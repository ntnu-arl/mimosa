#!/usr/bin/env python3
"""
IMU-to-IMU Extrinsic Calibration Script

This script finds the relative pose (rotation and translation) and time offset
between two rigidly mounted IMUs from a ROS1 bag file.

Approach:
1. Time offset: Cross-correlate gyroscope magnitude signals to find time shift
2. Rotation: Solve Wahba's problem using gyroscope measurements
3. Translation (lever arm): Estimate from accelerometer differences accounting for
   centripetal and tangential accelerations
4. Biases: Estimated as constant offsets during optimization
"""

import numpy as np
from scipy import interpolate, optimize, signal
from scipy.spatial.transform import Rotation
import rosbag
import argparse
from dataclasses import dataclass
from typing import Tuple, List, Optional
import warnings

# =============================================================================
# CONFIGURATION - Set your topic names here
# =============================================================================
IMU1_TOPIC = "/lidar_imu"
IMU2_TOPIC = "/vectornav_driver_node/imu/data"

# =============================================================================
# Data structures
# =============================================================================
@dataclass
class ImuData:
    """Container for IMU measurements."""
    timestamps: np.ndarray  # in seconds
    gyro: np.ndarray        # Nx3 angular velocity [rad/s]
    accel: np.ndarray       # Nx3 linear acceleration [m/s^2]
    
    def __len__(self):
        return len(self.timestamps)


@dataclass 
class CalibrationResult:
    """Container for calibration results."""
    rotation_xyzw: np.ndarray      # Quaternion (x, y, z, w) from IMU1 to IMU2
    translation_xyz: np.ndarray    # Translation [m] from IMU1 to IMU2 in IMU1 frame
    time_offset: float             # Time offset: t_imu2 = t_imu1 + offset [s]
    gyro_bias_imu1: np.ndarray     # Gyroscope bias IMU1 [rad/s]
    gyro_bias_imu2: np.ndarray     # Gyroscope bias IMU2 [rad/s]
    accel_bias_imu1: np.ndarray    # Accelerometer bias IMU1 [m/s^2]
    accel_bias_imu2: np.ndarray    # Accelerometer bias IMU2 [m/s^2]
    rotation_residual: float       # RMS angular velocity residual [rad/s]
    accel_residual: float          # RMS acceleration residual [m/s^2]
    excitation_sufficient: bool    # Whether motion excitation was sufficient


# =============================================================================
# Bag reading
# =============================================================================
def read_imu_from_bag(bag_path: str, topic: str) -> ImuData:
    """Read IMU messages from a ROS bag file."""
    timestamps = []
    gyro = []
    accel = []
    
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            # Use header timestamp if available, otherwise bag time
            if hasattr(msg, 'header') and msg.header.stamp.to_sec() > 0:
                timestamps.append(msg.header.stamp.to_sec())
            else:
                timestamps.append(t.to_sec())
            
            gyro.append([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])
            accel.append([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
    
    if len(timestamps) == 0:
        raise ValueError(f"No messages found on topic {topic}")
    
    return ImuData(
        timestamps=np.array(timestamps),
        gyro=np.array(gyro),
        accel=np.array(accel)
    )


# =============================================================================
# Excitation checking
# =============================================================================
def check_excitation(imu1: ImuData, imu2: ImuData, 
                     min_gyro_std: float = 0.1,      # rad/s
                     min_accel_std: float = 0.5,     # m/s^2
                     min_axis_ratio: float = 0.1) -> Tuple[bool, List[str]]:
    """
    Check if motion excitation is sufficient for calibration.
    
    Returns:
        (is_sufficient, list_of_warnings)
    """
    warnings_list = []
    
    # Check gyroscope excitation
    gyro1_std = np.std(imu1.gyro, axis=0)
    gyro2_std = np.std(imu2.gyro, axis=0)
    
    for i, axis in enumerate(['X', 'Y', 'Z']):
        if gyro1_std[i] < min_gyro_std and gyro2_std[i] < min_gyro_std:
            warnings_list.append(
                f"Low rotational excitation around {axis}-axis: "
                f"IMU1 std={gyro1_std[i]:.4f}, IMU2 std={gyro2_std[i]:.4f} rad/s "
                f"(min recommended: {min_gyro_std} rad/s)"
            )
    
    # Check if rotation is excited in multiple axes (needed for full rotation estimation)
    gyro_total_std = np.sqrt(gyro1_std**2 + gyro2_std**2) / np.sqrt(2)
    if np.max(gyro_total_std) > 0:
        axis_ratios = gyro_total_std / np.max(gyro_total_std)
        poorly_excited = axis_ratios < min_axis_ratio
        if np.sum(~poorly_excited) < 2:
            warnings_list.append(
                f"Rotation only excited around {np.sum(~poorly_excited)} axis/axes. "
                f"At least 2 axes recommended for robust rotation estimation. "
                f"Axis ratios: X={axis_ratios[0]:.2f}, Y={axis_ratios[1]:.2f}, Z={axis_ratios[2]:.2f}"
            )
    
    # Check accelerometer excitation (beyond gravity)
    accel1_std = np.std(imu1.accel, axis=0)
    accel2_std = np.std(imu2.accel, axis=0)
    
    accel_excited_axes = 0
    for i, axis in enumerate(['X', 'Y', 'Z']):
        if accel1_std[i] >= min_accel_std or accel2_std[i] >= min_accel_std:
            accel_excited_axes += 1
    
    if accel_excited_axes < 2:
        warnings_list.append(
            f"Low acceleration variation (only {accel_excited_axes} axes excited). "
            f"IMU1 accel std: [{accel1_std[0]:.3f}, {accel1_std[1]:.3f}, {accel1_std[2]:.3f}] m/s^2, "
            f"IMU2 accel std: [{accel2_std[0]:.3f}, {accel2_std[1]:.3f}, {accel2_std[2]:.3f}] m/s^2. "
            f"Lever arm estimation may be inaccurate."
        )
    
    # Check data duration
    duration1 = imu1.timestamps[-1] - imu1.timestamps[0]
    duration2 = imu2.timestamps[-1] - imu2.timestamps[0]
    if duration1 < 2.0 or duration2 < 2.0:
        warnings_list.append(
            f"Short data duration: IMU1={duration1:.2f}s, IMU2={duration2:.2f}s. "
            f"At least 2-3 seconds recommended."
        )
    
    is_sufficient = len(warnings_list) == 0
    return is_sufficient, warnings_list


# =============================================================================
# Time synchronization
# =============================================================================
def estimate_time_offset(imu1: ImuData, imu2: ImuData, 
                         max_offset: float = 0.01) -> Tuple[float, float]:
    """
    Estimate time offset between two IMUs using cross-correlation of gyro magnitude.
    
    Args:
        imu1, imu2: IMU data
        max_offset: Maximum expected offset in seconds
        
    Returns:
        (time_offset, correlation_score): offset such that t_imu2 = t_imu1 + offset
    """
    # Compute gyroscope magnitude
    gyro1_mag = np.linalg.norm(imu1.gyro, axis=1)
    gyro2_mag = np.linalg.norm(imu2.gyro, axis=1)
    
    # Determine common time range and resample to uniform grid
    t_start = max(imu1.timestamps[0], imu2.timestamps[0])
    t_end = min(imu1.timestamps[-1], imu2.timestamps[-1])
    
    # Use higher of the two sample rates
    dt1 = np.median(np.diff(imu1.timestamps))
    dt2 = np.median(np.diff(imu2.timestamps))
    dt = min(dt1, dt2)
    
    t_common = np.arange(t_start, t_end, dt)
    
    if len(t_common) < 100:
        warnings.warn("Very short overlapping time range for cross-correlation")
    
    # Interpolate to common time base
    interp1 = interpolate.interp1d(imu1.timestamps, gyro1_mag, 
                                    kind='linear', fill_value='extrapolate')
    interp2 = interpolate.interp1d(imu2.timestamps, gyro2_mag,
                                    kind='linear', fill_value='extrapolate')
    
    sig1 = interp1(t_common)
    sig2 = interp2(t_common)
    
    # Remove mean
    sig1 = sig1 - np.mean(sig1)
    sig2 = sig2 - np.mean(sig2)
    
    # Cross-correlation
    correlation = signal.correlate(sig2, sig1, mode='full')
    lags = signal.correlation_lags(len(sig2), len(sig1), mode='full')
    lag_times = lags * dt
    
    # Only consider lags within max_offset
    valid_mask = np.abs(lag_times) <= max_offset
    if not np.any(valid_mask):
        valid_mask = np.ones_like(lag_times, dtype=bool)
    
    correlation_valid = correlation.copy()
    correlation_valid[~valid_mask] = -np.inf
    
    best_idx = np.argmax(correlation_valid)
    time_offset = lag_times[best_idx]
    
    # Compute normalized correlation score
    norm = np.sqrt(np.sum(sig1**2) * np.sum(sig2**2))
    if norm > 0:
        corr_score = correlation[best_idx] / norm
    else:
        corr_score = 0.0
    
    return time_offset, corr_score


def refine_time_offset(imu1: ImuData, imu2: ImuData, 
                       initial_offset: float,
                       R_1_to_2: np.ndarray,
                       search_range: float = 0.002) -> float:
    """
    Refine time offset by minimizing gyroscope residuals after rotation alignment.
    """
    # Create interpolators for imu2 gyro
    interp2 = [
        interpolate.interp1d(imu2.timestamps, imu2.gyro[:, i],
                            kind='linear', fill_value='extrapolate')
        for i in range(3)
    ]
    
    def cost(offset):
        # Shift imu2 timestamps
        t2_shifted = imu1.timestamps + offset[0]
        
        # Interpolate imu2 gyro at shifted times
        gyro2_interp = np.column_stack([f(t2_shifted) for f in interp2])
        
        # Transform imu1 gyro to imu2 frame
        gyro1_in_2 = (R_1_to_2 @ imu1.gyro.T).T
        
        # Compute residual
        residual = gyro2_interp - gyro1_in_2
        return np.sum(residual**2)
    
    result = optimize.minimize(
        cost, 
        x0=[initial_offset],
        bounds=[(initial_offset - search_range, initial_offset + search_range)],
        method='L-BFGS-B'
    )
    
    return result.x[0]


# =============================================================================
# Rotation estimation (Wahba's problem)
# =============================================================================
def estimate_rotation_wahba(gyro1: np.ndarray, gyro2: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Estimate rotation from IMU1 frame to IMU2 frame using SVD solution to Wahba's problem.
    
    Given: gyro2 = R @ gyro1 (both Nx3)
    Find: R that minimizes sum of ||gyro2[i] - R @ gyro1[i]||^2
    
    Returns:
        R: 3x3 rotation matrix from IMU1 to IMU2
        residual: RMS residual in rad/s
    """
    # Build correlation matrix B = sum(gyro2[i] @ gyro1[i].T)
    B = gyro2.T @ gyro1
    
    # SVD
    U, S, Vt = np.linalg.svd(B)
    
    # Ensure proper rotation (det = +1)
    det = np.linalg.det(U @ Vt)
    D = np.diag([1, 1, det])
    
    R = U @ D @ Vt
    
    # Compute residual
    gyro1_rotated = (R @ gyro1.T).T
    residual = np.sqrt(np.mean(np.sum((gyro2 - gyro1_rotated)**2, axis=1)))
    
    return R, residual


def estimate_rotation_and_biases(gyro1: np.ndarray, gyro2: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Estimate rotation and gyroscope biases jointly.
    
    Model: gyro2 - b2 = R @ (gyro1 - b1)
    
    Returns:
        R: rotation matrix
        bias1: gyro bias for IMU1
        bias2: gyro bias for IMU2  
        residual: RMS residual
    """
    def cost(params):
        # params: [b1x, b1y, b1z, b2x, b2y, b2z, rx, ry, rz] (rotation as axis-angle)
        b1 = params[0:3]
        b2 = params[3:6]
        rvec = params[6:9]
        
        R = Rotation.from_rotvec(rvec).as_matrix()
        
        gyro1_corrected = gyro1 - b1
        gyro2_corrected = gyro2 - b2
        
        gyro1_rotated = (R @ gyro1_corrected.T).T
        residual = gyro2_corrected - gyro1_rotated
        
        return np.sum(residual**2)
    
    # Initial guess: zero biases, estimate rotation without bias correction
    R_init, _ = estimate_rotation_wahba(gyro1, gyro2)
    rvec_init = Rotation.from_matrix(R_init).as_rotvec()
    
    x0 = np.concatenate([np.zeros(3), np.zeros(3), rvec_init])
    
    result = optimize.minimize(cost, x0, method='L-BFGS-B')
    
    b1 = result.x[0:3]
    b2 = result.x[3:6]
    rvec = result.x[6:9]
    R = Rotation.from_matrix(Rotation.from_rotvec(rvec).as_matrix()).as_matrix()
    
    # Compute final residual
    gyro1_corrected = gyro1 - b1
    gyro2_corrected = gyro2 - b2
    gyro1_rotated = (R @ gyro1_corrected.T).T
    residual = np.sqrt(np.mean(np.sum((gyro2_corrected - gyro1_rotated)**2, axis=1)))
    
    return R, b1, b2, residual


# =============================================================================
# Translation (lever arm) estimation
# =============================================================================
def estimate_lever_arm(accel1: np.ndarray, accel2: np.ndarray,
                       gyro1: np.ndarray, R_1_to_2: np.ndarray,
                       gyro_bias1: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Estimate lever arm (translation) from accelerometer measurements.
    
    For rigidly connected IMUs:
    a2 = R @ a1 + R @ (omega_dot x r + omega x (omega x r))
    
    where r is the lever arm from IMU1 to IMU2 in IMU1 frame.
    
    We solve: a2 - R @ a1 = R @ A @ r + b2 - R @ b1
    where A is constructed from angular velocity terms.
    
    Returns:
        lever_arm: translation from IMU1 to IMU2 in IMU1 frame
        accel_bias1: accelerometer bias IMU1
        accel_bias2: accelerometer bias IMU2
        residual: RMS acceleration residual
    """
    N = len(accel1)
    
    # Correct gyro for bias
    omega = gyro1 - gyro_bias1
    
    # Estimate angular acceleration (numerical derivative)
    dt = 1.0 / N  # Approximate, will be normalized anyway
    omega_dot = np.gradient(omega, axis=0) / dt
    
    # Build the system: for each measurement
    # a2 - R @ a1 = R @ (omega_dot x r + omega x (omega x r))
    # This is linear in r
    
    # Transform accel1 to frame 2
    accel1_in_2 = (R_1_to_2 @ accel1.T).T
    
    # Residual accelerations
    delta_accel = accel2 - accel1_in_2
    
    # Build design matrix for lever arm
    # omega x (omega x r) = omega (omega . r) - r (omega . omega)
    # omega_dot x r is also linear in r
    
    def skew(v):
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
    
    # Build matrices for least squares
    # Model: delta_accel[i] = R @ A[i] @ r + (b2 - R @ b1)
    # Unknowns: r (3), b1 (3), b2 (3) = 9 parameters
    
    A_matrices = []
    for i in range(N):
        w = omega[i]
        w_dot = omega_dot[i]
        # centripetal + tangential: omega_dot x r + omega x (omega x r)
        # = skew(omega_dot) @ r + skew(omega) @ skew(omega) @ r
        A_i = skew(w_dot) + skew(w) @ skew(w)
        A_matrices.append(A_i)
    
    # Stack into big system
    # [R @ A_i, -R, I] @ [r; b1; b2] = delta_accel[i]
    
    H = np.zeros((3 * N, 9))
    y = np.zeros(3 * N)
    
    for i in range(N):
        H[3*i:3*i+3, 0:3] = R_1_to_2 @ A_matrices[i]
        H[3*i:3*i+3, 3:6] = -R_1_to_2
        H[3*i:3*i+3, 6:9] = np.eye(3)
        y[3*i:3*i+3] = delta_accel[i]
    
    # Solve least squares
    result, residuals, rank, s = np.linalg.lstsq(H, y, rcond=None)
    
    lever_arm = result[0:3]
    accel_bias1 = result[3:6]
    accel_bias2 = result[6:9]
    
    # Compute residual
    y_pred = H @ result
    residual = np.sqrt(np.mean((y - y_pred)**2))
    
    return lever_arm, accel_bias1, accel_bias2, residual


# =============================================================================
# Main calibration routine
# =============================================================================
def calibrate_imus(bag_path: str, 
                   imu1_topic: str = IMU1_TOPIC,
                   imu2_topic: str = IMU2_TOPIC,
                   max_time_offset: float = 0.01) -> CalibrationResult:
    """
    Main calibration function.
    
    Args:
        bag_path: Path to ROS bag file
        imu1_topic: Topic name for IMU1
        imu2_topic: Topic name for IMU2
        max_time_offset: Maximum expected time offset in seconds
        
    Returns:
        CalibrationResult with all estimated parameters
    """
    print(f"Reading bag file: {bag_path}")
    print(f"  IMU1 topic: {imu1_topic}")
    print(f"  IMU2 topic: {imu2_topic}")
    
    # Read data
    imu1 = read_imu_from_bag(bag_path, imu1_topic)
    imu2 = read_imu_from_bag(bag_path, imu2_topic)
    
    print(f"\nData loaded:")
    print(f"  IMU1: {len(imu1)} samples, {imu1.timestamps[-1] - imu1.timestamps[0]:.2f}s duration")
    print(f"  IMU2: {len(imu2)} samples, {imu2.timestamps[-1] - imu2.timestamps[0]:.2f}s duration")
    
    # Check excitation
    print("\nChecking motion excitation...")
    excitation_ok, excitation_warnings = check_excitation(imu1, imu2)
    
    if not excitation_ok:
        print("\n" + "="*60)
        print("WARNING: INSUFFICIENT MOTION EXCITATION DETECTED")
        print("="*60)
        for warn in excitation_warnings:
            print(f"  - {warn}")
        print("="*60)
        print("Results may be unreliable. Consider recording with more motion.\n")
    else:
        print("  Motion excitation appears sufficient.")
    
    # Step 1: Estimate time offset
    print("\nStep 1: Estimating time offset...")
    time_offset, corr_score = estimate_time_offset(imu1, imu2, max_time_offset)
    print(f"  Initial time offset: {time_offset*1000:.3f} ms (correlation: {corr_score:.3f})")
    
    if corr_score < 0.5:
        print("  WARNING: Low correlation score. Time offset estimate may be unreliable.")
    
    # Step 2: Interpolate IMU2 to IMU1 timestamps (with offset correction)
    print("\nStep 2: Aligning data temporally...")
    t1 = imu1.timestamps
    t2_target = t1 + time_offset
    
    # Check if target times are within imu2 range
    valid_mask = (t2_target >= imu2.timestamps[0]) & (t2_target <= imu2.timestamps[-1])
    if np.sum(valid_mask) < 100:
        raise ValueError("Insufficient overlapping data after time alignment")
    
    t1 = t1[valid_mask]
    t2_target = t2_target[valid_mask]
    gyro1 = imu1.gyro[valid_mask]
    accel1 = imu1.accel[valid_mask]
    
    # Interpolate IMU2 data
    gyro2_interp = np.column_stack([
        interpolate.interp1d(imu2.timestamps, imu2.gyro[:, i], kind='linear')(t2_target)
        for i in range(3)
    ])
    accel2_interp = np.column_stack([
        interpolate.interp1d(imu2.timestamps, imu2.accel[:, i], kind='linear')(t2_target)
        for i in range(3)
    ])
    
    print(f"  Using {len(t1)} aligned samples")
    
    # Step 3: Estimate rotation and gyro biases
    print("\nStep 3: Estimating rotation and gyroscope biases...")
    R_1_to_2, gyro_bias1, gyro_bias2, gyro_residual = estimate_rotation_and_biases(
        gyro1, gyro2_interp
    )
    
    # Convert to quaternion (x, y, z, w)
    rot = Rotation.from_matrix(R_1_to_2)
    quat_xyzw = rot.as_quat()  # scipy returns [x, y, z, w]
    euler_xyz = rot.as_euler('xyz', degrees=True)
    
    print(f"  Rotation (euler xyz): [{euler_xyz[0]:.2f}, {euler_xyz[1]:.2f}, {euler_xyz[2]:.2f}] deg")
    print(f"  Gyro bias IMU1: [{gyro_bias1[0]:.6f}, {gyro_bias1[1]:.6f}, {gyro_bias1[2]:.6f}] rad/s")
    print(f"  Gyro bias IMU2: [{gyro_bias2[0]:.6f}, {gyro_bias2[1]:.6f}, {gyro_bias2[2]:.6f}] rad/s")
    print(f"  Gyro residual: {gyro_residual:.6f} rad/s RMS")
    
    # Step 4: Refine time offset
    print("\nStep 4: Refining time offset...")
    time_offset_refined = refine_time_offset(imu1, imu2, time_offset, R_1_to_2)
    print(f"  Refined time offset: {time_offset_refined*1000:.3f} ms")
    
    # Re-interpolate with refined offset if significantly different
    if abs(time_offset_refined - time_offset) > 0.0001:
        t2_target = t1 + time_offset_refined
        valid_mask = (t2_target >= imu2.timestamps[0]) & (t2_target <= imu2.timestamps[-1])
        t1 = imu1.timestamps[valid_mask]
        t2_target = t1 + time_offset_refined
        gyro1 = imu1.gyro[valid_mask]
        accel1 = imu1.accel[valid_mask]
        
        gyro2_interp = np.column_stack([
            interpolate.interp1d(imu2.timestamps, imu2.gyro[:, i], kind='linear')(t2_target)
            for i in range(3)
        ])
        accel2_interp = np.column_stack([
            interpolate.interp1d(imu2.timestamps, imu2.accel[:, i], kind='linear')(t2_target)
            for i in range(3)
        ])
    
    # Step 5: Estimate lever arm (translation)
    print("\nStep 5: Estimating lever arm (translation)...")
    lever_arm, accel_bias1, accel_bias2, accel_residual = estimate_lever_arm(
        accel1, accel2_interp, gyro1, R_1_to_2, gyro_bias1
    )
    
    print(f"  Lever arm: [{lever_arm[0]:.4f}, {lever_arm[1]:.4f}, {lever_arm[2]:.4f}] m")
    print(f"  Accel bias IMU1: [{accel_bias1[0]:.4f}, {accel_bias1[1]:.4f}, {accel_bias1[2]:.4f}] m/s^2")
    print(f"  Accel bias IMU2: [{accel_bias2[0]:.4f}, {accel_bias2[1]:.4f}, {accel_bias2[2]:.4f}] m/s^2")
    print(f"  Accel residual: {accel_residual:.4f} m/s^2 RMS")
    
    # Build result
    result = CalibrationResult(
        rotation_xyzw=quat_xyzw,
        translation_xyz=lever_arm,
        time_offset=time_offset_refined,
        gyro_bias_imu1=gyro_bias1,
        gyro_bias_imu2=gyro_bias2,
        accel_bias_imu1=accel_bias1,
        accel_bias_imu2=accel_bias2,
        rotation_residual=gyro_residual,
        accel_residual=accel_residual,
        excitation_sufficient=excitation_ok
    )
    
    return result


def print_results(result: CalibrationResult):
    """Print calibration results in a clear format."""
    print("\n" + "="*60)
    print("CALIBRATION RESULTS")
    print("="*60)
    
    print(f"\nRelative Pose (IMU1 to IMU2):")
    print(f"  Translation (xyz): [{result.translation_xyz[0]:.6f}, {result.translation_xyz[1]:.6f}, {result.translation_xyz[2]:.6f}] m")
    print(f"  Rotation (xyzw):   [{result.rotation_xyzw[0]:.6f}, {result.rotation_xyzw[1]:.6f}, {result.rotation_xyzw[2]:.6f}, {result.rotation_xyzw[3]:.6f}]")
    
    # Also print as euler for readability
    euler = Rotation.from_quat(result.rotation_xyzw).as_euler('xyz', degrees=True)
    print(f"  Rotation (euler):  [{euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f}] deg")
    
    print(f"\nTime Offset:")
    print(f"  t_imu2 = t_imu1 + {result.time_offset*1000:.4f} ms")
    
    print(f"\nGyroscope Biases:")
    print(f"  IMU1: [{result.gyro_bias_imu1[0]:.6f}, {result.gyro_bias_imu1[1]:.6f}, {result.gyro_bias_imu1[2]:.6f}] rad/s")
    print(f"  IMU2: [{result.gyro_bias_imu2[0]:.6f}, {result.gyro_bias_imu2[1]:.6f}, {result.gyro_bias_imu2[2]:.6f}] rad/s")
    
    print(f"\nAccelerometer Biases:")
    print(f"  IMU1: [{result.accel_bias_imu1[0]:.4f}, {result.accel_bias_imu1[1]:.4f}, {result.accel_bias_imu1[2]:.4f}] m/s^2")
    print(f"  IMU2: [{result.accel_bias_imu2[0]:.4f}, {result.accel_bias_imu2[1]:.4f}, {result.accel_bias_imu2[2]:.4f}] m/s^2")
    
    print(f"\nResiduals:")
    print(f"  Gyroscope: {result.rotation_residual:.6f} rad/s RMS")
    print(f"  Accelerometer: {result.accel_residual:.4f} m/s^2 RMS")
    
    print(f"\nExcitation Sufficient: {result.excitation_sufficient}")
    
    print("="*60)


# =============================================================================
# Entry point
# =============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calibrate relative pose between two rigidly mounted IMUs"
    )
    parser.add_argument("bag_file", help="Path to ROS bag file")
    parser.add_argument("--imu1", default=IMU1_TOPIC, help=f"Topic name for IMU1 (default: {IMU1_TOPIC})")
    parser.add_argument("--imu2", default=IMU2_TOPIC, help=f"Topic name for IMU2 (default: {IMU2_TOPIC})")
    parser.add_argument("--max-offset", type=float, default=0.01, 
                        help="Maximum expected time offset in seconds (default: 0.01)")
    
    args = parser.parse_args()
    
    try:
        result = calibrate_imus(
            args.bag_file,
            imu1_topic=args.imu1,
            imu2_topic=args.imu2,
            max_time_offset=args.max_offset
        )
        print_results(result)
        
    except Exception as e:
        print(f"\nError: {e}")
        raise