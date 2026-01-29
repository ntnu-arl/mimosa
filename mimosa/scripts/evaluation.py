#!/usr/bin/env python3
"""
Trajectory evaluation script using evo tools.

This script evaluates odometry results from ROS bags against ground truth
by computing RPE (Relative Pose Error) and ATE (Absolute Trajectory Error).
"""

import subprocess
import shutil
from pathlib import Path
from typing import List

# =============================================================================
# CONFIGURATION - Set these values
# =============================================================================

# BASE_FOLDER = "~/rosbags/nk4/lab/bags/2025_11_20_runehamar/hornbill/results/flight8"
BASE_FOLDER = "/home/cyanide/rosbags/nk4/lab/bags/fyllingsdal/day_2/results/500m_7_fast"

# Method names and their corresponding odometry topics
METHODS = {
    "mimosa": "/mimosa_node/imu/manager/odometry",
    "fast_livo2": "/LIVO2/imu_propagate",
}

# =============================================================================
# END CONFIGURATION
# =============================================================================


def topic_to_filename(topic: str) -> str:
    """Convert a ROS topic name to the filename evo_traj generates."""
    # evo_traj replaces '/' with '_' and removes leading underscore
    filename = topic.replace("/", "_")
    if filename.startswith("_"):
        filename = filename[1:]
    return filename + ".tum"


def run_command(cmd: List[str], cwd: str = None) -> bool:
    """Run a command and return True if successful."""
    print(f"  Running: {' '.join(cmd)}")
    try:
        result = subprocess.run(
            cmd,
            cwd=cwd,
            capture_output=True,
            text=True,
            check=True
        )
        return True
    except subprocess.CalledProcessError as e:
        print(f"  Error: {e.stderr}")
        return False


def extract_tum_from_bag(bag_path: Path, topic: str, output_tum: Path) -> bool:
    """Extract trajectory from bag and save as TUM format."""
    bag_dir = bag_path.parent
    bag_name = bag_path.name
    
    # Run evo_traj to extract all topics as TUM
    cmd = ["evo_traj", "bag", bag_name, "--save_as_tum", "--all_topics"]
    if not run_command(cmd, cwd=str(bag_dir)):
        return False
    
    # Find the generated TUM file for our topic
    expected_tum_name = topic_to_filename(topic)
    generated_tum = bag_dir / expected_tum_name
    
    if not generated_tum.exists():
        print(f"  Error: Expected TUM file {generated_tum} not found")
        # List what was generated for debugging
        tum_files = list(bag_dir.glob("*.tum"))
        print(f"  Generated TUM files: {[f.name for f in tum_files]}")
        return False
    
    # Move to the desired output name
    shutil.move(str(generated_tum), str(output_tum))
    print(f"  Created: {output_tum.name}")
    
    # Clean up other generated TUM files
    for tum_file in bag_dir.glob("*.tum"):
        if tum_file != output_tum:
            tum_file.unlink()
    
    return True


def compute_rpe(gt_tum: Path, traj_tum: Path, output_zip: Path) -> bool:
    """Compute Relative Pose Error."""
    cmd = [
        "evo_rpe", "tum",
        str(gt_tum),
        str(traj_tum),
        "--align",
        "-r", "point_distance",
        "--delta", "10",
        "--delta_unit", "m",
        "--save_results", str(output_zip)
    ]
    return run_command(cmd)


def compute_ate(gt_tum: Path, traj_tum: Path, output_zip: Path) -> bool:
    """Compute Absolute Trajectory Error."""
    cmd = [
        "evo_ape", "tum",
        str(gt_tum),
        str(traj_tum),
        "--align",
        "-r", "trans_part",
        "--save_results", str(output_zip)
    ]
    return run_command(cmd)


def run_evo_res(result_zips: List[Path], output_path: Path) -> bool:
    """Run evo_res on a list of result zip files."""
    if not result_zips:
        print("  No result files to process")
        return False
    
    cmd = ["evo_res"] + [str(z) for z in result_zips] + ["--save_table", str(output_path)]
    return run_command(cmd)


def main():
    base_folder = Path(BASE_FOLDER).expanduser().resolve()
    gt_tum = base_folder / "gt_odometry.tum"
    result_bags_dir = base_folder / "result_bags"
    
    print(f"Base folder: {base_folder}")
    print(f"Ground truth: {gt_tum}")
    print(f"Methods: {list(METHODS.keys())}")
    
    if not gt_tum.exists():
        print(f"Error: Ground truth file not found: {gt_tum}")
        return 1
    
    if not result_bags_dir.exists():
        print(f"Error: Result bags directory not found: {result_bags_dir}")
        return 1
    
    all_rpe_results = []
    all_ate_results = []
    
    # Process each method
    for method, topic in METHODS.items():
        print(f"\n{'='*60}")
        print(f"Processing method: {method}")
        print(f"Topic: {topic}")
        print("=" * 60)
        
        method_dir = result_bags_dir / method
        if not method_dir.exists():
            print(f"Warning: Method directory not found: {method_dir}")
            continue
        
        # Find all bag files in the method directory
        bag_files = list(method_dir.glob("*.bag"))
        if not bag_files:
            print(f"Warning: No bag files found in {method_dir}")
            continue
        
        for bag_file in sorted(bag_files):
            bag_name = bag_file.stem  # filename without extension
            print(f"\n  Processing: {bag_file.name}")
            
            # Output paths
            traj_tum = method_dir / f"{bag_name}.tum"
            rpe_zip = method_dir / f"rpe_{bag_name}.zip"
            ate_zip = method_dir / f"ape_{bag_name}.zip"
            
            # Step 1: Extract TUM trajectory from bag
            if not extract_tum_from_bag(bag_file, topic, traj_tum):
                print(f"  Skipping {bag_name} due to extraction error")
                continue
            
            # Step 2: Compute RPE
            print(f"  Computing RPE...")
            if compute_rpe(gt_tum, traj_tum, rpe_zip):
                all_rpe_results.append(rpe_zip)
            
            # Step 3: Compute ATE
            print(f"  Computing ATE...")
            if compute_ate(gt_tum, traj_tum, ate_zip):
                all_ate_results.append(ate_zip)
    
    # Generate summary results
    print(f"\n{'='*60}")
    print("Generating summary results")
    print("=" * 60)
    
    if all_rpe_results:
        print("\nRPE Summary:")
        rpe_table = base_folder / "rpe_summary.csv"
        run_evo_res(all_rpe_results, rpe_table)
    
    if all_ate_results:
        print("\nATE Summary:")
        ate_table = base_folder / "ate_summary.csv"
        run_evo_res(all_ate_results, ate_table)
    
    print(f"\n{'='*60}")
    print("Evaluation complete!")
    print("=" * 60)
    
    return 0


if __name__ == "__main__":
    exit(main())