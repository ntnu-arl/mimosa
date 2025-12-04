#!/usr/bin/env python

"""
A script to iterate over a ROS bag, accumulate point clouds into a global frame,
and save the final combined and downsampled point cloud.

This script includes a section to programmatically inject a known-missing static
transform into the TF buffer, allowing it to process bags with incomplete TF data.
"""

import argparse
import sys
import os

try:
    import rosbag
    import rospy
    import sensor_msgs.point_cloud2 as pc2
    import tf2_ros
    import tf2_py as tf2
    from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
    from tqdm import tqdm

    # Import message types needed for manual transform injection
    from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
except ImportError as e:
    sys.exit(
        "Could not import required libraries. Please ensure ROS is sourced and dependencies are installed.\n"
        "ROS: source /opt/ros/noetic/setup.bash\n"
        "Dependencies: pip install numpy open3d-python tqdm\n"
        f"Original error: {e}"
    )

import numpy as np
import open3d as o3d


def accumulate_pointclouds(bag_file, pcd_topic, target_frame, voxel_size, output_file):
    if not os.path.exists(bag_file):
        sys.exit(f"Error: Bag file not found at {bag_file}")

    print("--- Step 1/4: Reading bag file and preparing TF buffer ---")
    bag = rosbag.Bag(bag_file, "r")

    bag_start_time = bag.get_start_time()
    bag_end_time = bag.get_end_time()
    bag_duration = bag_end_time - bag_start_time

    print(f"  Bag duration: {bag_duration:.2f} seconds")

    # --- First Pass: Populate TF Buffer from Bag ---
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(bag_duration + 5.0))
    tf_message_count = bag.get_message_count(topic_filters=["/tf", "/tf_static"])

    with tqdm(total=tf_message_count, desc="[Pass 1/2] Populating TF") as pbar:
        for topic, msg, t in bag.read_messages(topics=["/tf", "/tf_static"]):
            if topic == "/tf":
                for transform in msg.transforms:
                    tf_buffer.set_transform(transform, "bag_importer")
            elif topic == "/tf_static":
                for transform in msg.transforms:
                    tf_buffer.set_transform_static(transform, "bag_importer")
            pbar.update(1)

    print("  TF buffer populated from bag.")

    # --- INJECT MISSING STATIC TRANSFORM ---
    # This section programmatically adds the missing mimosa_lidar -> mimosa_body transform.
    print("  Injecting known static transform: mimosa_lidar -> mimosa_body")

    static_ts = TransformStamped()
    static_ts.header.stamp = rospy.Time.from_sec(
        bag_start_time
    )  # Available for the whole bag duration
    static_ts.header.frame_id = "mimosa_body"  # The parent frame
    static_ts.child_frame_id = "mimosa_lidar"  # The child frame

    # Translation and Rotation from your tf_echo output
    static_ts.transform.translation = Vector3(0.0, -0.06, -0.04)
    static_ts.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)

    # Add it to the buffer. The "authority" string is just for debugging.
    tf_buffer.set_transform_static(static_ts, "manual_injecion")
    print("  Injection complete.")
    # ----------------------------------------

    # --- Second Pass: Process Point Clouds ---
    print("\n--- Step 2/4: Processing and transforming point clouds ---")
    accumulated_points = []

    message_count = bag.get_message_count(topic_filters=[pcd_topic])
    if message_count == 0:
        sys.exit(f"Error: No messages found on topic '{pcd_topic}'")

    print(f"  Found {message_count} messages on topic '{pcd_topic}'.")

    with tqdm(total=message_count, desc="[Pass 2/2] Accumulating") as pbar:
        for topic, msg, t in bag.read_messages(topics=[pcd_topic]):
            try:
                transform_stamped = tf_buffer.lookup_transform(
                    target_frame,
                    msg.header.frame_id,
                    msg.header.stamp,
                    timeout=rospy.Duration(0.0),
                )

                cloud_transformed = do_transform_cloud(msg, transform_stamped)
                points_generator = pc2.read_points(
                    cloud_transformed, field_names=("x", "y", "z"), skip_nans=True
                )
                accumulated_points.extend(list(points_generator))

            except (
                tf2.LookupException,
                tf2.ConnectivityException,
                tf2.ExtrapolationException,
            ) as e:
                rospy.logwarn(
                    f"Could not transform point cloud from {msg.header.frame_id} to {target_frame} at time {msg.header.stamp.to_sec()}: {e}"
                )

            pbar.update(1)

    bag.close()

    if not accumulated_points:
        sys.exit(
            "Error: No points were accumulated. Check your topics, frames, and TF data."
        )

    print(f"  Accumulated a total of {len(accumulated_points):,} raw points.")

    # --- Step 3: Post-Processing with Open3D ---
    print("\n--- Step 3/4: Downsampling point cloud ---")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(accumulated_points))
    print(f"  Downsampling with a voxel grid of size {voxel_size}m...")
    pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
    num_points_after = len(pcd_downsampled.points)
    print(f"  Downsampling complete. Final point count: {num_points_after:,}")

    # --- Step 4: Saving to File ---
    print("\n--- Step 4/4: Saving final point cloud ---")
    print(f"  Saving to file: {output_file}")
    o3d.io.write_point_cloud(output_file, pcd_downsampled)

    print("\n" + "=" * 40)
    print("      Processing Complete!")
    print("=" * 40)
    print(f"Final point cloud saved at: {output_file}")


if __name__ == "__main__":
    # --- Argparse and Node Init (Unchanged) ---
    parser = argparse.ArgumentParser(
        description="Accumulate point clouds from a ROS bag into a single file."
    )
    parser.add_argument(
        "-b", "--bag", required=True, help="Path to the input ROS bag file."
    )
    parser.add_argument(
        "-o",
        "--output",
        required=True,
        help="Path for the output point cloud file (e.g., 'map.pcd', 'map.ply').",
    )
    parser.add_argument(
        "-t",
        "--topic",
        required=True,
        help="The PointCloud2 topic to process (e.g., '/lidar/points').",
    )
    parser.add_argument(
        "-f",
        "--frame",
        required=True,
        help="The target global frame to transform all point clouds into (e.g., 'map', 'odom').",
    )
    parser.add_argument(
        "-v",
        "--voxel-size",
        type=float,
        default=0.005,
        help="The voxel size for downsampling in meters. Default is 0.005 (0.5cm).",
    )
    args = parser.parse_args()
    try:
        rospy.init_node("pcd_accumulator_node", anonymous=True, disable_signals=True)
    except rospy.exceptions.ROSInitException:
        pass
    accumulate_pointclouds(
        bag_file=args.bag,
        pcd_topic=args.topic,
        target_frame=args.frame,
        voxel_size=args.voxel_size,
        output_file=args.output,
    )
