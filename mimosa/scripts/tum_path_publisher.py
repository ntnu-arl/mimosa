#!/usr/bin/env python3
"""
TUM Path Publisher - Publishes a Path ROS message from a TUM format file.
TUM format: timestamp tx ty tz qx qy qz qw (space-delimited)
"""

import rospy
import argparse
import csv
import sys
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import (
    quaternion_matrix,
    translation_matrix,
    concatenate_matrices,
    quaternion_from_matrix,
    translation_from_matrix,
)


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Publish a path msg from a file (in tum format)"
    )
    parser.add_argument("file_name", type=str, help="Path to TUM format file")
    parser.add_argument(
        "--frame_id", type=str, default="map", help="Frame ID for the path"
    )
    parser.add_argument(
        "--topic_name", type=str, default="tum_path", help="Topic to publish path on"
    )
    parser.add_argument("--hz", type=float, default=0.5, help="Publishing rate in Hz")

    # Add transform arguments
    parser.add_argument(
        "--transform",
        type=float,
        nargs=7,
        metavar=("x", "y", "z", "qx", "qy", "qz", "qw"),
        help="Transform to apply (x y z qx qy qz qw)",
        default=None,
    )
    parser.add_argument(
        "--transform_side",
        type=str,
        choices=["left", "right"],
        default="left",
        help="Apply transform on the left or right side of pose",
    )

    return parser.parse_args()


def apply_transform(pose, transform_matrix, side="left"):
    """
    Apply a transform to a pose.

    Args:
        pose: Pose object to transform
        transform_matrix: 4x4 homogeneous transformation matrix
        side: 'left' or 'right' - determines if transform is pre or post-multiplied

    Returns:
        Transformed pose
    """
    # Extract position and orientation from pose
    p_pos = [pose.position.x, pose.position.y, pose.position.z]
    p_quat = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]

    # Create pose transformation matrix
    p_trans = translation_matrix(p_pos)
    p_rot = quaternion_matrix(p_quat)
    p_matrix = concatenate_matrices(p_trans, p_rot)

    # Apply transform based on side
    if side == "left":
        result_matrix = concatenate_matrices(transform_matrix, p_matrix)
    else:  # right
        result_matrix = concatenate_matrices(p_matrix, transform_matrix)

    # Extract new position and orientation using proper tf functions
    new_pos = translation_from_matrix(result_matrix)
    new_quat = quaternion_from_matrix(result_matrix)

    # Create new pose
    new_pose = Pose()
    new_pose.position = Point(new_pos[0], new_pos[1], new_pos[2])
    new_pose.orientation = Quaternion(
        new_quat[0], new_quat[1], new_quat[2], new_quat[3]
    )

    return new_pose


def create_path_from_tum_file(
    file_path, frame_id, transform=None, transform_side="left"
):
    """Read TUM file and create a path message."""
    path_msg = Path()
    path_msg.header.frame_id = frame_id

    # Precompute transform matrix if a transform is provided
    transform_matrix = None
    if transform is not None:
        # Extract transform components
        t_pos = transform[0:3]
        t_quat = transform[3:7]

        # Create transformation matrix once
        t_trans = translation_matrix(t_pos)
        t_rot = quaternion_matrix(t_quat)
        transform_matrix = concatenate_matrices(t_trans, t_rot)

    try:
        with open(file_path) as tum_file:
            tum_reader = csv.reader(tum_file, delimiter=" ")
            for line in tum_reader:
                # Skip empty lines or comments
                if not line or line[0].startswith("#"):
                    continue

                # Filter out any empty strings that might come from extra spaces
                line = [item for item in line if item]

                # Ensure line has enough elements
                if len(line) < 8:
                    rospy.logwarn(f"Skipping invalid line: {line}")
                    continue

                try:
                    ps = PoseStamped()
                    ps.header.frame_id = frame_id
                    ps.header.stamp = rospy.Time.from_sec(float(line[0]))

                    # Create original pose
                    original_pose = Pose()
                    original_pose.position.x = float(line[1])
                    original_pose.position.y = float(line[2])
                    original_pose.position.z = float(line[3])
                    original_pose.orientation.x = float(line[4])
                    original_pose.orientation.y = float(line[5])
                    original_pose.orientation.z = float(line[6])
                    original_pose.orientation.w = float(line[7])

                    # Apply transform if provided
                    if transform_matrix is not None:
                        ps.pose = apply_transform(
                            original_pose, transform_matrix, transform_side
                        )
                    else:
                        ps.pose = original_pose

                    path_msg.poses.append(ps)
                except ValueError as e:
                    rospy.logwarn(f"Error parsing line {line}: {e}")

            rospy.loginfo(f"Loaded {len(path_msg.poses)} poses from {file_path}")

            if not path_msg.poses:
                rospy.logwarn("No valid poses found in file!")

    except FileNotFoundError:
        rospy.logerr(f"File not found: {file_path}")
        sys.exit(1)
    except Exception as e:
        rospy.logerr(f"Error reading file: {e}")
        sys.exit(1)

    return path_msg


def main():
    """Main function."""
    # Parse arguments
    args = parse_arguments()

    # Initialize ROS node
    rospy.init_node("tum_path_publisher", anonymous=True)

    # Create publisher
    pub = rospy.Publisher(args.topic_name, Path, queue_size=1, latch=True)

    # Read TUM file and create path
    path_msg = create_path_from_tum_file(
        args.file_name, args.frame_id, args.transform, args.transform_side
    )

    if args.transform:
        rospy.loginfo(f"Applied {args.transform_side} transform: {args.transform}")

    # Publish path at specified rate
    rate = rospy.Rate(args.hz)
    rospy.loginfo(f"Publishing path on topic {args.topic_name} at {args.hz} Hz")

    try:
        while not rospy.is_shutdown():
            path_msg.header.stamp = rospy.Time.now()
            pub.publish(path_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("Path publisher stopped")


if __name__ == "__main__":
    main()
