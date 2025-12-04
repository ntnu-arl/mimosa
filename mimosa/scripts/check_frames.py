#!/usr/bin/env python
import rosbag
import sys

if len(sys.argv) < 2:
    print("Usage: python check_frames.py <path_to_your_bag.bag>")
    sys.exit(1)

bag_file = sys.argv[1]
all_frames = set()

print(f"Analyzing frames in {bag_file}...")

with rosbag.Bag(bag_file, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=["/tf", "/tf_static"]):
        for transform in msg.transforms:
            all_frames.add(transform.header.frame_id)
            all_frames.add(transform.child_frame_id)

if not all_frames:
    print("\nError: No frames found in /tf or /tf_static topics.")
    print(
        "Please check if these topics were recorded in your bag file with 'rosbag info'."
    )
else:
    print("\nFound the following unique frames in the bag file:")
    for frame in sorted(list(all_frames)):
        print(f"- {frame}")
