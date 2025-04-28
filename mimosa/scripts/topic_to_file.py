#!/usr/bin/env python3

# Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

import rospy
from std_msgs.msg import String

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("topic", help="The topic to subscribe to")
parser.add_argument("file", help="The file to write to")
args = parser.parse_args()

rospy.init_node('topic_to_file')
msg = rospy.wait_for_message(args.topic, String)
with open(args.file, 'w') as f:
    f.write(msg.data)
