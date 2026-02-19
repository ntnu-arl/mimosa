#!/usr/bin/env python3
"""
Convert ROS 2 Jazzy/Iron rosbag2 metadata to Humble-compatible format.

Fixes:
1. Removes 'type_description_hash' fields (not supported in Humble)
2. Converts empty offered_qos_profiles from [] to '' (yaml-cpp compatibility)
3. Downgrades metadata version from 9 to 8
"""

import sys
import shutil
import re
from pathlib import Path


def convert_metadata(bag_path: str, backup: bool = True):
    """
    Convert a rosbag2 metadata.yaml to be Humble-compatible.
    Uses regex-based approach to avoid yaml library formatting issues.
    """
    bag_dir = Path(bag_path)
    metadata_file = bag_dir / "metadata.yaml"

    if not metadata_file.exists():
        print(f"Error: metadata.yaml not found in {bag_dir}")
        sys.exit(1)

    # Create backup
    if backup:
        backup_file = bag_dir / "metadata.yaml.jazzy_backup"
        if not backup_file.exists():
            shutil.copy(metadata_file, backup_file)
            print(f"Created backup: {backup_file}")
        else:
            print(f"Backup already exists: {backup_file}")

    # Read the raw file
    with open(metadata_file, 'r') as f:
        content = f.read()

    original_content = content

    # 1. Remove all type_description_hash lines
    content = re.sub(r'\n\s*type_description_hash:.*', '', content)

    # 2. Convert offered_qos_profiles: [] to offered_qos_profiles: ""
    # Humble expects a string, not an empty array
    content = re.sub(r"offered_qos_profiles:\s*\[\]", 'offered_qos_profiles: ""', content)

    # 3. Downgrade version from 9 to 8
    content = re.sub(r'version:\s*9\b', 'version: 8', content)

    # 4. Handle custom_data: null -> remove or convert
    # Some Humble versions don't like 'null'
    content = re.sub(r'\n\s*custom_data:\s*null', '\n  custom_data: {}', content)

    if content == original_content:
        print("No changes needed - metadata already appears Humble-compatible")
    else:
        # Write the modified metadata
        with open(metadata_file, 'w') as f:
            f.write(content)

        print("Successfully converted metadata.yaml for Humble compatibility:")
        print("  - Removed type_description_hash fields")
        print("  - Fixed offered_qos_profiles format")
        print("  - Downgraded version to 8")
        print("  - Fixed custom_data format")

    print(f"\nTry opening your bag now with: ros2 bag info {bag_dir}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python convert_bag_to_humble_v2.py <path_to_bag_directory>")
        print("\nExample: python convert_bag_to_humble_v2.py ~/bags/flight8")
        sys.exit(1)

    convert_metadata(sys.argv[1])
