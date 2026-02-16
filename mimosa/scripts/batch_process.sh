#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <base_dir>"
    exit 1
fi

BASE_DIR="$1"
MIMOSA_PKG_PATH=$(rospack find mimosa)

if [ -z "$MIMOSA_PKG_PATH" ]; then
    echo "Error: Could not find mimosa package"
    exit 1
fi

for folder in "$BASE_DIR"/*/; do
    folder_name=$(basename "$folder")
    bag_path="${BASE_DIR}/${folder_name}/processed/sensors_only_with_clouds.bag"
    
    if [ ! -f "$bag_path" ]; then
        echo "Warning: Bag not found, skipping: $bag_path"
        continue
    fi
    
    echo "Processing: $folder_name"
    
    # Run roslaunch
    roslaunch mimosa hornbill_rosbag.launch bag_name:="$bag_path"
    
    # Copy results
    cp "${MIMOSA_PKG_PATH}/data/mimosa_results.bag" "${BASE_DIR}/${folder_name}/processed/estimate.bag"
    
    echo "Finished: $folder_name"
    echo "---"
done
