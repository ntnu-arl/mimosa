#!/bin/bash

# Configuration
# BAG_NAME="/home/cyanide/rosbags/nk4/lab/bags/fyllingsdal/day_2/500m_7_fast_actually_615/processed/sensors_only_with_clouds.bag"
BAG_NAME="/home/cyanide/rosbags/nk4/lab/bags/2025_11_20_runehamar/hornbill/flight8/processed/sensors_only_with_clouds.bag"

MIMOSA_DIR="$(rospack find mimosa)"
OVERRIDES_DIR="$MIMOSA_DIR/scripts/param_change_evaluation/config_overrides"
RESULTS_DIR="$MIMOSA_DIR/data/param_change_evaluation_results"

# Create results directory if it doesn't exist
mkdir -p "$RESULTS_DIR"

# Check if overrides directory exists
if [ ! -d "$OVERRIDES_DIR" ]; then
    echo "Error: Overrides directory not found: $OVERRIDES_DIR"
    exit 1
fi

# Count yaml files
yaml_files=("$OVERRIDES_DIR"/*.yaml)
if [ ! -e "${yaml_files[0]}" ]; then
    echo "No YAML files found in $OVERRIDES_DIR"
    exit 1
fi

echo "Found ${#yaml_files[@]} override file(s)"
echo "=========================================="

# Iterate over all yaml files
for override_file in "$OVERRIDES_DIR"/*.yaml; do
    # Get filename without path and extension
    experiment_name=$(basename "$override_file" .yaml)
    
    echo ""
    echo "Running experiment: $experiment_name"
    echo "Override file: $override_file"
    echo "------------------------------------------"
    
    # Run the launch file with this override
    roslaunch mimosa hornbill_rosbag.launch \
        bag_name:="$BAG_NAME" \
        config_override:="$override_file"
    
    # Capture exit status
    exit_status=$?
    
    # Move/rename results if needed
    if [ -f "$(rospack find mimosa)/data/mimosa_results.bag" ]; then
        mv "$(rospack find mimosa)/data/mimosa_results.bag" \
           "$RESULTS_DIR/${experiment_name}_results.bag"
        echo "Results saved to: $RESULTS_DIR/${experiment_name}_results.bag"
    fi
    
    if [ $exit_status -ne 0 ]; then
        echo "Warning: Experiment $experiment_name exited with status $exit_status"
    fi
    
    echo "Finished: $experiment_name"
    echo "=========================================="
    
    # Optional: add a small delay between runs
    sleep 2
done

echo ""
echo "All experiments complete!"
echo "Results saved in: $RESULTS_DIR"
