#!/bin/bash

folder_path="/home/cyanide/rosbags/nk4/lab/bags/fyllingsdal/day_2/results/500m_7_fast"
gt_file_name="leica_result.tum"

estimate_file_names=(
  "lri.tum"
  "lri_4dof.tum"
  "lri_4dof_avar_imu_params.tum"
  "li.tum"
  "ri.tum"
  "ri_avar_imu_params.tum"
  "fast_livo_2.tum"
)

force_recompute=false

# Check if folder exists
if [[ ! -d "$folder_path" ]]; then
  echo "[ERROR] Directory does not exist: $folder_path"
  exit 1
fi

gt_file="$folder_path/$gt_file_name"

# Check if ground truth file exists
if [[ ! -f "$gt_file" ]]; then
  echo "[ERROR] Ground truth file $gt_file does not exist"
  exit 1
fi

for estimate_file_name in "${estimate_file_names[@]}"; do
  estimate_file="$folder_path/$estimate_file_name"

  # Check if estimate file exists
  if [[ ! -f "$estimate_file" ]]; then
    echo "[WARNING] Skipping $estimate_file - file does not exist"
    continue
  fi

  # Extract the base name without extension for --save_results
  base_name="${estimate_file_name%.tum}"

  results_zip="$folder_path/rpe_trans_$base_name.zip"
  # Check if the output file already exists and skip if force_recompute is false
  if [[ -f "$results_zip" && "$force_recompute" = false ]]; then
    echo "[INFO] Skipping $estimate_file - results already exist at $results_zip"
    continue
  fi
  result_figures="$folder_path/rpe_trans_$base_name"

  evo_rpe tum -r trans_part --all_pairs -a -d 10 -u m "$gt_file" "$estimate_file" --save_results "$results_zip" --save_plot "$result_figures" --no_warnings

  # evo_ape tum -r trans_part -a "$gt_file_name" "$estimate_file" --save_results "ape_trans_$base_name.zip" --no_warnings

done
