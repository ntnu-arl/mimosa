#!/usr/bin/env python3

# Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

import subprocess
import pty
import sys
from datetime import datetime
import os
import shutil
from evo.tools import log

log.configure_logging(verbose=True, debug=True, silent=False)

import pprint
import numpy as np

from evo.tools import plot
import matplotlib.pyplot as plt
from evo.tools import file_interface
from evo.core import sync
import copy
from evo.core import metrics
from evo.core.units import Unit

def run_command(cmd, stream_output=False, preserve_color=False):
    """
    Run a command in one of three ways:
      1) stream_output=False, preserve_color=False:
         - Non-streaming, capture all output into a string and return it.
      2) stream_output=True, preserve_color=False:
         - Streaming mode using Popen + stdout=PIPE; color may be lost.
      3) preserve_color=True (which implies streaming_output=True):
         - Streaming mode, but via a pseudo-tty (PTY) so color is preserved.
    """
    if not stream_output and not preserve_color:
        # -------------------------------------------------------
        # Mode 1: Non-streaming capture (simple, returns output)
        # -------------------------------------------------------
        print(f"\n=== Running (captured): {cmd} ===\n")
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"[ERROR] Command '{cmd}' returned code {result.returncode}.\n{result.stderr}")
        return result.stdout.strip()

    elif stream_output and not preserve_color:
        # -------------------------------------------------------
        # Mode 2: Streaming (stdout=PIPE), but no color
        # -------------------------------------------------------
        print(f"\n=== Running (streaming, no PTY): {cmd} ===\n")
        process = subprocess.Popen(
            cmd, shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )
        # Print lines as they arrive
        try:
            for line in iter(process.stdout.readline, ''):
                if not line:
                    break
                print(line, end='')
        except KeyboardInterrupt:
            process.terminate()
            print("\n[INFO] Command interrupted by user (Ctrl+C).")
            # Exit immediately
            sys.exit()

        return_code = process.wait()
        if return_code != 0:
            print(f"\n[ERROR] Command '{cmd}' returned code {return_code}.")
        return ""

    else:
        # -------------------------------------------------------
        # Mode 3: Streaming via PTY to preserve color
        # -------------------------------------------------------
        print(f"\n=== Running (streaming, PTY for color): {cmd} ===\n")

        # 1) Create a new pseudo-terminal
        master_fd, slave_fd = pty.openpty()

        # 2) Launch the subprocess with stdin/stdout/stderr attached to PTY
        process = subprocess.Popen(
            cmd,
            shell=True,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            text=True
        )

        # The child process sees 'slave_fd' as its terminal (PTY)
        # We close the 'slave_fd' in the parent, and read from 'master_fd'.
        os.close(slave_fd)

        try:
            # 3) Read from the master end of the PTY and write to our own stdout
            while True:
                # Reading 1024 bytes at a time
                output = os.read(master_fd, 1024)
                if not output:
                    break
                # Decode and write to real stdout (preserving color codes)
                sys.stdout.write(output.decode())
                sys.stdout.flush()
        except KeyboardInterrupt:
            # On Ctrl+C in the parent, terminate the child process
            process.terminate()
            print("\n[INFO] Command interrupted by user (Ctrl+C).")
            sys.exit()
        except OSError:
            # This can happen if the child process closes before we read everything
            pass
        finally:
            os.close(master_fd)

        # Wait for the child to finish
        return_code = process.wait()
        if return_code != 0:
            print(f"\n[ERROR] Command '{cmd}' returned code {return_code}.")
        return ""


##### Configuration
sequences = None
best_ates = None
best_rtes = None

max_diff = 0.01

#### ENWIDE Dataset
dataset_name = "enwide"
launch_file_name = "enwide_rosbag.launch"
dataset_path = "/Downloads/enwide"   ######################## CHANGE THIS TO YOUR PATH
sequences = [
    "tunnel_s",
    "tunnel_d",
    "intersection_s",
    "intersection_d",
    "runway_s",
    "runway_d",
    "field_s",
    "field_d",
    "katzensee_s",
    "katzensee_d"
]
# # Coin lio values
best_ates = [0.743, 0.487, 0.466, 1.912, 1.033, 2.437, 0.232, 0.581, 0.412, 0.592]
best_rtes = [1.60, 1.59, 1.25, 1.69, 1.89, 2.98, 0.85, 1.83, 0.99, 1.61]
####

# The dataset is organized as:
#   dataset_path/sequence_name/recorddate-sequence_name.bag

original_estimated_trajectory_path = "/tmp/mimosa/lidar_manager_odometry.log"
estimated_trajectory_filename = "pg_lio.tum"

# Every run will create a new directory with the current timestamp
# and store the results in that directory.
current_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
internal_results_dir = (f"PG-LIO/Results/{current_timestamp}")
results_dir = (
    f"/tmp/{internal_results_dir}"
)
os.makedirs(results_dir, exist_ok=True)

#######

def get_bold_markdown(text):
    return f"**{text}**"

def get_bold_latex(text):
    return f"\\textbf{{{text}}}"


if __name__ == "__main__":

    if sequences is None:
        # List all sequences in the dataset directory. Each sequence is a subdirectory.
        list_cmd = f"ls -d {dataset_path}/*/"
        output = run_command(list_cmd, stream_output=False)

        # Split the output by newlines to get individual sequence directory names
        sequences = output.split("\n") if output else []

        # Extract just the sequence name from the full path
        sequences = [seq.split("/")[-2] for seq in sequences if seq]

    ates = []
    rtes = []

    # Print the list of sequences
    print(f"Found {len(sequences)} sequences in {dataset_path}:")
    for seq in sequences:
        print(f" - {seq}")

    with open(f"{results_dir}/results.md", "a") as f:
        f.write(f"### {dataset_name} Results\n")

    for sequence in sequences:
        # Construct bag file path for the current sequence
        bag_name = f"{dataset_path}/{sequence}/*{sequence}.bag"

        # Print the bag file path
        print(f"Bag file for sequence '{sequence}': {bag_name}")

        # Launch mimosa with the bag file
        roslaunch_cmd = f"roslaunch mimosa {launch_file_name} bag_name:={bag_name}"
        run_command(roslaunch_cmd, stream_output=True, preserve_color=True)

        print(f"Finished roslaunch for sequence: {sequence}")
        print("-" * 60)

        # Create a directory for the current sequence
        os.makedirs(f"{results_dir}/{sequence}", exist_ok=True)

        # Copy the estimated trajectory to the sequence directory
        print(f"Copying estimated trajectory to sequence directory: {sequence}")
        shutil.copy2(
            original_estimated_trajectory_path,
            f"{results_dir}/{sequence}/{estimated_trajectory_filename}",
        )

        # Generate the evo results
        traj_ref = file_interface.read_tum_trajectory_file(
            f"{dataset_path}/{sequence}/gt-{sequence}.csv"
        )
        traj_est = file_interface.read_tum_trajectory_file(
            f"{results_dir}/{sequence}/{estimated_trajectory_filename}"
        )
        traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)
        traj_est_aligned = copy.deepcopy(traj_est)
        traj_est_aligned.align(traj_ref, correct_scale=False, correct_only_scale=False)

        data = (traj_ref, traj_est_aligned)

        # ATE
        pose_relation = metrics.PoseRelation.translation_part
        ape_metric = metrics.APE(pose_relation)
        ape_metric.process_data(data)
        ape_stats = ape_metric.get_all_statistics()
        ates.append(ape_stats['rmse'])
        pprint.pprint(ape_stats)

        seconds_from_start = [t - traj_est.timestamps[0] for t in traj_est.timestamps]
        fig = plt.figure()
        plot.error_array(
            fig.gca(),
            ape_metric.error,
            x_array=seconds_from_start,
            statistics={s: v for s, v in ape_stats.items() if s != "sse"},
            name="APE",
            title="APE w.r.t. " + ape_metric.pose_relation.value,
            xlabel="$t$ (s)",
        )
        plt.savefig(f"{results_dir}/{sequence}/pg_lio_ape.png")
        # plt.show()

        # RTE
        pose_relation = metrics.PoseRelation.point_distance
        delta = 10
        delta_unit = Unit.meters
        all_pairs = True
        rpe_metric = metrics.RPE(pose_relation=pose_relation, delta=delta, delta_unit=delta_unit, all_pairs=all_pairs)
        rpe_metric.process_data(data)
        rpe_stats = rpe_metric.get_all_statistics()
        rtes.append(rpe_stats['rmse'] * delta)
        pprint.pprint(rpe_stats)

        # important: restrict data to delta ids for plot
        traj_ref_plot = copy.deepcopy(traj_ref)
        traj_est_plot = copy.deepcopy(traj_est)
        # Note: the pose at index 0 is added for plotting purposes, although it has
        # no RPE value assigned to it since it has no previous pose.
        # (for each pair (i, j), the 'delta_ids' represent only j)
        delta_ids_with_first_pose = [0] + rpe_metric.delta_ids
        traj_ref_plot.reduce_to_ids(delta_ids_with_first_pose)
        traj_est_plot.reduce_to_ids(delta_ids_with_first_pose)
        seconds_from_start = [t - traj_est_plot.timestamps[0] for t in traj_est_plot.timestamps[1:]]

        print(f'error size: {rpe_metric.error.size}')
        print(f'seconds from start size: {len(seconds_from_start)}')

        fig = plt.figure()
        plot.error_array(fig.gca(), rpe_metric.error, x_array=seconds_from_start,
                        statistics={s:v for s,v in rpe_stats.items() if s != "sse"},
                        name="RPE", title="RPE w.r.t. " + rpe_metric.pose_relation.value, xlabel="$t$ (s)")
        plt.savefig(f"{results_dir}/{sequence}/pg_lio_rpe.png")
        # plt.show()

        # Open and append the rmse values into the results md file
        with open(f"{results_dir}/results.md", "a") as f:
            f.write(f"#### {sequence}\n")
            f.write(f"APE: RMSE (m): {ape_stats['rmse']}\n")
            f.write(f"RPE: RMSE (%): {rpe_stats['rmse'] * delta}\n")
            f.write("\n")
            f.write(f"![[{internal_results_dir}/{sequence}/pg_lio_ape.png]]\n")
            f.write(f"![[{internal_results_dir}/{sequence}/pg_lio_rpe.png]]\n")

        # # RTE
        # evo_cmd = f"evo_rpe tum {dataset_path}/{sequence}/gt-* {results_dir}/{estimated_trajectory_filename} --align --pose_relation point_distance --delta 10 --delta_unit m --all_pairs --verbose --save_plot {results_dir}/pg_lio_rpe"
        # output = run_command(evo_cmd, stream_output=False)
        # # Write the output to a file
        # with open(f"{results_dir}/pg_lio_evo_rpe.txt", "w") as f:
        #     f.write(output)

        # # ATE
        # evo_cmd = f"evo_ape tum {dataset_path}/{sequence}/gt-* {results_dir}/{estimated_trajectory_filename} --align --pose_relation trans_part --verbose --save_plot {results_dir}/pg_lio_ape"
        # output = run_command(evo_cmd, stream_output=False)
        # # Write the output to a file
        # with open(f"{results_dir}/pg_lio_evo_ape.txt", "w") as f:
        #     f.write(output)

    print("All roslaunch commands have completed.")
    with open(f"{results_dir}/results.md", "a") as f:
        f.write(f"## Collated results table with ATE/RTE:\n\n")

        # Write the table header
        line = "| "
        for seq in sequences:
            line += f"{seq} | "
        f.write(line + "\n")

        line = "| "
        for seq in sequences:
            line += f"--------- | "
        f.write(line + "\n")

        # Write the ATE and RTE values as ATE / RTE
        line = "| "
        for i in range(len(sequences)):
            if best_ates is not None:
                if ates[i] < best_ates[i]:
                    line += get_bold_markdown(ates[i])
                else:
                    line += str(ates[i])
            line += " / "
            if best_rtes is not None:
                if rtes[i] < best_rtes[i]:
                    line += get_bold_markdown(rtes[i])
                else:
                    line += str(rtes[i])
            line += " | "
        f.write(line + "\n")

        # Write the row out again for latex table formatting
        f.write("\n\n")
        line = "\\textbf{Ours} & "
        for i in range(len(sequences)):
            if best_ates is not None and ates[i] < best_ates[i]:
                formatted_ate = f"{ates[i]:.3f}"
                line += f"${get_bold_latex(formatted_ate)}$"
            else:
                line += f"${ates[i]:.3f}$"

            line += " / "

            if best_rtes is not None and rtes[i] < best_rtes[i]:
                formatted_rte = f"{rtes[i]:.2f}"
                line += f"${get_bold_latex(formatted_rte)}$"
            else:
                line += f"${rtes[i]:.2f}$"

            line += " & "
        # Remove the trailing " & "
        line = line[:-2]
        line += " \\\\ \n"
        f.write(line)
