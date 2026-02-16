# mimosa

![License: MIT](https://img.shields.io/badge/License-BSD-green.svg)
![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)

This package implements a tightly-coupled multi-modal fusion framework. It currently supports fusing LiDAR (Geometric, Photometric), Radar, any Odometry and IMU to provide robust state estimation in challenging environments. The framework is designed to be modular and easily extensible to add new sensors.

## Working Description

mimosa maintains a sliding window factor graph to fuse factors generated from multiple sensors. On arrival of a new measurement (a pointcloud from the LiDAR or Radar or an odometry message from an external odometry source like VIO), a new state is "declared" in the graph and connected with a preintegrated IMU factor. Then the measurement gets processed by the corresponding sensor manager to generate a new factor(s). This factor(s) is(are) then added to the graph and the graph is optimized. The optimized state is then published on `mimosa_node/graph/odometry` topic as a `nav_msgs/Odometry` message.

### IMU Factor

The IMU factor is based on GTSAM's provided `PreintegratedIMUFactor` but modified to also have gravity as a state in the preintegration. This is due to the fact that we consider the initial orientation of the IMU to be the map frame (whereas GTSAM assumes the map frame to be gravity aligned).

### LiDAR Factor

There are two types of LiDAR factors implemented:

1. Geometric Factor: This factor uses point-to-plane scan-to-map ICP residuals to constrain the LiDAR pose. The correspondences are found using a k-d tree and the residuals are computed using the point-to-plane distance.
2. Photometric Factor (Implemented only for Ouster LiDARs): This factor uses photometric error of patches in the intensity image to constrain the LiDAR pose.

### Radar Factor

The radar factor provides a single factor per pointcloud that utilizes the radial speed residuals from the radar measurements.

### Odometry Factor

Consecutive odometry measurements (e.g., from a VIO system) are used to create relative pose factors (Between factors) between the corresponding states in the graph.

## Setup

These instructions assume that `ros-noetic-desktop-full` is installed on your Ubuntu 20.04 system.

  ```bash
  # dependencies
  sudo apt install python3-catkin-tools \
  libgoogle-glog-dev \
  libspdlog-dev

  mkdir catkin_ws/src && cd catkin_ws/src

  git clone git@github.com:ntnu-arl/config_utilities.git -b dev/mimosa
  git clone git@github.com:ntnu-arl/gtsam.git -b feature/imu_factor_with_gravity
  git clone git@github.com:ntnu-arl/gtsam_points.git -b minimal_updated

  # get this code
  git clone git@github.com:ntnu-arl/mimosa.git

  # build it
  cd ..
  catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=OFF
  catkin build mimosa
  ```

## Usage

This package can be run either online (`mimosa_node`) or offline using a rosbag (`mimosa_rosbag`). The online version is for deployment on a robot, while the offline version is for testing and debugging. Both versions are identical in terms of functionality since they use the same callbacks. The offline version just allows you to read a rosbag and process the callbacks directly instead of receiving them over the network.

### Examples

#### LiDAR-Radar-IMU Fusion

##### [Unified Autonomy Stack Datasets](https://huggingface.co/datasets/ntnu-arl/unified_autonomy_stack_datasets)

Download any rosbag from [https://huggingface.co/datasets/ntnu-arl/unified_autonomy_stack_datasets](https://huggingface.co/datasets/ntnu-arl/unified_autonomy_stack_datasets). There are two variants of robots in this dataset: one which has an Ouster LiDAR and the other which has a Unipilot module (with a Robosense Airy LiDAR). As per the rosbag that you downloaded, you will need to build the [ouster_ros](https://github.com/ouster-lidar/ouster-ros) or [rslidar_sdk](https://github.com/ntnu-arl/rslidar_sdk/tree/develop) package respectively to be able to convert the raw LiDAR packets to pointclouds. Please follow the instructions in the respective repositories to build the drivers. Note that in the case of rslidar_sdk you will need to modify the config file to set the `lidar_type` as `RSAIRY`, the `common/msg_source` as `2` (for packet message comes from ros or ros2), `send_packet_ros` as `false` and `send_point_cloud_ros` as `true`. You can find a working config file [here](https://github.com/ntnu-arl/rslidar_sdk/blob/d02b59cc43d027e4bdaa1ee0762d36e77129ecc7/config/config.yaml).

The bags should be replayed with `--clock` option and the global `/use_sim_time` parameter set to true. To do this, a simple way is to create a launch file called `simcore.launch` with the following content:

```xml
<launch>
  <param name="/use_sim_time" value="true"/>
</launch>
```

Then you can launch the system as:

```bash
# Terminal 1
roslaunch simcore.launch

# Terminal 2
roslaunch ouster_ros replay.launch
# or
roslaunch rslidar_sdk start.launch

# Terminal 3
roslaunch mimosa hornbill.launch
# or
roslaunch mimosa magpie.launch

# Terminal 4
rosbag play --clock /path/to/your/rosbag.bag
```

#### LiDAR(Photometric-Geometric)-IMU Fusion

##### [ENWIDE Dataset](https://projects.asl.ethz.ch/datasets/enwide)

Download the dataset from [https://projects.asl.ethz.ch/datasets/enwide](https://projects.asl.ethz.ch/datasets/enwide).
After downloading any of the sequences in the dataset, you can run the following command to launch the example:

```bash
roslaunch mimosa enwide_rosbag.launch bag_name:="/path/to/your/rosbag.bag" viz:="true"
```

##### [Newer College Multi-Camera Dataset](https://ori-drs.github.io/newer-college-dataset/multi-cam/)

Download the dataset from [https://ori-drs.github.io/newer-college-dataset/download/](https://ori-drs.github.io/newer-college-dataset/download/).
After downloading any of the sequences in the dataset, you can run the following command to launch the example:

```bash
roslaunch mimosa newer_college_rosbag.launch bag_name:="/path/to/your/rosbag.bag" viz:="true"
```

##### Complete Dataset Example

There is also a script [dataset_evaluation.py](src/mimosa/scripts/dataset_evaluation.py) that can be used to run the complete dataset evaluation. This script will run the `mimosa_rosbag` node on all the sequences in the dataset and save the results in a folder. Before running this script make sure to set the correct `dataset_path` and `results_directory`. It is assumed that the dataset is in the following format:

```bash
dataset_path/
├── sequence_1
│   ├── xxxxxsequence_1.bag
|   ├── gt-sequence_1.csv
├── sequence_2
│   ├── xxxxxsequence_2.bag
|   ├── gt-sequence_2.csv
├── sequence_3
│   ├── xxxxxsequence_3.bag
|   ├── gt-sequence_3.csv
├── sequence_4
│   ├── xxxxxsequence_4.bag
|   ├── gt-sequence_4.csv
```

```bash
python dataset_evaluation.py
```

### On your own data

To run on your own data, you need to set up the following:

1. Take the most relevant launch file
2. Modify the remapping of the input topics to your sensor topics in the launch file
3. Modify parameters in the corresponding config file
   1. e.g. for velodyne the `point_skip_factor` must be 1
   2. Set your transforms for `T_B_S` for each of the sensors you are using. This is the transform from the Sensor frame to the Body frame (i.e. pose of Sensor in the Body frame).
4. Launch your launch file

## License

This project is licensed under the BSD-3-Clause License - see the [LICENSE](LICENSE) file for details.

## Citing

If you use this work in your research, please cite the relevant publications:

```bibtex
@misc{perception_pglio,
    title = {{PG}-{LIO}: {Photometric}-{Geometric} fusion for {Robust} {LiDAR}-{Inertial} {Odometry}},
    shorttitle = {{PG}-{LIO}},
    url = {http://arxiv.org/abs/2506.18583},
    doi = {10.48550/arXiv.2506.18583},
    publisher = {arXiv},
    author = {Khedekar, Nikhil and Alexis, Kostas},
    month = jun,
    year = {2025},
    note = {arXiv:2506.18583 [cs]},
    keywords = {Computer Science - Robotics},
}

@inproceedings{perception_dlrio,
    title = {Degradation {Resilient} {LiDAR}-{Radar}-{Inertial} {Odometry}},
    url = {https://ieeexplore.ieee.org/document/10611444},
    doi = {10.1109/ICRA57147.2024.10611444},
    booktitle = {2024 {IEEE} {International} {Conference} on {Robotics} and {Automation} ({ICRA})},
    author = {Nissov, Morten and Khedekar, Nikhil and Alexis, Kostas},
    month = may,
    year = {2024},
    keywords = {Degradation, Estimation, Laser radar, Odometry, Prevention and mitigation, Robot sensing systems, Sensors},
    pages = {8587--8594},
}

@ARTICLE{perception_jplRadar,
    author={Nissov, Morten and Edlund, Jeffrey A. and Spieler, Patrick and Padgett, Curtis and Alexis, Kostas and Khattak, Shehryar},
    journal={IEEE Robotics and Automation Letters},
    title={Robust High-Speed State Estimation for Off-Road Navigation Using Radar Velocity Factors},
    year={2024},
    volume={9},
    number={12},
    pages={11146-11153},
    keywords={Radar;Sensors;Velocity measurement;Radar measurements;Laser radar;Odometry;State estimation;Robustness;Robot sensing systems;Field robots;localization;sensor fusion},
    doi={10.1109/lra.2024.3486189}
}
```

## Questions

You can open an issue or contact us for any questions:

- [Nikhil Khedekar](mailto:nikhil.v.khedekar@ntnu.no)
- [Morten Nissov](mailto:morten.nissov@ntnu.no)
- [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)
