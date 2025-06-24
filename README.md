# mimosa

![License: MIT](https://img.shields.io/badge/License-BSD-green.svg)
![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)

This package implements a tightly-coupled multi-modal fusion framework. It currently suppports LiDAR (Geometric, Photometric) and IMU and will be extended to fuse additional sensors.

## Working Description

mimosa maintains a sliding window factor graph to fuse constraints generated from multiple sensors. On arrival of a new measurement, a new state is "declared" in the graph and connected with a preintegrated IMU factor. Then the measurement gets processed by the corresponding sensor manager to generate a new factor(s). This factor(s) is(are) then added to the graph and the graph is optimized.

## Setup

These instructions assume that `ros-noetic-desktop-full` is installed on your Ubuntu 20.04 system.

1. Install the required binaries:

  ```bash
  sudo apt install python-catkin-tools \
  libgoogle-glog-dev \
  libspdlog-dev
  ```

2. Install the required libraries from source:

  ```bash
  mkdir src_installs
  cd src_installs

  # config_utilities
  git clone git@github.com:ntnu-arl/config_utilities.git
  cd config_utilities/config_utilities
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make -j4
  sudo make install
  cd ../..

  # GTSAM
  git clone git@github.com:ntnu-arl/gtsam.git -b feature/imu_factor_with_gravity
  cd gtsam
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=OFF ..
  make -j4
  sudo make install
  cd ../..

  # gtsam_points
  git clone git@github.com:ntnu-arl/gtsam_points.git -b minimal_updated
  cd gtsam_points
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make -j4
  sudo make install
  cd ../..
  ```

3. Build this repository in `Release` mode:

  ```bash
  cd ~/catkin_ws/src
  git clone git@github.com:ntnu-arl/mimosa.git
  cd ..
  catkin build -DCMAKE_BUILD_TYPE=Release
  ```

## Usage

This package can be run either online (`mimosa_node`) or offline using a rosbag (`mimosa_rosbag`). The online version is for deployment on a robot, while the offline version is for testing and debugging. Both versions are identical in terms of functionality since they use the same callbacks. The offline version just allows you to read a rosbag and process the callbacks directly instead of recieving them over the network.

### Examples

#### [ENWIDE Dataset](https://projects.asl.ethz.ch/datasets/enwide)

Download the dataset from [https://projects.asl.ethz.ch/datasets/enwide](https://projects.asl.ethz.ch/datasets/enwide).
After downloading any of the sequences in the dataset, you can run the following command to launch the example:

```bash
roslaunch mimosa enwide_rosbag.launch bag:="/path/to/your/rosbag.bag" viz:="true"
```

#### [Newer College Multi-Camera Dataset](https://ori-drs.github.io/newer-college-dataset/multi-cam/)

Download the dataset from [https://ori-drs.github.io/newer-college-dataset/download/](https://ori-drs.github.io/newer-college-dataset/download/).
After downloading any of the sequences in the dataset, you can run the following command to launch the example:

```bash
roslaunch mimosa newer_college_rosbag.launch bag:="/path/to/your/rosbag.bag" viz:="true"
```

#### Complete Dataset Example

There is also a script [dataset_evaluation.py](src/mimosa/scripts/dataset_evaluation.py) that can be used to run the complete dataset evaluation. This script will run the `mimosa_rosbag` node on all the sequences in the dataset and save the results in a folder. Before running this script make sure to set the currect `dataset_path` and `results_directory`. It is assumed that the dataset is in the following format:

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
2. Set correct topics
3. Set correct parameter file
4. Set correct parameters in the launch file (For velodyne the `point_skip_factor` must be 1)
5. Set your transforms for T_B_S for each of the sensors you are using in the parameter file
6. Launch your launch file

## License

This project is licensed under the BSD-3-Clause License - see the [LICENSE](LICENSE) file for details. Intended for civilian use.

## Citing

If you use this work in your research, please cite the following publications:

```
@misc{khedekar2025pgliophotometricgeometricfusionrobust,
      title={PG-LIO: Photometric-Geometric fusion for Robust LiDAR-Inertial Odometry}, 
      author={Nikhil Khedekar and Kostas Alexis},
      year={2025},
      eprint={2506.18583},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2506.18583}, 
}
```

## Questions

You can open an issue or contact us for any questions:

- [Nikhil Khedekar](mailto:nikhil.v.khedekar@ntnu.no)
- [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)
