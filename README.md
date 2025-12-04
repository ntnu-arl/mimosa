# mimosa

![License: MIT](https://img.shields.io/badge/License-BSD-green.svg)
![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)

This package implements a tightly-coupled multi-modal fusion framework. It currently suppports fusing LiDAR (Geometric, Photometric), Radar, any Odometry and IMU.

## Working Description

mimosa maintains a sliding window factor graph to fuse constraints generated from multiple sensors. On arrival of a new measurement, a new state is "declared" in the graph and connected with a preintegrated IMU factor. Then the measurement gets processed by the corresponding sensor manager to generate a new factor(s). This factor(s) is(are) then added to the graph and the graph is optimized.

## Setup

These instructions assume that `ros-noetic-desktop-full` is installed on your Ubuntu 20.04 system.

  ```bash
  # dependancies
  sudo apt install python-catkin-tools \
  libgoogle-glog-dev \
  libspdlog-dev

  mkdir catkin_ws/src && cd catkin_ws/src

  git clone git@github.com:ntnu-arl/config_utilities.git
  git clone git@github.com:ntnu-arl/gtsam.git -b feature/imu_factor_with_gravity
  git clone git@github.com:ntnu-arl/gtsam_points.git -b minimal_updated

  # get this code
  git clone git@github.com:ntnu-arl/mimosa.git

  # build it
  catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=OFF ..
  catkin build mimosa
  ```

## Usage

This package can be run either online (`mimosa_node`) or offline using a rosbag (`mimosa_rosbag`). The online version is for deployment on a robot, while the offline version is for testing and debugging. Both versions are identical in terms of functionality since they use the same callbacks. The offline version just allows you to read a rosbag and process the callbacks directly instead of recieving them over the network.

### Examples

#### LiDAR-Radar-IMU Fusion

Coming soon...

#### LiDAR(Photometric-Geometric)-IMU Fusion

##### [ENWIDE Dataset](https://projects.asl.ethz.ch/datasets/enwide)

Download the dataset from [https://projects.asl.ethz.ch/datasets/enwide](https://projects.asl.ethz.ch/datasets/enwide).
After downloading any of the sequences in the dataset, you can run the following command to launch the example:

```bash
roslaunch mimosa enwide_rosbag.launch bag:="/path/to/your/rosbag.bag" viz:="true"
```

##### [Newer College Multi-Camera Dataset](https://ori-drs.github.io/newer-college-dataset/multi-cam/)

Download the dataset from [https://ori-drs.github.io/newer-college-dataset/download/](https://ori-drs.github.io/newer-college-dataset/download/).
After downloading any of the sequences in the dataset, you can run the following command to launch the example:

```bash
roslaunch mimosa newer_college_rosbag.launch bag:="/path/to/your/rosbag.bag" viz:="true"
```

##### Complete Dataset Example

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
2. Remap to the correct topics in the launch file
3. Modify parameters in the config file
   1. For velodyne the `point_skip_factor` must be 1
   2. Set your transforms for T_B_S for each of the sensors you are using
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
