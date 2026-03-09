from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mimosa')

    bag_name_arg = DeclareLaunchArgument('bag_name')
    s_arg = DeclareLaunchArgument('s', default_value='0.0')
    viz_arg = DeclareLaunchArgument('viz', default_value='false')
    lidar_collection_delay_arg = DeclareLaunchArgument(
        'lidar_collection_delay', default_value='0.113')

    mimosa_node = Node(
        package='mimosa',
        executable='mimosa_rosbag',
        name='mimosa_node',
        output='screen',
        parameters=[{
            'config_path': os.path.join(pkg_share, 'config', 'parrot', 'params.yaml'),
            'bag_name': LaunchConfiguration('bag_name'),
            's': LaunchConfiguration('s'),
            'lidar_collection_delay': LaunchConfiguration('lidar_collection_delay'),
        }],
        remappings=[
            ('~/imu/manager/imu_in', '/vectornav_driver_node/imu/data'),
            ('~/lidar/manager/lidar_in', '/lidar_points'),
            ('~/radar/manager/radar_in', '/radar/cloud'),
            ('~/odometry/manager/odometry_in', '/rovio/odometry'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'mimosa.rviz')],
        condition=IfCondition(LaunchConfiguration('viz')),
    )

    return LaunchDescription([
        bag_name_arg,
        s_arg,
        viz_arg,
        lidar_collection_delay_arg,
        mimosa_node,
        rviz_node,
    ])
