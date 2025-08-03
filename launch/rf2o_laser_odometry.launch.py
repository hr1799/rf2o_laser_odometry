import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/autodrive/roboracer_1/lidar',
                    'odom_topic' : '/odom_rf2o',
                    'publish_tf' : False,
                    'base_frame_id' : 'roboracer_1',
                    'odom_frame_id' : 'map',
                    'init_pose_from_topic' : '/initialpose',
                    'freq' : 20.0,
                    # Laser TF parameters
                    'base_link_to_laser_tf.x': 0.273,
                    'base_link_to_laser_tf.y': 0.0,
                    'base_link_to_laser_tf.z': 0.096,
                    # Initial pose parameters w.r.t to world
                    'initial_pose.x': 0.7406,
                    'initial_pose.y': 3.1583,
                    'initial_pose.z': 0.0592,
                    'initial_pose.yaw': -1.5707963,
                }],
            ),
    ])
