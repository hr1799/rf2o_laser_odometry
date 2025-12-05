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
            DeclareLaunchArgument('laser_scan_topic', default_value='/autodrive/roboracer_1/lidar'),
            DeclareLaunchArgument('odom_topic', default_value='/odom_rf2o'),
            DeclareLaunchArgument('publish_tf', default_value='False'),
            DeclareLaunchArgument('base_frame_id', default_value='roboracer_1'),
            DeclareLaunchArgument('odom_frame_id', default_value='map'),
            DeclareLaunchArgument('init_pose_from_topic', default_value='/initialpose'),
            DeclareLaunchArgument('freq', default_value='20.0'),
            DeclareLaunchArgument('base_link_to_laser_tf_x', default_value='0.273'),
            DeclareLaunchArgument('base_link_to_laser_tf_y', default_value='0.0'),
            DeclareLaunchArgument('base_link_to_laser_tf_z', default_value='0.096'),
            DeclareLaunchArgument('initial_pose_x', default_value='0.7406'),
            DeclareLaunchArgument('initial_pose_y', default_value='3.1583'),
            DeclareLaunchArgument('initial_pose_z', default_value='0.0592'),
            DeclareLaunchArgument('initial_pose_yaw', default_value='-1.5707963'),

            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic': LaunchConfiguration('laser_scan_topic'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'publish_tf': LaunchConfiguration('publish_tf'),
                    'base_frame_id': LaunchConfiguration('base_frame_id'),
                    'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                    'init_pose_from_topic': LaunchConfiguration('init_pose_from_topic'),
                    'freq': LaunchConfiguration('freq'),
                    'base_link_to_laser_tf.x': LaunchConfiguration('base_link_to_laser_tf_x'),
                    'base_link_to_laser_tf.y': LaunchConfiguration('base_link_to_laser_tf_y'),
                    'base_link_to_laser_tf.z': LaunchConfiguration('base_link_to_laser_tf_z'),
                    'initial_pose.x': LaunchConfiguration('initial_pose_x'),
                    'initial_pose.y': LaunchConfiguration('initial_pose_y'),
                    'initial_pose.z': LaunchConfiguration('initial_pose_z'),
                    'initial_pose.yaw': LaunchConfiguration('initial_pose_yaw'),
                }],
            ),
    ])
