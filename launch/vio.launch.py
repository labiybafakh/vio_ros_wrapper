from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('vio_ros_wrapper')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'vio.rviz')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'left_image_topic',
            default_value='/camera/left/image_raw',
            description='Left camera image topic'
        ),
        DeclareLaunchArgument(
            'right_image_topic',
            default_value='/camera/right/image_raw',
            description='Right camera image topic'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='IMU data topic'
        ),
        DeclareLaunchArgument(
            'queue_size',
            default_value='10000',
            description='Subscriber queue size'
        ),
        DeclareLaunchArgument(
            'config_file',
            description='VIO configuration file path (REQUIRED!)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2'
        ),

        # VIO Node
        Node(
            package='vio_ros_wrapper',
            executable='vio_node',
            name='vio_node',
            output='screen',
            parameters=[{
                'left_image_topic': LaunchConfiguration('left_image_topic'),
                'right_image_topic': LaunchConfiguration('right_image_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'queue_size': LaunchConfiguration('queue_size'),
                'config_file': LaunchConfiguration('config_file'),
            }]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])
