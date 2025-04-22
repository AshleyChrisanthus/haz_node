from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    aiil_gazebo_dir = get_package_share_directory('aiil_gazebo')
    aiil_rosbot_demo_dir = get_package_share_directory('aiil_rosbot_demo')
    
    return LaunchDescription([
        # Launch SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(aiil_gazebo_dir, 'launch', 'slam.launch.py')
            )
        ),

        # Launch Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(aiil_gazebo_dir, 'launch', 'nav.launch.py')
            )
        ),

        # Launch Find Object 2D (with gui:=false)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(aiil_rosbot_demo_dir, 'launch', 'find_object_2d.launch.py')
            ),
            launch_arguments={'gui': 'false'}.items()
        ),

        # Run hazard_detector node
        Node(
            package='my_robot_challenge_pkg',
            executable='hazard_detector',
            name='hazard_detector',
            output='screen'
        )
    ])
