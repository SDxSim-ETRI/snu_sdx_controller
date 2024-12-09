from launch import LaunchDescription
import launch
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_name = launch.substitutions.LaunchConfiguration('robot_name')
    config_file = os.path.join(
        get_package_share_directory('mujoco_ros'),
        'config'
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('robot_name', default_value='fr3'),
    
        Node(
        package='mujoco_ros',
        executable='manipulator_sim',
        name='manipulator_sim',
        output='screen',
        parameters=[{'robot_name': robot_name}]
        ),

    ])
