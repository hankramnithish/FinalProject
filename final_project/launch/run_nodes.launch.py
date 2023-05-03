from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_project',
            executable='robot_recognition',
            output = 'screen'
        ),
        Node(
            package='final_project',
            executable='obstacle_avoidance',
            output = 'screen'
        ),
        Node(
            package='final_project',
            executable='robot_control',
            output = 'screen'
        ),
    ])