from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    launch_file1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot4_bringup'),'launch/oakd.launch.py')
            ),
        launch_arguments = {'usePreview':'True', 'previewWidth':'300','previewHeight':'150'}.items()
    )
    launch_file2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot4_bringup'),'launch/rplidar.launch.py')
            ),
    )
    ld.add_action(launch_file1)
    ld.add_action(launch_file2)
    return ld