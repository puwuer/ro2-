import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="True")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_pkg = get_package_share_directory("mycar_slam_cartographer")
    nav2_pkg = get_package_share_directory("mycar_navigation2")

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg,'launch',
                                                    'cartographer.launch.py')),
        launch_arguments=[('use_sim_time', use_sim_time)]
        )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg,'launch', 
                                                    'nav2.launch.py')),
        launch_arguments=[('use_sim_time', use_sim_time)]
        )


    return LaunchDescription([declare_use_sim_time, 
                              slam_launch, 
                              nav2_launch
                              ])