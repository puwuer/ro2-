from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="False")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # 允许换 lua 配置(仿真建图用 mycar_sim_mapping.lua: 关闭在线位姿图优化, 避免 map->odom 跳变)
    declare_lua = DeclareLaunchArgument("configuration_basename", default_value="mycar.lua")
    lua_basename = LaunchConfiguration("configuration_basename")
    cartograpner_node=Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-configuration_directory",os.path.join(get_package_share_directory("mycar_slam_cartographer"), "params"),
            "-configuration_basename", lua_basename
        ],
        # remappings=[("odom", "/odometry/filtered")]
    )
    cartographer_occupancy_grid_node=Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    return LaunchDescription([declare_use_sim_time, declare_lua, cartograpner_node, cartographer_occupancy_grid_node])
