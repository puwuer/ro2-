from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # 实车默认值: use_sim_time=false + map/y_map.yaml + params/amcl.yaml
    # 仿真: use_sim_time:=true map_yaml:=/home/ros/Robot_ws/map/sim_map.yaml
    #       amcl_params:=.../params/sim_amcl.yaml
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    declare_map_yaml = DeclareLaunchArgument(
        "map_yaml", default_value="map/y_map.yaml")
    declare_amcl_params = DeclareLaunchArgument(
        "amcl_params",
        default_value=os.path.join(
            get_package_share_directory("mycar_navigation2"), "params", "amcl.yaml"))
    # 是否直接给 AMCL 初值(仿真里用, 省得手动点 2D Pose Estimate)
    declare_set_initial_pose = DeclareLaunchArgument(
        "set_initial_pose", default_value="false")
    declare_initial_x = DeclareLaunchArgument("initial_x", default_value="0.0")
    declare_initial_y = DeclareLaunchArgument("initial_y", default_value="0.0")
    declare_initial_yaw = DeclareLaunchArgument("initial_yaw", default_value="0.0")
    # AMCL 的里程计输入话题(仿真里可指向 EKF 融合后的 /odometry/filtered)
    declare_amcl_odom_topic = DeclareLaunchArgument(
        "amcl_odom_topic", default_value="odom")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml")
    amcl_params = LaunchConfiguration("amcl_params")
    set_initial_pose = LaunchConfiguration("set_initial_pose")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    amcl_odom_topic = LaunchConfiguration("amcl_odom_topic")

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {"yaml_filename": map_yaml},
            {"use_sim_time": use_sim_time}
        ]
    )
    amcl_node=Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl_node',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            amcl_params,
            # 初值必须和机器人出生点一致, 否则一开始就定位错
            {"set_initial_pose": set_initial_pose,
             "initial_pose.x": initial_x,
             "initial_pose.y": initial_y,
             "initial_pose.z": 0.0,
             "initial_pose.yaw": initial_yaw},
        ],
        remappings=[("odom", amcl_odom_topic)],
    )
    manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
                    {"node_names": [
                        "map_server",
                        "amcl_node"
                    ]}]
    )
    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        declare_amcl_params,
        declare_set_initial_pose,
        declare_initial_x,
        declare_initial_y,
        declare_initial_yaw,
        declare_amcl_odom_topic,
        map_server_node,
        amcl_node,
        manager_node,
    ])
