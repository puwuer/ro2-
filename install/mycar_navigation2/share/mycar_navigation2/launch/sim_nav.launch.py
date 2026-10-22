"""仿真 + 定位 + 导航 一层启动 (第一层)

启动内容:
  1. demo_gazebo_sim/gazebo_sim_world.launch.py  Factory 世界(带墙和小人) + 机器人 + 桥接 + RViz
  2. EKF(robot_localization)                      融合 /odom + /imu -> /odometry/filtered
  3. map_server + AMCL                            用 map/Factory_map.yaml 定位(初值=出生点)
  4. nav2.launch.py(sim:=true)                    仿真参数下的完整导航栈
  5. camera_link -> camera_optical_frame 静态 TF  供视觉跟踪节点反投影用

用法:
  ros2 launch mycar_navigation2 sim_nav.launch.py
  ros2 launch mycar_navigation2 sim_nav.launch.py world:=house.sdf world_name:=empty \
      map_yaml:=/home/ros/Robot_ws/map/sim_map.yaml spawn_x:=7.03 spawn_y:=-4.92
  ros2 launch mycar_navigation2 sim_nav.launch.py enable_ekf:=false    # 不融合, AMCL 直接用 /odom
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_pkg = get_package_share_directory("mycar_navigation2")
    sim_pkg = get_package_share_directory("demo_gazebo_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_ekf = LaunchConfiguration("enable_ekf")
    map_yaml = LaunchConfiguration("map_yaml")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_enable_ekf = DeclareLaunchArgument(
        "enable_ekf", default_value="true",
        description="是否用 robot_localization 融合 /odom + /imu (AMCL 使用融合里程计)")
    declare_map_yaml = DeclareLaunchArgument(
        "map_yaml",
        default_value="/home/ros/Robot_ws/1_map.yaml",
        description="仿真地图(默认用 cartographer 建的外围图 1_map.yaml)")
    declare_world = DeclareLaunchArgument(
        "world", default_value="Factory.sdf",
        description="world 目录下的世界文件")
    declare_world_name = DeclareLaunchArgument(
        "world_name", default_value="Factory",
        description="SDF 里的 <world name=...>")
    declare_spawn_x = DeclareLaunchArgument("spawn_x", default_value="4.225")
    declare_spawn_y = DeclareLaunchArgument("spawn_y", default_value="-3.975")
    declare_spawn_yaw = DeclareLaunchArgument("spawn_yaw", default_value="0.0")
    # AMCL 初值用的是"地图坐标系"下的坐标, 不是世界坐标:
    #   * 1_map.yaml (cartographer 建的外围图, map 原点=出生点): (0, 0, 0)
    #   * Factory_map.yaml (origin (-11,-18), 与世界对齐): 出生点世界坐标 (4.225, -3.975)
    declare_amcl_initial_x = DeclareLaunchArgument("amcl_initial_x", default_value="0.0")
    declare_amcl_initial_y = DeclareLaunchArgument("amcl_initial_y", default_value="0.0")
    declare_amcl_initial_yaw = DeclareLaunchArgument("amcl_initial_yaw", default_value="0.0")
    amcl_initial_x = LaunchConfiguration("amcl_initial_x")
    amcl_initial_y = LaunchConfiguration("amcl_initial_y")
    amcl_initial_yaw = LaunchConfiguration("amcl_initial_yaw")
    declare_start_delay = DeclareLaunchArgument(
        "start_delay", default_value="20.0",
        description="等 Gazebo 起来(虚拟机负载高)再启动定位和导航的秒数")
    start_delay = LaunchConfiguration("start_delay")

    # 1. 仿真(世界 + 机器人 + 小人 + 桥接 + RViz)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, "launch", "gazebo_sim_world.launch.py")),
        launch_arguments={
            "world": world,
            "world_name": world_name,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )

    # 2. 地图 + AMCL 定位(初值 = 出生点, 不用手动点 2D Pose Estimate)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, "launch", "mycar_loca.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "map_yaml": map_yaml,
            "amcl_params": os.path.join(nav2_pkg, "params", "sim_amcl.yaml"),
            "amcl_odom_topic": "/odometry/filtered",
            "set_initial_pose": "true",
            "initial_x": amcl_initial_x,
            "initial_y": amcl_initial_y,
            "initial_yaw": amcl_initial_yaw,
        }.items(),
    )

    # 3. nav2 导航栈(仿真参数)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, "launch", "nav2.launch.py")),
        launch_arguments={"use_sim_time": "true", "sim": "true"}.items(),
    )

    # 4. 相机光学坐标系(视觉反投影需要): x 向右, y 向下, z 向前
    # 3.5 EKF: 融合 /odom(速度) 与 /imu(角速度), 输出 /odometry/filtered
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(enable_ekf),
        parameters=[
            os.path.join(nav2_pkg, "params", "sim_ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # 4. 相机光学坐标系(视觉反投影需要): x 向右, y 向下, z 向前
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_optical_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "-1.5707963", "--pitch", "0", "--yaw", "-1.5707963",
            "--frame-id", "camera_link", "--child-frame-id", "camera_optical_frame",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # 定位和导航等仿真起来后再拉, 否则 map_server 的 lifecycle 服务会因为 CPU 抢占超时
    localization_and_nav = TimerAction(
        period=LaunchConfiguration("start_delay"),
        actions=[ekf_node, localization, nav2],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_ekf,
        declare_map_yaml,
        declare_world,
        declare_world_name,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_yaw,
        declare_amcl_initial_x,
        declare_amcl_initial_y,
        declare_amcl_initial_yaw,
        declare_start_delay,
        gazebo,
        camera_optical_tf,
        localization_and_nav,
    ])
