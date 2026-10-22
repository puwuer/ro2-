"""Factory 仿真 + cartographer 建图 + nav2 + RViz 一键启动

启动内容:
  1. demo_gazebo_sim/gazebo_sim_world.launch.py  Factory 世界 + 机器人 + 桥接 + RViz
  2. cartographer                                建图(发布 /map 与 map->odom)
  3. nav2 (可选, nav2:=true)                     用 RViz 的 "Nav2 Goal" 点目标点自动跑图
  4. teleop_twist_keyboard (可选, teleop:=true)   键盘遥控(经速度平滑器, 可与 nav2 共存)

用法:
  ros2 launch mycar_slam_cartographer sim_mapping.launch.py
  ros2 launch mycar_slam_cartographer sim_mapping.launch.py teleop:=false   # 自己另开终端遥控
  ros2 launch mycar_slam_cartographer sim_mapping.launch.py nav2:=false     # 只键控, 不起 nav2
  # 另开终端遥控: ros2 run teleop_twist_keyboard teleop_twist_keyboard

建图方式(二选一):
  A. 键盘: 在启动终端按 i/j/l/,/. 直接开
  B. nav2 目标点: RViz 工具栏点 "Nav2 Goal", 在地图上点一个点+拖方向, 小车自己走过去;
     初始地图很小, 先点附近的目标, 地图长出来后再点远的; 注意不要点到墙里

建完图保存地图(另开终端, 等地图稳定后再执行):
  ros2 run nav2_map_server map_saver_cli -f /home/ros/Robot_ws/map/Factory_cartographer \
      --occ 0.65 --free 0.25
  然后把 map/Factory_cartographer.yaml 设为 sim_nav 的默认地图:
  ros2 launch mycar_navigation2 sim_nav.launch.py map_yaml:=/home/ros/Robot_ws/map/Factory_cartographer.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory("demo_gazebo_sim")
    carto_pkg = get_package_share_directory("mycar_slam_cartographer")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    teleop = LaunchConfiguration("teleop")
    rviz = LaunchConfiguration("rviz")
    nav2 = LaunchConfiguration("nav2")
    teleop_topic = LaunchConfiguration("teleop_topic")
    gui = LaunchConfiguration("gui")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_world = DeclareLaunchArgument("world", default_value="Factory.sdf")
    declare_world_name = DeclareLaunchArgument("world_name", default_value="Factory")
    # 出生点 = 世界原点偏移; cartographer 的 map 原点就是出生点
    declare_spawn_x = DeclareLaunchArgument("spawn_x", default_value="4.225")
    declare_spawn_y = DeclareLaunchArgument("spawn_y", default_value="-3.975")
    declare_spawn_yaw = DeclareLaunchArgument("spawn_yaw", default_value="0.0")
    declare_teleop = DeclareLaunchArgument(
        "teleop", default_value="true",
        description="是否在本终端启动键盘遥控(teleop_twist_keyboard)")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="true")
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true",
        description="是否启动 Gazebo GUI (无人值守建图可设 false 提速)")
    declare_nav2 = DeclareLaunchArgument(
        "nav2", default_value="true",
        description="是否启动 nav2 (可用 RViz 的 Nav2 Goal 工具点目标点自动跑图)")
    # nav2 的 velocity_smoother 会一直发 /cmd_vel, 所以键盘遥控要发到 /cmd_vel_nav
    declare_teleop_topic = DeclareLaunchArgument(
        "teleop_topic", default_value="/cmd_vel_nav",
        description="键盘遥控发布的话题; 起 nav2 时用 /cmd_vel_nav, 不起 nav2 时用 /cmd_vel")

    # 1. 仿真(Factory + 机器人 + 桥接 + RViz)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, "launch", "gazebo_sim_world.launch.py")),
        launch_arguments={
            "world": world,
            "world_name": world_name,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_yaw": spawn_yaw,
            "gui": gui,
            "rviz": rviz,
        }.items(),
    )

    # 2. cartographer 建图
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(carto_pkg, "launch", "cartographer.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            # 仿真建图用关闭在线优化的配置: map->odom 不跳变, 局部代价地图不会丢激光(防撞墙)
            "configuration_basename": "mycar_sim_mapping.lua",
        }.items(),
    )

    # 3. nav2 导航栈(用 RViz 点目标点自动跑图; 地图来自 cartographer 的 /map)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mycar_navigation2"),
                         "launch", "nav2.launch.py")),
        launch_arguments={"use_sim_time": "true", "sim": "true"}.items(),
        condition=IfCondition(nav2),
    )

    # 4. 键盘遥控(默认在本终端按 i/j/l 驱动; 有 nav2 时发到 /cmd_vel_nav 才能生效)
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        condition=IfCondition(teleop),
        remappings=[("cmd_vel", teleop_topic)],
    )

    hint = LogInfo(msg=[
        "建图已启动: 用键盘(i/j/l/,/.)驱动小车走遍厂房, 覆盖 4 个角; ",
        "完成后另开终端执行 map_saver_cli 保存到 map/Factory_cartographer",
    ])

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_world_name,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_yaw,
        declare_teleop,
        declare_rviz,
        declare_gui,
        declare_nav2,
        declare_teleop_topic,
        hint,
        gazebo,
        cartographer,
        nav2_launch,
        teleop_node,
    ])
