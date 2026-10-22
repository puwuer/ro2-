"""
YOLO 识别 + Nav2 导航跟随 一键启动 launch

启动内容：
  1. mycar_navigation2 的 nav2.launch.py（导航栈，含 DWB 局部避障）
  2. yolo_tracker_node_unn 识别节点（YOLO + CSRT + 深度转 map 坐标 + 发导航目标）
  3. base_link -> camera_link 静态 TF（相机安装位姿）

用法：
  ros2 launch tracker yolo_nav2.launch.py
  ros2 launch tracker yolo_nav2.launch.py cam_x:=0.2 cam_z:=0.1
  ros2 launch tracker yolo_nav2.launch.py publish_camera_optical_tf:=True

注意：底盘驱动、雷达、相机、SLAM(Cartographer) 需要按现有方式单独启动，
本 launch 只负责导航栈 + YOLO 识别跟随。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---------- 参数声明（可在命令行覆盖）----------
    # 是否使用仿真时间
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="False")
    # 相机在机器人 base_link 系下的安装位置和朝向（按实车实际安装调整）
    declare_cam_x = DeclareLaunchArgument("cam_x", default_value="0.13")
    declare_cam_y = DeclareLaunchArgument("cam_y", default_value="0.0")
    declare_cam_z = DeclareLaunchArgument("cam_z", default_value="0.11")
    declare_cam_yaw = DeclareLaunchArgument("cam_yaw", default_value="0.0")
    # 深度图像话题（如果使用对齐到彩色的深度，可改为 /camera/aligned_depth_to_color/image_raw）
    declare_depth_topic = DeclareLaunchArgument(
        "depth_topic", default_value="/camera/depth/image_raw")
    # YOLOv8 ONNX 模型路径
    declare_model_path = DeclareLaunchArgument(
        "model_path", default_value="/home/ros/Robot_ws/src/tracker/models/yolov8n.onnx")
    # 是否由本 launch 发布 camera_link -> camera_color_optical_frame 的 TF
    # （相机驱动如果自己发布该 TF，就保持默认 False）
    declare_publish_optical_tf = DeclareLaunchArgument(
        "publish_camera_optical_tf", default_value="False")

    # 把命令行参数包装成 LaunchConfiguration，供下面节点使用
    use_sim_time = LaunchConfiguration("use_sim_time")
    cam_x = LaunchConfiguration("cam_x")
    cam_y = LaunchConfiguration("cam_y")
    cam_z = LaunchConfiguration("cam_z")
    cam_yaw = LaunchConfiguration("cam_yaw")
    depth_topic = LaunchConfiguration("depth_topic")
    model_path = LaunchConfiguration("model_path")
    publish_optical_tf = LaunchConfiguration("publish_camera_optical_tf")

    # ---------- 0. 启动 Astra Pro 相机 ----------
    # 先清理残留进程，再启动相机驱动
    astra_camera_script = ExecuteProcess(
        cmd=["bash", "/home/ros/astra-pro-ros2/clean_and_start_astra_pro_camera.sh"],
        name="astra_pro_camera",
        output="screen",
    )

    # ---------- 1. 引入 Nav2 导航栈 ----------
    # nav2.launch.py 启动 planner / controller(DWB) / bt_navigator 等
    # DWB 局部规划器使用 /scan 做障碍物避障，无需改动
    nav2_pkg = get_package_share_directory("mycar_navigation2")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, "launch", "nav2.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    local_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, "launch", "mycar_loca.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ---------- 2. YOLO 识别节点 ----------
    # 检测到人 -> 深度反投影 + TF 转 map 系 -> 发布 /yolo/target_pose
    # -> 调用 /navigate_to_pose action 持续跟随
    yolo_node = Node(
        package="tracker",
        executable="yolo_tracker_node_unn",
        name="yolo_tracker_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "model_path": model_path,
            "depth_topic": depth_topic,
            "camera_info_topic": "/camera/depth/camera_info",
            "camera_frame": "camera_depth_optical_frame",
            "goal_frame": "map",
            "stop_distance": 1.0,       # 距离目标 1m 内停止发导航目标
            "goal_min_delta": 0.3,      # 目标移动超过 0.3m 才更新导航目标
            "lost_cancel_timeout": 5.0, # 目标丢失 5s 后取消导航
            "mirror_flip": True,        # 彩色图是镜像图，取深度时翻转 u 坐标
            "enable_direct_control": False,  # 默认交给 Nav2，不直接发 /cmd_vel
        }],
    )

    # ---------- 3. 相机静态 TF ----------
    # base_link -> camera_link：相机相对机器人底盘的位置和朝向
    # 默认 x=0.15、z=0.15、朝前，实车需要按实际安装位置修改
    base_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", cam_x, "--y", cam_y, "--z", cam_z,
            "--yaw", cam_yaw,
            "--frame-id", "base_link", "--child-frame-id", "camera_link",
        ],
    )

    # camera_link -> camera_color_optical_frame（可选）
    # 光学坐标系约定：x 向右、y 向下、z 朝前，所以 roll=-pi/2、yaw=-pi/2
    # 仅在相机驱动没有发布该 TF 时开启：publish_camera_optical_tf:=True
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0", "-1.5707963", "0", "-1.5707963",
            "camera_link", "camera_color_optical_frame",
        ],
        condition=IfCondition(publish_optical_tf),
    )

    # ---------- 组装 LaunchDescription ----------
    return LaunchDescription([
        declare_use_sim_time,
        declare_cam_x,
        declare_cam_y,
        declare_cam_z,
        declare_cam_yaw,
        declare_depth_topic,
        declare_model_path,
        declare_publish_optical_tf,
        astra_camera_script,
        nav2_launch,
        yolo_node,
        base_camera_tf,
        camera_optical_tf,
    ])