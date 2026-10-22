import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


# 上一次仿真残留的进程模式：如果同时存在两套 Gazebo/bridge，/clock 与 /tf 会出现两个发布者，
# 时间戳交错会让 RViz 不断 reset（模型闪烁）、TF 缓存反复清空，所以启动前先清掉。
STALE_PATTERNS = [
    'ign gazebo',
    'ros_gz_bridge/parameter_bridge',
    'lib/rviz2/rviz2',
    'robot_state_publisher/robot_state_publisher',
    'cartographer',          # 残留的 SLAM 会和 AMCL 抢 map -> odom 变换
    'patrol_node',
    'yolo_tracker_node',
    'tf2_ros/static_transform_publisher',
    '/opt/ros/humble/lib/nav2_',   # 残留的 nav2 节点(重名会让 lifecycle 启动失败)
]


def _find_pids(pattern):
    try:
        out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True)
    except FileNotFoundError:
        return []
    return [int(pid) for pid in out.stdout.split()]


def clean_stale_processes(context):
    if str(context.launch_configurations.get('clean_stale', 'true')).lower() in ('false', '0', 'no'):
        return []

    killed = []
    for pattern in STALE_PATTERNS:
        for pid in _find_pids(pattern):
            if pid == os.getpid():
                continue
            try:
                os.kill(pid, signal.SIGINT)
                killed.append((pid, pattern))
            except ProcessLookupError:
                pass
    if not killed:
        return []

    time.sleep(3.0)
    # 没退干净的（例如挂住的 gz server）再补一刀
    for pid, _ in killed:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    summary = ', '.join(f'{pid}({pattern})' for pid, pattern in killed)
    return [LogInfo(msg=f'清理上一次仿真残留进程: {summary}')]


def generate_launch_description():

    this_pkg = get_package_share_directory('demo_gazebo_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    clean_stale = DeclareLaunchArgument(
        name='clean_stale',
        default_value='true',
        description='启动前清理上一次仿真的残留进程（避免出现两套 bridge 导致 RViz 抖动）'
    )
    # 世界文件与其 SDF 里的 <world name>: Factory.sdf -> Factory, house.sdf -> empty
    declare_world = DeclareLaunchArgument(
        'world', default_value='Factory.sdf',
        description='world 目录下的世界文件')
    declare_world_name = DeclareLaunchArgument(
        'world_name', default_value='Factory',
        description='SDF 里的 <world name=...>, 用于 /world/<name>/... 话题前缀')
    world_file = PathJoinSubstitution([this_pkg, 'world', LaunchConfiguration('world')])
    # 机器人出生点(Factory 地图中部房间; AMCL 初值必须与此一致)
    spawn_x = DeclareLaunchArgument('spawn_x', default_value='4.225')
    spawn_y = DeclareLaunchArgument('spawn_y', default_value='-3.975')
    spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.01')
    spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    # 小人模型(DoctorFemaleWalk)所在目录, 供 world 里的 model:// 解析
    actor_skins = os.path.join(
        get_package_share_directory('gazebo_ros_actor_plugin'), 'config', 'skins')
    declare_software_gl = DeclareLaunchArgument(
        'software_gl', default_value='1',
        description='LIBGL_ALWAYS_SOFTWARE: 1=软件渲染(默认, 虚拟机稳定), 0=硬件渲染')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='是否启动 Gazebo GUI (建图时设 false 可省一半 CPU)')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动 RViz (建图时设 false)')
    # 这台虚拟机的硬件 GL 会让 Gazebo GUI 在渲染相机纹理时 abort
    # (场景被创建两次 -> Actor skin already exists -> OGRE 断言), 软件渲染可稳定运行
    set_software_gl = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value=LaunchConfiguration('software_gl'))
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
               ':', actor_skins])
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
               ':', actor_skins])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # gui:=false 时用 -s (只跑 server, 不启 GUI)
            'gz_args': [
                PythonExpression([
                    "'-s -r ' if '", LaunchConfiguration('gui'),
                    "'.lower() in ('false', '0', 'no') else '-r '"
                ]),
                world_file,
            ]
        }.items(),
    )
    # 加载display.launch.py（联合仿真时关闭 joint_state_publisher_gui）
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_pkg, 'launch', 'display.launch.py')),
        launch_arguments={
            "use_gui": "false",
            "use_sim_time": "true",
            "rviz": LaunchConfiguration('rviz'),
        }.items()
    )
    urdf_path = os.path.join(this_pkg, "urdf", "mycar.urdf")
    # 加载机器人模型（直接从 URDF 文件读取）
    spawn = Node(package="ros_gz_sim", executable="create",
                arguments=[
                "-name", "mycar",
                "-x", LaunchConfiguration('spawn_x'),
                "-y", LaunchConfiguration('spawn_y'),
                "-z", LaunchConfiguration('spawn_z'),
                "-Y", LaunchConfiguration('spawn_yaw'),
                "-file", urdf_path],
            output="screen")
    # 建立桥接
    bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    # '[' = Gazebo -> ROS，']' = ROS -> Gazebo。
    # 传感器/里程计只走单向，避免消息回灌 Gazebo 形成回环
    arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/model/mycar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/model/mycar/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                ['/world/', LaunchConfiguration('world_name'),
                 '/model/mycar/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
                # 小人是静态模型, 用 set_pose 服务来挪动它(person_teleop 节点用)
                ['/world/', LaunchConfiguration('world_name'),
                 '/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                # "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                "/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                # IMU(陀螺仪+加速度计) -> /imu, 供 EKF 融合
                "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                # 小人(actor)速度指令: 键盘遥控用, 与机器人 /cmd_vel 分开
                "/actor/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                ],
    parameters=[{"qos_overrides./model/mycar.subscriber.reliability": "reliable"}],
    remappings=[
            ("/model/mycar/tf", "/tf"),
            (['/world/', LaunchConfiguration('world_name'),
              '/model/mycar/joint_state'], "joint_states"),
            ("/model/mycar/odometry","/odom")
        ],
    output="screen"
    )
    return LaunchDescription([
        clean_stale,
        OpaqueFunction(function=clean_stale_processes),
        declare_software_gl,
        set_software_gl,
        declare_world,
        declare_world_name,
        declare_gui,
        declare_rviz,
        spawn_x,
        spawn_y,
        spawn_z,
        spawn_yaw,
        set_ign_resource_path,
        set_gz_resource_path,
        gz_sim,
        display,
        spawn,
        bridge,
    ])
