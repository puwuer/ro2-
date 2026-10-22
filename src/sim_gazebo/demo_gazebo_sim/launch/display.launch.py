from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    demo_gazebo_sim_dir = get_package_share_directory("demo_gazebo_sim")
    default_model_path = os.path.join(demo_gazebo_sim_dir,"urdf","mycar.urdf")
    default_rviz_path = os.path.join(demo_gazebo_sim_dir,"rviz","robot.rviz")

    # 读取 URDF 文件（纯 URDF，无需 xacro 处理）
    with open(default_model_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # 是否启动 joint_state_publisher_gui（联合仿真时设为 false，由 Gazebo 提供关节状态）
    use_gui = DeclareLaunchArgument(
        name="use_gui",
        default_value="true",
        description="启动 joint_state_publisher_gui"
    )
    # 是否使用仿真时钟（与 Gazebo 联仿时设为 true，单独看模型时保持 false）
    use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="使用 /clock 仿真时间"
    )
    # 是否启动 rviz2 (建图/无界面跑仿真时可关掉省 CPU)
    show_rviz = DeclareLaunchArgument(
        name="rviz",
        default_value="true",
        description="是否启动 rviz2"
    )

    # 启动 robot_state_publisher 节点并加载 URDF 描述
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }]
    )
    # 启动 joint_state_publisher 节点（仅 use_gui=true 时启动）
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("use_gui")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    # rviz2 节点
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    return LaunchDescription([
        use_gui,
        use_sim_time,
        show_rviz,
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])
