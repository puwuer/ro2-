import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(text):
    return str(text).lower() in ("true", "1", "yes")


def launch_setup(context, *args, **kwargs):
    # 注意: 这里必须转成 bool, 否则会以字符串传给参数服务, 触发 use_sim_time 类型错误
    use_sim_time = _is_true(LaunchConfiguration("use_sim_time").perform(context))
    sim = _is_true(LaunchConfiguration("sim").perform(context))

    current_pkg = get_package_share_directory("mycar_navigation2")

    def params(name):
        return os.path.join(current_pkg, "params", name)

    # sim:=true 时在基础参数之后叠加仿真覆盖(footprint/膨胀半径等)
    extra_params = []
    if sim:
        extra_params = [os.path.join(current_pkg, "params_sim", "override.yaml")]

    # 计划器节点
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            params("planner.yaml")] + extra_params,
        )
    # 控制器节点
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            params("controller.yaml")] + extra_params,
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )
    # 恢复行为节点
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            params("behavior.yaml")],
    )
    # 路点跟随节点
    waypoint_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            params("waypoint.yaml")]
    )
    # 速度平滑节点
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn_delay=2.0,
        parameters=[
            {"use_sim_time": use_sim_time},
            params("velocity.yaml")],
        remappings=
                [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
    )
    # 路径平滑节点
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            params("smoother.yaml")],
    )
    # BT导航器节点
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            params("bt.yaml"),
            {"default_nav_to_pose_bt_xml": os.path.join(current_pkg,"bts","bt_planner_controller_behavior.xml")},
            {"default_nav_through_poses_bt_xml": os.path.join(current_pkg,"bts","bt_planner_controller_behavior_poses.xml")}
            ],
        )
    # 生命周期管理器节点
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        # 注意: 不能和 mycar_loca.launch.py 里的 manager 重名, 否则两个节点同名冲突
        name='lifecycle_manager_nav2',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            {'autostart': True},
                    {'node_names': [
                        'bt_navigator',
                        'planner_server',
                        'controller_server',
                        'behavior_server',
                        'waypoint_follower',
                        'velocity_smoother',
                        'smoother_server'
                    ]}])

    return [
        lifecycle_manager_node,
        bt_navigator_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        waypoint_node,
        velocity_smoother_node,
        smoother_server_node,
    ]


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="False")
    declare_sim = DeclareLaunchArgument(
        "sim", default_value="False",
        description="true 时叠加 params_sim/override.yaml (仿真车 footprint)")

    return LaunchDescription([
        declare_use_sim_time,
        declare_sim,
        OpaqueFunction(function=launch_setup),
    ])
