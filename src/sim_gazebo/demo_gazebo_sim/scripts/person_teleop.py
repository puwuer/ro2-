#!/usr/bin/env python3
"""键盘遥控 Gazebo 里的小人(静态模型)

原理: 订阅 /person/cmd_vel, 按仿真时间积分出位姿, 以 20Hz 调
      /world/<world>/set_pose (ros_gz_interfaces/srv/SetEntityPose) 把人挪过去。

用法:
  # 终端A (仿真起来后)
  ros2 run demo_gazebo_sim person_teleop.py
  # 终端B 键盘遥控小人 (i 前进 / , 后退 / j l 转向 / k 停)
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/person/cmd_vel

也可以一次性瞬移(世界坐标):
  ign service -s /world/Factory/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
    --timeout 5000 --req 'name: "person", position: {x: -2.125, y: -1.575, z: 0.0}'
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from ros_gz_interfaces.srv import SetEntityPose
from rosgraph_msgs.msg import Clock
from rclpy.node import Node


class PersonTeleop(Node):
    def __init__(self):
        super().__init__("person_teleop")
        self.declare_parameter("entity_name", "person")
        self.declare_parameter("world_name", "Factory")
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("initial_x", -2.125)
        self.declare_parameter("initial_y", -1.575)
        self.declare_parameter("initial_yaw", math.pi)
        self.declare_parameter("max_linear", 1.0)
        self.declare_parameter("max_angular", 1.5)

        self.entity = self.get_parameter("entity_name").value
        world = self.get_parameter("world_name").value
        self.x = self.get_parameter("initial_x").value
        self.y = self.get_parameter("initial_y").value
        self.yaw = self.get_parameter("initial_yaw").value
        self.max_v = self.get_parameter("max_linear").value
        self.max_w = self.get_parameter("max_angular").value

        self.vx = 0.0
        self.wz = 0.0
        self.sim_time = None
        self.last_sim = None
        self.busy = False
        self.moved = False

        self.create_subscription(Twist, "/person/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.cli = self.create_client(SetEntityPose, f"/world/{world}/set_pose")
        rate = float(self.get_parameter("update_rate").value)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.on_timer)
        self.get_logger().info(
            "小人遥控就绪: 发速度到 /person/cmd_vel (起点 world(%.2f, %.2f, yaw=%.2f))"
            % (self.x, self.y, self.yaw))

    def on_cmd(self, msg: Twist):
        self.vx = max(-self.max_v, min(self.max_v, msg.linear.x))
        self.wz = max(-self.max_w, min(self.max_w, msg.angular.z))

    def on_clock(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_timer(self):
        if self.sim_time is None:
            return
        if self.last_sim is None:
            self.last_sim = self.sim_time
            return
        dt = self.sim_time - self.last_sim
        self.last_sim = self.sim_time
        dt = max(0.0, min(dt, 0.5))

        # 按速度积分位姿 (仿真时间)
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        self.yaw += self.wz * dt

        if abs(self.vx) < 1e-4 and abs(self.wz) < 1e-4 and not self.moved:
            return  # 没速度且还没动过, 不用调服务
        self.moved = True
        if self.busy or not self.cli.service_is_ready():
            return

        req = SetEntityPose.Request()
        req.entity.name = self.entity
        req.pose.position.x = self.x
        req.pose.position.y = self.y
        req.pose.position.z = 0.0
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = math.sin(self.yaw * 0.5)
        req.pose.orientation.w = math.cos(self.yaw * 0.5)

        self.busy = True
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_done)

    def on_done(self, future):
        self.busy = False
        try:
            res = future.result()
            if res is not None and not res.success:
                self.get_logger().warn("set_pose 返回失败", throttle_duration_sec=5.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"set_pose 调用异常: {exc}", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = PersonTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
