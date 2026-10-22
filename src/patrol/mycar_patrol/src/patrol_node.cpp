// 巡检 / 跟踪 / 搜索 三状态任务节点
//
// 状态机:
//   PATROL: 依次调用 /navigate_to_pose 走巡检点, 循环
//   TRACK : 检测到人后, 导航到"人前方 approach_distance 处"的接近点并跟随
//   SEARCH: 目标丢失超过 lost_timeout, 原地旋转搜索, 超时回 PATROL
//
// 只有本节点调用 /navigate_to_pose; /cmd_vel 只在 SEARCH 期间由本节点直接发布。

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mycar_interfaces/msg/system_status.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
struct Waypoint
{
  double x;
  double y;
  double yaw;
};

// ============ 巡检点 (世界坐标, 单位 m/rad) ============
// Factory 世界里厂房四角(来自地图像素 (75,91)/(590,95)/(569,462)/(89,485)):
//   (-7.22,  5.93) / (18.53, 5.73) / (17.48, -12.62) / (-6.52, -13.77)
// 顺序就是巡检顺序, 走完最后一个回到第一个; 改坐标时按世界坐标填即可。
//
// 注意: 地图坐标系原点不一定在世界原点 ——
//   * 用自带的 map/Factory_map.yaml (origin (-11,-18), 与世界对齐): 偏移 = (0, 0)
//   * 用 cartographer 现场建的地图: 其 map 原点 = 出生点, 偏移 = 出生点世界坐标 (4.225, -3.975)
// 通过 launch 参数 map_offset_x/map_offset_y 传入, 节点按 map = 世界 - 偏移 换算。
const std::vector<Waypoint> kWaypoints = {
  {-7.22, 5.93, -0.66},
  {18.53, 5.73, -2.52},
  {17.48, -12.62, 2.51},
  {-6.52, -13.77, 0.71},
};
// ==============================================================================

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

class PatrolNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using SystemStatus = mycar_interfaces::msg::SystemStatus;

  enum class Mode
  {
    PATROL = SystemStatus::PATROL,
    TRACK = SystemStatus::TRACK,
    SEARCH = SystemStatus::SEARCH,
  };

  PatrolNode()
  : Node("patrol_node"), mode_(Mode::PATROL)
  {
    // ---------------- 参数 ----------------
    goal_timeout_ = declare_parameter("goal_timeout", 60.0);
    max_retries_ = declare_parameter("max_retries", 1);
    lost_timeout_ = declare_parameter("lost_timeout", 2.0);
    search_timeout_ = declare_parameter("search_timeout", 20.0);
    search_angular_vel_ = declare_parameter("search_angular_vel", 0.6);
    approach_distance_ = declare_parameter("approach_distance", 1.0);
    goal_min_delta_ = declare_parameter("goal_min_delta", 0.3);
    confidence_min_ = declare_parameter("confidence_min", 0.4);
    track_enter_count_ = declare_parameter("track_enter_count", 2);
    track_update_period_ = declare_parameter("track_update_period", 1.0);
    status_rate_ = declare_parameter("status_rate", 10.0);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    robot_frame_ = declare_parameter("robot_frame", std::string("base_link"));
    map_offset_x_ = declare_parameter("map_offset_x", 0.0);
    map_offset_y_ = declare_parameter("map_offset_y", 0.0);

    // ---------------- 发布/订阅 ----------------
    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    status_pub_ = create_publisher<SystemStatus>("/system_status", status_qos);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    detection_sub_ = create_subscription<SystemStatus>(
      "/yolo/detection", status_qos,
      std::bind(&PatrolNode::onDetection, this, std::placeholders::_1));

    // ---------------- 导航 action client (唯一调用者) ----------------
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // ---------------- TF ----------------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---------------- 定时器 ----------------
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, status_rate_)),
      std::bind(&PatrolNode::publishStatus, this));
    search_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&PatrolNode::onSearchTimer, this));
    check_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PatrolNode::checkTimeouts, this));
    // 目标补发: 启动瞬间 action server 可能还没发现, 或者目标被取消后没有续发
    goal_retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PatrolNode::ensureGoalAlive, this));

    last_detection_time_ = now();
    RCLCPP_INFO(get_logger(), "巡检节点已启动: %zu 个巡检点, 初始状态 PATROL", kWaypoints.size());
    sendWaypointGoal(0);
  }

private:
  // ---------------------------------------------------------------- 状态发布
  void publishStatus()
  {
    SystemStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.mode = static_cast<uint8_t>(mode_);
    msg.target_visible = target_visible_;
    msg.target_pose = last_target_pose_;
    msg.target_pose.header.stamp = now();
    msg.target_pose.header.frame_id = map_frame_;
    msg.confidence = last_confidence_;
    msg.waypoint_index = current_waypoint_;
    msg.detail = detail_;
    status_pub_->publish(msg);
  }

  void setMode(Mode mode, const std::string & reason)
  {
    if (mode_ == mode && reason == detail_) {
      return;
    }
    mode_ = mode;
    detail_ = reason;
    RCLCPP_INFO(get_logger(), "状态切换 -> %s (%s)", modeName(mode).c_str(), reason.c_str());
    publishStatus();
  }

  static std::string modeName(Mode mode)
  {
    switch (mode) {
      case Mode::PATROL: return "PATROL";
      case Mode::TRACK: return "TRACK";
      default: return "SEARCH";
    }
  }

  // ---------------------------------------------------------------- 检测回调
  void onDetection(const SystemStatus::SharedPtr msg)
  {
    target_visible_ = msg->target_visible;
    last_confidence_ = msg->confidence;
    if (msg->target_visible && msg->confidence >= static_cast<float>(confidence_min_)) {
      last_target_pose_ = msg->target_pose;
      last_detection_time_ = now();
      ++detection_streak_;
      if (mode_ != Mode::TRACK) {
        // 连续若干帧检测到目标才切换, 避免检测抖动导致状态来回跳
        if (detection_streak_ >= track_enter_count_) {
          RCLCPP_INFO(get_logger(), "连续 %d 帧检测到目标 (置信度 %.2f), 切换到跟踪",
                      detection_streak_, msg->confidence);
          cancelNavGoal();
          stopSearchMotion();
          setMode(Mode::TRACK, "检测到目标, 开始跟踪");
          track_goal_sent_ = false;
        } else {
          return;
        }
      }
      updateTrackGoal();
    } else if (!msg->target_visible) {
      detection_streak_ = 0;
      // 是否进入 SEARCH 交给 checkTimeouts 按 lost_timeout 判断
    }
  }

  // ---------------------------------------------------------------- 跟踪目标
  void updateTrackGoal()
  {
    if (mode_ != Mode::TRACK || !target_visible_) {
      return;
    }
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "跟踪中: 暂时拿不到 %s -> %s 的 TF", map_frame_.c_str(),
                           robot_frame_.c_str());
      return;
    }

    const double dx = last_target_pose_.pose.position.x - robot_pose.pose.position.x;
    const double dy = last_target_pose_.pose.position.y - robot_pose.pose.position.y;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-3) {
      return;
    }

    // 已经足够近: 取消当前导航目标, 停在原地盯着目标(不要继续往人身上开)
    if (dist <= approach_distance_) {
      if (goal_active_ || goal_pending_) {
        cancelNavGoal();
      }
      return;
    }

    // 限速: 目标点在抖动时不要以检测频率反复下发导航目标
    if (has_track_goal_time_ &&
      (now() - last_track_goal_time_).seconds() < track_update_period_)
    {
      return;
    }

    // 接近点: 沿"机器人 -> 目标"方向, 停在目标前方 approach_distance 处
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.pose.position.x = last_target_pose_.pose.position.x - approach_distance_ * dx / dist;
    goal.pose.position.y = last_target_pose_.pose.position.y - approach_distance_ * dy / dist;
    goal.pose.position.z = 0.0;
    goal.pose.orientation = quaternionFromYaw(std::atan2(dy, dx));  // 朝向目标

    if (track_goal_sent_) {
      const double moved = std::hypot(
        goal.pose.position.x - track_goal_.pose.position.x,
        goal.pose.position.y - track_goal_.pose.position.y);
      if (moved < goal_min_delta_) {
        return;  // 目标没怎么动, 不重复下发
      }
    }
    track_goal_ = goal;
    track_goal_sent_ = true;
    last_track_goal_time_ = now();
    has_track_goal_time_ = true;
    sendGoal(goal);
  }

  // ---------------------------------------------------------------- 巡检点
  void sendWaypointGoal(size_t index, bool reset_retry = true)
  {
    if (kWaypoints.empty()) {
      RCLCPP_ERROR(get_logger(), "巡检点列表为空");
      return;
    }
    current_waypoint_ = static_cast<int>(index % kWaypoints.size());
    const Waypoint & wp = kWaypoints[current_waypoint_];

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    // 世界坐标 -> 地图坐标
    goal.pose.position.x = wp.x - map_offset_x_;
    goal.pose.position.y = wp.y - map_offset_y_;
    goal.pose.position.z = 0.0;
    goal.pose.orientation = quaternionFromYaw(wp.yaw);

    if (reset_retry) {
      retry_count_ = 0;
    }
    sendGoal(goal);
    RCLCPP_INFO(get_logger(), "巡检点 %d/%zu: (%.2f, %.2f, %.2f)", current_waypoint_ + 1,
                kWaypoints.size(), wp.x, wp.y, wp.yaw);
  }

  void advanceWaypoint(const std::string & reason)
  {
    setMode(Mode::PATROL, reason);
    const size_t next = static_cast<size_t>(current_waypoint_ + 1) % kWaypoints.size();
    sendWaypointGoal(next);
  }

  // ---------------------------------------------------------------- action
  void sendGoal(const geometry_msgs::msg::PoseStamped & pose)
  {
    last_goal_attempt_ = now();
    has_goal_attempt_ = true;
    if (goal_pending_) {
      return;  // 上一个请求还没回复
    }
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "等待 /navigate_to_pose action server ...");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = pose;
    goal.pose.header.stamp = now();

    goal_pending_ = true;
    goal_sent_time_ = now();

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & handle) {
        goal_pending_ = false;
        if (handle) {
          goal_handle_ = handle;
          goal_active_ = true;
        } else {
          RCLCPP_WARN(get_logger(), "导航目标被拒绝");
          goal_active_ = false;
          onGoalFailed("目标被拒绝");
        }
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        goal_pending_ = false;
        goal_active_ = false;
        goal_handle_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          if (mode_ == Mode::PATROL) {
            advanceWaypoint("到达巡检点");
          } else if (mode_ == Mode::TRACK) {
            track_goal_sent_ = false;  // 到达接近点, 目标再动就重新下发
          }
        } else {
          onGoalFailed("导航未成功");
        }
      };

    nav_client_->async_send_goal(goal, options);
  }

  void onGoalFailed(const std::string & reason)
  {
    if (mode_ != Mode::PATROL) {
      return;  // 跟踪/搜索期间失败不推进巡检点
    }
    if (retry_count_ < max_retries_) {
      ++retry_count_;
      RCLCPP_WARN(get_logger(), "巡检点 %d 导航失败(%s), 重试 %d/%d", current_waypoint_ + 1,
                  reason.c_str(), retry_count_, max_retries_);
      // 注意: 重试时不要把 retry_count_ 清零, 否则会无限重试
      sendWaypointGoal(static_cast<size_t>(current_waypoint_), false);
    } else {
      RCLCPP_WARN(get_logger(), "巡检点 %d 导航失败(%s), 跳过", current_waypoint_ + 1,
                  reason.c_str());
      advanceWaypoint("巡检点失败, 跳过");
    }
  }

  void cancelNavGoal()
  {
    goal_pending_ = false;
    if (goal_handle_) {
      nav_client_->async_cancel_goal(goal_handle_);
      goal_handle_.reset();
    } else {
      nav_client_->async_cancel_all_goals();
    }
    goal_active_ = false;
  }

  // ---------------------------------------------------------------- 搜索
  void startSearch(const std::string & reason)
  {
    cancelNavGoal();
    setMode(Mode::SEARCH, reason);
    search_start_time_ = now();
  }

  void stopSearchMotion()
  {
    if (mode_ == Mode::SEARCH) {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
    }
  }

  void onSearchTimer()
  {
    if (mode_ != Mode::SEARCH) {
      return;
    }
    geometry_msgs::msg::Twist twist;
    twist.angular.z = search_angular_vel_;
    cmd_vel_pub_->publish(twist);
  }

  // ---------------------------------------------------------------- 超时检查
  // 没有目标在执行时补发(最多 2 秒一次), 避免启动竞态导致状态机空转
  void ensureGoalAlive()
  {
    if (goal_pending_ || goal_active_) {
      return;
    }
    if (has_goal_attempt_ && (now() - last_goal_attempt_).seconds() < 2.0) {
      return;
    }
    if (mode_ == Mode::PATROL) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "巡检点 %d 暂无导航目标, 重新下发", current_waypoint_ + 1);
      sendWaypointGoal(static_cast<size_t>(current_waypoint_));
    } else if (mode_ == Mode::TRACK) {
      track_goal_sent_ = false;
      updateTrackGoal();
    }
  }

  void checkTimeouts()
  {
    const rclcpp::Time stamp = now();

    if (mode_ == Mode::TRACK) {
      const double lost = (stamp - last_detection_time_).seconds();
      // 注意: 单帧丢检不算丢失, 要连续 lost_timeout 秒没有检测到目标才进搜索
      if (lost > lost_timeout_) {
        RCLCPP_WARN(get_logger(), "目标丢失 %.1fs, 进入搜索", lost);
        startSearch("目标丢失, 原地搜索");
      }
      return;
    }

    if (mode_ == Mode::SEARCH) {
      const double searching = (stamp - search_start_time_).seconds();
      if (searching > search_timeout_) {
        RCLCPP_WARN(get_logger(), "搜索 %.1fs 未发现目标, 返回巡检", searching);
        stopSearchMotion();
        setMode(Mode::PATROL, "搜索超时, 返回巡检");
        sendWaypointGoal(static_cast<size_t>(current_waypoint_));  // 从当前点继续
      }
      return;
    }

    // PATROL: 单个巡检点超时保护
    if (goal_active_ || goal_pending_) {
      const double elapsed = (stamp - goal_sent_time_).seconds();
      if (elapsed > goal_timeout_) {
        RCLCPP_WARN(get_logger(), "巡检点 %d 超时 %.1fs, 取消", current_waypoint_ + 1, elapsed);
        cancelNavGoal();
        onGoalFailed("导航超时");
      }
    }
  }

  // ---------------------------------------------------------------- 工具
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
      pose.header = tf.header;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      return false;
    }
  }

  // ---------------- 成员 ----------------
  Mode mode_;
  std::string detail_ = "启动巡检";

  double goal_timeout_ = 60.0;
  int max_retries_ = 1;
  double lost_timeout_ = 2.0;
  double search_timeout_ = 20.0;
  double search_angular_vel_ = 0.6;
  double approach_distance_ = 1.0;
  double goal_min_delta_ = 0.3;
  double confidence_min_ = 0.4;
  int track_enter_count_ = 2;
  double track_update_period_ = 1.0;
  double status_rate_ = 10.0;
  std::string map_frame_ = "map";
  std::string robot_frame_ = "base_link";
  double map_offset_x_ = 0.0;
  double map_offset_y_ = 0.0;

  int current_waypoint_ = 0;
  int retry_count_ = 0;
  int detection_streak_ = 0;

  bool target_visible_ = false;
  float last_confidence_ = 0.0F;
  geometry_msgs::msg::PoseStamped last_target_pose_;
  rclcpp::Time last_detection_time_;
  rclcpp::Time search_start_time_;
  rclcpp::Time goal_sent_time_;

  bool goal_pending_ = false;
  bool goal_active_ = false;
  GoalHandle::SharedPtr goal_handle_;

  bool track_goal_sent_ = false;
  geometry_msgs::msg::PoseStamped track_goal_;
  rclcpp::Time last_track_goal_time_;
  bool has_track_goal_time_ = false;

  rclcpp::Publisher<SystemStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<SystemStatus>::SharedPtr detection_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr search_timer_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::TimerBase::SharedPtr goal_retry_timer_;
  rclcpp::Time last_goal_attempt_;
  bool has_goal_attempt_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolNode>());
  rclcpp::shutdown();
  return 0;
}
