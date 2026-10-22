#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mycar_interfaces/msg/system_status.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
// 【必须添加】引入 ONNX Runtime 核心头文件
// #include <onnxruntime/core/session/onnxruntime_cxx_api.h>
// #include <onnxruntime/core/providers/cpu/cpu_provider_factory.h>
#include <onnxruntime_cxx_api.h>

#include <memory>
#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

/**
 * YOLO 识别 + CSRT 局部跟踪 + Nav2 导航跟随节点
 *
 * 工作流程：
 *  1. 订阅彩色图 / 深度图（时间同步）
 *  2. YOLOv8 (ONNX Runtime) 检测人 -> 得到目标框
 *  3. 目标框交给 CSRT 做帧间局部跟踪
 *  4. 用深度值 + 相机内参把目标反投影成相机系 3D 点
 *  5. 通过 TF 把坐标转到 map 系，发布 /yolo/target_pose
 *  6. 调用 Nav2 的 /navigate_to_pose action 持续发送跟随目标
 */
class YoloTrackerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  YoloTrackerNode()
  : Node("yolo_tracker_node"), frame_counter_(0),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // 1. 【ONNX Runtime 初始化核心】
    // 模型路径：请务必修改这里为真实的绝对路径
    std::string model_path = "/home/ros/Robot_ws/src/tracker/models/yolov8n.onnx";

    // 2. 声明可以在运行中动态修改的参数
    declare_parameter("model_path", "/home/ros/Robot_ws/src/tracker/models/yolov8n.onnx");
    declare_parameter("detection_interval", 30);          // 每隔多少帧用 YOLO 重新检测一次
    declare_parameter("angular_gain", 0.02);              // 旧版直接控制模式的角度增益
    declare_parameter("rgb_topic", "/camera/color/image_mirrored");  // 彩色图话题
    declare_parameter("depth_topic", "/camera/depth/image_raw");
    declare_parameter("camera_info_topic", "/camera/depth/camera_info");
    declare_parameter("camera_frame", "camera_depth_optical_frame");  // 深度相机光学坐标系
    declare_parameter("goal_frame", "map");               // 导航目标所在的坐标系
    declare_parameter("stop_distance", 1.0);              // 距离目标多近时停止发导航目标
    declare_parameter("goal_min_delta", 0.3);             // 目标移动超过多少米才更新导航目标
    declare_parameter("lost_cancel_timeout", 5.0);        // 目标丢失多久后取消导航
    declare_parameter("target_min_depth", 0.2);           // 有效深度范围：最小距离（米）
    declare_parameter("target_max_depth", 3.0);           // 有效深度范围：最大距离（米）
    declare_parameter("mirror_flip", true);               // 彩色图是镜像图时翻转 u 坐标
    declare_parameter("enable_direct_control", false);    // 是否使用旧版直接发 /cmd_vel 的模式
    declare_parameter("enable_nav_client", true);         // 是否由本节点调用 /navigate_to_pose
                                                          // (仿真里交给 mycar_patrol 巡检节点, 置 false)
    declare_parameter("confidence_min", 0.4);             // 发布"检测到目标"的最低置信度
    declare_parameter("detect_threshold", 0.25);          // YOLO 框出人的最低分数
    declare_parameter("fallback_fx", 554.0);              // 无 camera_info 时的 Astra Pro S 默认内参
    declare_parameter("fallback_fy", 521.0);
    declare_parameter("fallback_cx", 320.0);
    declare_parameter("fallback_cy", 240.0);

    // 读取参数到成员变量
    get_parameter("model_path", model_path);
    get_parameter("rgb_topic", rgb_topic_);
    get_parameter("depth_topic", depth_topic_);
    get_parameter("camera_info_topic", camera_info_topic_);
    get_parameter("camera_frame", camera_frame_);
    get_parameter("goal_frame", goal_frame_);
    get_parameter("stop_distance", stop_distance_);
    get_parameter("goal_min_delta", goal_min_delta_);
    get_parameter("lost_cancel_timeout", lost_cancel_timeout_);
    get_parameter("target_min_depth", target_min_depth_);
    get_parameter("target_max_depth", target_max_depth_);
    get_parameter("mirror_flip", mirror_flip_);
    get_parameter("enable_direct_control", enable_direct_control_);
    get_parameter("enable_nav_client", enable_nav_client_);
    get_parameter("confidence_min", confidence_min_);
    get_parameter("detect_threshold", detect_threshold_);
    get_parameter("fallback_fx", fx_);
    get_parameter("fallback_fy", fy_);
    get_parameter("fallback_cx", cx_);
    get_parameter("fallback_cy", cy_);

    // 设置环境变量和 Session 配置
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "yolo_tracker");
    session_options_.SetIntraOpNumThreads(2); // 虚拟机 CPU 资源少，设为 1 防止资源竞争卡顿
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    try {
      session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_options_);
      Ort::AllocatorWithDefaultOptions allocator;
      // 记录模型的输入/输出节点名，推理时按名字传张量
      for (size_t i = 0; i < session_->GetInputCount(); ++i) {
        input_names_.push_back(session_->GetInputNameAllocated(i, allocator).get());
      }
      for (size_t i = 0; i < session_->GetOutputCount(); ++i) {
        output_names_.push_back(session_->GetOutputNameAllocated(i, allocator).get());
      }
      RCLCPP_INFO(get_logger(), "ONNX loaded, input=%s output=%s",
                  input_names_[0].c_str(), output_names_[0].c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load ONNX model: %s", e.what());
      throw;
    }

    // 3. 初始化发布者和订阅者
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);      // 旧版直接控制
    debug_pub_ = image_transport::create_publisher(this, "/tracking/debug_image");   // 调试图像
    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/yolo/target_pose", 10);  // map 系目标点
    // 检测结果发布给巡检/跟踪状态机(mycar_patrol)
    detection_pub_ = create_publisher<mycar_interfaces::msg::SystemStatus>(
      "/yolo/detection", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    // 订阅系统权威状态, 用于填充检测消息里的 mode 字段
    status_sub_ = create_subscription<mycar_interfaces::msg::SystemStatus>(
      "/system_status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](mycar_interfaces::msg::SystemStatus::ConstSharedPtr msg) {
        system_mode_ = msg->mode;
      });

    // 4. 初始化 Nav2 导航 action client（用于持续跟随）
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // 5. 订阅深度相机内参（用于把像素坐标反投影成 3D 点）
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        // K 矩阵：[fx, 0, cx, 0, fy, cy, 0, 0, 1]
        if (msg->k.size() >= 9 && msg->k[0] > 0.0) {
          fx_ = msg->k[0];
          fy_ = msg->k[4];
          cx_ = msg->k[2];
          cy_ = msg->k[5];
          ci_width_ = static_cast<int>(msg->width);
          ci_height_ = static_cast<int>(msg->height);
          ci_valid_ = true;
        }
      });

    // 6. 订阅彩色图和深度图
    rgb_sub_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);

    // 7. 时间同步器（保证彩色和深度图帧对齐）
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&YoloTrackerNode::sync_callback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    last_detection_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "YOLO + CSRT + Nav2 node initialized.");
  }

private:
  // 统一把图像缩小到这个分辨率，降低 CPU 开销
  static constexpr int kProcW = 320;
  static constexpr int kProcH = 240;

  // 订阅与发布
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  image_transport::Publisher debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<mycar_interfaces::msg::SystemStatus>::SharedPtr detection_pub_;
  rclcpp::Subscription<mycar_interfaces::msg::SystemStatus>::SharedPtr status_sub_;

  // 追踪器状态
  cv::Ptr<cv::Tracker> tracker_;
  cv::Rect current_bbox_;
  bool is_tracking_ = false;
  int frame_counter_;
  int detection_interval_ = 30;
  float last_confidence_ = 0.0F;      // 最近一次 YOLO 检测置信度
  float last_detection_score_ = 0.0F; // detect_yolo_onnxruntime 返回框的置信度
  uint8_t system_mode_ = mycar_interfaces::msg::SystemStatus::PATROL;

  // TF 坐标变换（camera -> base_link -> odom -> map）
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Nav2 导航目标状态
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandle::SharedPtr goal_handle_;               // 当前已接受的导航目标句柄
  bool goal_send_pending_ = false;                  // 是否有目标请求还没收到回复
  bool cancel_pending_ = false;                     // 请求在途中时收到取消，接受后立即取消
  bool goal_valid_ = false;                         // 当前是否有一个有效的导航目标
  geometry_msgs::msg::PoseStamped pending_goal_;    // 正在发送的目标（用于回调里确认）
  geometry_msgs::msg::PoseStamped last_goal_;       // 上一次已接受的目标（用于判断是否要更新）
  std::array<uint8_t, 16> current_goal_id_{};       // 当前目标 ID，用于区分新旧目标的回调
  double angular_gain_ = 0.02;

  // 参数成员
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_;
  std::string goal_frame_;
  double stop_distance_ = 1.0;
  double goal_min_delta_ = 0.3;
  double lost_cancel_timeout_ = 5.0;
  double target_min_depth_ = 0.2;
  double target_max_depth_ = 3.0;
  bool mirror_flip_ = true;
  bool enable_direct_control_ = false;
  bool enable_nav_client_ = true;
  double confidence_min_ = 0.4;
  double detect_threshold_ = 0.25;

  // 相机内参（优先用 camera_info，否则用 Astra Pro S 默认值）
  double fx_ = 554.0;
  double fy_ = 521.0;
  double cx_ = 320.0;
  double cy_ = 240.0;
  int ci_width_ = 640;
  int ci_height_ = 480;
  bool ci_valid_ = false;

  // 最近一次成功检测的时间，用于“目标丢失超时取消导航”
  std::chrono::steady_clock::time_point last_detection_time_;

  // ONNX Runtime 核心成员
  std::unique_ptr<Ort::Env> env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;

  // 核心回调函数：彩色图 + 深度图同步到达后处理一帧
  void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
  {
    try {
      // 1. 转换图像与降低分辨率
      cv::Mat frame = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
      // 深度编码自适应: 仿真深度相机发 32FC1(米), 实车 Astra 发 16UC1(毫米)
      cv::Mat depth_frame;
      if (depth_msg->encoding == "32FC1" || depth_msg->encoding == "32FC1;") {
        depth_frame = cv_bridge::toCvShare(depth_msg, "32FC1")->image.clone();
      } else {
        depth_frame = cv_bridge::toCvShare(depth_msg, "16UC1")->image.clone();
      }
      cv::resize(frame, frame, cv::Size(kProcW, kProcH));
      cv::resize(depth_frame, depth_frame, cv::Size(kProcW, kProcH));

      frame_counter_++;
      this->get_parameter("detection_interval", detection_interval_);

      cv::Rect bbox;
      bool detection_success = false;

      // 2. 触发 YOLO 推理逻辑
      // 触发时机：目标跟丢了，或者每隔 detection_interval 帧重新检测一次
      if (!is_tracking_ || (frame_counter_ % detection_interval_ == 0)) {
        bbox = detect_yolo_onnxruntime(frame);
        if (bbox.width > 0 && bbox.height > 0) {
          // 初始化或重新初始化 CSRT 追踪器
          if (!tracker_) tracker_ = cv::TrackerCSRT::create();
          tracker_->init(frame, bbox);
          current_bbox_ = bbox;
          last_confidence_ = last_detection_score_;
          is_tracking_ = true;
          detection_success = true;
        } else {
          is_tracking_ = false;
        }
      }

      // 3. OpenCV CSRT 局部跟踪
      // 在 YOLO 两次检测之间，用 CSRT 在局部区域持续跟踪目标
      if (is_tracking_ && tracker_) {
        bool ret = tracker_->update(frame, current_bbox_);
        if (ret) {
          bbox = current_bbox_;
          detection_success = true;
        } else {
          is_tracking_ = false;
          detection_success = false;
        }
      }

      // 4. 深度处理与导航目标输出
      if (detection_success) {
        last_detection_time_ = std::chrono::steady_clock::now();

        int cx = bbox.x + bbox.width / 2;
        int cy = bbox.y + bbox.height / 2;

        // 把检测框中心 -> 相机系 3D 点 -> map 系目标位姿
        geometry_msgs::msg::PoseStamped goal_pose;
        double target_depth_m = 0.0;
        double dist_map = 0.0;
        if (compute_target_in_map(depth_frame, depth_msg, cx, cy, goal_pose, target_depth_m, dist_map)) {
          // 发布 map 系目标点，供 RViz 调试
          target_pose_pub_->publish(goal_pose);
          publishDetection(true, goal_pose);

          if (enable_direct_control_) {
            // 旧版模式：直接发 /cmd_vel 控制底盘
            handle_direct_control(cx, target_depth_m);
          } else if (enable_nav_client_) {
            // 新版模式：把目标交给 Nav2，由 DWB 规划避障路径
            handle_navigation(goal_pose, dist_map);
          }
          // enable_nav_client_=false: 只上报检测结果, 导航由 mycar_patrol 负责
        } else {
          publishDetection(false, geometry_msgs::msg::PoseStamped());
        }

        // 绘制调试框
        cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);
        std::string dist_text = cv::format("%.2fm", target_depth_m);
        cv::putText(frame, dist_text, cv::Point(bbox.x, bbox.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
      } else {
        // 目标丢失：超过 lost_cancel_timeout 秒还没重新检测到，就取消导航
        publishDetection(false, geometry_msgs::msg::PoseStamped());
        auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_detection_time_).count();
        if (elapsed > lost_cancel_timeout_ && enable_nav_client_) {
          cancel_nav_goal();
        }
      }

      // 发布带有识别框的调试图像
      debug_pub_.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Processing error: %s", e.what());
    }
  }

  // 目标坐标换算：深度像素 -> 相机系 3D 点 -> TF 转到 map 系
  bool compute_target_in_map(const cv::Mat& depth_frame,
                             const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                             int bbox_cx, int bbox_cy,
                             geometry_msgs::msg::PoseStamped& goal_pose,
                             double& target_depth_m, double& dist_map)
  {
    int w = depth_frame.cols;
    int h = depth_frame.rows;
    int u = bbox_cx;
    int v = bbox_cy;
    // 彩色图是镜像图（image_mirrored），深度图不是，取深度前先把 u 翻转
    if (mirror_flip_) {
      u = (w - 1) - u;
    }
    if (u < 3 || v < 3 || u >= w - 3 || v >= h - 3) {
      return false;
    }

    // 提取真实有效深度：取目标中心 5x5 区域的平均值排除噪声
    // 16UC1: 单位 mm; 32FC1(仿真): 单位 m, 无效像素为 NaN
    cv::Mat roi = depth_frame(cv::Range(v - 2, v + 3), cv::Range(u - 2, u + 3));
    cv::Mat valid_mask = (roi > 0) & (roi == roi);
    cv::Scalar mean_depth = cv::mean(roi, valid_mask);
    if (mean_depth[0] <= 0.0) {
      return false;
    }
    const bool depth_in_meters = (depth_msg->encoding == "32FC1" || depth_msg->encoding == "32FC1;");
    target_depth_m = depth_in_meters ? mean_depth[0] : mean_depth[0] / 1000.0;
    if (target_depth_m < target_min_depth_ || target_depth_m > target_max_depth_) {
      return false;
    }

    // 把相机内参从原始分辨率缩放到处理后的 320x240 分辨率
    double orig_w = depth_msg->width > 0 ? static_cast<double>(depth_msg->width)
                                         : static_cast<double>(ci_width_);
    double orig_h = depth_msg->height > 0 ? static_cast<double>(depth_msg->height)
                                          : static_cast<double>(ci_height_);
    double sx = static_cast<double>(w) / orig_w;
    double sy = static_cast<double>(h) / orig_h;
    double fx = fx_ * sx;
    double fy = fy_ * sy;
    double pxc = cx_ * sx;
    double pyc = cy_ * sy;

    // 相机内参反投影：像素 (u, v) + 深度 Z -> 相机系 3D 点 (X, Y, Z)
    geometry_msgs::msg::PointStamped pt_cam;
    pt_cam.header.frame_id = camera_frame_;
    pt_cam.header.stamp = depth_msg->header.stamp;
    pt_cam.point.x = (u - pxc) * target_depth_m / fx;
    pt_cam.point.y = (v - pyc) * target_depth_m / fy;
    pt_cam.point.z = target_depth_m;

    // 通过 TF 把相机系坐标转到 map 系
    geometry_msgs::msg::PointStamped pt_map;
    try {
      auto transform = tf_buffer_.lookupTransform(
        goal_frame_, camera_frame_, depth_msg->header.stamp, tf2::durationFromSec(1.0));
      tf2::doTransform(pt_cam, pt_map, transform);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "TF transform failed: %s", e.what());
      return false;
    }

    // 组装导航目标位姿（map 系）
    goal_pose.header.frame_id = goal_frame_;
    goal_pose.header.stamp = depth_msg->header.stamp;
    goal_pose.pose.position.x = pt_map.point.x;
    goal_pose.pose.position.y = pt_map.point.y;
    goal_pose.pose.position.z = 0.0;

    // 目标朝向：从当前机器人位置指向目标；同时算出 map 平面上的距离
    double yaw = 0.0;
    dist_map = target_depth_m;
    try {
      auto tf_robot = tf_buffer_.lookupTransform(
        goal_frame_, "base_link", depth_msg->header.stamp, tf2::durationFromSec(0.2));
      double dx = pt_map.point.x - tf_robot.transform.translation.x;
      double dy = pt_map.point.y - tf_robot.transform.translation.y;
      dist_map = std::hypot(dx, dy);
      yaw = std::atan2(dy, dx);
    } catch (const tf2::TransformException&) {
      // 拿不到 base_link TF 时退化为深度距离、朝向 0
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    goal_pose.pose.orientation = tf2::toMsg(q);
    return true;
  }

  // 导航跟随逻辑：太近就取消，目标明显移动就更新，否则保持当前目标
  void handle_navigation(const geometry_msgs::msg::PoseStamped& goal, double dist_map)
  {
    if (dist_map > stop_distance_) {
      // 还没有有效目标，或新目标离旧目标超过 goal_min_delta，就发送/替换目标
      if (!goal_valid_ || pose_delta_2d(last_goal_, goal) > goal_min_delta_) {
        send_nav_goal(goal);
      }
    } else {
      // 已经到达目标附近，取消导航目标，避免原地反复规划
      cancel_nav_goal();
    }
  }

  // 旧版直接控制（可选）：不经过 Nav2，直接根据像素偏差发 /cmd_vel
  void handle_direct_control(int bbox_cx, double target_depth_m)
  {
    this->get_parameter("angular_gain", angular_gain_);
    geometry_msgs::msg::Twist twist;
    if (target_depth_m > target_min_depth_ && target_depth_m < target_max_depth_) {
      twist.linear.x = 0.2;  // 目标在有效距离内，控制前进
    } else {
      twist.linear.x = 0.0;
    }
    // 将横向像素误差转换为转向角速度
    twist.angular.z = angular_gain_ * (kProcW / 2.0 - bbox_cx);
    cmd_vel_pub_->publish(twist);
  }

  // 发送导航目标（Nav2 会自动用新目标替换正在执行的旧目标）
  void send_nav_goal(const geometry_msgs::msg::PoseStamped& goal)
  {
    if (goal_send_pending_) {
      return;  // 上一个请求还没收到回复，先不重复发送
    }
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Nav2 action server /navigate_to_pose not available");
      return;
    }

    NavigateToPose::Goal action_goal;
    action_goal.pose = goal;
    pending_goal_ = goal;
    goal_send_pending_ = true;
    cancel_pending_ = false;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // 目标回复回调：服务端接受或拒绝目标
    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr& gh) {
        goal_send_pending_ = false;
        if (gh) {
          goal_handle_ = gh;
          std::copy(gh->get_goal_id().data(), gh->get_goal_id().data() + 16,
                    current_goal_id_.begin());
          if (cancel_pending_) {
            // 发送期间收到了“取消”指令：接受后立刻取消
            cancel_pending_ = false;
            nav_client_->async_cancel_goal(gh);
            goal_handle_.reset();
            RCLCPP_WARN(get_logger(), "Nav2 goal accepted then canceled immediately");
          } else {
            last_goal_ = pending_goal_;
            goal_valid_ = true;
            RCLCPP_INFO(get_logger(), "Nav2 goal accepted");
          }
        } else {
          RCLCPP_WARN(get_logger(), "Nav2 goal rejected");
        }
      };
    // 结果回调：目标执行结束（成功 / 被替换 / 被取消）
    options.result_callback =
      [this](const GoalHandle::WrappedResult& result) {
        goal_send_pending_ = false;
        // 只有是当前目标的回调才清空句柄，避免旧目标的结束回调误删新目标
        if (std::equal(result.goal_id.data(), result.goal_id.data() + 16,
                       current_goal_id_.begin())) {
          goal_handle_.reset();
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(get_logger(), "Nav2 goal finished with code %d",
                      static_cast<int>(result.code));
        }
      };

    nav_client_->async_send_goal(action_goal, options);
  }

  // 取消当前导航目标
  void cancel_nav_goal()
  {
    goal_valid_ = false;
    if (goal_handle_) {
      nav_client_->async_cancel_goal(goal_handle_);
      goal_handle_.reset();
    } else if (goal_send_pending_) {
      // 目标还在发送途中，标记为接受后立刻取消
      cancel_pending_ = true;
    }
  }

  // 计算两个 map 系目标点在 XY 平面上的距离
  double pose_delta_2d(const geometry_msgs::msg::PoseStamped& a,
                       const geometry_msgs::msg::PoseStamped& b) const
  {
    return std::hypot(a.pose.position.x - b.pose.position.x,
                      a.pose.position.y - b.pose.position.y);
  }

  // 【替换为 ONNX Runtime 推理】YOLOv8 检测：输入一帧图，返回置信度最高的人形框
  cv::Rect detect_yolo_onnxruntime(const cv::Mat& frame)
  {
    cv::Rect best_bbox;
    if (!session_) return best_bbox;

    int img_h = frame.rows;
    int img_w = frame.cols;
    // 图像预处理(转换成 640x640 的 blob)：缩放 + 归一化 + BGR 转 RGB
    cv::Mat blob = cv::dnn::blobFromImage(
      frame, 1 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);

    // 1. 构建 ONNX Runtime 的输入张量
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<int64_t> input_shape = {1, 3, 640, 640};  // NCHW：1 张图、3 通道、640x640
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info, blob.ptr<float>(), blob.total(),
      input_shape.data(), input_shape.size());

    std::vector<const char*> in_names = {input_names_[0].c_str()};
    std::vector<const char*> out_names = {output_names_[0].c_str()};

    // 2. 执行模型推理
    std::vector<Ort::Value> output_tensors;
    try {
      output_tensors = session_->Run(Ort::RunOptions{nullptr},
                                     in_names.data(), &input_tensor, 1,
                                     out_names.data(), 1);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Inference failed: %s", e.what());
      return best_bbox;
    }

    // 3. 解析 YOLOv8 输出 (形状为 1, 84, 8400)
    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    std::vector<int64_t> out_shape =
      output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    int num_anchors = static_cast<int>(out_shape[2]);  // 8400 个候选位置
    int num_classes = static_cast<int>(out_shape[1]) - 4;  // 84 - 4 坐标 = 80 类

    // 尺寸映射：模型输出坐标在 640x640 图上，要映射回当前 320x240 图
    float scale_x = img_w / 640.0f;
    float scale_y = img_h / 640.0f;

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;

    // 遍历 8400 个检测点
    for (int i = 0; i < num_anchors; ++i) {
      // 计算坐标（前 4 个数：中心 x、中心 y、宽、高）
      float cx = output_data[0 * num_anchors + i] * scale_x;
      float cy = output_data[1 * num_anchors + i] * scale_y;
      float w = output_data[2 * num_anchors + i] * scale_x;
      float h = output_data[3 * num_anchors + i] * scale_y;

      // 查找该检测点的最高置信度类别（后 80 个数是各类别得分）
      float max_score = 0.0f;
      int best_class = -1;
      for (int j = 0; j < num_classes; ++j) {
        float score = output_data[(4 + j) * num_anchors + i];
        if (score > max_score) {
          max_score = score;
          best_class = j;
        }
      }

      // 如果是人(类别0), 且置信度大于阈值(默认 0.25, 仿真低模小人可调低)
      if (best_class == 0 && max_score > detect_threshold_) {
        int x = static_cast<int>(cx - w / 2);
        int y = static_cast<int>(cy - h / 2);
        boxes.emplace_back(x, y, static_cast<int>(w), static_cast<int>(h));
        confidences.push_back(max_score);
      }
    }

    // 4. 非极大值抑制 (NMS)：去掉重叠的框，只保留置信度最高的那个
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, static_cast<float>(detect_threshold_), 0.4, indices);
    if (!indices.empty()) {
      // 取置信度最高的那个框
      int best_idx = indices[0];
      for (int idx : indices) {
        if (confidences[idx] > confidences[best_idx]) {
          best_idx = idx;
        }
      }
      best_bbox = boxes[best_idx];
      last_detection_score_ = confidences[best_idx];
    }
    return best_bbox;
  }

  // 上报检测结果给巡检/跟踪状态机 (mycar_patrol)
  void publishDetection(bool visible, const geometry_msgs::msg::PoseStamped& pose)
  {
    mycar_interfaces::msg::SystemStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = goal_frame_;
    msg.mode = system_mode_;
    msg.target_visible = visible && (last_confidence_ >= static_cast<float>(confidence_min_));
    msg.target_pose = pose;
    msg.target_pose.header.frame_id = goal_frame_;
    msg.target_pose.header.stamp = msg.header.stamp;
    msg.confidence = msg.target_visible ? last_confidence_ : 0.0F;
    msg.waypoint_index = -1;
    msg.detail = msg.target_visible ? "detected" : "lost";
    detection_pub_->publish(msg);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
