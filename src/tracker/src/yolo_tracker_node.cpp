#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <memory>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class YoloTrackerNode : public rclcpp::Node
{
public:
    YoloTrackerNode() : Node("yolo_tracker_node"), frame_counter_(0)
    {
        // 1. 【删除 YOLO 模型加载】这里直接初始化 MOG2 背景减除器
        bg_subtractor_ = cv::createBackgroundSubtractorMOG2(500, 16, false);

        // 2. 声明可以在运行中动态修改的参数
        this->declare_parameter("detection_interval", 30);
        this->declare_parameter("angular_gain", 0.02);

        // 3. 初始化发布者和订阅者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        debug_pub_ = image_transport::create_publisher(this, "/tracking/debug_image");

        // 4. 订阅彩色图和深度图
        rgb_sub_.subscribe(this, "/camera/color/image_mirrored", rmw_qos_profile_sensor_data);
        depth_sub_.subscribe(this, "/camera/depth/image_raw", rmw_qos_profile_sensor_data);

        // 5. 时间同步器（保证彩色和深度图帧对齐）
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&YoloTrackerNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Motion Tracker (MOG2 + CSRT) Node initialized.");
    }

private:
    // 订阅与发布
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    image_transport::Publisher debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // OpenCV 核心成员
    cv::Ptr<cv::BackgroundSubtractor> bg_subtractor_;
    cv::Ptr<cv::Tracker> tracker_;
    cv::Rect current_bbox_;
    
    // 运行状态标志
    bool is_tracking_ = false;
    int frame_counter_;
    int detection_interval_ = 30;
    double angular_gain_ = 0.02;

    // 核心回调函数
    void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg, 
                       const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
    {
        try {
            // 1. ROS 消息转 OpenCV 矩阵
            cv::Mat frame = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
            cv::Mat depth_frame = cv_bridge::toCvShare(depth_msg, "16UC1")->image;
            
            // 2. 【性能优化】在虚拟机里，强制缩小图像到 320x240
            cv::resize(frame, frame, cv::Size(320, 240));
            cv::resize(depth_frame, depth_frame, cv::Size(320, 240));
            
            frame_counter_++;
            this->get_parameter("detection_interval", detection_interval_);
            
            cv::Rect bbox;
            bool detection_success = false;

            // 3. 【运动检测触发时机】跟丢了，或者每隔 detection_interval 帧触发一次
            if (!is_tracking_ || (frame_counter_ % detection_interval_ == 0)) {
                bbox = detect_motion(frame); // 调用 MOG2 运动检测
                if (bbox.width > 0 && bbox.height > 0) {
                    // 初始化或重新初始化 CSRT 追踪器
                    if (!tracker_) {
                        tracker_ = cv::TrackerCSRT::create();
                    }
                    tracker_->init(frame, bbox);
                    current_bbox_ = bbox;
                    is_tracking_ = true;
                    detection_success = true;
                } else {
                    is_tracking_ = false;
                }
            }

            // 4. 【持续跟踪】如果追踪器正常，调用 CSRT 更新
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

            // 5. 【控制逻辑输出】如果检测/跟踪成功
            if (detection_success) {
                int img_h = frame.rows, img_w = frame.cols;
                int cx = bbox.x + bbox.width / 2;
                int cy = bbox.y + bbox.height / 2;
                int center_x = img_w / 2;
                int center_y = img_h / 2;

                // 获取目标区域的深度值 (取 5x5 区域中值排除噪声)
                float target_depth = 0.0f;
                if (cx < img_w - 3 && cy < img_h - 3 && cx > 3 && cy > 3) {
                    cv::Mat depth_roi = depth_frame(cv::Range(cy-2, cy+3), cv::Range(cx-2, cx+3));
                    cv::Scalar mean_depth = cv::mean(depth_roi, depth_roi > 0);
                    target_depth = mean_depth[0] / 1000.0f; // mm 转 m
                }

                // 计算像素偏差
                float error_x = center_x - cx;
                float error_y = center_y - cy;
                this->get_parameter("angular_gain", angular_gain_);
                
                // 发布 /cmd_vel 控制机器人底盘
                geometry_msgs::msg::Twist twist;
                if (target_depth > 0.2f && target_depth < 3.0f) {
                    twist.linear.x = 0.2; // 目标在0.2~3米内，则控制前进
                } else {
                    twist.linear.x = 0.0;
                }
                // 将横向像素误差转换为转向角速度
                twist.angular.z = angular_gain_ * error_x; 
                cmd_vel_pub_->publish(twist);

                // 绘制调试框
                cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, "MOG2+CSRT Track", cv::Point(bbox.x, bbox.y - 10), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                std::string depth_text = "Dist: " + std::to_string(target_depth) + "m";
                cv::putText(frame, depth_text, cv::Point(10, 30), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
            }

            // 发布带有识别框的调试图像（看是否锁住目标）
            debug_pub_.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing frame: %s", e.what());
        }
    }

    // 【核心替换】用 MOG2 运动检测替换 YOLO
    cv::Rect detect_motion(const cv::Mat& frame)
    {
        cv::Rect best_bbox;
        cv::Mat fg_mask;
        
        // 1. 执行背景减除，得到前景运动掩码
        bg_subtractor_->apply(frame, fg_mask);

        // 2. 形态学去噪（膨胀 + 腐蚀，让运动区域连成一片，去掉孤立噪点）
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_CLOSE, kernel);

        // 3. 在掩码中寻找所有轮廓
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(fg_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 4. 找出面积最大的轮廓（最大运动区域，也就是人的下半身）
        double max_area = 0.0;
        int max_idx = -1;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_idx = i;
            }
        }

        // 5. 面积超过 200 像素才判定为目标（过滤掉过小的噪点反光）
        if (max_idx >= 0 && max_area > 200.0) {
            best_bbox = cv::boundingRect(contours[max_idx]);
        }
        return best_bbox;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloTrackerNode>());
    rclcpp::shutdown();
    return 0;
}