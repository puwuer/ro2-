#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <mutex>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

namespace ros2_stm32 {
namespace driver {

enum ParseStatus { HEADER, TYPE, DATA, CHECKSUM, END };

enum MessageType { VOLTAGE = 1, WHEEL_ENCODE = 2, ANGULAR_VELOCITY = 3, ACCELERATION = 4, ORIENTATION = 5, SPEED = 6, PID = 7, SERVO = 8 };

#define PI 3.1415926

//机器人通信帧头定义
#define  ID_CPR2ROS_DATA    0x10    //下位机向ROS发送的综合数据
#define  ID_ROS2CTR_VEL     0x50    //ROS向下位机发送的速度数据
#define  ID_ROS2CTR_IMU     0x51    //ROS向下位机发送的IMU陀螺仪零偏校准指令
#define  ID_ROS2CTR_RTY     0x5a    //ROS向下位机发送的车型参数
#define  ID_ROS2CTR_LGT     0x52    //ROS向下位机发送的灯光调试数据
#define  ID_ROS2CTR_LST     0x53    //ROS向下位机发送的灯光保存数据
#define  ID_ROS2CTR_BEEP    0x54    //ROS向下位机发送的蜂鸣器数据

#define  LIGHT_M1    0x10    //单色模式  
#define  LIGHT_M2    0x20    //呼吸模式  

//IMU加速度计量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACC_RATIO 	  (2*9.8/32768)

//IMU陀螺仪量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位
#define GYRO_RATIO   ((500*PI/180)/32768)

//机器人数据处理周期,单位S
#define DATA_PERIOD   0.02f

typedef boost::shared_ptr<boost::asio::serial_port> serial_ptr;

//IMU数据结构体
struct imu_data
{
    float acc_x;
    float acc_y;
    float acc_z;    

    float gyro_x;
    float gyro_y;
    float gyro_z;
};

//IMU四元数结构体
struct imu_orientation_data
{
    float w;
    float x;
    float y;
    float z;    
};

//机器人速度数据结构体
struct velocity_data
{
    float linear_x;
    float linear_y;
    float angular_z;    
};

//机器人位置数据结构体
struct pose_data
{
    float pos_x;
    float pos_y;
    float angular_z;    
};

class MiniDriver : public rclcpp::Node {
 public:
  MiniDriver();
  ~MiniDriver();

  void Run();

 private:
  void Pid();
  bool Init();
  void ParseMessage();
  void DistributeMessage(MessageType type, uint8_t* payload);
  void CheckAndPublishImu();
  int CalculateDelta(int &current_encode, int receive_encode);
  void HandleEncodeMessage(short left_encode, short right_encode);
  void TwistHandleCallback(const geometry_msgs::msg::Twist & msg);
  void SendTimerCallback();
  void SpeedCommand(short left,short right);
  void recvCallback();
  void recvDataHandle(uint8_t* buffer_data);
  void calculateImuQuaternion(struct imu_data imu_cel);
  void sendSerialPacket(uint8_t *pbuf, uint8_t len, uint8_t num);
  void publishOdom();
  void publishOdomTF();
  void publishImu();

  std::string PrintHex(uint8_t* data, int length) {
    std::stringstream ss;
    for (int i = 0; i < length; i++) {
      ss << std::setw(2) << std::setfill('0') << std::hex << (int)data[i] << " ";
    }
    return ss.str();
  }

 private:
    //数据定义
  struct imu_data imu_data_;    //IMU数据
  struct imu_orientation_data orient_data_;  //IMU四元数姿态数据
  struct velocity_data vel_data_;    //机器人的速度
  struct pose_data pos_data_;    //机器人的位置
  float  bat_vol_data_;    //机器人电池电压数据
  bool parse_flag_;
  bool pub_odom_tf_;
  
  boost::system::error_code ec_;
  boost::asio::io_service io_service_;
  serial_port_ptr port_;

  std::mutex mutex_;
  std::mutex twist_mutex_;

  std::string port_name_;
  int baud_rate_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr voltage_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr wheel_encode_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscribe_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  

  sensor_msgs::msg::Imu imu_msg_;
  bool imu_msg_flag_[3]{false, false, false};  //  [ANGULAR_VELOCITY, ACCELERATION, ORIENTATION]

  int control_rate_;
  std::string odom_frame_, base_frame_, imu_frame_; 

  double maximum_encoding_;
  double pulse_per_cycle_, encoder_resolution_, reduction_ratio_, pid_rate_;
  double model_param_cw_, model_param_acw_, wheel_diameter_;

  bool start_flag_;
  double delta_time_;
  double accumulation_x_, accumulation_y_, accumulation_th_;
  int current_left_encode_, current_right_encode_;

  rclcpp::Time now_, last_time_, last_twist_time_;
  geometry_msgs::msg::Twist current_twist_;
  
};

}  // namespace driver
}  // namespace ros2_stm32
