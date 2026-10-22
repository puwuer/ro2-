#include "ros2_stm32_bridge/ros2_stm32_driver_node.hpp"

#include "iostream"

namespace ros2_stm32 {
namespace driver {

MiniDriver::MiniDriver() : Node("mini_driver"), parse_flag_(false), start_flag_(true) {

  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.read_only = true;

  this->declare_parameter<std::string>("port_name", "/dev/mycar",desc);
  this->declare_parameter<int>("baud_rate", 230400,desc);
  this->declare_parameter<std::string>("odom_frame", "odom",desc);
  this->declare_parameter<std::string>("base_frame", "base_link",desc);
  this->declare_parameter<int>("control_rate", 10,desc);
  this->declare_parameter<double>("maximum_encoding", 100.0);
  this->declare_parameter<double>("encoder_resolution", 44.0,desc);
  this->declare_parameter<double>("reduction_ratio", 90.0,desc);
  this->declare_parameter<double>("model_param_cw", 0.216);
  this->declare_parameter<double>("model_param_acw", 0.216);
  this->declare_parameter<double>("wheel_diameter", 0.065);
  this->declare_parameter<int>("kp", 25);
  this->declare_parameter<int>("ki", 0);
  this->declare_parameter<int>("kd", 30);
  this->declare_parameter<bool>("pub_odom_tf", true);

  this->get_parameter("port_name", port_name_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("control_rate", control_rate_);
  // this->get_parameter("maximum_encoding", maximum_encoding_);
  this->get_parameter("encoder_resolution", encoder_resolution_);
  this->get_parameter("reduction_ratio", reduction_ratio_);
  this->get_parameter("pub_odom_tf", pub_odom_tf_);
  // this->get_parameter("model_param_cw", model_param_cw_);
  // this->get_parameter("model_param_acw", model_param_acw_);
  // this->get_parameter("wheel_diameter", wheel_diameter_);
  // pulse_per_cycle_ = reduction_ratio_ * encoder_resolution_ / (M_PI * wheel_diameter_ * pid_rate_);
  pid_rate_ = 25.0; 
  last_twist_time_ = this->get_clock()->now();

  voltage_publisher_ = this->create_publisher<std_msgs::msg::UInt16>("voltage", 10);
  wheel_encode_publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("wheel_encode", 10);
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);

  twist_subscribe_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MiniDriver::TwistHandleCallback, this, std::placeholders::_1));

  send_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000 / control_rate_)),
                                        std::bind(&MiniDriver::SendTimerCallback, this));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  Run();
}

MiniDriver::~MiniDriver() {
  mutex_.lock();
  parse_flag_ = false;
  SpeedCommand(0,0);
  if (port_) {
    port_->cancel();
    port_->close();
    port_.reset();
  }
  io_service_.stop();
  io_service_.reset();
  mutex_.unlock();
}

bool MiniDriver::Init() {
  if (port_) {
    RCLCPP_ERROR(this->get_logger(), "error : port is already opened...");
    return false;
  }

  port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
  port_->open(port_name_, ec_);
  if (ec_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "error : port_->open() failed...port_name="
                                               << port_name_ << ", e=" << ec_.message().c_str() << "\n");
    return false;
  }
  // option settings...
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  port_->set_option(boost::asio::serial_port_base::character_size(8));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  return true;
}

void MiniDriver::ParseMessage() {

}

//协议解析变量
uint8_t rx_con = 0;  //接收计数器
uint8_t rx_checksum; //帧头部分校验和
uint8_t rx_buf[60];  //接收缓冲
/*
 * @功  能  串口接收回调函数，接收塔克X-Protocol
 */
void MiniDriver::recvCallback()
{
    //接收数据
	uint8_t res;

    while(1)
    {
      // RCLCPP_INFO(this->get_logger(), "接收消息...");
        //读取串口数据 boost::asio::read(*port_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
        boost::asio::read(*port_.get(), boost::asio::buffer(&res, 1), ec_);
        // RCLCPP_INFO(this->get_logger(), "接收数据：%d", res);
        //塔克X-Protocol协议解析数据
        if(rx_con < 3)    //=========接收帧头 + 长度
        {
            if(rx_con == 0)  //接收帧头1 0xAA
            {
                if(res == 0xAA)
                {
                    rx_buf[0] = res;
                    rx_con = 1;					
                    // RCLCPP_INFO(this->get_logger(), "接收数据：%d", res);
                }
                else
                {
                    
                }
            }else if(rx_con == 1) //接收帧头2 0x55
            {
                if(res == 0x55)
                {
                    rx_buf[1] = res;
                    rx_con = 2;
                    // RCLCPP_INFO(this->get_logger(), "接收数据：%d", res);
                }
                else
                {
                    rx_con = 0;						
                }				
            }
            else  //接收数据长度
            {
                rx_buf[2] = res;
                rx_con = 3;
                rx_checksum = (0xAA+0x55) + res;	//计算校验和
            }
        }
        else    //=========接收数据
        {
            if(rx_con < (rx_buf[2]-1) )
            {
                rx_buf[rx_con] = res;
                rx_con++;
                rx_checksum = rx_checksum + res;					
            }
            else    //判断最后1位
            {
                //接收完成，恢复初始状态
                rx_con = 0;					
                
                //数据校验
                if( res == rx_checksum )  //校验正确
                {	
                    //调用串口数据处理函数
                    // RCLCPP_INFO(this->get_logger(), "收到完整帧，帧码：%d", rx_buf[3]);
                    recvDataHandle(rx_buf);
                }	
                else
                {
                    // RCLCPP_ERROR(this->get_logger(), "校验错误，帧码：%d", rx_buf[3]);
                }
            }
        }         
    }
}

/*
 * @功  能  串口接收数据处理
 */
void MiniDriver::recvDataHandle(uint8_t* buffer_data)
{
    //机器人数据
    if(buffer_data[3] == ID_CPR2ROS_DATA)
    {
        //解析IMU加速度数据
        imu_data_.acc_x =  ((double)((int16_t)(buffer_data[4]*256+buffer_data[5]))*ACC_RATIO);
        imu_data_.acc_y =  ((double)((int16_t)(buffer_data[6]*256+buffer_data[7]))*ACC_RATIO);
        imu_data_.acc_z =  ((double)((int16_t)(buffer_data[8]*256+buffer_data[9]))*ACC_RATIO);

        //解析IMU陀螺仪数据
        imu_data_.gyro_x  = ((double)((int16_t)(buffer_data[10]*256+buffer_data[11]))*GYRO_RATIO);
        imu_data_.gyro_y  = ((double)((int16_t)(buffer_data[12]*256+buffer_data[13]))*GYRO_RATIO);
        imu_data_.gyro_z  = ((double)((int16_t)(buffer_data[14]*256+buffer_data[15]))*GYRO_RATIO);

        //解析机器人速度
        vel_data_.linear_x = ((double)((int16_t)(buffer_data[16]*256+buffer_data[17]))/1000);
        vel_data_.linear_y = ((double)((int16_t)(buffer_data[18]*256+buffer_data[19]))/1000);
        vel_data_.angular_z = ((double)((int16_t)(buffer_data[20]*256+buffer_data[21]))/1000);

        //解析电压值
        bat_vol_data_ = (double)(((buffer_data[22]<<8)+buffer_data[23]))/100;

        //计算里程计数据
        pos_data_.pos_x += (vel_data_.linear_x*cos(pos_data_.angular_z) - vel_data_.linear_y*sin(pos_data_.angular_z)) * DATA_PERIOD; 
        pos_data_.pos_y += (vel_data_.linear_x*sin(pos_data_.angular_z) + vel_data_.linear_y*cos(pos_data_.angular_z)) * DATA_PERIOD; 
        pos_data_.angular_z += vel_data_.angular_z * DATA_PERIOD;   //绕Z轴的角位移，单位：rad 

        //计算IMU四元数数据
        calculateImuQuaternion(imu_data_);

        //发布话题消息
        publishOdom();     //发布里程计话题
        publishImu();      //发布IMU传感器话题
        // publishBatVol();   //发布电池电压话题
        std_msgs::msg::UInt16 voltage_msg;
        voltage_msg.data = (uint16_t)(bat_vol_data_*100);
        voltage_publisher_->publish(voltage_msg);
        //发布odom里程计TF坐标变换（odom --> base_footprint）
        if(pub_odom_tf_ == true)
        {
          publishOdomTF();
        }
    }
}

/*
 * @功  能  发布IMU话题消息
 */
void MiniDriver::publishImu()
{
    imu_msg_.header.stamp    = this->now();
    imu_msg_.header.frame_id = "imu_link";
    imu_msg_.angular_velocity.x = imu_data_.gyro_x;
    imu_msg_.angular_velocity.y = imu_data_.gyro_y;
    imu_msg_.angular_velocity.z = imu_data_.gyro_z;
    imu_msg_.linear_acceleration.x = imu_data_.acc_x;
    imu_msg_.linear_acceleration.y = imu_data_.acc_y;
    imu_msg_.linear_acceleration.z = imu_data_.acc_z;
    imu_msg_.orientation.x = orient_data_.x;
    imu_msg_.orientation.y = orient_data_.y;
    imu_msg_.orientation.z = orient_data_.z;
    imu_msg_.orientation.w = orient_data_.w;

    // 不使用姿态角度
    imu_msg_.orientation.x = 0;
    imu_msg_.orientation.y = 0;

    // 协方差矩阵
    imu_msg_.orientation_covariance = {
        1e6, 0,    0,
        0,   1e6,  0,
        0,   0,    1e-6
    };
    imu_msg_.angular_velocity_covariance = {
        1e6, 0,    0,
        0,   1e6,  0,
        0,   0,    1e-6
    };
    imu_msg_.linear_acceleration_covariance = {
        0, 0, 0,
        0, 0, 0,
        0, 0, 0
    };

    imu_publisher_->publish(imu_msg_);
}

/*
 * @功  能  发送串口数据包，塔克X-Protocol协议
 * @参  数  *pbuf：发送数据指针
 *          len：发送数据长度个数，长度小于64字节
 *          num：帧号，帧编码
 * @返回值	 无
 */
void MiniDriver::sendSerialPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
    uint8_t tx_checksum = 0;//发送校验和
    uint8_t tx_buf[64];
	
    //判断是否超出长度
	if(len <= 64)
	{
		//获取数据
		tx_buf[0] = 0xAA;    //帧头
		tx_buf[1] = 0x55;    //
		tx_buf[2] = len+5;  //根据输出长度计算帧长度
		tx_buf[3] = num;    //帧编码
		
		for(i=0; i<len; i++)
		{
			tx_buf[4+i] = *(pbuf+i);
		}
		
		//计算校验和	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + tx_buf[i];
		}
		tx_buf[i] = tx_checksum;

        //计算帧长度
        cnt = len+5;
		
        //发送数据
        boost::asio::write(*port_.get(),boost::asio::buffer(tx_buf,cnt),ec_);
        //boost::asio::write(*port_.get(), boost::asio::buffer(data, 12), ec_);
	}
}

/*
 * @功  能  发布odom里程计话题消息
 */
void MiniDriver::publishOdom()
{
   //计算里程计四元数
    nav_msgs::msg::Odometry odom_msg;
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0,0,pos_data_.angular_z);       

    //获取数据
    odom_msg.header.stamp    = this->get_clock()->now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id  = base_frame_;
    odom_msg.pose.pose.position.x = pos_data_.pos_x;
    odom_msg.pose.pose.position.y = pos_data_.pos_y;
    odom_msg.pose.pose.position.z = 0;  //高度为0
    odom_msg.pose.pose.orientation.x = odom_quat.getX();
    odom_msg.pose.pose.orientation.y = odom_quat.getY();
    odom_msg.pose.pose.orientation.z = odom_quat.getZ();
    odom_msg.pose.pose.orientation.w = odom_quat.getW();
    odom_msg.twist.twist.linear.x = vel_data_.linear_x;
    odom_msg.twist.twist.linear.y = vel_data_.linear_y;
    odom_msg.twist.twist.angular.z = vel_data_.angular_z;

    //里程计协防差矩阵，用于robt_pose_ekf功能包，静止和运动使用不同的参数
    if(vel_data_.linear_x==0 && vel_data_.linear_y==0 && vel_data_.angular_z==0)
    {
        //机器人静止时，IMU水平陀螺仪会存在零飘，编码器没有误差，编码器数据权重增加
        odom_msg.pose.covariance = {   1e-9,    0,    0,    0,    0,    0, 
                                            0, 1e-3, 1e-9,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0, 1e-9 };

        odom_msg.twist.covariance = {  1e-9,    0,    0,    0,    0,    0, 
                                            0, 1e-3, 1e-9,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0, 1e-9 };
    }
    else
    {
        //机器人运动时，轮子滑动编码器误差增加，IMU陀螺仪数据更加准确，IMU数据权重增加
        odom_msg.pose.covariance = {   1e-3,    0,    0,    0,    0,    0, 
                                            0, 1e-3,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  1e3 };

        odom_msg.twist.covariance = {  1e-3,    0,    0,    0,    0,    0, 
                                            0, 1e-3,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  1e3 };
    }

    //发布
    odom_publisher_->publish(odom_msg);
}

/*
 * @功  能  发布里程计到base_footprint的TF坐标变换
 */
void MiniDriver::publishOdomTF()
{
    //发布里程计到footprint坐标变换
    if(pub_odom_tf_ == true)
    {
        /*    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = odom_frame_;
    transform_stamped.child_frame_id = base_frame_;
    transform_stamped.transform.translation.x = accumulation_x_;
    transform_stamped.transform.translation.y = accumulation_y_;
    transform_stamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, accumulation_th_);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform_stamped);*/
        //计算里程计TF四元数
        geometry_msgs::msg::TransformStamped transform_stamped;
        tf2::Quaternion q;
        q.setRPY(0,0,pos_data_.angular_z);

        //填充数据
        transform_stamped.header.stamp    = this->get_clock()->now();
        transform_stamped.header.frame_id = odom_frame_;
        transform_stamped.child_frame_id  = base_frame_;

        transform_stamped.transform.translation.x = pos_data_.pos_x;
        transform_stamped.transform.translation.y = pos_data_.pos_y;
        transform_stamped.transform.translation.z = 0.0;

        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        //发布TF坐标变换
        tf_broadcaster_->sendTransform(transform_stamped);
    }
}

/***************四元数计算**************************************************/
volatile float twoKp = 1.0f;     // 2 * proportional gain (Kp)
volatile float twoKi = 0.0f;     // 2 * integral gain (Ki)
 // quaternion of sensor frame relative to auxiliary frame
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;         
 // integral error terms scaled by Ki
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
volatile const float sampling_period  = DATA_PERIOD;

float invSqrt(float number);
/*
 * @功  能  平方根倒数 求四元数
 */
float invSqrt(float x)
{
    volatile long i;
    volatile float halfx, y;
    volatile const float f = 1.5F;

    halfx = x * 0.5F;
    y = x;
    i = * (( long * ) &y);
    
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( halfx * y * y ) );

    return y;
}
/*
 * @功  能  计算IMU四元数
 */
void MiniDriver::calculateImuQuaternion(struct imu_data imu_cel)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float roll,pitch,yaw ;

    //首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = invSqrt(imu_cel.acc_x * imu_cel.acc_x + imu_cel.acc_y * imu_cel.acc_y + imu_cel.acc_z * imu_cel.acc_z);

    imu_cel.acc_x *= recipNorm;
    imu_cel.acc_y *= recipNorm;
    imu_cel.acc_z *= recipNorm;      

    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    //误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (imu_cel.acc_y * halfvz - imu_cel.acc_z * halfvy);
    halfey = (imu_cel.acc_z * halfvx - imu_cel.acc_x * halfvz);
    halfez = (imu_cel.acc_x * halfvy - imu_cel.acc_y * halfvx);

    // 计算并应用积分反馈（如果启用）
    if(twoKi > 0.0f) 
    {
        integralFBx += twoKi * halfex * sampling_period;  // integral error scaled by Ki
        integralFBy += twoKi * halfey * sampling_period;
        integralFBz += twoKi * halfez * sampling_period;
        imu_cel.gyro_x += integralFBx;        // apply integral feedback
        imu_cel.gyro_y += integralFBy;
        imu_cel.gyro_z += integralFBz;
    }
    else 
    {
        integralFBx = 0.0f;       // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    // Apply proportional feedback
    imu_cel.gyro_x += twoKp * halfex;
    imu_cel.gyro_y += twoKp * halfey;
    imu_cel.gyro_z += twoKp * halfez;        

    // Integrate rate of change of quaternion
    imu_cel.gyro_x *= (0.5f * sampling_period);   // pre-multiply common factors
    imu_cel.gyro_y *= (0.5f * sampling_period);
    imu_cel.gyro_z *= (0.5f * sampling_period);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * imu_cel.gyro_x - qc * imu_cel.gyro_y - q3 * imu_cel.gyro_z);
    q1 += (qa * imu_cel.gyro_x + qc * imu_cel.gyro_z - q3 * imu_cel.gyro_y);
    q2 += (qa * imu_cel.gyro_y - qb * imu_cel.gyro_z + q3 * imu_cel.gyro_x);
    q3 += (qa * imu_cel.gyro_z + qb * imu_cel.gyro_y - qc * imu_cel.gyro_x); 

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //计算结果赋值到
    orient_data_.w = q0;
    orient_data_.x = q1;
    orient_data_.y = q2;
    orient_data_.z = q3;

    //计算欧拉角
    // roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	// pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	// yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
    // ROS_INFO("IMU:%f  %f  %f ",roll,pitch,yaw);

}


void MiniDriver::DistributeMessage(MessageType type, uint8_t* payload) {
  if (type == VOLTAGE) {
    std_msgs::msg::UInt16 voltage_msg;
    voltage_msg.data = (payload[0] << 8 & 0xff00) | (payload[1] & 0x00ff);
    voltage_publisher_->publish(voltage_msg);
  } else if (type == WHEEL_ENCODE) {
    std_msgs::msg::UInt16MultiArray wheel_encode_msg;
    for (int i = 0; i < 4; i++) {
      wheel_encode_msg.data.push_back((payload[2 * i] << 8 & 0xff00) | (payload[2 * i + 1] & 0x00ff));
    }
    wheel_encode_publisher_->publish(wheel_encode_msg);
    // HandleEncodeMessage(wheel_encode_msg.data.at(0), wheel_encode_msg.data.at(1));
  } else if (type == ANGULAR_VELOCITY) {
    imu_msg_.angular_velocity.x = (int16_t)((payload[0] << 8 & 0xff00) | (payload[1] & 0x00ff)) * 0.1;
    imu_msg_.angular_velocity.y = (int16_t)((payload[2] << 8 & 0xff00) | (payload[3] & 0x00ff)) * 0.1;
    imu_msg_.angular_velocity.z = (int16_t)((payload[4] << 8 & 0xff00) | (payload[5] & 0x00ff)) * 0.1;
    imu_msg_flag_[0] = true;
    CheckAndPublishImu();
  } else if (type == ACCELERATION) {
    imu_msg_.linear_acceleration.x = (int16_t)((payload[0] << 8 & 0xff00) | (payload[1] & 0x00ff)) * 0.1;
    imu_msg_.linear_acceleration.y = (int16_t)((payload[2] << 8 & 0xff00) | (payload[3] & 0x00ff)) * 0.1;
    imu_msg_.linear_acceleration.z = (int16_t)((payload[4] << 8 & 0xff00) | (payload[5] & 0x00ff)) * 0.1;
    imu_msg_flag_[1] = true;
    CheckAndPublishImu();
  } else if (type == ORIENTATION) {
    tf2::Quaternion quaternion;
    quaternion.setRPY((int16_t)((payload[0] << 8 & 0xff00) | (payload[1] & 0x00ff)) * 0.001744444,
                      -(int16_t)((payload[2] << 8 & 0xff00) | (payload[3] & 0x00ff)) * 0.001744444,
                      (int16_t)((payload[4] << 8 & 0xff00) | (payload[5] & 0x00ff)) * 0.001744444);
    imu_msg_.orientation.x = quaternion.getX();
    imu_msg_.orientation.y = quaternion.getY();
    imu_msg_.orientation.z = quaternion.getZ();
    imu_msg_.orientation.w = quaternion.getW();
    imu_msg_flag_[2] = true;
    CheckAndPublishImu();
  }
}

void MiniDriver::CheckAndPublishImu() {
  if (imu_msg_flag_[0] && imu_msg_flag_[1] && imu_msg_flag_[2]) {
    imu_msg_.header.frame_id = "imu_link";
    imu_msg_.header.stamp = this->get_clock()->now();
    imu_publisher_->publish(imu_msg_);
    std::fill_n(std::begin(imu_msg_flag_), 3, false);
  }
}

int MiniDriver::CalculateDelta(int& current_encode, int receive_encode) {
  int delta;
  if (receive_encode > current_encode) {
    delta = (receive_encode - current_encode) < (current_encode - receive_encode + 65535)
                ? (receive_encode - current_encode)
                : (receive_encode - current_encode - 65535);
  } else {
    delta = (current_encode - receive_encode) < (receive_encode - current_encode + 65535)
                ? (receive_encode - current_encode)
                : (receive_encode - current_encode + 65535);
  }
  current_encode = receive_encode;
  return delta;
}

void MiniDriver::HandleEncodeMessage(short left_encode, short right_encode) {
  now_ = this->get_clock()->now();
  if (start_flag_) {
    accumulation_x_ = 0.0;
    accumulation_y_ = 0.0;
    accumulation_th_ = 0.0;
    current_left_encode_ = left_encode;
    current_right_encode_ = right_encode;
    last_time_ = now_;
    start_flag_ = false;
    return;
  }
  int delta_left = CalculateDelta(current_left_encode_, left_encode);
  int delta_right = CalculateDelta(current_right_encode_, right_encode);
  delta_time_ = (now_ - last_time_).seconds();
  if (delta_time_ > 0.02) {
    double model_param;
    this->get_parameter("model_param_cw", model_param_cw_);
    this->get_parameter("model_param_acw", model_param_acw_);
    if (delta_right <= delta_left) {
      model_param = model_param_cw_;
    } else {
      model_param = model_param_acw_;
    }
    this->get_parameter("wheel_diameter", wheel_diameter_);
    pulse_per_cycle_ = reduction_ratio_ * encoder_resolution_ / (M_PI * wheel_diameter_ * pid_rate_);
    double delta_theta = (delta_right - delta_left) / (pulse_per_cycle_ * pid_rate_ * model_param);
    double v_theta = delta_theta / delta_time_;
    
    double delta_dis = (delta_right + delta_left) / (pulse_per_cycle_ * pid_rate_ * 2.0);
    double v_dis = delta_dis / delta_time_;
    double delta_x, delta_y;
    if (delta_theta == 0) {
      delta_x = delta_dis;
      delta_y = 0.0;
    } else {
      delta_x = delta_dis * (sin(delta_theta) / delta_theta);
      delta_y = delta_dis * ((1 - cos(delta_theta)) / delta_theta);
    }

    accumulation_x_ += (cos(accumulation_th_) * delta_x - sin(accumulation_th_) * delta_y);
    accumulation_y_ += (sin(accumulation_th_) * delta_x + cos(accumulation_th_) * delta_y);
    accumulation_th_ += delta_theta;

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = odom_frame_;
    transform_stamped.child_frame_id = base_frame_;
    transform_stamped.transform.translation.x = accumulation_x_;
    transform_stamped.transform.translation.y = accumulation_y_;
    transform_stamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, accumulation_th_);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform_stamped);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = accumulation_x_;
    odom_msg.pose.pose.position.y = accumulation_y_;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation.x = q.getX();
    odom_msg.pose.pose.orientation.y = q.getY();
    odom_msg.pose.pose.orientation.z = q.getZ();
    odom_msg.pose.pose.orientation.w = q.getW();
    odom_msg.twist.twist.linear.x = v_dis;
    
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = v_theta;
    odom_publisher_->publish(odom_msg);
  }
  last_time_ = now_;
}

//cmd_vel话题回调函数
void MiniDriver::TwistHandleCallback(const geometry_msgs::msg::Twist& msg) {
  twist_mutex_.lock();
  last_twist_time_ = this->get_clock()->now();
  current_twist_ = msg;
  // Pid();
  twist_mutex_.unlock();
}

void MiniDriver::SpeedCommand(short left,short right){
  uint8_t data[12] = {0xfc, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdf};
  data[2] = data[6] = (left >> 8) & 0xff;
  data[3] = data[7] = left & 0xff;
  data[4] = data[8] = (right >> 8) & 0xff;
  data[5] = data[9] = right & 0xff;
  for (int i = 0; i < 10; i++) {
    data[10] ^= data[i];
  }
  boost::asio::write(*port_.get(), boost::asio::buffer(data, 12), ec_);
}

void MiniDriver::SendTimerCallback() {
  double left_d, right_d, radio;
  double model_param;
  short left, right;

  double speed_x,speed_y, speed_w;
  if ((this->get_clock()->now() - last_twist_time_).seconds() <= 1.0) {
    speed_x = current_twist_.linear.x;
    speed_y = current_twist_.linear.y;
    speed_w = current_twist_.angular.z;
  } else {
    speed_x = 0;
    speed_y = 0;
    speed_w = 0;
  }

  static uint8_t vel_data[11];

  //数据转换
  vel_data[0] = (int16_t)(speed_x*1000)>>8;
  vel_data[1] = (int16_t)(speed_x*1000);
  vel_data[2] = (int16_t)(speed_y*1000)>>8;
  vel_data[3] = (int16_t)(speed_y*1000);
  vel_data[4] = (int16_t)(speed_w*1000)>>8;
  vel_data[5] = (int16_t)(speed_w*1000);
  // RCLCPP_ERROR(this->get_logger(), "发送数据：%2f, %2f, %2f", speed_x, speed_y, speed_w);
  //发送串口数据
  sendSerialPacket(vel_data, 6, ID_ROS2CTR_VEL);
  // if (angular_speed <= 0) {
  //   model_param = model_param_cw_;
  // } else {
  //   model_param = model_param_acw_;
  // }

  // left_d = (linear_speed - model_param / 2 * angular_speed) * pulse_per_cycle_;
  // right_d = (linear_speed + model_param / 2 * angular_speed) * pulse_per_cycle_;

  // this->get_parameter("maximum_encoding", maximum_encoding_);
  // radio = std::max(std::max(std::abs(left_d), std::abs(right_d)) / maximum_encoding_, 1.0);

  // left = static_cast<short>(left_d / radio);
  // right = static_cast<short>(right_d / radio);

  // SpeedCommand(left,right);
  
}

void MiniDriver::Pid(){

}
void MiniDriver::Run() {
  if (Init()) {
    std::thread parse_thread(&MiniDriver::recvCallback, this);
    parse_thread.detach();
  }
}

}  // namespace driver
}  // namespace ros2_stm32

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_stm32::driver::MiniDriver>());
  rclcpp::shutdown();
  return 0;
}