#include "../include/udp_connection/laptop/imu_node.hpp"

float KalmanFilter::update(float measurement) {
    if (first_run) {
        X = measurement;
        first_run = false;
        return X;
    }

    P = P + Q;
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;

    return X;
}

ImuNode::ImuNode()
{
    node = rclcpp::Node::make_shared("imu_node");

    sub_imu_ = node->create_subscription<std_msgs::msg::String>("/ebimu_data", 10, std::bind(&ImuNode::imuCallback, this, std::placeholders::_1));
    pub_roll_ = node->create_publisher<std_msgs::msg::Float32>("/imu/roll", 10);
    pub_pitch_ = node->create_publisher<std_msgs::msg::Float32>("/imu/pitch", 10);
    pub_yaw_ = node->create_publisher<std_msgs::msg::Float32>("/imu/yaw", 10);

    RCLCPP_INFO(node->get_logger(), "imu_node 초기화 완료");

    initialized_ = true;
}

ImuNode::~ImuNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void ImuNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}

bool ImuNode::isInitialized() const
{
    return initialized_; // 초기화 상태 반환
}


void ImuNode::quaternionToEuler(const double qx, const double qy, const double qz, const double qw, double &roll, double &pitch, double &yaw) {
    
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    normalizeAngle(roll);
    normalizeAngle(yaw);
    
    if (pitch > 90.0) pitch = -90.0;
    if (pitch < -90.0) pitch = 90.0;
}

void ImuNode::resetAngles() {
    // yaw 오프셋만 현재 값으로 설정
    yaw_offset = yaw_kf.getState();
    
    // yaw 관련 필터만 초기화
    yaw_kf = KalmanFilter();
}


void ImuNode::imuCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::vector<double> values;
    std::string data = msg->data.substr(1, msg->data.length() - 2);
    std::stringstream ss2(data);
    std::string token;
    
    while (std::getline(ss2, token, ',')) {
        values.push_back(std::stod(token));
    }
    
    if (values.size() >= 4) {
        processImuData(values);
    }
}

void ImuNode::processImuData(const std::vector<double>& data)
{
    double roll, pitch, yaw;
    quaternionToEuler(data[2], data[1], data[0], data[3], roll, pitch, yaw);

    // yaw에만 오프셋 적용
    yaw -= yaw_offset;
    
    // Kalman 필터 적용
    float final_roll = roll_kf.update(roll);
    float final_pitch = pitch_kf.update(pitch);
    float final_yaw = yaw_kf.update(yaw);

    auto roll_msg = std_msgs::msg::Float32();
    auto pitch_msg = std_msgs::msg::Float32();
    auto yaw_msg = std_msgs::msg::Float32();

    // 필터링된 값을 메시지에 할당
    roll_msg.data = static_cast<float>(final_roll);
    pitch_msg.data = static_cast<float>(final_pitch);
    yaw_msg.data = static_cast<float>(final_yaw);
    
    pub_roll_->publish(roll_msg);
    pub_pitch_->publish(pitch_msg);
    pub_yaw_->publish(yaw_msg);
}

void ImuNode::normalizeAngle(double &angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
}