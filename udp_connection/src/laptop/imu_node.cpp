#include "../include/udp_connection/laptop/imu_node.hpp"

float KalmanFilter::update(float measurement)
{
    if (first_run)
    {
        X = measurement;
        P = 1.0; // 초기 불확실성 설정
        first_run = false;
        return X;
    }
    P = P + Q;

    // Kalman 게인 계산
    K = P / (P + R);

    // 측정값과 예측값의 차이가 너무 크면 이를 제한
    float innovation = measurement - X;
    if (std::abs(innovation) > 180.0)
    {
        if (innovation > 0)
        {
            innovation -= 360.0;
        }
        else
        {
            innovation += 360.0;
        }
    }

    // 상태 업데이트
    X = X + K * innovation;

    // 불확실성 업데이트
    P = (1 - K) * P;

    // 각도 정규화
    while (X > 180.0)
        X -= 360.0;
    while (X < -180.0)
        X += 360.0;

    return X;
}

ImuNode::ImuNode()
{
    node = rclcpp::Node::make_shared("imu_node");

    sub_imu_ = node->create_subscription<std_msgs::msg::String>("/ebimu_data", 10, std::bind(&ImuNode::imuCallback, this, std::placeholders::_1));
    pub_roll_ = node->create_publisher<std_msgs::msg::Float32>("/imu/roll", 10);
    pub_pitch_ = node->create_publisher<std_msgs::msg::Float32>("/imu/pitch", 10);
    pub_yaw_ = node->create_publisher<std_msgs::msg::Float32>("/imu/yaw", 10);

    reset_service_ = node->create_service<std_srvs::srv::Trigger>("reset_imu", std::bind(&ImuNode::handleResetSignal, this, std::placeholders::_1, std::placeholders::_2));

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
    // 기존의 변환 코드
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

    // 라디안에서 도로 변환
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    // 각도 정규화 및 제한
    normalizeAngle(roll);
    normalizeAngle(pitch);
    normalizeAngle(yaw);

    // pitch 제한
    if (pitch > 90.0)
        pitch = 90.0;
    if (pitch < -90.0)
        pitch = -90.0;
}

void ImuNode::resetAngles() {
    // yaw 오프셋만 현재 값으로 설정
    yaw_offset = yaw_kf.getState();
    roll_kf = KalmanFilter();
    pitch_kf = KalmanFilter();
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

    // Kalman 필터 적용 (offset 적용 전)
    float final_roll = roll_kf.update(roll);
    float final_pitch = pitch_kf.update(pitch);
    int final_yaw = yaw_kf.update(yaw);

    // yaw offset을 필터링된 값에 적용
    final_yaw -= yaw_offset;

    // 각도 정규화 (offset 적용 후)
    while (final_yaw > 180.0)
        final_yaw -= 360.0;
    while (final_yaw < -180.0)
        final_yaw += 360.0;

    auto roll_msg = std_msgs::msg::Float32();
    auto pitch_msg = std_msgs::msg::Float32();
    auto yaw_msg = std_msgs::msg::Float32();

    roll_msg.data = final_roll;
    pitch_msg.data = final_pitch;
    yaw_msg.data = final_yaw;

    std_msgs::msg::Float32 final_msg;
    final_msg.data = final_yaw;
    pub_yaw_->publish(final_msg);

    // 디버깅을 위한 로그 추가
    RCLCPP_INFO(node->get_logger(), "Raw Yaw: %.2f, Filtered: %.2f, Offset: %.2f, Final: %d",
                yaw, yaw_kf.getState(), yaw_offset, final_yaw);
}

void ImuNode::normalizeAngle(double &angle)
{
    while (angle > 180.0)
        angle -= 360.0;
    while (angle < -180.0)
        angle += 360.0;
}

void ImuNode::handleResetSignal(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    resetAngles();
    response->success = true;
    response->message = "IMU angles have been reset";
    RCLCPP_INFO(node->get_logger(), "IMU angles reset triggered");
}