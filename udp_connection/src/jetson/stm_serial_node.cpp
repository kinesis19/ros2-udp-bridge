#include "../include/udp_connection/jetson/stm_serial_node.hpp"

StmSerialNode::StmSerialNode() {
    node = rclcpp::Node::make_shared("stm_serial_node");

    RCLCPP_INFO(node->get_logger(), "stm_serial_node 초기화 완료");

    initialized_ = true;

    // 시리얼 포트 설정
    try {
        serial_.setPort("/dev/ttyUSB0");
        serial_.setBaudrate(115200);  // 보드레이트 설정
        serial_.open();
    } catch (serial::IOException &e) {
        RCLCPP_ERROR(node->get_logger(), "Unable to open port: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    if (serial_.isOpen()) {
        RCLCPP_INFO(node->get_logger(), "Serial port opened successfully");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to open serial port");
        rclcpp::shutdown();
        return;
    }

    // 구독자 및 퍼블리셔 생성
    sub_ = node->create_subscription<std_msgs::msg::String>("serial_node/input", 10, std::bind(&StmSerialNode::writeCallback, this, std::placeholders::_1));
    pub_ = node->create_publisher<std_msgs::msg::String>("serial_node/output", 10);
    pub_adc1_ = node->create_publisher<std_msgs::msg::Int32>("adc_value_right", 10);
    pub_adc2_ = node->create_publisher<std_msgs::msg::Int32>("adc_value_front", 10);
    pub_adc3_ = node->create_publisher<std_msgs::msg::Int32>("adc_value_left", 10);

    linear_vel_sub_ = node->create_subscription<std_msgs::msg::Int32>("linear_vel", 10, std::bind(&StmSerialNode::linearVelCallback, this, std::placeholders::_1));
    angular_vel_sub_ = node->create_subscription<std_msgs::msg::Int32>("angular_vel", 10, std::bind(&StmSerialNode::angularVelCallback, this, std::placeholders::_1));

    // 타이머 설정
    read_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StmSerialNode::readCallback, this));
}

StmSerialNode::~StmSerialNode() {
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void StmSerialNode::run() {
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}

bool StmSerialNode::isInitialized() const {
    return initialized_; // 초기화 상태 반환
}

// ========== [메시지 수신 콜백: 수신한 데이터를 시리얼 포트를 통해 전송] ==========

void StmSerialNode::writeCallback(const std_msgs::msg::String::SharedPtr msg) {
    serial_.write(msg->data);
}

void StmSerialNode::linearVelCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    linear_vel_ = msg->data;
    sendVelocityData();
}

void StmSerialNode::angularVelCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    angular_vel_ = msg->data;
    sendVelocityData();
}

void StmSerialNode::sendVelocityData() {
    uint8_t packet[10];

    packet[0] = 0x19;

    packet[1] = (linear_vel_ >> 24) & 0xFF; // 상위 8비트
    packet[2] = (linear_vel_ >> 16) & 0xFF;
    packet[3] = (linear_vel_ >> 8) & 0xFF;
    packet[4] = linear_vel_ & 0xFF;         // 하위 8비트

    packet[5] = (angular_vel_ >> 24) & 0xFF; // 상위 8비트
    packet[6] = (angular_vel_ >> 16) & 0xFF;
    packet[7] = (angular_vel_ >> 8) & 0xFF;
    packet[8] = angular_vel_ & 0xFF;         // 하위 8비트

    packet[9] = 0x03;

    serial_.write(packet, sizeof(packet));

    RCLCPP_INFO(node->get_logger(), "Sent packet: linear_vel: %d, angular_vel: %d", linear_vel_, angular_vel_);
}

void StmSerialNode::readCallback() {
    try {
        size_t available_bytes = serial_.available();

        if (available_bytes >= 14) {
            std::vector<uint8_t> buffer(available_bytes);
            serial_.read(buffer.data(), buffer.size());

            if(buffer[0] == 0x08 && buffer[13] == 0x20) {
                int32_t adc_value_1 = (buffer[1] << 24) | (buffer[2] << 16) | (buffer[3] << 8) | buffer[4];
                int32_t adc_value_2 = (buffer[5] << 24) | (buffer[6] << 16) | (buffer[7] << 8) | buffer[8];
                int32_t adc_value_3 = (buffer[9] << 24) | (buffer[10] << 16) | (buffer[11] << 8) | buffer[12];

                std_msgs::msg::Int32 msg1;
                std_msgs::msg::Int32 msg2;
                std_msgs::msg::Int32 msg3;

                msg1.data = adc_value_1;
                msg2.data = adc_value_2;
                msg3.data = adc_value_3;

                pub_adc1_->publish(msg1);
                pub_adc2_->publish(msg2);
                pub_adc3_->publish(msg3);
            }
        }
    } catch (serial::IOException &e) {
        RCLCPP_ERROR(node->get_logger(), "IOException: %s", e.what());
        serial_.close();
        rclcpp::shutdown();
    }
}