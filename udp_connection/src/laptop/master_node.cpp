#include "../include/udp_connection/laptop/master_node.hpp"

MasterNode::MasterNode() : isDetectYellowLine(false), isDetectWhiteLine(false)
{
    node = rclcpp::Node::make_shared("master_node");

    RCLCPP_INFO(node->get_logger(), "master_node 초기화 완료");

    initialized_ = true;

    // ========== [Line Detect 서브스크라이버] ==========
    sub_yellow_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/yellow_line_detected", 10, std::bind(&MasterNode::detectYellowLine, this, std::placeholders::_1));
    sub_white_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/white_line_detected", 10, std::bind(&MasterNode::detectWhiteLine, this, std::placeholders::_1));

    // ========== [Dynamixel 퍼블리셔 생성] ==========
    pub_dxl_linear_vel_ = node->create_publisher<std_msgs::msg::Int32>("/stm32/dxl_linear_vel", 10);
    pub_dxl_angular_vel_ = node->create_publisher<std_msgs::msg::Int32>("/stm32/dxl_linear_vel", 10);


    ctlDxlLeft();
}

MasterNode::~MasterNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void MasterNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}

bool MasterNode::isInitialized() const
{
    return initialized_; // 초기화 상태 반환
}

// ========== [Line Detect 서브스크라이브] ==========
void MasterNode::detectYellowLine(const std_msgs::msg::Bool::SharedPtr msg) {
    isDetectYellowLine = msg->data;
    if (isDetectYellowLine) {
        isDetectYellowLine = true;
        RCLCPP_INFO(node->get_logger(), "노란색 라인 감지");
    } else {
        isDetectYellowLine = false;
    }
}

void MasterNode::detectWhiteLine(const std_msgs::msg::Bool::SharedPtr msg) {
    isDetectWhiteLine = msg->data;
    if (isDetectWhiteLine) {
        isDetectWhiteLine = true;
        RCLCPP_INFO(node->get_logger(), "흰색 라인 감지");
    } else {
        isDetectWhiteLine = false;
    }
}

// ========== [Dxl Control 메서드] ==========
void MasterNode::ctlDxlLeft() {
    auto msg_linear_ = std_msgs::msg::Int32();
    auto msg_angular_ = std_msgs::msg::Int32();

    msg_linear_.data = 0;
    msg_angular_.data = 10;

    pub_dxl_linear_vel_->publish(msg_linear_); // 퍼블리시
    pub_dxl_angular_vel_->publish(msg_angular_);
    RCLCPP_INFO(node->get_logger(), "Published to /stm32/dxl_linear_vel: %d", msg_linear_.data);
}


void MasterNode::ctlDxlRight() {

    auto msg_linear_ = std_msgs::msg::Int32();
    auto msg_angular_ = std_msgs::msg::Int32();

    msg_linear_.data = 0;
    msg_angular_.data = -10;

    pub_dxl_linear_vel_->publish(msg_linear_); // 퍼블리시
    pub_dxl_angular_vel_->publish(msg_angular_);
    RCLCPP_INFO(node->get_logger(), "Published to /stm32/dxl_linear_vel: %d", msg_linear_.data);

}