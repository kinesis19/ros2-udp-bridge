#include "../include/udp_connection/laptop/psd_manager_node.hpp"

PsdManagerNode::PsdManagerNode()
{
    node = rclcpp::Node::make_shared("psd_manager_node");

    RCLCPP_INFO(node->get_logger(), "psd_manager_node 초기화 완료");

    initialized_ = true;

    // ========== [STM32의 PSD(ADC) Value 서브스크라이브] ==========
    sub_stm32_psd_adc_right_ = node->create_subscription<std_msgs::msg::Int32>("/stm32/psd_adc_value_right", 10, std::bind(&PsdManagerNode::psdRightCallback, this, std::placeholders::_1));
    sub_stm32_psd_adc_front_ = node->create_subscription<std_msgs::msg::Int32>("/stm32/psd_adc_value_front", 10, std::bind(&PsdManagerNode::psdFrontCallback, this, std::placeholders::_1));
    sub_stm32_psd_adc_left_ = node->create_subscription<std_msgs::msg::Int32>("/stm32/psd_adc_value_left", 10, std::bind(&PsdManagerNode::psdLeftCallback, this, std::placeholders::_1));
}

PsdManagerNode::~PsdManagerNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void PsdManagerNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}

bool PsdManagerNode::isInitialized() const
{
    return initialized_; // 초기화 상태 반환
}

// ========== [STM32 PSD Value Callback Method] ==========
void PsdManagerNode::psdRightCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // RCLCPP_INFO(node->get_logger(), "PSD Right: %d", msg->data);
    emit stmPsdRightReceived(msg->data);
}

void PsdManagerNode::psdFrontCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // RCLCPP_INFO(node->get_logger(), "PSD Front: %d", msg->data);
    emit stmPsdFrontReceived(msg->data);
}

void PsdManagerNode::psdLeftCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // RCLCPP_INFO(node->get_logger(), "PSD Left: %d", msg->data);
    emit stmPsdLeftReceived(msg->data);
}