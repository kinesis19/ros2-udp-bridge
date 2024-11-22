#include "../include/udp_connection/jetson/stm_serial_node.hpp"

StmSerialNode::StmSerialNode()
{
    node = rclcpp::Node::make_shared("stm_serial_node");

    RCLCPP_INFO(node->get_logger(), "stm_serial_node 초기화 완료");

    initialized_ = true;
}

StmSerialNode::~StmSerialNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void StmSerialNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}

bool StmSerialNode::isInitialized() const
{
    return initialized_; // 초기화 상태 반환
}