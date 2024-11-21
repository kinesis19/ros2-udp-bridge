#include "../include/udp_connection/laptop/imu_node.hpp"

ImuNode::ImuNode()
{
    node = rclcpp::Node::make_shared("imu_node");

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