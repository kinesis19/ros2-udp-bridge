#include "../include/udp_connection/laptop/psd_manager_node.hpp"

PsdManagerNode::PsdManagerNode()
{
    node = rclcpp::Node::make_shared("psd_manager_node");

    RCLCPP_INFO(node->get_logger(), "psd_manager_node 초기화 완료");
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