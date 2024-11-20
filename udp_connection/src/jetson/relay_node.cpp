#include "../include/udp_connection/jetson/relay_node.hpp"

RelayNode::RelayNode()
{
    node = rclcpp::Node::make_shared("relay_node");

    RCLCPP_INFO(node->get_logger(), "relay_node 초기화 완료");
}

RelayNode::~RelayNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void RelayNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}