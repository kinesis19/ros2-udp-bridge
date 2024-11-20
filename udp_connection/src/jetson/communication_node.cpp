#include "../include/udp_connection/jetson/communication_node.hpp"

CommunicationNode::CommunicationNode()
{
    node = rclcpp::Node::make_shared("communication_node");

    RCLCPP_INFO(node->get_logger(), "communication_node 초기화 완료");
}

CommunicationNode::~CommunicationNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void CommunicationNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}