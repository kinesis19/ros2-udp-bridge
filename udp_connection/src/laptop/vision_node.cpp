#include "../include/udp_connection/laptop/vision_node.hpp"

VisionNode::VisionNode()
{
    node = rclcpp::Node::make_shared("vision_node");

    RCLCPP_INFO(node->get_logger(), "vision_node 초기화 완료");
}

VisionNode::~VisionNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void VisionNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}