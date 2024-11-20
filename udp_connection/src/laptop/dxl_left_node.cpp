#include "../include/udp_connection/laptop/dxl_left_node.hpp"

DxlLeftNode::DxlLeftNode()
{
    node = rclcpp::Node::make_shared("dxl_left_node");

    RCLCPP_INFO(node->get_logger(), "dxl_left_node 초기화 완료");
}

DxlLeftNode::~DxlLeftNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void DxlLeftNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}