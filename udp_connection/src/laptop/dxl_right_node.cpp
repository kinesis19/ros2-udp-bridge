#include "../include/udp_connection/laptop/dxl_right_node.hpp"

DxlRightNode::DxlRightNode()
{
    node = rclcpp::Node::make_shared("dxl_right_node");

    RCLCPP_INFO(node->get_logger(), "dxl_right_node 초기화 완료");
}

DxlRightNode::~DxlRightNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void DxlRightNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}