#include "../include/udp_connection/laptop/master_node.hpp"

MasterNode::MasterNode()
{
    node = rclcpp::Node::make_shared("master_node");

    RCLCPP_INFO(node->get_logger(), "master_node 초기화 완료");
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