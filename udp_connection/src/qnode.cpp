#include "../include/udp_connection/qnode.hpp"

QNode::QNode(const std::string& nodeName)
{
  node = rclcpp::Node::make_shared(nodeName);
  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  // rclcpp::shutdown();
  Q_EMIT rosShutDown();
}
