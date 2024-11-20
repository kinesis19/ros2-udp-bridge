#ifndef UDP_CONNECTION_RELAY_NODE_HPP
#define UDP_CONNECTION_RELAY_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class RelayNode : public QThread
{
    Q_OBJECT

public:
    RelayNode();
    ~RelayNode();

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
};

#endif // UDP_CONNECTION_RELAY_NODE_HPP