#ifndef UDP_CONNECTION_DXL_LEFT_NODE_HPP
#define UDP_CONNECTION_DXL_LEFT_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>

class DxlLeftNode : public QThread
{
    Q_OBJECT

public:
    DxlLeftNode();
    ~DxlLeftNode();

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
};

#endif // UDP_CONNECTION_DXL_LEFT_NODE_HPP