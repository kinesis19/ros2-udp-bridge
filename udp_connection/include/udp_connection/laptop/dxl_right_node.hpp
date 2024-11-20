#ifndef UDP_CONNECTION_DXL_RIGHT_NODE_HPP
#define UDP_CONNECTION_DXL_RIGHT_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>

class DxlRightNode : public QThread
{
    Q_OBJECT

public:
    DxlRightNode();
    ~DxlRightNode();

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
};

#endif // UDP_CONNECTION_DXL_RIGHT_NODE_HPP