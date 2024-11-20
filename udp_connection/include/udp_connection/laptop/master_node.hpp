#ifndef UDP_CONNECTION_MASTER_NODE_HPP
#define UDP_CONNECTION_MASTER_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>

class MasterNode : public QThread
{
    Q_OBJECT

public:
    MasterNode();
    ~MasterNode();

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
};

#endif // UDP_CONNECTION_MASTER_NODE_HPP