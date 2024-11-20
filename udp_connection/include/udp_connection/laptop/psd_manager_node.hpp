#ifndef UDP_CONNECTION_PSD_MANAGER_NODE_HPP
#define UDP_CONNECTION_PSD_MANAGER_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>

class PsdManagerNode : public QThread
{
    Q_OBJECT

public:
    PsdManagerNode();
    ~PsdManagerNode();

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
};

#endif // UDP_CONNECTION_PSD_MANAGER_NODE_HPP