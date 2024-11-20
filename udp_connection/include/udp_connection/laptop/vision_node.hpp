#ifndef UDP_CONNECTION_VISION_HPP
#define UDP_CONNECTION_VISION_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>

class VisionNode : public QThread
{
    Q_OBJECT

public:
    VisionNode();
    ~VisionNode();

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
};

#endif // UDP_CONNECTION_VISION_HPP