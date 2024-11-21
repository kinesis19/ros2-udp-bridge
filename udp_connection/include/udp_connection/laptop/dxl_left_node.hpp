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
    bool isInitialized() const; // 초기화 상태 확인 메서드

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
    bool initialized_; // 초기화 상태 확인 변수
};

#endif // UDP_CONNECTION_DXL_LEFT_NODE_HPP