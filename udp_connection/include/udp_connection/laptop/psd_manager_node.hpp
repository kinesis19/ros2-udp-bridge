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
    bool isInitialized() const; // 초기화 상태 확인 메서드

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
    bool initialized_; // 초기화 상태 확인 변수
};

#endif // UDP_CONNECTION_PSD_MANAGER_NODE_HPP