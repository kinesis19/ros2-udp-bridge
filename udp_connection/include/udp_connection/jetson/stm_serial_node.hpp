#ifndef UDP_CONNECTION_STM_SERIAL_HPP
#define UDP_CONNECTION_STM_SERIAL_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>

class StmSerialNode : public QThread
{
    Q_OBJECT

public:
    StmSerialNode();
    ~StmSerialNode();
    bool isInitialized() const; // 초기화 상태 확인 메서드

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
    bool initialized_; // 초기화 상태 확인 변수
};

#endif // UDP_CONNECTION_STM_SERIAL_HPP