#ifndef UDP_CONNECTION_STM_SERIAL_HPP
#define UDP_CONNECTION_STM_SERIAL_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

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

    serial::Serial serial_;  // 시리얼 포트 객체
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_stm32_psd_adc_right_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_stm32_psd_adc_front_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_stm32_psd_adc_left_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_stm32_dxl_linear_vel_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_stm32_dxl_angular_vel_;
    int32_t linear_vel_ = 0;
    int32_t angular_vel_ = 0;
    rclcpp::TimerBase::SharedPtr read_timer_;


    bool initialized_; // 초기화 상태 확인 변수

    void writeCallback(const std_msgs::msg::String::SharedPtr msg);
    void linearVelCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void angularVelCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void sendVelocityData();
    void readCallback();
};

#endif // UDP_CONNECTION_STM_SERIAL_HPP