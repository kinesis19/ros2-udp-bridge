#ifndef UDP_CONNECTION_PSD_MANAGER_NODE_HPP
#define UDP_CONNECTION_PSD_MANAGER_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

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

    // ========== [Psd-Adc-Value 서브스크라이브] ==========
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_stm32_psd_adc_right_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_stm32_psd_adc_front_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_stm32_psd_adc_left_;

    bool initialized_; // 초기화 상태 확인 변수
    
    // ========== [STM32 PSD Value Callback Method] ==========
    void psdRightCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void psdFrontCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void psdLeftCallback(const std_msgs::msg::Int32::SharedPtr msg);
};

#endif // UDP_CONNECTION_PSD_MANAGER_NODE_HPP