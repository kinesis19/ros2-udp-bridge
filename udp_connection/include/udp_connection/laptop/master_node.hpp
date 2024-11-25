#ifndef UDP_CONNECTION_MASTER_NODE_HPP
#define UDP_CONNECTION_MASTER_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

class MasterNode : public QThread
{
    Q_OBJECT

public:
    MasterNode();
    ~MasterNode();
    bool isInitialized() const; // 초기화 상태 확인 메서드
    void stopDxl();
    void updateDxlData(int linearVel, int angularVel); // Dxl 데이터를 UI로 입력 받아 제어하기
    void runDxl(int linearVel, int angularVel); // Dxl 원격 제어(버튼)

private slots:
    // void onStopButtonClicked();

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_yellow_detected_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_white_detected_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_dxl_linear_vel_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_dxl_angular_vel_;

    bool initialized_; // 초기화 상태 확인 변수
    bool isRobotRun_; // 로봇의 동작 여부를 나타내는 변수(토글로 사용)

    // 선 감지 확인 변수
    bool isDetectYellowLine;
    bool isDetectWhiteLine;

    // linear, angular 변수(UI -> msg)
    int linear_vel_;
    int angular_vel_;

    // ========== [Line Detect 메서드] ==========
    void detectYellowLine(const std_msgs::msg::Bool::SharedPtr msg);
    void detectWhiteLine(const std_msgs::msg::Bool::SharedPtr msg);

    // ========== [Dxl Control 메서드] ==========
    void ctlDxlFront();
    void ctlDxlLeft();
    void ctlDxlRight();
    void ctlDxlBack();


};

#endif // UDP_CONNECTION_MASTER_NODE_HPP