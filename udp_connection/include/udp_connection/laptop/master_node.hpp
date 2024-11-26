#ifndef UDP_CONNECTION_MASTER_NODE_HPP
#define UDP_CONNECTION_MASTER_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

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

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_yellow_line_x_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_white_line_x_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_imu_yaw_;

    bool initialized_; // 초기화 상태 확인 변수
    bool isRobotRun_; // 로봇의 동작 여부를 나타내는 변수(토글로 사용)

    int stage_number_; // 현재 스테이지 나타내는 변수

    // 선 감지 확인 변수
    bool isDetectYellowLine;
    bool isDetectWhiteLine;

    // 감지된 선의 x 좌표 변수
    float yellow_line_x_;
    float white_line_x_;

    // linear, angular 변수(UI -> msg)
    int linear_vel_;
    int angular_vel_;

    // IMU의 yaw 데이터 변수
    float imu_yaw_;

    // ========== [Line Detect 메서드] ==========
    void detectYellowLine(const std_msgs::msg::Bool::SharedPtr msg);
    void detectWhiteLine(const std_msgs::msg::Bool::SharedPtr msg);
    void getYellowLineX(const std_msgs::msg::Float32::SharedPtr msg);
    void getWhiteLineX(const std_msgs::msg::Float32::SharedPtr msg);

    // ========== [IMU 메서드] ==========
    void getImuYaw(const std_msgs::msg::Float32::SharedPtr msg);

    // ========== [Dxl Control 메서드] ==========
    void ctlDxlFront(int linearVel, int angularVel);
    void ctlDxlLeft(int linearVel, int angularVel);
    void ctlDxlRight(int linearVel, int angularVel);
    void ctlDxlBack(int linearVel, int angularVel);

    // ========== [스테이지별 이동 처리 메서드] ==========
    void runRobotStage1(); // 스테이지1 일때의 이동처리 로직

};

#endif // UDP_CONNECTION_MASTER_NODE_HPP