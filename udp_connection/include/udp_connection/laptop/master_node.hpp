#ifndef UDP_CONNECTION_MASTER_NODE_HPP
#define UDP_CONNECTION_MASTER_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath> 

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

    void responePidTest();


    // ========== [PID Control 메서드] ==========
    void ctlDxlYaw(float target_yaw);


signals:
    void stmPsdRightReceived(const int &psdRight);
    void stmPsdFrontReceived(const int &psdFront);
    void stmPsdLeftReceived(const int &psdLeft);
    void updateCurrentStage(const int &stageNum);
    
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


    // ========== [Psd-Adc-Value 서브스크라이브] ==========
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_stm32_psd_adc_right_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_stm32_psd_adc_front_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_stm32_psd_adc_left_;

    // ========== [흰색 선의 점들(x, y) 서브스크라이브] ==========
    // 점1: index0, index1(x, y)
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_white_line_points_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_yellow_line_points_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_yellow_angle_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_white_angle_;

    // ========== [차단바 감지 서브스크라이브] ==========
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_barrier_detected_;




    bool initialized_; // 초기화 상태 확인 변수
    bool isRobotRun_; // 로봇의 동작 여부를 나타내는 변수(토글로 사용)

    int stage_number_; // 현재 스테이지 나타내는 변수

    // 선 감지 확인 변수
    bool isDetectYellowLine;
    bool isDetectWhiteLine;

    // 차단바 감지 확인 변수
    bool isDetectBarrier;

    // 감지된 선의 x 좌표 변수
    float yellow_line_x_;
    float white_line_x_;

    // linear, angular 변수(UI -> msg)
    int linear_vel_;
    int angular_vel_;

    // IMU의 yaw 데이터 변수
    float imu_yaw_;

    // PSD 값 저장 변수
    int psd_adc_left_;
    int psd_adc_front_;
    int psd_adc_right_;

    // 흰 선의 좌표 저장 벡터 변수
    std::vector<float> white_line_points_;
    std::vector<float> yellow_line_points_;

    // 선의 각도(angle) 저장 변수
    float white_line_angle_;
    float yellow_line_angle_;

    // PID Control 변수    
    const float kP = 0.1f;
    const float kI = 0.0f;
    const float kD = 0.0f;
    const float yaw_ok = 10.0f;
    const float max_angular_vel = 5.0f;
    const float min_angular_vel = 1.0f;
    bool playYawFlag = false;
    float intergral = 0.0f;
    float angular_vel_pid = 0.0f;
    float target_yaw_ = 0.0f;
    float yaw_error = 0.0f;
    float pre_yaw_error = 0.0f;


    // ========== [Stage2 감지 플래그 변수] ==========
    bool isDetectSecondObjectStage2; // 오브젝트2 정면 감지 변수
    bool isDetectThirdObjectStage2; // 오브젝트3 정면 감지 변수
    bool isPassSecondObjectStage2; // 오브젝트2 통과 감지 변수
    bool isDetectYellowLineStage2; // 오브젝트2 통과 이후, 직진을 위한 Yellow Line 감지 변수
    bool isDetectWhiteLineStage2; // 오브젝트2 통과 이후, 라인트레이싱을 위한 White Line 감지 변수
    bool isStartPidRightTurnStage2; // 오브젝트2 통과 이후, 노란색 선 감지하고 정해진 각도만큼 최초 1회만 PID 제어를 감지하는 변수
    

    // ========== [Line Detect 메서드] ==========
    void detectYellowLine(const std_msgs::msg::Bool::SharedPtr msg);
    void detectWhiteLine(const std_msgs::msg::Bool::SharedPtr msg);
    void getYellowLineX(const std_msgs::msg::Float32::SharedPtr msg);
    void getWhiteLineX(const std_msgs::msg::Float32::SharedPtr msg);

    void getWhiteLinePoints(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void getYellowLinePoints(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void getWhiteLineAngle(const std_msgs::msg::Float32::SharedPtr msg);
    void getYellowLineAngle(const std_msgs::msg::Float32::SharedPtr msg);

    // ========== [STM32 PSD Value Callback Method] ==========
    void psdRightCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void psdFrontCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void psdLeftCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // ========== [IMU 메서드] ==========
    void getImuYaw(const std_msgs::msg::Float32::SharedPtr msg);

    // ========== [차단바 감지 메서드] ==========
    void detectBarrier(const std_msgs::msg::Bool::SharedPtr msg);

    // ========== [Dxl Control 메서드] ==========
    void ctlDxlFront(int linearVel, int angularVel);
    void ctlDxlLeft(int linearVel, int angularVel);
    void ctlDxlRight(int linearVel, int angularVel);
    void ctlDxlBack(int linearVel, int angularVel);

    // ========== [스테이지별 이동 처리 메서드] ==========
    void runRobotStage1(); // 스테이지1 일때의 이동처리 로직
    void runRobotStage2();


};

#endif // UDP_CONNECTION_MASTER_NODE_HPP