#ifndef UDP_CONNECTION_MASTER_NODE_HPP
#define UDP_CONNECTION_MASTER_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>
#include <chrono>

#define GAIN_LINEAR 0.001
#define GAIN_CORNER 0.002

class MasterNode : public QThread
{
    Q_OBJECT

public:

    int stage_number_; // 현재 스테이지 나타내는 변수

    MasterNode();
    ~MasterNode();
    bool isInitialized() const; // 초기화 상태 확인 메서드
    void stopDxl();
    void updateDxlData(float linearVel, float angularVel); // Dxl 데이터를 UI로 입력 받아 제어하기
    void runDxl(float linearVel, float angularVel); // Dxl 원격 제어(버튼)

    void responePidTest();

    void resetValue();

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

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_dxl_linear_vel_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_dxl_angular_vel_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_yellow_line_x_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_white_line_x_;

    // ========== [IMU 서브스크라이브 & 퍼블리시] ==========
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_imu_yaw_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_imu_reset_;


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

    // ========== [주차 표지판 감지 메서드] ==========
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_blue_sign_detected_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_left_blue_sign_detected_; // 삼거리 탈출 표지판 감지

    // ========== [차단바 감지 서브스크라이브] ==========
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_barrier_detected_;

    // ========== [Line Detect  중앙값 서브스크라이브] ==========
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_yellow_line_center_dist_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_white_line_center_dist_;

    // ========== [Red Line Detect 서브스크라이브] ==========
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_red_line_detected_;

    bool initialized_; // 초기화 상태 확인 변수
    bool isRobotRun_; // 로봇의 동작 여부를 나타내는 변수(토글로 사용)

    // 이미지 관련 변수
    int image_center_point_x_ = 160; // 현재 Qt에서 이미지 Scale은 320 * 240

    // 선 감지 확인 변수
    bool isDetectYellowLine;
    bool isDetectWhiteLine;
    bool isDetectRedLine = false;

    // 주차 표지판 감지 확인 변수
    bool isDetectBlueSign = false;
    bool isDetectLeftBlueSign = false;

    // 차단바 감지 확인 변수
    bool isDetectBarrier;

    // 감지된 선의 x 좌표 변수
    float yellow_line_x_;
    float white_line_x_;

    // linear, angular 변수(UI -> msg)
    float linear_vel_;
    float angular_vel_;

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
    const float kP = 0.005f;
    const float kI = 0.0f;
    const float kD = 0.0f;
    const float yaw_ok = 10.0f;
    const float max_angular_vel = 0.4f;
    const float min_angular_vel = 0.05f;
    bool playYawFlag = false;
    float intergral = 0.0f;
    float angular_vel_pid = 0.0f;
    float target_yaw_ = 0.0f;
    float yaw_error = 0.0f;
    float pre_yaw_error = 0.0f;

    // Line Detect Center 변수
    float dist_yellow_line_ = 0.0;
    float dist_white_line_ = 0.0;
    float pixel_gap = 0.0; // 중앙선 기준 오차

    // ========== [Stage1 감지 플래그 변수] ==========
    bool isDetectObject1Stage1 = false; // 스테이지2 진입 전, 오브젝트1과 흰색 코너 감지를 나타내는 변수.

    // ========== [Stage2 감지 플래그 변수] ==========
    // 사용 안 하는 플래그 변수들
    bool isDetectSecondObjectStage2; // 오브젝트2 정면 감지 변수
    bool isDetectThirdObjectStage2; // 오브젝트3 정면 감지 변수
    bool isPassSecondObjectStage2; // 오브젝트2 통과 감지 변수
    bool isDetectYellowLineStage2; // 오브젝트2 통과 이후, 직진을 위한 Yellow Line 감지 변수
    bool isDetectWhiteLineStage2; // 오브젝트2 통과 이후, 라인트레이싱을 위한 White Line 감지 변수
    bool isStartPidRightTurnStage2; // 오브젝트2 통과 이후, 노란색 선 감지하고 정해진 각도만큼 최초 1회만 PID 제어를 감지하는 변수
    bool isDetectBlueSignStage2 = false; // 오브젝트3 통과 이후, 파란 표지판 감지 변수

    bool isDetectObject1andObject2 = false; // 스테이지2 진입 후, 오브젝트 1과 2 동시에 감지를 나타내는 변수.
    bool isWorkedPIDControlToTurnRightStage2 = false; // 스테이지 2 진입후, 노란색 선 감지 후 PID 제어를 나타내는 변수.
    
    // ========== [Stage3 감지 플래그 변수] ==========
    bool isMissYellowLineStage3 = false;
    bool isDetectBlueSignStage3 = false; // 삼거리에서 표지판 감지 플래그 -> 삼거리 PID 처리를 위한 변수
    bool isReadyPidControlThreeWayStreetInStage3 = false; // 삼거리 입장 플래그
    bool isDonePidControlThreeWayStreetInStage3 = false; // 삼거리 입장 후 PID 제어 완료시 플래그

    bool isMissBlueSignStage3 = false; // Stage3 진입 후, 파란색 표지판 잊어버렸을 때를 위한 플래그
    bool isDetectYellowLineinThreeStreetStage3 = false; // Stage3 진입 후, 삼거리에서 노란색을 재 감지 여부를 나타내는 플래그
    bool isStartPidTurnLeftThreeStreetStage3 = false; // Stage3 진입 후, 삼거리에서 왼쪽 PID 제어 감지 여부를 나태내는 플래그
    bool isDetectWhiteDottedLineStage3 = false; // Stage3 진입 후, 흰 색 점선 감지 여부를 나타내는 플래그
    bool isDetectYellowLineAfterDetectWhiteDottedLineStage3 = false; // Stage3 진입 후, 흰 색 점선 감지 이후 노란색 선 감지 여부를 나타내는 플래그
    bool isDonePidControlParkingStationInStage3 = false; // 주차장에서 오브젝트가 없는 방향으로 이동하기 위한 PID 제어 진입 완료 여부를 나타내는 플래그
    bool isDonePidControlParkingStationOutStage3 = false; // 주차장에서 오브젝트가 없는 방향으로 이동하기 위한 PID 제어 아웃 완료 여부를 나타내는 플래그
    bool isReadyToParking = false; // 주차장에서 오브젝트가 있는 방향으로 바라보고, 주차 준비가 된 상태일 때 여부를 나타내는 플래그
    int detectObjectNumParkingStationStage3 = 0; // 주차장에서 오브젝트가 어느 위치에 위치해 있는지 저장하는 변수 (0: null, 1: 왼쪽, 2: 오른쪽)
    bool isReadyToTurnLeftStage3 = false; // Stage3 코너에서 Stage4로 넘어가기 위해 필요한 좌회전 조건 플래그
    bool isReadyToGoStage4 = false; // 삼거리 탈출할 때 파란 표지판 감지하고, Stage4로 넘어가는 여부를 나타내는 플래그

    // ========== [Stage4 감지 플래그 변수] ==========
    bool isDetectBarrierStage4 = false; // isDetectBarrier가 너무 빠르게 변해서 만든 플래그

    // ========== [Stage5 감지 플래그 변수] ==========
    bool isDetectBarrierStage5 = false; // isDetectBarrier가 너무 빠르게 변해서 만든 플래그
    bool isDetectEndLineStage5 = false; // isDetectRedLine이 너무 빠르게 변해서 만든 플래그

    // ========== [Line Detect 메서드] ==========
    void detectYellowLine(const std_msgs::msg::Bool::SharedPtr msg);
    void detectWhiteLine(const std_msgs::msg::Bool::SharedPtr msg);
    void getYellowLineX(const std_msgs::msg::Float32::SharedPtr msg);
    void getWhiteLineX(const std_msgs::msg::Float32::SharedPtr msg);

    void getWhiteLinePoints(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void getYellowLinePoints(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void getWhiteLineAngle(const std_msgs::msg::Float32::SharedPtr msg);
    void getYellowLineAngle(const std_msgs::msg::Float32::SharedPtr msg);

    void detectRedLine(const std_msgs::msg::Bool::SharedPtr msg);

    // ========== [STM32 PSD Value Callback Method] ==========
    void psdRightCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void psdFrontCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void psdLeftCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // ========== [IMU 메서드] ==========
    void getImuYaw(const std_msgs::msg::Float32::SharedPtr msg);
    void resetIMU(); // IMU reset을 publish 하기 위한 메서드

    // ========== [주차 표지판 감지 메서드] ==========
    void detectBlueSign(const std_msgs::msg::Bool::SharedPtr msg);
    void detectLeftBlueSign(const std_msgs::msg::Bool::SharedPtr msg);

    // ========== [차단바 감지 메서드] ==========
    void detectBarrier(const std_msgs::msg::Bool::SharedPtr msg);

    // ========== [Line Detect  중앙값 서브스크라이브] ==========
    void getDistYellowLineCenter(const std_msgs::msg::Float32::SharedPtr msg);
    void getDistWhiteLineCenter(const std_msgs::msg::Float32::SharedPtr msg);

    // ========== [Dxl Control 메서드] ==========
    void ctlDxlFront(float linearVel, float angularVel);
    void ctlDxlLeft(float linearVel, float angularVel);
    void ctlDxlRight(float linearVel, float angularVel);
    void ctlDxlBack(float linearVel, float angularVel);

    // ========== [스테이지별 이동 처리 메서드] ==========
    void runRobotStage1(); // 스테이지1 일때의 이동처리 로직
    void runRobotStage2();
    void runRobotStage3();
    void runRobotStage4();
    void runRobotStage5();


};

#endif // UDP_CONNECTION_MASTER_NODE_HPP