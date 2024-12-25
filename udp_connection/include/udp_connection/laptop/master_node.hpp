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

    int stage_number_ = 0; // 현재 스테이지 나타내는 변수

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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_straight_blue_sign_detected; // 삼거리 탈출 표지판 감지

    // ========== [차단바 감지 서브스크라이브] ==========
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_barrier_detected_;

    // ========== [Line Detect  중앙값 서브스크라이브] ==========
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_yellow_line_center_dist_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_white_line_center_dist_;

    // ========== [Red Line Detect 서브스크라이브] ==========
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_red_line_detected_;

    bool initialized_; // 초기화 상태 확인 변수
    bool isRobotRun_ = false; // 로봇의 동작 여부를 나타내는 변수(토글로 사용)

    // 이미지 관련 변수
    int image_center_point_x_ = 160; // 현재 Qt에서 이미지 Scale은 320 * 240

    // 선 감지 확인 변수
    bool isDetectYellowLine = false;
    bool isDetectWhiteLine = false;
    bool isDetectRedLine = false;

    // 주차 표지판 감지 확인 변수
    bool isDetectBlueSign = false;
    bool isDetectLeftBlueSign = false;
    bool isDetectStraightBlueSign = false;

    // 차단바 감지 확인 변수
    bool isDetectBarrier = false;

    // 감지된 선의 x 좌표 변수
    float yellow_line_x_ = 0.0;
    float white_line_x_ = 0.0;

    // linear, angular 변수(UI -> msg)
    float linear_vel_ = 0.0;
    float angular_vel_ = 0.0;

    // IMU의 yaw 데이터 변수
    float imu_yaw_ = 0.0;

    // PSD 값 저장 변수
    int psd_adc_left_ = 0;
    int psd_adc_front_ = 0;
    int psd_adc_right_ = 0;

    // 흰 선의 좌표 저장 벡터 변수
    // std::vector<float> white_line_points_(8, 0.0f);
    // std::vector<float> yellow_line_points_(8, 0.0f);
    std::vector<float> white_line_points_ = std::vector<float>(8, 0.0f);  // 등호 초기화
    std::vector<float> yellow_line_points_ = std::vector<float>(8, 0.0f);


    // 선의 각도(angle) 저장 변수
    float white_line_angle_ = 0.0;
    float yellow_line_angle_ = 0.0;

    // PID Control 변수    
    const float kP = 0.005f;
    const float kI = 0.0f;
    const float kD = 0.0f;
    const float yaw_ok = 0.5f;
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
    bool isDetectObject1andObject2 = false; // 스테이지2 진입 후, 오브젝트 1과 2 동시에 감지를 나타내는 변수.
    bool isWorkedPIDControlToTurnRightStage2 = false; // 스테이지 2 진입후, 노란색 선 감지 후 PID 제어를 나타내는 변수.
    bool isMissYellowLineStage2 = false;

    int nowModeStage2 = 0; // 스테이지2일 때 모드 판단
    bool isOkayPidControlLeftStage2 = false; // Stage2에서 처음 PID 제어를 위한 플래그 변수
    bool isTurnRightStage2 = false; // Stage2에서 오른쪽 회전했을 때 나타내는 변수
    bool isDetectWhite1Stage2 = false;
    bool isDetectYellowLine1Stage2 = false;
    int areaNumberStage2 = 0; // Stage2 내에서 세부적인 area 넘버링을 나타내는 변수

    bool isTurnLeftMode1Stage2 = false; // Stage2에서, mode1일 때 왼쪽 회전 여부를 나타내는 플래그 변수
    bool isDetectYellowLineMode1Stage2 = false; // Stage2에서, mode1일 때 노란색 라인 감지 여부를 나타내는 플래그
    bool isDetectWhiteLineMode1Stage2 = false; // Stage2에서, mode1일 때, 탈출을 위한 흰색 라인을 감지 여부를 나타내는 플래그 
    
    // ========== [Stage3 감지 플래그 변수] ==========
    bool isMissBlueSignStage3 = false; // Stage3 진입 후, 파란색 표지판 잊어버렸을 때를 위한 플래그
    bool isDetectYellowLineinThreeStreetStage3 = false; // Stage3 진입 후, 삼거리에서 노란색을 재 감지 여부를 나타내는 플래그
    bool isStartPidTurnLeftThreeStreetStage3 = false; // Stage3 진입 후, 삼거리에서 왼쪽 PID 제어 감지 여부를 나태내는 플래그
    bool isDetectWhiteDottedLineStage3 = false; // Stage3 진입 후, 흰 색 점선 감지 여부를 나타내는 플래그
    bool isDetectYellowLineAfterDetectWhiteDottedLineStage3 = false; // Stage3 진입 후, 흰 색 점선 감지 이후 노란색 선 감지 여부를 나타내는 플래그
    bool isDonePidControlParkingStationInStage3 = false; // 주차장에서 오브젝트가 없는 방향으로 이동하기 위한 PID 제어 진입 완료 여부를 나타내는 플래그
    bool isDonePidControlParkingStationOutStage3 = false; // 주차장에서 오브젝트가 없는 방향으로 이동하기 위한 PID 제어 아웃 완료 여부를 나타내는 플래그
    bool isReadyToParking = false; // 주차장에서 오브젝트가 있는 방향으로 바라보고, 주차 준비가 된 상태일 때 여부를 나타내는 플래그
    int detectObjectNumParkingStationStage3 = 0; // 주차장에서 오브젝트가 어느 위치에 위치해 있는지 저장하는 변수 (0: null, 1: 왼쪽, 2: 오른쪽)
    bool isTurnLeftToGoToStage4 = false; // 삼거리에서 Stage4로 가기 위해 좌회전 하고 있는지 여부를 나타내는 플래그
    bool isRunWithYellowLineParkingStationOutStage3 = false; // 주차 이후, 나갈 때

    // ========== [Stage4 감지 플래그 변수] ==========
    bool isDetectBarrierStage4 = false; // isDetectBarrier가 너무 빠르게 변해서 만든 플래그

    bool tempIsEnableTurnStage4 = false;

    // ========== [Stage5 감지 플래그 변수] ==========
    bool isDetectBarrierStage5 = false; // isDetectBarrier가 너무 빠르게 변해서 만든 플래그
    bool isDetectEndLineStage5 = false; // isDetectRedLine이 너무 빠르게 변해서 만든 플래그
    bool isDonePidControlEndLineStage5 = false; // End Line에서 PID 제어 180도 회전 했는지 여부를 나타내는 플래그

    // ========== [Stage6 감지 플래그 변수] ==========
    bool isDetectBarrierStage6 = false; // isDetectBarrier가 너무 빠르게 변해서 만든 플래그

    // ========== [Stage7 감지 플래그 변수] ==========
    bool isDetectBarrierStage7 = false;


    // ========== [Stage9 감지 플래그 변수] ==========
    bool isOkayResetIMUStage9 = false;
    bool isDetectObject1andObject2Stage9 = false; // 스테이지9 진입 후, 오브젝트 3과 ar2 동시에 감지를 나타내는 변수.
    bool isWorkedPIDControlToTurnRightStage9 = false;
    bool isTempDoneTurnLeftRangeStage9 = false; // 스테이지9 진입 후, PID 제어 완료 했는지 여부를 나타내는 플래그

    bool isReadyToPIDControlToTurnLeftStage9 = false;


    int nowModeStage9 = 0; // 스테이지9일 때 모드 판단
    bool isOkayPidControlRightStage9 = false;
    bool isTurnLeftStage9 = false;
    bool isDetectWhite1Stage9 = false;
    bool isDetectYellowLine1Stage9 = false;
    bool isTempDoneTurnRightRangeStage9 = false; // 스테이지9 진입 후, PID 제어 완료 했는지 여부를 나타내는 플래그
    float past_imu_yaw_ = 0.0;

    bool isTurnRightMode1Stage9 = false; // 스테이지9 진입 후, mode1일 때 오른쪽 회전 했는지 여부를 나타내는 플래그
    bool isDetectYellowLineMode1Stage9 = false; // 스테이지9 진입 후, mode1일 때 노란색 감지 여부를 나타내는 플래그
    bool isDetectWhiteLine1Mode1Stage9 = false;

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
    void detectStraightBlueSign(const std_msgs::msg::Bool::SharedPtr msg);

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

    void runRobotStage6(); // reverse stage5
    void runRobotStage7(); // reverse stage4
    void runRobotStage8(); // reverse stage3 without parking
    void runRobotStage9(); // reverse stage2
    void runRobotStage10(); // reverse stage1


};

#endif // UDP_CONNECTION_MASTER_NODE_HPP