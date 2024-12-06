#include "../include/udp_connection/laptop/master_node.hpp"

MasterNode::MasterNode() : isDetectYellowLine(false), isDetectWhiteLine(false), isRobotRun_(false), linear_vel_(0), angular_vel_(0), yellow_line_x_(0.0), white_line_x_(0.0), stage_number_(0), imu_yaw_(0.0), psd_adc_left_(0), psd_adc_front_(0), psd_adc_right_(0), white_line_points_(8, 0.0), yellow_line_points_(8, 0.0), white_line_angle_(0.0), yellow_line_angle_(0.0), isDetectBarrier(false), isDetectSecondObjectStage2(false), isPassSecondObjectStage2(false), isDetectYellowLineStage2(false), isDetectWhiteLineStage2(false)
{
    node = rclcpp::Node::make_shared("master_node");

    RCLCPP_INFO(node->get_logger(), "master_node 초기화 완료");

    initialized_ = true;

    // ========== [Line Detect 서브스크라이버] ==========
    sub_yellow_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/yellow_line_detected", 10, std::bind(&MasterNode::detectYellowLine, this, std::placeholders::_1));
    sub_white_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/white_line_detected", 10, std::bind(&MasterNode::detectWhiteLine, this, std::placeholders::_1));
    sub_yellow_line_x_ = node->create_subscription<std_msgs::msg::Float32>("/vision/yellow_line_x", 10, std::bind(&MasterNode::getYellowLineX, this, std::placeholders::_1));
    sub_white_line_x_ = node->create_subscription<std_msgs::msg::Float32>("/vision/white_line_x", 10, std::bind(&MasterNode::getWhiteLineX, this, std::placeholders::_1));

    // ========== [Dynamixel 퍼블리셔 생성] ==========s
    pub_dxl_linear_vel_ = node->create_publisher<std_msgs::msg::Float32>("/stm32/dxl_linear_vel", 10);
    pub_dxl_angular_vel_ = node->create_publisher<std_msgs::msg::Float32>("/stm32/dxl_angular_vel", 10);

    // ========== [IMU 서브스크라이버] ==========
    sub_imu_yaw_ = node->create_subscription<std_msgs::msg::Float32>("/imu/yaw", 10, std::bind(&MasterNode::getImuYaw, this, std::placeholders::_1));

    // ========== [STM32의 PSD(ADC) Value 서브스크라이브] ==========
    sub_stm32_psd_adc_right_ = node->create_subscription<std_msgs::msg::Int32>("/stm32/psd_adc_value_right", 10, std::bind(&MasterNode::psdRightCallback, this, std::placeholders::_1));
    sub_stm32_psd_adc_front_ = node->create_subscription<std_msgs::msg::Int32>("/stm32/psd_adc_value_front", 10, std::bind(&MasterNode::psdFrontCallback, this, std::placeholders::_1));
    sub_stm32_psd_adc_left_ = node->create_subscription<std_msgs::msg::Int32>("/stm32/psd_adc_value_left", 10, std::bind(&MasterNode::psdLeftCallback, this, std::placeholders::_1));

    // ========== [White & Yellow Line 감지 및 좌표 서브스크라이브] ==========
    sub_white_line_points_ = node->create_subscription<std_msgs::msg::Float32MultiArray>("/vision/white_line_points", 10, std::bind(&MasterNode::getWhiteLinePoints, this, std::placeholders::_1));
    sub_yellow_line_points_ = node->create_subscription<std_msgs::msg::Float32MultiArray>("/vision/yellow_line_points", 10, std::bind(&MasterNode::getYellowLinePoints, this, std::placeholders::_1));

    // ========== [White & Yellow Angle(각도) 서브스크라이브] ==========
    sub_white_angle_ = node->create_subscription<std_msgs::msg::Float32>("/vision/white_line_angle", 10, std::bind(&MasterNode::getWhiteLineAngle, this, std::placeholders::_1));
    sub_yellow_angle_ = node->create_subscription<std_msgs::msg::Float32>("/vision/yellow_line_angle", 10, std::bind(&MasterNode::getYellowLineAngle, this, std::placeholders::_1));

    // ========== [주차 표지판 감지 서브스크라이브] ==========
    sub_blue_sign_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/blue_sign_detected", 10, std::bind(&MasterNode::detectBlueSign, this, std::placeholders::_1));

    // ========== [차단바 감지 서브스크라이브] ==========
    sub_barrier_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/barrier_detected", 10, std::bind(&MasterNode::detectBarrier, this, std::placeholders::_1));

    // ========== [Line Detect  중앙값 서브스크라이브] ==========
    sub_yellow_line_center_dist_ = node->create_subscription<std_msgs::msg::Float32>("/vision/yellow_line_center_dist", 10, std::bind(&MasterNode::getDistYellowLineCenter, this, std::placeholders::_1));
    sub_white_line_center_dist_ = node->create_subscription<std_msgs::msg::Float32>("/vision/white_line_center_dist", 10, std::bind(&MasterNode::getDistWhiteLineCenter, this, std::placeholders::_1));



    stage_number_ = 1; // 최초 시작: 스테이지1
}

MasterNode::~MasterNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void MasterNode::run()
{
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        // rclcpp::spin_some(node);

        /* 주행 로직
        * 각 line별 detected랑 position X값 사용해서 주행 로직 구현하기
        * 노란색 처리 우선(빛 반사 적음)
        */

        emit updateCurrentStage(stage_number_);
        RCLCPP_INFO(node->get_logger(), "Y Angle: %.2f | W Angle: %.2f || pixel_gap: %.2f || dist_yellow_line_: %.2f | dist_white_line_: %.2f || angular_vel_: %.2f || ", yellow_line_angle_, white_line_angle_, pixel_gap, dist_yellow_line_, dist_white_line_, angular_vel_);

        if (stage_number_ == 1) {
            runRobotStage1();
        } 
        else if (stage_number_ == 2) {
            runRobotStage2();
        } else if (stage_number_ == 3) {
            runRobotStage3();
        }

        // 현재 상태를 유지하며 지속적으로 퍼블리시
        auto msg_linear_ = std_msgs::msg::Float32();
        auto msg_angular_ = std_msgs::msg::Float32();

        msg_linear_.data = linear_vel_;
        msg_angular_.data = angular_vel_;

        pub_dxl_linear_vel_->publish(msg_linear_);
        pub_dxl_angular_vel_->publish(msg_angular_);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}

bool MasterNode::isInitialized() const
{
    return initialized_; // 초기화 상태 반환
}

// ========== [스테이지별 이동 처리 메서드] ==========
void MasterNode::runRobotStage1() {
    // stopDxl();
    // RCLCPP_INFO(node->get_logger(), "스테이지1");

    // if (((yellow_line_x_ <= -0.5 && white_line_x_ >= 0.5) && (500 < white_line_points_[0] && white_line_points_[0] < 600)) || ((500 < white_line_points_[0] && white_line_angle_ < 1))) {
    //     ctlDxlFront(0.65, 0);
    // } else if ((isDetectYellowLine && !isDetectWhiteLine) && ((0.5 >= yellow_line_x_) && (yellow_line_x_ >= -0.9))) { // 우회전 
    //     // 첫 번째 오른쪽 코너
    //     ctlDxlRight(0.35, 0.1);
    //     RCLCPP_INFO(node->get_logger(), "1");
    // } else if (((!isDetectYellowLine && isDetectWhiteLine) && ((0.8 >= white_line_x_) && (white_line_x_ >= 0.35)))) { // 좌회전
    //     // 주행 도중 라인 유지
    //     if ((0 <= white_line_angle_ && white_line_angle_ <= 1) && white_line_points_[0] > 430) {
    //         return;
    //     }
    //     ctlDxlLeft(0.35, 0.1);
    // }

    // 기본 주행 모드
    float center_x = 320.0; // 카메라 화면 중심 (예: 640x480 해상도의 중심 x좌표)
    linear_vel_ = 0.45;

    if ((isDetectYellowLine && isDetectWhiteLine) && dist_yellow_line_ < dist_white_line_) {
        pixel_gap = center_x - (int)(((dist_yellow_line_ * -1) + dist_white_line_) / 2);
        angular_vel_= (double)pixel_gap * GAIN_LINEAR;
        RCLCPP_INFO(node->get_logger(), "D-1");
    } else if ((isDetectYellowLine && !isDetectWhiteLine)) { // 노란색 선만 감지됨
        RCLCPP_INFO(node->get_logger(), "D-22222222");
        if (88 <= yellow_line_angle_ && yellow_line_angle_ <= 95) { // 예외 처리: 근사항 직진 주행
            // angular_vel_ = (220 + dist_yellow_line_) * GAIN_LINEAR; 
            if (-240 < dist_yellow_line_ && dist_yellow_line_ < 0) { // 노란색 차선이 직진 주행 각도로 감지되었으나, 직진 라인이 아닌 경우
                angular_vel_ = (dist_yellow_line_) * GAIN_LINEAR;
            RCLCPP_INFO(node->get_logger(), "D-2-0-0");
            } else if (0 < dist_yellow_line_) {
                angular_vel_ = (dist_yellow_line_ - 220) * GAIN_LINEAR;
                RCLCPP_INFO(node->get_logger(), "D-2-0-1");
            } else {
                angular_vel_ = 0.0;
                RCLCPP_INFO(node->get_logger(), "D-2-0-2");
            }
        } else if (95 <= yellow_line_angle_ && yellow_line_angle_ <= 95) {  // 좌회전 처리: (약 ~ 중)
            angular_vel_ = (180 + dist_yellow_line_) * GAIN_CORNER;
            RCLCPP_INFO(node->get_logger(), "D-2-1");
        } else if (95 <= yellow_line_angle_) { // 좌회전 처리: (중 ~ 강)
            // angular_vel_ = (25 + dist_yellow_line_) * GAIN_LINEAR;
            if (0 < dist_yellow_line_) {
                angular_vel_ = (dist_yellow_line_ - 280) * GAIN_LINEAR;
                RCLCPP_INFO(node->get_logger(), "D-2-2-0");
            } else {
                angular_vel_ = (dist_yellow_line_) * GAIN_LINEAR;
                RCLCPP_INFO(node->get_logger(), "D-2-2-1");
            }
        }
    } else if (!isDetectYellowLine && isDetectWhiteLine) {
        RCLCPP_INFO(node->get_logger(), "D-33333333");
        if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
            // angular_vel_ = (dist_white_line_ - 220) * GAIN_LINEAR;
            // if (0 < dist_white_line_ && dist_white_line_ < 240) { // 흰색 차선이 직진 주행 각도로 감지되었으나, 직진 라인이 아닌 경우
            //     angular_vel_ = (dist_white_line_) * GAIN_LINEAR;
            //     RCLCPP_INFO(node->get_logger(), "D-3-0-0"); 
            // } else if (dist_white_line_ < 0) {
            //     angular_vel_ = (dist_white_line_ + 220) * GAIN_LINEAR;
            //     RCLCPP_INFO(node->get_logger(), "D-3-0-1"); 
            // } else {
            //     angular_vel_ = 0.0;
            //     RCLCPP_INFO(node->get_logger(), "D-3-0-2"); 
            // }
            angular_vel_ = 0.0;
            RCLCPP_INFO(node->get_logger(), "D-3-0-3"); 
        } else if (83 < white_line_angle_ && white_line_angle_ <= 88) {
            if (dist_white_line_ < 0) {
                angular_vel_ = (dist_white_line_ + 200) * GAIN_CORNER;
            } else {
                angular_vel_ = (dist_white_line_) * GAIN_CORNER;
            }
            // angular_vel_ = (dist_white_line_ - 150) * GAIN_CORNER;
            RCLCPP_INFO(node->get_logger(), "D-3-1");
        } else if (yellow_line_angle_ < 83) {

            if (dist_white_line_ < 0) {
                angular_vel_ = (dist_white_line_ + 200) * GAIN_CORNER;
            } else {
                angular_vel_ = (dist_white_line_) * GAIN_LINEAR;
            }
            // angular_vel_ = (dist_white_line_ - 25) * GAIN_LINEAR;
            RCLCPP_INFO(node->get_logger(), "D-3-2");
        } else {

        }
    } else {
        // // 선이 감지되지 않을 경우
        // linear_vel_ = 0.0;
        angular_vel_ = 0.0;
        RCLCPP_INFO(node->get_logger(), "D-4");
    }


    // Stage2 감지
    // if (((psd_adc_left_ >= 2000) && (320 < white_line_points_[0] && white_line_points_[0] < 630)) && (75 < white_line_angle_ && white_line_angle_ <= 90)) {
    //     if (0.45 < white_line_x_ && white_line_x_ < 0.62) {
    //         stage_number_ = 2;
    //     }
    // }
}

void MasterNode::runRobotStage2() {
    // stopDxl();
    // RCLCPP_INFO(node->get_logger(), "스테이지2");
}

void MasterNode::runRobotStage3() {
    // stopDxl();
    // RCLCPP_INFO(node->get_logger(), "스테이지3");
}

// ========== [Line Detect 서브스크라이브] ==========
// 노랑 라인 감지 메서드
void MasterNode::detectYellowLine(const std_msgs::msg::Bool::SharedPtr msg) {
    isDetectYellowLine = msg->data;
    if (isDetectYellowLine) {
        isDetectYellowLine = true;
    } else {
        isDetectYellowLine = false;
    }
}

// 흰색 라인 감지 메서드
void MasterNode::detectWhiteLine(const std_msgs::msg::Bool::SharedPtr msg) {
    isDetectWhiteLine = msg->data;
    if (isDetectWhiteLine) {
        isDetectWhiteLine = true;
    } else {
        isDetectWhiteLine = false;
    }
}

// ========== [차단바 감지 서브스크라이브] ==========
void MasterNode::detectBarrier(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        stopDxl();
        isDetectBarrier = true;
    } else {
        isDetectBarrier = false;
    }
}


// ========== [라인 감지 서브스크라이브] ==========
void MasterNode::getYellowLineX(const std_msgs::msg::Float32::SharedPtr msg) {
    yellow_line_x_ = msg->data;
}

void MasterNode::getWhiteLineX(const std_msgs::msg::Float32::SharedPtr msg) {
    white_line_x_ = msg->data;
}

void MasterNode::getWhiteLinePoints(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    for (size_t i = 0; i < white_line_points_.size(); i++) {
        white_line_points_[i] = msg->data[i];
    }
}

void MasterNode::getYellowLinePoints(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    for (size_t i = 0; i < yellow_line_points_.size(); i++) {
        yellow_line_points_[i] = msg->data[i];
    }
}

void MasterNode::getWhiteLineAngle(const std_msgs::msg::Float32::SharedPtr msg) {
    white_line_angle_ = msg->data;
}

void MasterNode::getYellowLineAngle(const std_msgs::msg::Float32::SharedPtr msg) {
    yellow_line_angle_ = msg->data;
}


// ========== [IMU 서브스크라이브] ==========
void MasterNode::getImuYaw(const std_msgs::msg::Float32::SharedPtr msg) {
    imu_yaw_ = msg->data;
    if(playYawFlag) {
        ctlDxlYaw(target_yaw_);
    }
}


// ========== [Dxl Control 메서드] ==========
// Dxl 직진 메서드
void MasterNode::ctlDxlFront(float linearVel, float angularVel) {
    if (isRobotRun_) {
        linear_vel_ = linearVel;  // 상태 갱신
        angular_vel_ = angularVel;
    }
}

// Dxl 좌회전 메서드
void MasterNode::ctlDxlLeft(float linearVel, float angularVel) {
    if (isRobotRun_) {
        linear_vel_ = linearVel;  // 상태 갱신
        angular_vel_ = angularVel;
    }
}

// Dxl 우회전 메서드
void MasterNode::ctlDxlRight(float linearVel, float angularVel) {
    if (isRobotRun_) {
        linear_vel_ = linearVel;  // 상태 갱신
        angular_vel_ = -angularVel;
    }
}

// Dxl 후진 메서드
void MasterNode::ctlDxlBack(float linearVel, float angularVel) {
    if (isRobotRun_) {
        linear_vel_ = linearVel;  // 상태 갱신
        angular_vel_ = angularVel;
    }
}

// Dxl 정지 메서드
void MasterNode::stopDxl() {
    isRobotRun_ = !isRobotRun_;

    stage_number_ = 0;
    linear_vel_ = 0;  // 상태 갱신
    angular_vel_ = 0;

    auto msg_linear_ = std_msgs::msg::Float32();
    auto msg_angular_ = std_msgs::msg::Float32();

    msg_linear_.data = 0;
    msg_angular_.data = 0;

    pub_dxl_linear_vel_->publish(msg_linear_);
    pub_dxl_angular_vel_->publish(msg_angular_);

    RCLCPP_INFO(node->get_logger(), "STOPSTOPSTOP!");
}

void MasterNode::ctlDxlYaw(float target_yaw) {
    if (!playYawFlag) {
        RCLCPP_INFO(node->get_logger(), "예외");
        return;
    }
    RCLCPP_INFO(node->get_logger(), "진입");

    yaw_error = target_yaw - imu_yaw_;

    if(yaw_error > 180.0f)
        yaw_error -= 360.0f;
    if(yaw_error < -180.0f)
        yaw_error += 360.0f;

    if(std::fabs(yaw_error) < yaw_ok) {
        intergral = 0;
        playYawFlag = false;
        angular_vel_pid = 0;
        target_yaw_ = imu_yaw_;
        RCLCPP_INFO(node->get_logger(), "조건문");
        stopDxl();
        return;
    }

    intergral += yaw_error;
    float d_term = yaw_error - pre_yaw_error;
    angular_vel_pid = (kP * yaw_error) + (kI * intergral) + (kD * d_term);
    if(angular_vel_pid > max_angular_vel)
        angular_vel_pid = max_angular_vel;
    if(angular_vel_pid < -max_angular_vel)
        angular_vel_pid = -max_angular_vel;

    if (angular_vel_pid > 0 && angular_vel_pid < min_angular_vel)
        angular_vel_pid = min_angular_vel;
    if (angular_vel_pid < 0 && angular_vel_pid > -min_angular_vel)
        angular_vel_pid = -min_angular_vel;

    runDxl(0, static_cast<int>(-angular_vel_pid));
    pre_yaw_error = yaw_error;
}


// (사용X) 수동 조작 모드일 때, GUI에서 입력한 linear와 angular 데이터 반영
void MasterNode::updateDxlData(float linearVel, float angularVel) {
    linear_vel_ = linearVel;
    angular_vel_ = angularVel;
    RCLCPP_INFO(node->get_logger(), "UPDATEUPDATE!");
}


// Qt에서 단축키로 로봇을 제어하기 위한 메서드
void MasterNode::runDxl(float linearVel, float angularVel) {
    isRobotRun_ = false;
    linear_vel_ = linearVel;  // 상태 저장
    angular_vel_ = angularVel;  // 상태 저장

    auto msg_linear_ = std_msgs::msg::Float32();
    auto msg_angular_ = std_msgs::msg::Float32();

    msg_linear_.data = linearVel;
    msg_angular_.data = angularVel;

    pub_dxl_linear_vel_->publish(msg_linear_);
    pub_dxl_angular_vel_->publish(msg_angular_);
}


// ========== [STM32 PSD Value Callback Method] ==========
void MasterNode::psdRightCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    psd_adc_right_ = msg->data;
    emit stmPsdRightReceived(msg->data);
}

void MasterNode::psdFrontCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    psd_adc_front_ = msg->data;
    emit stmPsdFrontReceived(msg->data);
}

void MasterNode::psdLeftCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    psd_adc_left_ = msg->data;
    emit stmPsdLeftReceived(msg->data);
}


void MasterNode::responePidTest() {
    target_yaw_ = -90.0;
    playYawFlag = true;
}


// ========== [주차 표지판 감지 서브스크라이브] ==========
void MasterNode::detectBlueSign(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == true) {
        isDetectBlueSign = true;
    } else {
        isDetectBlueSign = false;
    }
}

void MasterNode::getDistYellowLineCenter(const std_msgs::msg::Float32::SharedPtr msg) {
    dist_yellow_line_ = msg->data;
}

void MasterNode::getDistWhiteLineCenter(const std_msgs::msg::Float32::SharedPtr msg) {
    dist_white_line_ = msg->data;
}

