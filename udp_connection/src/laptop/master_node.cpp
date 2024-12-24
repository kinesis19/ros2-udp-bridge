#include "../include/udp_connection/laptop/master_node.hpp"

MasterNode::MasterNode()
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

    // ========== [IMU 서브스크라이버 & 퍼블리셔] ==========
    sub_imu_yaw_ = node->create_subscription<std_msgs::msg::Float32>("/imu/yaw", 10, std::bind(&MasterNode::getImuYaw, this, std::placeholders::_1));
    pub_imu_reset_ = node->create_publisher<std_msgs::msg::Bool>("/imu/reset", 10);


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
    sub_left_blue_sign_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/left_blue_sign_detected", 10, std::bind(&MasterNode::detectLeftBlueSign, this, std::placeholders::_1));
    sub_straight_blue_sign_detected = node->create_subscription<std_msgs::msg::Bool>("/vision/straight_blue_sign_detected", 10, std::bind(&MasterNode::detectStraightBlueSign, this, std::placeholders::_1));

    // ========== [차단바 감지 서브스크라이브] ==========
    sub_barrier_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/barrier_detected", 10, std::bind(&MasterNode::detectBarrier, this, std::placeholders::_1));

    // ========== [Line Detect  중앙값 서브스크라이브] ==========
    sub_yellow_line_center_dist_ = node->create_subscription<std_msgs::msg::Float32>("/vision/yellow_line_center_dist", 10, std::bind(&MasterNode::getDistYellowLineCenter, this, std::placeholders::_1));
    sub_white_line_center_dist_ = node->create_subscription<std_msgs::msg::Float32>("/vision/white_line_center_dist", 10, std::bind(&MasterNode::getDistWhiteLineCenter, this, std::placeholders::_1));


    // ========== [Red Line Detect 서브스크라이브] ==========
    sub_red_line_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/red_line_detected", 10, std::bind(&MasterNode::detectRedLine, this, std::placeholders::_1));

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
        RCLCPP_INFO(node->get_logger(), "Y Angle: %.2f | W Angle: %.2f || pixel_gap: %.2f || dist_yellow_line_: %.2f | dist_white_line_: %.2f || angular_vel_: %.2f || imu_yaw_: %.2f ", yellow_line_angle_, white_line_angle_, pixel_gap, dist_yellow_line_, dist_white_line_, angular_vel_, imu_yaw_);

        if (stage_number_ == 1) {
            runRobotStage1();
        } else if (stage_number_ == 2) {
            runRobotStage2();
        } else if (stage_number_ == 3) {
            runRobotStage3();
        } else if (stage_number_ == 4) {
            runRobotStage4();
        } else if (stage_number_ == 5) {
            runRobotStage5();
        } else if (stage_number_ == 6) { // reverse stage5
            runRobotStage6();
        } else if (stage_number_ == 7) {
            runRobotStage7();
        } else if (stage_number_ == 8) {
            runRobotStage8();
        } else if (stage_number_ == 9) {
            runRobotStage9();
        } else if (stage_number_ == 10) {
            runRobotStage10();
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

    // 기본 주행 모드
    float center_x = 320.0; // 카메라 화면 중심 (예: 640x480 해상도의 중심 x좌표)
    linear_vel_ = 0.45;

    if ((isDetectYellowLine && isDetectWhiteLine) && (white_line_points_[0] < yellow_line_points_[0])) {
        if (88 <= yellow_line_angle_ && yellow_line_angle_ <= 95) { // 예외 처리: 근사항 직진 주행
            angular_vel_ = ((310 + dist_yellow_line_) / 3000) * -1;
        } else if (95 <= yellow_line_angle_ && yellow_line_angle_ <= 100) {  // 좌회전 처리: (약 ~ 중)
            angular_vel_ = ((310 + dist_yellow_line_) / 2500) * -1;
        } else if (100 <= yellow_line_angle_) { // 좌회전 처리: (중 ~ 강)
            if ((((310 + dist_yellow_line_) / 1000) * -1) < -0.4) {
                angular_vel_ = -0.4;
            } else {
                angular_vel_ = (((310 + dist_yellow_line_) / 1000) * -1);
            }
        }
    } else if ((isDetectYellowLine && isDetectWhiteLine) && dist_yellow_line_ < dist_white_line_) {
        angular_vel_ = 0.0;
    } else if ((isDetectYellowLine && !isDetectWhiteLine)) { // 노란색 선만 감지됨
        if (88 <= yellow_line_angle_ && yellow_line_angle_ <= 95) { // 예외 처리: 근사항 직진 주행
            angular_vel_ = ((320 + dist_yellow_line_) / 3000) * -1;
        } else if (95 <= yellow_line_angle_ && yellow_line_angle_ <= 100) {  // 좌회전 처리: (약 ~ 중)
            angular_vel_ = ((320 + dist_yellow_line_) / 2500) * -1;
        } else if (100 <= yellow_line_angle_) { // 좌회전 처리: (중 ~ 강)
            if ((((320 + dist_yellow_line_) / 800) * -1) < -0.4) {
                angular_vel_ = -0.4;
            } else {
                angular_vel_ = (((320 + dist_yellow_line_) / 800) * -1);
            }
        }
    } else if (!isDetectYellowLine && isDetectWhiteLine) {
        if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
            angular_vel_ = ((320 - dist_white_line_) / 3000) * 1;
        } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
            angular_vel_ = ((320 - dist_white_line_) / 3000) * 1;
        } else if (100 < white_line_angle_ || white_line_angle_ < 88) {  
            if ((((320 - dist_white_line_) / 800) * 1) > 0.35) {
                angular_vel_ = 0.35;
            } else {
                angular_vel_ = (((320 - dist_white_line_) / 800) * 1);
            }
        }
    } else {
        // // 선이 감지되지 않을 경우
        angular_vel_ = 0.0;
    }

    // Stage2 진입 감지 처리
    if (((psd_adc_left_ >= 2000) && (320 < white_line_points_[0] && white_line_points_[0] < 630)) && (75 < white_line_angle_ && white_line_angle_ <= 90)) {
        if (0.45 < white_line_x_ && white_line_x_ < 0.62) {
            stage_number_ = 2;
        }
    }
}

void MasterNode::runRobotStage2() {

    // 주행 로직
    if (!isDetectObject1andObject2 && (!isDetectYellowLine && isDetectWhiteLine)) {
        if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
            angular_vel_ = ((240 - dist_white_line_) / 2500) * 1;
        } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
            angular_vel_ = ((310 - dist_white_line_) / 3000) * 1;
        } else if (100 < white_line_angle_ || white_line_angle_ < 88) {  
            if ((((310 - dist_white_line_) / 1200) * 1) > 0.35) {
                angular_vel_ = 0.35;
            } else {
                angular_vel_ = (((310 - dist_white_line_) / 1200) * 1);
            }
        }
    }

    // 스테이지2 진입 후, 오브젝트1과 오브젝트2를 동시에 감지했을 때
    if (!isDetectObject1andObject2 && (psd_adc_front_ > 1300)) {
        // PID 제어로 왼쪽으로 회전하기
        if (imu_yaw_ - 50.0 < -180) {
            target_yaw_ = 360 + (imu_yaw_ - 50.0); // 범위 보정
        } else {
            target_yaw_ = imu_yaw_ - 50.0;
        }
        playYawFlag = true;
        isDetectObject1andObject2 = true;
    } else if ((isDetectObject1andObject2 && !playYawFlag) && !isWorkedPIDControlToTurnRightStage2){
        // PID 제어로 왼쪽 회전 및 직진 처리하기: 노란색 선이 보이기 전까지
        linear_vel_ = 0.35;
        angular_vel_ = 0.0;

        if (isDetectWhiteLine && !isDetectYellowLine) {
            isWorkedPIDControlToTurnRightStage2 = true;
        }
    }

    // PID 제어로 왼쪽 회전 및 직진 이후 노란색 선이 조건 범위 내에서 감지될 때
    if (isDetectObject1andObject2 && ((isDetectYellowLine && !isDetectWhiteLine)) && dist_yellow_line_ > 50) { // 노란색 선만 감지됨
        if (imu_yaw_ + 65.0 > 180) {
            target_yaw_ = 360 - (imu_yaw_ + 65.0); // 범위 보정 (양수에서 초과할 경우 음수로 변환)
        } else {
            target_yaw_ = imu_yaw_ + 65.0;
        }
        playYawFlag = true;
    }

    // PID 제어로 우회전 이후의 주행 로직
    if (isWorkedPIDControlToTurnRightStage2 && !playYawFlag) {
        if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
            angular_vel_ = ((310 - dist_white_line_) / 2500) * 1;
        } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
            angular_vel_ = ((310 - dist_white_line_) / 3000) * 1;
        } else if (100 < white_line_angle_ || white_line_angle_ < 88) {  
            if ((((310 - dist_white_line_) / 1200) * 1) > 0.35) {
                angular_vel_ = 0.35;
            } else {
                angular_vel_ = (((310 - dist_white_line_) / 1200) * 1);
            }
        }
    }

    // Stage3 진입 감지 처리: 주차 표지판을 감지했을 때 (직진 주행임과 동시에)
    if (isDetectBlueSign) {
        stage_number_ = 3;
    }

}

void MasterNode::runRobotStage3() {
    
    // 기본 주행 (흰색)
    if (!isDetectYellowLineinThreeStreetStage3 && !isStartPidTurnLeftThreeStreetStage3) {
        if (88 <= white_line_angle_ && white_line_angle_ <= 93) { // 직진
            angular_vel_ = ((315 - dist_white_line_) / 2500) * 1;
        } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
            angular_vel_ = ((315 - dist_white_line_) / 3000) * 1;
        }
        RCLCPP_INFO(node->get_logger(), "처음 흰 로직");
    }

    // 파란색 표지판을 놓쳤을 때
    if (!isDetectBlueSign && !isMissBlueSignStage3) {
        isMissBlueSignStage3 = true; // 플래그 처리
    }

    if (!isDetectYellowLine) {
        isMissYellowLineStage2 = true;
    }

    // 파란색 표지판을 놓치고, 노란색 라인을 재감지 했을 때
    if ((isMissBlueSignStage3 && isMissYellowLineStage2) && isDetectYellowLine) {
        isDetectYellowLineinThreeStreetStage3 = true; // 삼거리 감지 플래그 처리 (현재 위치: 삼거리 중앙, 서쪽 방향)
    }

    // 삼거리에서, 서쪽 방향을 바라보고 있을 때
    if ((isDetectYellowLineinThreeStreetStage3 && !isStartPidTurnLeftThreeStreetStage3)) {
        // PID 좌회전 처리하기
        if (imu_yaw_ - 88.0 < -180) {
            target_yaw_ = 360 + (imu_yaw_ - 88.0); // 범위 보정
        } else {
            target_yaw_ = imu_yaw_ - 88.0;
        }
        playYawFlag = true;
        // 플래그 처리
        isDetectYellowLineinThreeStreetStage3 = false; // 삼거리 입장 플래그 비활성화 
        isStartPidTurnLeftThreeStreetStage3 = true; // 삼거리 입장 후 PID 좌회전 처리 플래그 활성화
    }
    
    // 좌회전 PID 제어 이후, 노란색 라인만을 감지했을 때
    if (((isStartPidTurnLeftThreeStreetStage3 && !playYawFlag) && (isDetectYellowLine && !isDetectWhiteLine)) && !isDonePidControlParkingStationOutStage3) {
        RCLCPP_INFO(node->get_logger(), "PID 제어 이후의 주차장 진입");

        linear_vel_ = 0.4;

        // 양 옆이 노란색 라인일 때의 주행 처리
        if (isDetectYellowLine && !isDetectWhiteLine) {
            if (dist_yellow_line_ < -200) {
                if (75 <= yellow_line_angle_ && yellow_line_angle_ <= 88) { // 예외 처리: 근사항 직진 주행
                    angular_vel_ = ((310 - fabs(dist_yellow_line_)) / 2000) * 1;
                } else if (88 < yellow_line_angle_ && yellow_line_angle_ < 90) {  // 좌회전 처리: (약 ~ 중)
                    angular_vel_ = 0.0;
                } else if (90 <= yellow_line_angle_) {
                    angular_vel_ = ((310 - fabs(dist_yellow_line_)) / 2000) * -1.1;
                }
                RCLCPP_INFO(node->get_logger(), "노랑 진입 직진");
            }
        }
    } else if (!isDetectYellowLine && isDetectWhiteLine) { // 주차장에서 점선 감지했을 때w d
        // 주차장 입장 전까지 조건 : detectObjectNumParkingStationStage3
        angular_vel_ = 0.0;
        isDetectWhiteDottedLineStage3 = true; // 점선 감지 플래그 활성화
        RCLCPP_INFO(node->get_logger(), "흰색 점선");
    }

    // 흰색 점선이 감지된 이후의 상태일 때
    if (isDetectWhiteDottedLineStage3 && !isDetectYellowLineAfterDetectWhiteDottedLineStage3) { 
        // 건너편에 있는 노란색 선을 감지했을 때
        if (isDetectYellowLine && !isDetectWhiteLine) {
            isDetectYellowLineAfterDetectWhiteDottedLineStage3 = true;
        }
    }

    // 흰색 점선 이후에 노란색 라인을 감지함과 동시에 왼쪽 혹은 오른쪽에 오브젝트가 위치해 있을 때의 처리
    if (((isDetectYellowLineAfterDetectWhiteDottedLineStage3 && !isDonePidControlParkingStationInStage3) && (psd_adc_left_ > 2500 || psd_adc_right_ > 2500)) && !playYawFlag) {
        float target_yaw_vel_; // 오브젝트 위치에 따른 PID제어의 타겟 값 

        if (psd_adc_left_ > 2500 && psd_adc_right_ < 1800) { // 주차장 왼쪽에 오브젝트가 위치해 있을 때,
            target_yaw_vel_ = 90.0;
            detectObjectNumParkingStationStage3 = 1; // 왼쪽에 있다는 것을 저장.
        } else if (psd_adc_left_ < 1800 && psd_adc_right_ > 2500) { // 주차장 오른쪽에 오브젝트가 위치해 있을 때,
            target_yaw_vel_ = -90.0;
            detectObjectNumParkingStationStage3 = 2; // 오른쪽에 있다는 것을 저장.
        }

        // PID 제어로 왼쪽으로 회전하기
        if (imu_yaw_ - target_yaw_vel_ < -180) {
            target_yaw_ = 360 + (imu_yaw_ - target_yaw_vel_); // 범위 보정
        } else {
            target_yaw_ = imu_yaw_ - target_yaw_vel_;
        }

        playYawFlag = true;
        isDonePidControlParkingStationInStage3 = true; // 주차장에서 오브젝트가 없는 방향으로 이동하기 위한 PID 제어 진입 완료 여부를 나타내는 플래그 활성화
    }

    // 주차장에서 PID 제어 이후, 오브젝트가 있는 방향으로 바라보았을 때
    if ((isDonePidControlParkingStationInStage3 && !isReadyToParking) && (!playYawFlag && !isDonePidControlParkingStationOutStage3)) {

        isReadyToParking = true; // 주차 준비 플래그 활성화

        linear_vel_ = -0.2; // 후진 속도 설정
        angular_vel_ = 0.0;

        auto msg_linear = std_msgs::msg::Float32();
        auto msg_angular = std_msgs::msg::Float32();

        msg_linear.data = linear_vel_;
        msg_angular.data = angular_vel_;

        pub_dxl_linear_vel_->publish(msg_linear);
        pub_dxl_angular_vel_->publish(msg_angular);

        int target_seconds_move_back_;
        int target_seconds_move_front_;

        float target_yaw_vel_out_; // 오브젝트 위치에 따른 PID제어의 타겟 값 (나갈 때)

        if (detectObjectNumParkingStationStage3 == 1) { // 진입 기준 왼쪽에 오브젝트가 있을 때
            target_seconds_move_back_ = 2000;
            target_seconds_move_front_ = 2200;
            target_yaw_vel_out_ = 90.0;
        } else if (detectObjectNumParkingStationStage3 == 2) { // 진입 기준 오른쪽에 오브젝트가 있을 때
            target_seconds_move_back_ = 2000;
            target_seconds_move_front_ = 2000;
            target_yaw_vel_out_ = -60.0; // -55.0 ~ -60.0
        }

        rclcpp::sleep_for(std::chrono::milliseconds(target_seconds_move_back_)); // 첫 번째 딜레이: 후진

        stopDxl();
        rclcpp::sleep_for(std::chrono::milliseconds(1000)); // 두 번째 딜레이: 정지

        linear_vel_ = 0.2;
        angular_vel_ = 0.0;

        msg_linear.data = linear_vel_;
        msg_angular.data = angular_vel_;

        pub_dxl_linear_vel_->publish(msg_linear);
        pub_dxl_angular_vel_->publish(msg_angular);

        RCLCPP_INFO(node->get_logger(), "주차 이후: 직진");

        rclcpp::sleep_for(std::chrono::milliseconds(target_seconds_move_front_)); // 세 번째 딜레이: 직진

        stopDxl();
        // rclcpp::sleep_for(std::chrono::milliseconds(1000)); // 네 번째 딜레이: 정지
    }
    

    // 주차 이후, 주차장에서 나오지 않았을 때
    if (isReadyToParking && !isDonePidControlParkingStationOutStage3) {

        linear_vel_ = 0.0;

        if (detectObjectNumParkingStationStage3 == 1) { // 진입 기준 왼쪽에 오브젝트가 있을 때
            // angular_vel_ = 0.08;
            angular_vel_ = 0.1;
        } else if (detectObjectNumParkingStationStage3 == 2) { // 진입 기준 오른쪽에 오브젝트가 있을 때
            // angular_vel_ = -0.08;
            angular_vel_ = -0.1;
        }

        RCLCPP_INFO(node->get_logger(), "주차장 퇴출을 위한 회전 처리 중");

        if (isDetectYellowLine && isDetectWhiteLine) {
            if (((isDetectYellowLine && dist_yellow_line_ < -200) && (90 <= yellow_line_angle_ && yellow_line_angle_ <= 91)) && (psd_adc_front_ < 1200 && (psd_adc_left_ > 2000 || psd_adc_right_ > 2000))) {
                isDonePidControlParkingStationOutStage3 = true;
                RCLCPP_INFO(node->get_logger(), "주차장 퇴출 준비 완료");
            }
        } else if (isDetectYellowLine && !isDetectWhiteLine) {
            if (((isDetectYellowLine && dist_yellow_line_ < -200) && (90 <= yellow_line_angle_ && yellow_line_angle_ <= 91)) && (psd_adc_front_ < 1200 && (psd_adc_left_ > 2000 || psd_adc_right_ > 2000))) {
                isDonePidControlParkingStationOutStage3 = true;
                RCLCPP_INFO(node->get_logger(), "주차장 퇴출 준비 완료");
            }
        } else if (!isDetectYellowLine && isDetectWhiteLine) {
            if (((isDetectWhiteLine && dist_white_line_ > 200) && (90 <= white_line_angle_ && white_line_angle_ <= 91)) && (psd_adc_front_ < 1200 && (psd_adc_left_ > 2000 || psd_adc_right_ > 2000))) {
                // 흰 선이 오른쪽에서 수직선으로 되어 있으며, psd 조건을 충족했을 때
                isDonePidControlParkingStationOutStage3 = true;
                RCLCPP_INFO(node->get_logger(), "주차장 퇴출 준비 완료");
            }
        }

        // // 노란선이 왼쪽에서 수직선으로 되어 있으며, psd 조건을 충족했을 때r
        // if (((isDetectYellowLine && dist_yellow_line_ < -200) && (90 <= yellow_line_angle_ && yellow_line_angle_ <= 91)) && (psd_adc_front_ < 1200 && (psd_adc_left_ > 2000 || psd_adc_right_ > 2000))) {
        //     isDonePidControlParkingStationOutStage3 = true;
        //     RCLCPP_INFO(node->get_logger(), "주차장 퇴출 준비 완료");
        // } else if (((isDetectWhiteLine && dist_white_line_ > 200) && (90 <= white_line_angle_ && white_line_angle_ <= 91)) && (psd_adc_front_ < 1200 && (psd_adc_left_ > 2000 || psd_adc_right_ > 2000))) {
        //     // 흰 선이 오른쪽에서 수직선으로 되어 있으며, psd 조건을 충족했을 때
        //     isDonePidControlParkingStationOutStage3 = true;
        //     RCLCPP_INFO(node->get_logger(), "주차장 퇴출 준비 완료");
        // }
    }


    // 주차장에서 퇴출 준비가 완료된 상태일 때
    if ((isDonePidControlParkingStationOutStage3 && !isTurnLeftToGoToStage4)) {
        linear_vel_ = 0.4;

        // 양 옆이 노란선일 때의 주행 처리
        if (isDetectYellowLine && !isDetectWhiteLine) {

            // 양 옆이 노란색 라인일 때의 주행 처리
            if (dist_yellow_line_ < -200) {
                if (75 <= yellow_line_angle_ && yellow_line_angle_ <= 88) { // 예외 처리: 근사항 직진 주행
                    angular_vel_ = ((310 - fabs(dist_yellow_line_)) / 2000) * 1;
                } else if (88 < yellow_line_angle_ && yellow_line_angle_ < 90) {  // 좌회전 처리: (약 ~ 중)
                    // angular_vel_ = 0.0;
                    angular_vel_ = ((310 - fabs(dist_yellow_line_)) / 2500) * 1;
                } else if (90 <= yellow_line_angle_) {
                    angular_vel_ = ((310 - fabs(dist_yellow_line_)) / 2000) * -1.1;
                }
                RCLCPP_INFO(node->get_logger(), "노랑 탈출");
            }

        } else {
            angular_vel_ = 0.0;
            RCLCPP_INFO(node->get_logger(), "몰라1");
        }
    }

    // 삼거리에서 좌회전 표지판을 감지하고, 주차장에서 퇴출이 완료된 상태일 때
    if ((isDetectLeftBlueSign && isDonePidControlParkingStationOutStage3) || isTurnLeftToGoToStage4) {
        // 아래 하위 조건이 감지 되기 전까지 계속 좌회전 하기
        linear_vel_ = 0.0;
        angular_vel_ = 0.07;
        // angular_vel_ = 0.1;
        isTurnLeftToGoToStage4 = true;
        RCLCPP_INFO(node->get_logger(), "회전한다");

        if ((isDetectYellowLine && !isDetectWhiteLine) && (89 <= yellow_line_angle_ && yellow_line_angle_ <= 91)) {
            // stopDxl();
            stage_number_ = 4;
            RCLCPP_INFO(node->get_logger(), "스테이지4 플래그 완료");
        }
    }
}

void MasterNode::runRobotStage4() {
    if (isDetectBarrier) {
        isDetectBarrierStage4 = true;
    }
    
    if (isDetectBarrierStage4) {
        stage_number_ = 5;
    } else {
        linear_vel_ = 0.25;
        if ((isDetectYellowLine && isDetectWhiteLine) && dist_yellow_line_ < dist_white_line_) {
            angular_vel_ = 0.0;
        } else if ((isDetectYellowLine && !isDetectWhiteLine)) { // 노란색 선만 감지됨
            if (88 <= yellow_line_angle_ && yellow_line_angle_ <= 95) { // 예외 처리: 근사항 직진 주행
                angular_vel_ = ((310 + dist_yellow_line_) / 2500) * -1;
            } else if (95 <= yellow_line_angle_ && yellow_line_angle_ <= 100) {  // 좌회전 처리: (약 ~ 중)
                angular_vel_ = ((310 + dist_yellow_line_) / 2500) * -1;
            } else if (100 <= yellow_line_angle_) { // 좌회전 처리: (중 ~ 강)
                if ((((310 + dist_yellow_line_) / 800) * -1) < -0.4) {
                    angular_vel_ = -0.4;
                } else {
                    angular_vel_ = (((310 + dist_yellow_line_) / 800) * -1);
                }
            }
        } else if (!isDetectYellowLine && isDetectWhiteLine) {
            if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
                angular_vel_ = ((310 - dist_white_line_) / 3000) * 1;
            } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
                angular_vel_ = ((310 - dist_white_line_) / 3000) * 1;
            } else if (100 < white_line_angle_ || white_line_angle_ < 88) {  
                if ((((310 - dist_white_line_) / 2000) * 1) > 0.35) {
                    angular_vel_ = 0.35;
                } else {
                    angular_vel_ = (((310 - dist_white_line_) / 2000) * 1);
                }
            }
        } else {
            // // 선이 감지되지 않을 경우
            angular_vel_ = 0.0;
        }
    }
}

void MasterNode::runRobotStage5() {
    if (isDetectBarrier) {
        isDetectBarrierStage5 = true;
    } else {
        isDetectBarrierStage5 = false;
    }
    
    if (isDetectBarrierStage5) {
        stopDxl();
    } else if ((!isDetectBarrierStage5 && !isDetectRedLine) && !isDonePidControlEndLineStage5) {
        linear_vel_ = 0.45;
        if ((isDetectYellowLine && isDetectWhiteLine) && dist_yellow_line_ < dist_white_line_) {
            angular_vel_ = 0.0;
        } else if ((isDetectYellowLine && !isDetectWhiteLine)) { // 노란색 선만 감지됨
            if (88 <= yellow_line_angle_ && yellow_line_angle_ <= 95) { // 예외 처리: 근사항 직진 주행
                angular_vel_ = ((310 + dist_yellow_line_) / 2700) * -1;
            } else if (95 <= yellow_line_angle_ && yellow_line_angle_ <= 100) {  // 좌회전 처리: (약 ~ 중)
                angular_vel_ = ((310 + dist_yellow_line_) / 2000) * -1;
            } else if (100 <= yellow_line_angle_) { // 좌회전 처리: (중 ~ 강)
                if ((((310 + dist_yellow_line_) / 800) * -1) < -0.4) {
                    angular_vel_ = -0.4;
                } else {
                    angular_vel_ = (((310 + dist_yellow_line_) / 800) * -1);
                }
            }
        } else if (!isDetectYellowLine && isDetectWhiteLine) {
            if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
                angular_vel_ = ((310 - dist_white_line_) / 2700) * 1;
            } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
                angular_vel_ = ((310 - dist_white_line_) / 2700) * 1;
            } else if (100 < white_line_angle_ || white_line_angle_ < 88) {  
                if ((((310 - dist_white_line_) / 1200) * 1) > 0.35) {
                    angular_vel_ = 0.35;
                } else {
                    angular_vel_ = (((310 - dist_white_line_) / 1200) * 1);
                }
            }
        } else {
            // // 선이 감지되지 않을 경우
            angular_vel_ = 0.0;
        }
    }

    if (isDetectRedLine && !isDonePidControlEndLineStage5) {
        isDetectEndLineStage5 = true;
    }

    if (isDetectEndLineStage5 && !isDonePidControlEndLineStage5) {

        // PID 180도 회전 처리하기
        if (imu_yaw_ - 180.0 < -180) {
            target_yaw_ = 360 + (imu_yaw_ - 180.0); // 범위 보정
        } else {
            target_yaw_ = imu_yaw_ - 180.0;
        }

        playYawFlag = true;
        isDonePidControlEndLineStage5 = true;

        // angular_vel_ = 0.1;

        // if (((isDetectWhiteLine && dist_white_line_ < -270) && (90 <= white_line_angle_ && white_line_angle_ <= 91))) {
        //     isDonePidControlEndLineStage5 = true;
        //     RCLCPP_INFO(node->get_logger(), "제어 완료");
        // }
    }

    if ((isDetectEndLineStage5 && !playYawFlag) && isDonePidControlEndLineStage5) {
        // stopDxl();
        stage_number_ = 6; // reverse stage5
    }
}

void MasterNode::runRobotStage6() {
    if (isDetectBarrier) {
        isDetectBarrierStage6 = true;
    } 
    
    if (isDetectBarrierStage6) {
        stage_number_ = 7;
    } else {
        linear_vel_ = 0.25;
        if ((isDetectYellowLine && isDetectWhiteLine) && dist_yellow_line_ > dist_white_line_) {
            angular_vel_ = 0.0;
        } else if ((isDetectYellowLine && !isDetectWhiteLine)) { // 노란색 선만 감지됨
            if (88 <= yellow_line_angle_ && yellow_line_angle_ <= 95) { // 예외 처리: 근사항 직진 주행
                angular_vel_ = ((235 - dist_yellow_line_) / 3000) * 1;
                RCLCPP_INFO(node->get_logger(), "S6 - 1");
            } else if (95 <= yellow_line_angle_ && yellow_line_angle_ <= 100) {  // 좌회전 처리: (약 ~ 중)
                angular_vel_ = ((235 - dist_yellow_line_) / 2500) * 1;
                RCLCPP_INFO(node->get_logger(), "S6 - 2");
            } else if (100 < yellow_line_angle_ || yellow_line_angle_ < 88) { // 좌회전 처리: (중 ~ 강)
                if ((((235 - dist_yellow_line_) / 2000) * 1) > 0.4) {
                    angular_vel_ = 0.4;
                    RCLCPP_INFO(node->get_logger(), "S6 - 3");
                } else {
                    angular_vel_ = (((235 - dist_yellow_line_) / 2000) * 1);
                    RCLCPP_INFO(node->get_logger(), "S6 - 4");
                }
            }
        } else if (!isDetectYellowLine && isDetectWhiteLine) {
            if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
                angular_vel_ = ((235 + dist_white_line_) / 2500) * -1;
            } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
                angular_vel_ = ((235 + dist_white_line_) / 2200) * -1;
            } else if (100 < white_line_angle_) {  
                if ((((235 + dist_white_line_) / 1200) * -1) < -0.35) {
                    angular_vel_ = -0.35;
                } else {
                    angular_vel_ = (((235 + dist_white_line_) / 1200) * -1);
                }
            }
        } else {
            // // 선이 감지되지 않을 경우
            angular_vel_ = 0.0;
        }
    }
}

void MasterNode::runRobotStage7() {
    if (isDetectBarrier) {
        isDetectBarrierStage7 = true;
    } else {
        isDetectBarrierStage7 = false;
    }
    
    if (isDetectBarrierStage7) {
        stopDxl();
    } else if ((!isDetectBarrierStage5 && !isDetectRedLine) && !isDetectBarrierStage7) {
        linear_vel_ = 0.25;

        // reverse용 코드
        if ((isDetectYellowLine && isDetectWhiteLine) && (white_line_points_[0] > yellow_line_points_[0])) {
            if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
                angular_vel_ = ((235 + dist_white_line_) / 2500) * -1;
            } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
                angular_vel_ = ((235 + dist_white_line_) / 2200) * -1;
            } else if (100 < white_line_angle_) {  
                if ((((235 + dist_white_line_) / 1200) * -1) < -0.35) {
                    angular_vel_ = -0.35;
                } else {
                    angular_vel_ = (((235 + dist_white_line_) / 1200) * -1);
                }
            }
        } else if ((isDetectYellowLine && isDetectWhiteLine) && dist_yellow_line_ > dist_white_line_) {
            angular_vel_ = 0.0;
        } else if ((isDetectYellowLine && !isDetectWhiteLine)) { // 노란색 선만 감지됨
            if (88 <= yellow_line_angle_ && yellow_line_angle_ <= 95) { // 예외 처리: 근사항 직진 주행
                angular_vel_ = ((235 - dist_yellow_line_) / 2500) * 1;
            } else if (95 <= yellow_line_angle_ && yellow_line_angle_ <= 100) {  // 좌회전 처리: (약 ~ 중)
                angular_vel_ = ((235 - dist_yellow_line_) / 2000) * 1;
            } else if (100 < yellow_line_angle_ || yellow_line_angle_ < 88) { // 좌회전 처리: (중 ~ 강)
                if ((((235 - dist_yellow_line_) / 2200) * 1) > 0.4) {
                    angular_vel_ = 0.4;
                } else {
                    angular_vel_ = (((235 - dist_yellow_line_) / 2200) * 1);
                }
            }
        } else if (!isDetectYellowLine && isDetectWhiteLine) {
            if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
                angular_vel_ = ((235 + dist_white_line_) / 2500) * -1;
            } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
                angular_vel_ = ((235 + dist_white_line_) / 2200) * -1;
            } else if (100 < white_line_angle_) {  
                if ((((235 + dist_white_line_) / 1200) * -1) < -0.35) {
                    angular_vel_ = -0.35;
                } else {
                    angular_vel_ = (((235 + dist_white_line_) / 1200) * -1);
                }
            }
        } else {
            // // 선이 감지되지 않을 경우
            angular_vel_ = 0.0;
        }
    }

    if ((isDetectStraightBlueSign && isDetectWhiteLine) && (90 <= white_line_angle_ && white_line_angle_ <= 91)) {
        stage_number_ = 8;
    } else if (((psd_adc_right_ >= 2000) && (0 < white_line_points_[0] && white_line_points_[0] < 320)) && (90 <= white_line_angle_ && white_line_angle_ <= 105)) {
        if (-0.62 < white_line_x_ && white_line_x_ < 0.45) {
            stage_number_ = 9;
        }
    }
}

void MasterNode::runRobotStage8() {

    linear_vel_ = 0.45;

    // reverse용 코드
    if ((isDetectYellowLine && isDetectWhiteLine) && (white_line_points_[0] > yellow_line_points_[0])) {
        if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
            angular_vel_ = ((235 + dist_white_line_) / 2500) * -1;
        } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
            angular_vel_ = ((235 + dist_white_line_) / 2200) * -1;
        } else if (100 < white_line_angle_) {  
            if ((((235 + dist_white_line_) / 800) * -1) < -0.35) {
                angular_vel_ = -0.35;
            } else {
                angular_vel_ = (((235 + dist_white_line_) / 800) * -1);
            }
        }
    } else if ((isDetectYellowLine && isDetectWhiteLine) && dist_yellow_line_ > dist_white_line_) {
        angular_vel_ = 0.0;
    } else if ((isDetectYellowLine && !isDetectWhiteLine)) { // 노란색 선만 감지됨
        if (88 <= yellow_line_angle_ && yellow_line_angle_ <= 95) { // 예외 처리: 근사항 직진 주행
            angular_vel_ = ((235 - dist_yellow_line_) / 2500) * 1;
        } else if (95 <= yellow_line_angle_ && yellow_line_angle_ <= 100) {  // 좌회전 처리: (약 ~ 중)
            angular_vel_ = ((235 - dist_yellow_line_) / 2000) * 1;
        } else if (100 < yellow_line_angle_ || yellow_line_angle_ < 88) { // 좌회전 처리: (중 ~ 강)
            if ((((235 - dist_yellow_line_) / 2000) * 1) > 0.4) {
                angular_vel_ = 0.4;
            } else {
                angular_vel_ = (((235 - dist_yellow_line_) / 2000) * 1);
            }
        }
    } else if (!isDetectYellowLine && isDetectWhiteLine) {
        if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
            angular_vel_ = ((235 + dist_white_line_) / 2500) * -1;
        } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
            angular_vel_ = ((235 + dist_white_line_) / 2200) * -1;
        } else if (100 < white_line_angle_) {  
            if ((((235 + dist_white_line_) / 800) * -1) < -0.35) {
                angular_vel_ = -0.35;
            } else {
                angular_vel_ = (((235 + dist_white_line_) / 800) * -1);
            }
        }
    } else {
        // // 선이 감지되지 않을 경우
        angular_vel_ = 0.0;
    }

    // Stage9 진입 감지 처리
    if (((psd_adc_right_ >= 2000) && (0 < white_line_points_[0] && white_line_points_[0] < 320)) && (90 <= white_line_angle_ && white_line_angle_ <= 105)) {
        if (-0.62 < white_line_x_ && white_line_x_ < 0.45) {
            stage_number_ = 9;
        }
    }
}

void MasterNode::runRobotStage9() {

    // 주행 로직
    if (!isDetectObject1andObject2Stage9 && (!isDetectYellowLine && isDetectWhiteLine)) {
        if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
            angular_vel_ = ((235 + dist_white_line_) / 2500) * -1;
        } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
            angular_vel_ = ((235 + dist_white_line_) / 2200) * -1;
        } else if (100 < white_line_angle_) {  
            if ((((235 + dist_white_line_) / 800) * -1) < -0.35) {
                angular_vel_ = -0.35;
            } else {
                angular_vel_ = (((235 + dist_white_line_) / 800) * -1);
            }
        }
    }

    // 스테이지9 진입 후, 오브젝트3과 오브젝트2를 동시에 감지했을 때
    if (!isDetectObject1andObject2Stage9 && (psd_adc_front_ > 1500)) {

        if (!isOkayResetIMUStage9) {
            resetIMU();
            isOkayResetIMUStage9 = true;
        } else if (imu_yaw_ == 0.0) {            
            linear_vel_ = 0.0;
            angular_vel_ = -0.1;
            isDetectObject1andObject2Stage9 = true;
        }
    } else if ((isDetectObject1andObject2Stage9 && !playYawFlag) && (!isTempDoneTurnLeftRangeStage9 && !isTempDoneTurnRightRangeStage9)){
        if (!isTempDoneTurnRightRangeStage9) {
            if (35.0 <= imu_yaw_ && imu_yaw_ <= 40.0) {
                stopDxl();
                isTempDoneTurnRightRangeStage9 = true;
                linear_vel_ = 0.35;
                angular_vel_ = 0.0;
            } else if (0.0 <= imu_yaw_ && imu_yaw_ < 35.0) {
                angular_vel_ = -0.1;
            } else if (40.0 < imu_yaw_) {
                angular_vel_ = 0.1;
            } else if (imu_yaw_ < 0.0) {
                angular_vel_ = -0.1;
            }
        }
    }

    // PID 제어로 왼쪽 회전 및 직진 이후 노란색 선이 조건 범위 내에서 감지될 때
    if (((isDetectObject1andObject2Stage9 && !isTempDoneTurnLeftRangeStage9) && (isDetectYellowLine && !isDetectWhiteLine)) && (dist_yellow_line_ < -180 && !isReadyToPIDControlToTurnLeftStage9)) { // 노란색 선만 감지됨
        isReadyToPIDControlToTurnLeftStage9 = true;
    }

    if (isReadyToPIDControlToTurnLeftStage9 && !isTempDoneTurnLeftRangeStage9) {
        linear_vel_ = 0.0;

        RCLCPP_INFO(node->get_logger(), "아아아아아아아아아아아아");
        if (-35.0 <= imu_yaw_ && imu_yaw_ <= -15.0) {
            stopDxl();
            isTempDoneTurnLeftRangeStage9 = true;
            RCLCPP_INFO(node->get_logger(), "노랑에서 제어");
            linear_vel_ = 0.35;
            angular_vel_ = 0.0;
            RCLCPP_INFO(node->get_logger(), "흰색을 향해");
        } else if (-15.0 < imu_yaw_) {
            angular_vel_ = 0.1;
            RCLCPP_INFO(node->get_logger(), "로그1");
        } else if (imu_yaw_ < -35.0) {
            angular_vel_ = -0.1;
            RCLCPP_INFO(node->get_logger(), "로그2");
        } else {
            angular_vel_ = 0.1;
            RCLCPP_INFO(node->get_logger(), "로그3");
        }
    }

    if ((isDetectWhiteLine && !isDetectYellowLine) && isTempDoneTurnLeftRangeStage9) {
        isWorkedPIDControlToTurnRightStage9 = true;
        RCLCPP_INFO(node->get_logger(), "흰색 감지 완료");
    }

    // PID 제어로 우회전 이후의 주행 로직
    if (isWorkedPIDControlToTurnRightStage9) {
        if ((isDetectYellowLine && isDetectWhiteLine) && (white_line_points_[0] > yellow_line_points_[0])) {
            if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
                angular_vel_ = ((235 + dist_white_line_) / 2500) * -1;
            } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
                angular_vel_ = ((235 + dist_white_line_) / 2200) * -1;
            } else if (100 < white_line_angle_) {  
                if ((((235 + dist_white_line_) / 800) * -1) < -0.35) {
                    angular_vel_ = -0.35;
                } else {
                    angular_vel_ = (((235 + dist_white_line_) / 800) * -1);
                }
            }
        } else if ((isDetectYellowLine && isDetectWhiteLine) && dist_yellow_line_ > dist_white_line_) {
            angular_vel_ = 0.0;
        } else if ((isDetectYellowLine && !isDetectWhiteLine)) { // 노란색 선만 감지됨
            if (88 <= yellow_line_angle_ && yellow_line_angle_ <= 95) { // 예외 처리: 근사항 직진 주행
                angular_vel_ = ((235 - dist_yellow_line_) / 2500) * 1;
            } else if (95 <= yellow_line_angle_ && yellow_line_angle_ <= 100) {  // 좌회전 처리: (약 ~ 중)
                angular_vel_ = ((235 - dist_yellow_line_) / 2000) * 1;
            } else if (100 < yellow_line_angle_ || yellow_line_angle_ < 88) { // 좌회전 처리: (중 ~ 강)
                if ((((235 - dist_yellow_line_) / 2000) * 1) > 0.4) {
                    angular_vel_ = 0.4;
                } else {
                    angular_vel_ = (((235 - dist_yellow_line_) / 2000) * 1);
                }
            }
        } else if (!isDetectYellowLine && isDetectWhiteLine) {
            if (88 <= white_line_angle_ && white_line_angle_ <= 93) {
                angular_vel_ = ((235 + dist_white_line_) / 2500) * -1;
            } else if (93 < white_line_angle_ && white_line_angle_ <= 100) {
                angular_vel_ = ((235 + dist_white_line_) / 2200) * -1;
            } else if (100 < white_line_angle_) {  
                if ((((235 + dist_white_line_) / 800) * -1) < -0.35) {
                    angular_vel_ = -0.35;
                } else {
                    angular_vel_ = (((235 + dist_white_line_) / 800) * -1);
                }
            }
        } else {
            // // 선이 감지되지 않을 경우
            angular_vel_ = 0.0;
        }
    }

    if (isDetectRedLine && isWorkedPIDControlToTurnRightStage9) {
        stopDxl(); // 종료
    }
}

void MasterNode::runRobotStage10() {

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

void MasterNode::detectRedLine(const std_msgs::msg::Bool::SharedPtr msg) {
    isDetectRedLine = msg->data;
    if (isDetectRedLine) {
        isDetectRedLine = true;
        RCLCPP_INFO(node->get_logger(), "빨간 라인 감지됨");
    } else {
        isDetectRedLine = false;
    }
}

// ========== [차단바 감지 서브스크라이브] ==========
void MasterNode::detectBarrier(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
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


// ========== [IMU 서브스크라이브 & 퍼블리시 ] ==========
void MasterNode::getImuYaw(const std_msgs::msg::Float32::SharedPtr msg) {
    imu_yaw_ = msg->data;
    if(playYawFlag) {
        ctlDxlYaw(target_yaw_);
    }
}

void MasterNode::resetIMU() {
    bool isRequestResetIMU = true;

    auto msg_isRequestResetIMU = std_msgs::msg::Bool();
    msg_isRequestResetIMU.data = isRequestResetIMU;
    pub_imu_reset_->publish(msg_isRequestResetIMU);
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

// for Qt Button
void MasterNode::resetValue() {

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

    isDetectObject1andObject2 = false;
    isWorkedPIDControlToTurnRightStage2 = false;

    isDetectObject1Stage1 = false;

    isDetectObject1andObject2 = false;
    isWorkedPIDControlToTurnRightStage2 = false;
    
    isMissBlueSignStage3 = false;
    isDetectYellowLineinThreeStreetStage3 = false;
    isStartPidTurnLeftThreeStreetStage3 = false;
    isDetectWhiteDottedLineStage3 = false;
    isDetectYellowLineAfterDetectWhiteDottedLineStage3 = false;
    isDonePidControlParkingStationInStage3 = false;
    isDonePidControlParkingStationOutStage3 = false;
    isReadyToParking = false;
    detectObjectNumParkingStationStage3 = false;

    isDetectBarrierStage4 = false;

    isDetectBarrierStage5 = false;
    isDetectEndLineStage5 = false;
    isDonePidControlEndLineStage5 = false;

    isDetectBarrierStage6 = false;

    isDetectBarrierStage7 = false;
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

    runDxl(0, (-angular_vel_pid));
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

void MasterNode::detectLeftBlueSign(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == true) {
        isDetectLeftBlueSign = true;
        RCLCPP_INFO(node->get_logger(), "탈출 표지판 감지 완료!");
    } else {
        isDetectLeftBlueSign = false;
    }
}

void MasterNode::detectStraightBlueSign(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == true) {
        isDetectStraightBlueSign = true;
        RCLCPP_INFO(node->get_logger(), "탈출 표지판 감지 완료!");
    } else {
        isDetectStraightBlueSign = false;
    }
}

void MasterNode::getDistYellowLineCenter(const std_msgs::msg::Float32::SharedPtr msg) {
    dist_yellow_line_ = msg->data;
}

void MasterNode::getDistWhiteLineCenter(const std_msgs::msg::Float32::SharedPtr msg) {
    dist_white_line_ = msg->data;
}

