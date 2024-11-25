#include "../include/udp_connection/laptop/master_node.hpp"

MasterNode::MasterNode() : isDetectYellowLine(false), isDetectWhiteLine(false), isRobotRun_(false), linear_vel_(0), angular_vel_(0), yellow_line_x_(0.0), white_line_x_(0.0), stage_number_(0)
{
    node = rclcpp::Node::make_shared("master_node");

    RCLCPP_INFO(node->get_logger(), "master_node 초기화 완료");

    initialized_ = true;

    // ========== [Line Detect 서브스크라이버] ==========
    sub_yellow_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/yellow_line_detected", 10, std::bind(&MasterNode::detectYellowLine, this, std::placeholders::_1));
    sub_white_detected_ = node->create_subscription<std_msgs::msg::Bool>("/vision/white_line_detected", 10, std::bind(&MasterNode::detectWhiteLine, this, std::placeholders::_1));
    sub_yellow_line_x_ = node->create_subscription<std_msgs::msg::Float32>("/vision/yellow_line_x", 10, std::bind(&MasterNode::getYellowLineX, this, std::placeholders::_1));
    sub_white_line_x_ = node->create_subscription<std_msgs::msg::Float32>("/vision/white_line_x", 10, std::bind(&MasterNode::getWhiteLineX, this, std::placeholders::_1));

    // ========== [Dynamixel 퍼블리셔 생성] ==========
    pub_dxl_linear_vel_ = node->create_publisher<std_msgs::msg::Int32>("/stm32/dxl_linear_vel", 10);
    pub_dxl_angular_vel_ = node->create_publisher<std_msgs::msg::Int32>("/stm32/dxl_angular_vel", 10);

    stage_number_ = 1; // 최초 시작: 스테이지1
    isStage1TurnLeft = false;
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
        /* 주행 로직
        * 각 line별 detected랑 position X값 사용해서 주행 로직 구현하기
        * 노란색 처리 우선(빛 반사 적음)
        */
        RCLCPP_INFO(node->get_logger(), "Yellow Pos: %.2f | White Pos: %.2f", yellow_line_x_, white_line_x_);
        
        if (stage_number_ == 1) {
            runRobotStage1();
        }
        
        // if (isDetectYellowLine && isDetectWhiteLine) {
        //     ctlDxlFront();
        // } else if (isDetectYellowLine && !isDetectWhiteLine) {
        //     ctlDxlRight();
        // } else if (!isDetectYellowLine && isDetectWhiteLine) {
        //     ctlDxlLeft();
        // } 
        
        // else if (!isDetectYellowLine && !isDetectWhiteLine) {
        //     // ctlDxlBack();
        // }

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
    if (yellow_line_x_ <= -0.5 && white_line_x_ >= 0.5) { // 직진 주행
        ctlDxlFront(5, 3);
    } else if (((0.15 >= yellow_line_x_) && (yellow_line_x_ >= -0.5))) { // 우회전 
        // 우회전 예외처리 추가해도 됨(가속)
        ctlDxlRight(7, 3);
    } else if (((0.95 >= white_line_x_) && (white_line_x_ >= 0.35))) { // 좌회전
        // 우회전 예외처리 추가해도 됨(가속)
        ctlDxlLeft(7, 3);
    }
}

// ========== [Line Detect 서브스크라이브] ==========
void MasterNode::detectYellowLine(const std_msgs::msg::Bool::SharedPtr msg) {
    isDetectYellowLine = msg->data;
    if (isDetectYellowLine) {
        isDetectYellowLine = true;
        RCLCPP_INFO(node->get_logger(), "노란색 라인 감지");
        // ctlDxlRight();
    } else {
        isDetectYellowLine = false;
    }
}

void MasterNode::detectWhiteLine(const std_msgs::msg::Bool::SharedPtr msg) {
    isDetectWhiteLine = msg->data;
    if (isDetectWhiteLine) {
        isDetectWhiteLine = true;
        RCLCPP_INFO(node->get_logger(), "흰색 라인 감지");
        // ctlDxlLeft();
    } else {
        isDetectWhiteLine = false;
    }
}

void MasterNode::getYellowLineX(const std_msgs::msg::Float32::SharedPtr msg) {
    yellow_line_x_ = msg->data;
    // RCLCPP_INFO(node->get_logger(), "value: %.2f", yellow_line_x_);
}

void MasterNode::getWhiteLineX(const std_msgs::msg::Float32::SharedPtr msg) {
    white_line_x_ = msg->data;
    // RCLCPP_INFO(node->get_logger(), "value: %.2f", white_line_x_);
}





// ========== [Dxl Control 메서드] ==========
void MasterNode::ctlDxlFront(int linearVel, int angularVel) {
    if (isRobotRun_) {
        auto msg_linear_ = std_msgs::msg::Int32();
        auto msg_angular_ = std_msgs::msg::Int32();

        // msg_linear_.data = 10;
        // msg_angular_.data = 0;
        msg_linear_.data = linear_vel_ + linearVel;
        msg_angular_.data = 0;

        pub_dxl_linear_vel_->publish(msg_linear_);
        pub_dxl_angular_vel_->publish(msg_angular_);
        RCLCPP_INFO(node->get_logger(), "직진 이동");
    }
}

void MasterNode::ctlDxlLeft(int linearVel, int angularVel) {
    if (isRobotRun_) {
        auto msg_linear_ = std_msgs::msg::Int32();
        auto msg_angular_ = std_msgs::msg::Int32();

        // msg_linear_.data = 2;
        // msg_angular_.data = 5;
        msg_linear_.data = linear_vel_ + linearVel;
        msg_angular_.data = angular_vel_ + angularVel;

        pub_dxl_linear_vel_->publish(msg_linear_); // 퍼블리시
        pub_dxl_angular_vel_->publish(msg_angular_);
        RCLCPP_INFO(node->get_logger(), "왼쪽으로 이동");
    }
}


void MasterNode::ctlDxlRight(int linearVel, int angularVel) {
    if (isRobotRun_) {
        auto msg_linear_ = std_msgs::msg::Int32();
        auto msg_angular_ = std_msgs::msg::Int32();

        // msg_linear_.data = 2;
        // msg_angular_.data = -5;
        msg_linear_.data = linear_vel_ + linearVel;
        msg_angular_.data = -angular_vel_ + (-angularVel);


        pub_dxl_linear_vel_->publish(msg_linear_); // 퍼블리시
        pub_dxl_angular_vel_->publish(msg_angular_);
        RCLCPP_INFO(node->get_logger(), "오른쪽으로 이동");
    }
}

void MasterNode::ctlDxlBack(int linearVel, int angularVel) {
    if (isRobotRun_) {
        auto msg_linear_ = std_msgs::msg::Int32();
        auto msg_angular_ = std_msgs::msg::Int32();

        // msg_linear_.data = -10;
        // msg_angular_.data = 0;
        msg_linear_.data = -linear_vel_ + (-linearVel);
        msg_angular_.data = 0;

        pub_dxl_linear_vel_->publish(msg_linear_);
        pub_dxl_angular_vel_->publish(msg_angular_);
        RCLCPP_INFO(node->get_logger(), "후진 이동");
    }
}


void MasterNode::stopDxl() {
    isRobotRun_ = !isRobotRun_;

    auto msg_linear_ = std_msgs::msg::Int32();
    auto msg_angular_ = std_msgs::msg::Int32();

    msg_linear_.data = 0;
    msg_angular_.data = 0;

    pub_dxl_linear_vel_->publish(msg_linear_);
    pub_dxl_angular_vel_->publish(msg_angular_);
}

void MasterNode::updateDxlData(int linearVel, int angularVel) {
    linear_vel_ = linearVel;
    angular_vel_ = angularVel;
}

void MasterNode::runDxl(int linearVel, int angularVel) {

    auto msg_linear_ = std_msgs::msg::Int32();
    auto msg_angular_ = std_msgs::msg::Int32();

    msg_linear_.data = linearVel;
    msg_angular_.data = angularVel;

    pub_dxl_linear_vel_->publish(msg_linear_);
    pub_dxl_angular_vel_->publish(msg_angular_);
}