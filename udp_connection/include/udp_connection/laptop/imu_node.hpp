#ifndef UDP_CONNECTION_IMU_HPP
#define UDP_CONNECTION_IMU_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>
#include <sstream>


class KalmanFilter {
private:
    float Q, R, P, X, K;
    bool first_run;
public:
    KalmanFilter(float q = 0.1, float r = 0.1) : Q(q), R(r), first_run(true) {}
    float update(float measurement);
    float getState() const { return X; }
};

class ImuNode : public QThread
{
    Q_OBJECT

public:
    ImuNode();
    ~ImuNode();
    bool isInitialized() const; // 초기화 상태 확인 메서드
    void resetAngles();  // yaw 각도 초기화 함수

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_imu_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_roll_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_pitch_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    // LowPassFilter roll_lpf{0.1}, pitch_lpf{0.1}, yaw_lpf{0.1};
    // KalmanFilter roll_kf{0.1, 0.1}, pitch_kf{0.1, 0.1}, yaw_kf{0.1, 0.1};

    bool initialized_; // 초기화 상태 확인 변수

    // yaw 오프셋 변수 추가
    double yaw_offset = 0.0;
    
    KalmanFilter roll_kf;
    KalmanFilter pitch_kf;
    KalmanFilter yaw_kf;


    void imuCallback(const std_msgs::msg::String::SharedPtr msg);
    void processImuData(const std::vector<double>& data);
    void normalizeAngle(double &angle);
    void quaternionToEuler(const double qx, const double qy, const double qz, const double qw, double &roll, double &pitch, double &yaw);
    
    // 서비스 콜백 함수 추가
    void handleResetSignal(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

};

#endif // UDP_CONNECTION_IMU_HPP

