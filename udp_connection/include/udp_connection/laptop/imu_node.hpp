#ifndef UDP_CONNECTION_IMU_HPP
#define UDP_CONNECTION_IMU_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <sstream>


class LowPassFilter {
private:
    float alpha;
    float filtered_value;
    bool first_run;
public:
    LowPassFilter(float a = 0.1) : alpha(a), first_run(true) {}
    float update(float input);
};

class KalmanFilter {
private:
    float Q, R, P, X, K;
    bool first_run;
public:
    KalmanFilter(float q = 0.1, float r = 0.1) : Q(q), R(r), first_run(true) {}
    float update(float measurement);
};

class ImuNode : public QThread
{
    Q_OBJECT

public:
    ImuNode();
    ~ImuNode();
    bool isInitialized() const; // 초기화 상태 확인 메서드

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_imu_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_roll_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_pitch_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw_;

    LowPassFilter roll_lpf{0.1}, pitch_lpf{0.1}, yaw_lpf{0.1};
    KalmanFilter roll_kf{0.1, 0.1}, pitch_kf{0.1, 0.1}, yaw_kf{0.1, 0.1};

    bool initialized_; // 초기화 상태 확인 변수

    void imuCallback(const std_msgs::msg::String::SharedPtr msg);
    void processImuData(const std::vector<double>& data);
    void quaternionToEuler(const double q0, const double q1, const double q2, const double q3, double& roll, double& pitch, double& yaw);
};

#endif // UDP_CONNECTION_IMU_HPP

