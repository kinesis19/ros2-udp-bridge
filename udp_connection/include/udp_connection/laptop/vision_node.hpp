#ifndef UDP_CONNECTION_VISION_HPP
#define UDP_CONNECTION_VISION_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <array>  // std::array를 위해 추가
#include <QImage>
#include <QPixmap>

class VisionNode : public QThread
{
    Q_OBJECT

public:
    VisionNode();
    ~VisionNode();
    bool isInitialized() const; // 초기화 상태 확인 메서드

signals:
    void imageReceived(const QPixmap &pixmapOriginal, const QPixmap &pixmapDetected, const QPixmap &pixmapYellowMask, const QPixmap &pixmapWhiteMask);

protected:
    void run() override;

private:
    rclcpp::Node::SharedPtr node;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_original_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_yellow_mask_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_white_mask_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_line_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_yellow_detected_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_white_detected_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yellow_pos_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_white_pos_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_blue_sign_detected_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_white_line_points_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_yellow_line_points_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yellow_angle_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_white_angle_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_barrier_detected_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_barrier_yellow_line_detected_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_barrier_white_line_detected_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_barrier_yellow_line_angle_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_barrier_white_line_angle_;



    bool initialized_; // 초기화 상태 확인 변수
    bool barrier_detected;
    bool yellow_line_detected; // 노란 선 감지
    bool white_line_detected; // 흰 선 감지
    int yellow_line_count; // 노란선 카운트
    int white_line_count; // 흰 선 카운트
    bool blue_sign_detected; // 삼거리 표지판 감지 변수
    int array_index; // 배열 숫자(라인을 실제로 감지했는지, 배열에 저장해서 계속 유지되는 값인지 확인)
    bool yellow_line_valid; // 배열 검사에 따른 노란선 유효값
    bool white_line_valid; // 배열 검사에 따른 흰선 유효값
    float yellow_line_x; // 노란선 x좌표
    float white_line_x; // 흰선의 x좌표 두 좌표를 통해 선의 각도 산출


    static const int ARRAY_SIZE = 10;
    static const int DETECTION_THRESHOLD = 7;
    std::array<bool, 10> yellow_detection_array;
    std::array<bool, 10> white_detection_array;


    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    bool isLineValid(std::array<bool, 10> &detection_array, bool current_detection);
};

#endif // UDP_CONNECTION_VISION_HPP