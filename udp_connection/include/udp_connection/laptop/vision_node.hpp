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

struct RANSACLine
{
    cv::Point2f start;
    cv::Point2f end;
    float angle;
    float center_x;
    float center_y;
    int num_inliers;
    bool valid;
};

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
    bool yellow_line_detected;
    bool white_line_detected;
    int yellow_line_count;
    int white_line_count;
    bool blue_sign_detected; // 삼거리 표지판 감지 변수

    static const int ARRAY_SIZE = 10;
    static const int DETECTION_THRESHOLD = 7;
    std::array<bool, 10> yellow_detection_array;
    std::array<bool, 10> white_detection_array;
    int array_index;
    bool yellow_line_valid;
    bool white_line_valid;

    float yellow_line_x;
    float white_line_x;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    bool isLineValid(std::array<bool, 10> &detection_array, bool current_detection);
};

#endif // UDP_CONNECTION_VISION_HPP