#ifndef UDP_CONNECTION_VISION_HPP
#define UDP_CONNECTION_VISION_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
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
    bool initialized_; // 초기화 상태 확인 변수

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_original_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_yellow_mask_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_white_mask_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_line_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif // UDP_CONNECTION_VISION_HPP