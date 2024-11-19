#include <QApplication>
#include <iostream>

#include "../include/udp_connection/laptop/laptop_window.hpp"
#include "../include/udp_connection/jetson/jetson_window.hpp"

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);

  // ROS2 초기화
  rclcpp::init(argc, argv);

  // 실행 인자를 통해 디바이스를 구분해서 GUI를 실행.
  if (argc < 2) {
    qFatal("Usage: %s [laptop|jetson]", argv[0]);
  }

  // 실행 인자: "laptop" 또는 "jetson"
  QString deviceType = argv[1]; 

  QMainWindow* window = nullptr;

  // Window 객체를 Heap으로 관리하도록 아키텍처를 설계.
  if (deviceType == "laptop") {
    window = new LaptopWindow();
  } else if (deviceType == "jetson") {
    window = new JetsonWindow();
  } else {
    qFatal("Invalid device type! Use 'laptop' or 'jetson'.");
  }

  // 선택한 인자에 대한 GUI show.
  if (window) {
    window->show();
  }

  int ret = a.exec();

  // ROS 2 종료
  rclcpp::shutdown();

  delete window;
  return ret;
}
