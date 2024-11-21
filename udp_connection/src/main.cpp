#include <QApplication>
#include <iostream>
#include <memory>

#include "../include/udp_connection/laptop/laptop_window.hpp"
#include "../include/udp_connection/jetson/jetson_window.hpp"
#include "../include/udp_connection/laptop/master_node.hpp"
#include "../include/udp_connection/laptop/vision_node.hpp"
#include "../include/udp_connection/laptop/psd_manager_node.hpp"
#include "../include/udp_connection/laptop/imu_node.hpp"
#include "../include/udp_connection/laptop/dxl_left_node.hpp"
#include "../include/udp_connection/laptop/dxl_right_node.hpp"
#include "../include/udp_connection/jetson/relay_node.hpp"


template <typename T>
void cleanupThread(T*& thread) {
  if (thread) {
    thread->quit();
    thread->wait();
    delete thread;
    thread = nullptr;
  }
}

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);

  // ROS2 초기화
  rclcpp::init(argc, argv);

  // 실행 인자를 통해 디바이스를 구분해서 GUI를 실행
  if (argc < 2) {
    qFatal("Usage: %s [laptop|jetson]", argv[0]);
  }

  // 실행 인자: "laptop" 또는 "jetson"
  QString deviceType = argv[1]; 

  QMainWindow* window = nullptr;
  
  std::shared_ptr<MasterNode> master_node = nullptr;
  std::shared_ptr<VisionNode> vision_node = nullptr;
  std::shared_ptr<PsdManagerNode> psd_manager_node = nullptr;
  std::shared_ptr<ImuNode> imu_node = nullptr;
  std::shared_ptr<DxlLeftNode> dxl_left_node = nullptr;
  std::shared_ptr<DxlRightNode> dxl_right_node = nullptr;
  std::shared_ptr<RelayNode> relay_node = nullptr;

  // Window 객체를 Heap으로 관리하도록 아키텍처를 설계
  if (deviceType == "laptop") {
    master_node = std::make_shared<MasterNode>();
    vision_node = std::make_shared<VisionNode>();
    psd_manager_node = std::make_shared<PsdManagerNode>();
    imu_node = std::make_shared<ImuNode>();
    dxl_left_node = std::make_shared<DxlLeftNode>();
    dxl_right_node = std::make_shared<DxlRightNode>();
    relay_node = std::make_shared<RelayNode>();
    
    // QThread 실행
    master_node->start();
    vision_node->start();
    psd_manager_node->start();
    imu_node->start();
    dxl_left_node->start();
    dxl_right_node->start();
    relay_node->start();

    window = new LaptopWindow(master_node, vision_node, psd_manager_node, imu_node, dxl_left_node, dxl_right_node, relay_node);
  } else if (deviceType == "jetson") {
    relay_node = std::make_shared<RelayNode>();
    relay_node->start();
    window = new JetsonWindow(relay_node);
  } else {
    qFatal("Invalid device type! Use 'laptop' or 'jetson'.");
  }

  // 선택한 인자에 대한 GUI show.
  if (window) {
    window->show();
  }

  int ret = a.exec();

  // ROS2 종료
  rclcpp::shutdown();

  delete window;
  return ret;
}


