#include <QApplication>
#include <iostream>

#include "../include/udp_connection/laptop/laptop_window.hpp"
#include "../include/udp_connection/jetson/jetson_window.hpp"
#include "../include/udp_connection/jetson/relay_node.hpp"
#include "../include/udp_connection/laptop/vision_node.hpp"
#include "../include/udp_connection/laptop/psd_manager_node.hpp"
#include "../include/udp_connection/laptop/dxl_left_node.hpp"
#include "../include/udp_connection/laptop/dxl_right_node.hpp"
#include "../include/udp_connection/laptop/master_node.hpp"


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
  RelayNode* relay_node_ = nullptr;
  VisionNode* vision_node_ = nullptr;
  PsdManagerNode* psd_manager_node_ = nullptr;
  DxlLeftNode* dxl_left_node_ = nullptr;
  DxlRightNode* dxl_right_node_ = nullptr;
  MasterNode* master_node_ = nullptr;

  // Window 객체를 Heap으로 관리하도록 아키텍처를 설계
  if (deviceType == "laptop") {
    window = new LaptopWindow();
    vision_node_ = new VisionNode();
    psd_manager_node_ = new PsdManagerNode();
    dxl_left_node_ = new DxlLeftNode();
    dxl_right_node_ = new DxlRightNode();
    master_node_ = new MasterNode();

    vision_node_->start(); // QThread 실행
    psd_manager_node_->start();
    dxl_left_node_->start();
    dxl_right_node_->start();
    master_node_->start();

  } else if (deviceType == "jetson") {
    window = new JetsonWindow();
    relay_node_ = new RelayNode();
    relay_node_->start(); // QThread 실행
  } else {
    qFatal("Invalid device type! Use 'laptop' or 'jetson'.");
  }

  // 선택한 인자에 대한 GUI show.
  if (window) {
    window->show();
  }

  int ret = a.exec();

  /* Cleanup (Thread 메모리 누수 방지 및 Clean quit)
  * Template로 처리해서 간결하게 구현함
  */ 
  cleanupThread(relay_node_);
  cleanupThread(vision_node_);
  cleanupThread(psd_manager_node_);
  cleanupThread(dxl_left_node_);
  cleanupThread(dxl_right_node_);
  cleanupThread(master_node_);


  // ROS2 종료
  rclcpp::shutdown();

  delete window;
  return ret;
}


