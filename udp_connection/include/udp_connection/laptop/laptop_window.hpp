#ifndef udp_connection_LAPTOP_WINDOW_H
#define udp_connection_LAPTOP_WINDOW_H

#include <QMainWindow>
#include <QNetworkInterface>
#include <QString>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "QIcon"
#include "../qnode.hpp"
#include "ui_laptopwindow.h"

#include "../include/udp_connection/jetson/relay_node.hpp"
#include "../include/udp_connection/laptop/vision_node.hpp"
#include "../include/udp_connection/laptop/psd_manager_node.hpp"
#include "../include/udp_connection/laptop/dxl_left_node.hpp"
#include "../include/udp_connection/laptop/dxl_right_node.hpp"
#include "../include/udp_connection/laptop/master_node.hpp"

class LaptopWindow : public QMainWindow
{
  Q_OBJECT

public:
  LaptopWindow(std::shared_ptr<RelayNode> relayNode, std::shared_ptr<VisionNode> visionNode, std::shared_ptr<PsdManagerNode> psdManagerNode, std::shared_ptr<DxlLeftNode> dxlLeftNode, std::shared_ptr<DxlRightNode> dxlRightNode, std::shared_ptr<MasterNode> masterNode,QWidget* parent = nullptr);
  ~LaptopWindow();

  QNode* qnode;

private slots:
  void onConnectToTargetDeviceClicked();
  void onSendButtonClicked();

private:
  Ui::LaptopWindowDesign* ui;

  std::shared_ptr<RelayNode> relayNode_;
  std::shared_ptr<VisionNode> visionNode_;
  std::shared_ptr<PsdManagerNode> psdManagerNode_;
  std::shared_ptr<DxlLeftNode> dxlLeftNode_;
  std::shared_ptr<DxlRightNode> dxlRightNode_;
  std::shared_ptr<MasterNode> masterNode_;

  void closeEvent(QCloseEvent* event);
  void updateConnectionStatus(bool connected);  // 연결 상태 업데이트 메서드
  QString getLocalIPAddress(); // 현재 디바이스의 IP 주소 가져오는 메서드
};

#endif  // udp_connection_LAPTOP_WINDOW_H
