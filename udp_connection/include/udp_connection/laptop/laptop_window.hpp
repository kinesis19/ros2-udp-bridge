#ifndef udp_connection_LAPTOP_WINDOW_H
#define udp_connection_LAPTOP_WINDOW_H

#include <QMainWindow>
#include <QNetworkInterface>
#include <QString>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <QDebug>
#include <QImage>
#include <QPixmap>
#include "QIcon"
#include "../qnode.hpp"
#include "ui_laptopwindow.h"

#include "../include/udp_connection/laptop/vision_node.hpp"
#include "../include/udp_connection/laptop/psd_manager_node.hpp"
#include "../include/udp_connection/laptop/imu_node.hpp"
#include "../include/udp_connection/laptop/dxl_left_node.hpp"
#include "../include/udp_connection/laptop/dxl_right_node.hpp"
#include "../include/udp_connection/laptop/master_node.hpp"

class LaptopWindow : public QMainWindow
{
  Q_OBJECT

public:
  LaptopWindow(std::shared_ptr<MasterNode> masterNode, std::shared_ptr<VisionNode> visionNode, std::shared_ptr<PsdManagerNode> psdManagerNode, std::shared_ptr<ImuNode> imuNode, std::shared_ptr<DxlLeftNode> dxlLeftNode, std::shared_ptr<DxlRightNode> dxlRightNode, QWidget* parent = nullptr);
  ~LaptopWindow();

  QNode* qnode;

private slots:
  void onConnectToTargetDeviceClicked();
  void onSendButtonClicked();

private:
  Ui::LaptopWindowDesign* ui;

  std::shared_ptr<MasterNode> masterNode_;
  std::shared_ptr<VisionNode> visionNode_;
  std::shared_ptr<PsdManagerNode> psdManagerNode_;
  std::shared_ptr<ImuNode> imuNode_;
  std::shared_ptr<DxlLeftNode> dxlLeftNode_;
  std::shared_ptr<DxlRightNode> dxlRightNode_;

  bool isReceiveAddress_;

  void closeEvent(QCloseEvent* event);
  void updateConnectionStatus(bool connected);  // 연결 상태 업데이트 메서드
  QString getLocalIPAddress(); // 현재 디바이스의 IP 주소 가져오는 메서드

  void checkAllNodeInitializeCondition(); // 전체 노드 초기화 상태 학인 메서드
  void checkMasterNodeInitializeCondition(); // Master 노드 초기화 상태 학인 메서드
  void checkVisionNodeInitializeCondition(); // Vision 노드 초기화 상태 학인 메서드
  void checkPsdManagerNodeInitializeCondition(); // PsdManager 노드 초기화 상태 학인 메서드
  void checkImuNodeInitializeCondition(); // Imu 노드 초기화 상태 학인 메서드
  void checkDxlLeftNodeInitializeCondition(); // DxlLeft 노드 초기화 상태 학인 메서드
  void checkDxlRightNodeInitializeCondition(); // DxlRight 노드 초기화 상태 학인 메서드

};

#endif  // udp_connection_LAPTOP_WINDOW_H
