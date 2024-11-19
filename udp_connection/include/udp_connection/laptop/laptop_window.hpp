#ifndef udp_connection_LAPTOP_WINDOW_H
#define udp_connection_LAPTOP_WINDOW_H

#include <QMainWindow>
#include <QNetworkInterface>
#include <QString>
#include "QIcon"
#include "../qnode.hpp"
#include "ui_laptopwindow.h"

class LaptopWindow : public QMainWindow
{
  Q_OBJECT

public:
  LaptopWindow(QWidget* parent = nullptr);
  ~LaptopWindow();

  QNode* qnode;

private slots:
  void onConnectToTargetDeviceClicked();
  void onSendButtonClicked();

private:
  Ui::LaptopWindowDesign* ui;

  void closeEvent(QCloseEvent* event);
  void updateConnectionStatus(bool connected);  // 연결 상태 업데이트 메서드
  QString getLocalIPAddress(); // 현재 디바이스의 IP 주소 가져오는 메서드
};

#endif  // udp_connection_LAPTOP_WINDOW_H
