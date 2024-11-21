#ifndef udp_connection_JETSON_WINDOW_H
#define udp_connection_JETSON_WINDOW_H

#include <QMainWindow>
#include <QNetworkInterface>
#include <QString>
#include "QIcon"
#include "../qnode.hpp"
#include "ui_jetsonwindow.h"
#include "../include/udp_connection/jetson/relay_node.hpp"

class JetsonWindow : public QMainWindow
{
  Q_OBJECT

public:
  JetsonWindow(std::shared_ptr<RelayNode> relayNode, QWidget* parent = nullptr);
  ~JetsonWindow();
  QNode* qnode;

private slots:
  void onConnectToTargetDeviceClicked();
  void onSendButtonClicked();

private:
  Ui::JetsonWindowDesign* ui;

  std::shared_ptr<RelayNode> relayNode_;

  void closeEvent(QCloseEvent* event);
  void updateConnectionStatus(bool connected);  // 연결 상태 업데이트 메서드
  QString getLocalIPAddress();
};

#endif  // udp_connection_JETSON_WINDOW_H
