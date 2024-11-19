#ifndef udp_connection_JETSON_WINDOW_H
#define udp_connection_JETSON_WINDOW_H

#include <QMainWindow>
#include "QIcon"
#include "../qnode.hpp"
#include "ui_jetsonwindow.h"

class JetsonWindow : public QMainWindow
{
  Q_OBJECT

public:
  JetsonWindow(QWidget* parent = nullptr);
  ~JetsonWindow();
  QNode* qnode;

private:
  Ui::JetsonWindowDesign* ui;
  void closeEvent(QCloseEvent* event);
};

#endif  // udp_connection_JETSON_WINDOW_H
