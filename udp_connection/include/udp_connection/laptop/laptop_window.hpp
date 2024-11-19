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

private:
  Ui::LaptopWindowDesign* ui;

  void closeEvent(QCloseEvent* event);
  QString getLocalIPAddress();
};

#endif  // udp_connection_LAPTOP_WINDOW_H
