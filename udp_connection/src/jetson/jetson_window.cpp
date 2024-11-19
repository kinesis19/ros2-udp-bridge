#include "../include/udp_connection/jetson/jetson_window.hpp"

JetsonWindow::JetsonWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::JetsonWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  // qnode = new QNode();
  qnode = new QNode("jetson_qnode");

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void JetsonWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

JetsonWindow::~JetsonWindow()
{
  delete ui;
}
