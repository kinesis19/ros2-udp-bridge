#include "../include/udp_connection/laptop/laptop_window.hpp"

LaptopWindow::LaptopWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::LaptopWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void LaptopWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

LaptopWindow::~LaptopWindow()
{
  delete ui;
}
