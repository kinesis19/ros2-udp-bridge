#include "../include/udp_connection/jetson/jetson_window.hpp"

JetsonWindow::JetsonWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::JetsonWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  // qnode = new QNode();
  qnode = new QNode("jetson_qnode");

  // Now Device의 IP 주소 설정
  QString ipAddress = getLocalIPAddress();
  ui->labelNowDeviceIPAddress->setText("IP Address: " + ipAddress);

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


QString JetsonWindow::getLocalIPAddress() {
  QList<QHostAddress> addrList = QNetworkInterface::allAddresses();
  for (const QHostAddress &addr : addrList) {
      // IPv4 주소만 선택하고, 로컬호스트(127.0.0.1)는 제외
      if (addr.protocol() == QAbstractSocket::IPv4Protocol && addr != QHostAddress::LocalHost) {
        return addr.toString();
      }
  }
  // 유효한 IPv4 주소를 찾지 못한 경우
  return "Unknown";  
}
