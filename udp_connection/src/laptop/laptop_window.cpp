#include "../include/udp_connection/laptop/laptop_window.hpp"

LaptopWindow::LaptopWindow(std::shared_ptr<RelayNode> relayNode, std::shared_ptr<VisionNode> visionNode, std::shared_ptr<PsdManagerNode> psdManagerNode, std::shared_ptr<DxlLeftNode> dxlLeftNode, std::shared_ptr<DxlRightNode> dxlRightNode, std::shared_ptr<MasterNode> masterNode, QWidget* parent) : QMainWindow(parent), ui(new Ui::LaptopWindowDesign), relayNode_(relayNode), visionNode_(visionNode), psdManagerNode_(psdManagerNode), dxlLeftNode_(dxlLeftNode), dxlRightNode_(dxlRightNode), masterNode_(masterNode)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode("laptop_qnode");

  // Now Device의 IP 주소 설정
  QString ipAddress = getLocalIPAddress();
  ui->labelNowDeviceIPAddress->setText("IP Address: " + ipAddress);

  connect(ui->btnConnectToTargetDevice, &QPushButton::clicked, this, &LaptopWindow::onConnectToTargetDeviceClicked);
  connect(ui->btnConnectCheckSend, &QPushButton::clicked, this, &LaptopWindow::onSendButtonClicked);

  connect(qnode, &QNode::messageReceived, this, [this](const QString& msg) {
    if (msg == "PONG") {
      updateConnectionStatus(true);  // 연결 성공 시 업데이트
    } else {
      ui->labelConnectCheckReceiveData->setText(msg);  // 일반 메시지는 출력
    }
  });

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

QString LaptopWindow::getLocalIPAddress() {
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

void LaptopWindow::updateConnectionStatus(bool connected) {
  if (connected) {
    ui->labelTargetDeviceCondition->setText("Connected!");
    ui->labelTargetDeviceCondition->setStyleSheet("color: green;");
  } else {
    ui->labelTargetDeviceCondition->setText("Disconnected");
    ui->labelTargetDeviceCondition->setStyleSheet("color: red;");
  }
}

void LaptopWindow::onConnectToTargetDeviceClicked() {
  QString ipAddress = ui->lineEditTargetDeviceIPAddress->text();
  if (!ipAddress.isEmpty()) {
    qnode->setReceiverIPAddress(ipAddress.toStdString());

    // PING 메시지 전송
    qnode->sendUDPMessage("Connected Message");

    updateConnectionStatus(false);
  } else {
    qnode->sendUDPMessage("Disconnected Message");
    updateConnectionStatus(false);
  }
}


void LaptopWindow::onSendButtonClicked() {
  QString message = ui->lineEditConnectCheckSendData->text();
  if (!message.isEmpty()) {
    qnode->sendUDPMessage(message.toStdString());
  }
}