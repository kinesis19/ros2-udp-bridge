#include "../include/udp_connection/jetson/jetson_window.hpp"

JetsonWindow::JetsonWindow(std::shared_ptr<RelayNode> relayNode, QWidget* parent) : QMainWindow(parent), ui(new Ui::JetsonWindowDesign), relayNode_(relayNode), isReceiveAddress_(false)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  // qnode = new QNode();
  qnode = new QNode("jetson_qnode");

  // Now Device의 IP 주소 설정
  QString ipAddress = getLocalIPAddress();
  ui->labelNowDeviceIPAddress->setText("IP Address: " + ipAddress);

  connect(ui->btnConnectToTargetDevice, &QPushButton::clicked, this, &JetsonWindow::onConnectToTargetDeviceClicked);
  connect(ui->btnConnectCheckSend, &QPushButton::clicked, this, &JetsonWindow::onSendButtonClicked);

  connect(qnode, &QNode::messageReceived, this, [this](const QString& msg) {
    if (msg == "PONG") {
      updateConnectionStatus(true);  // 연결 성공 시 업데이트
    } else {
      ui->labelConnectCheckReceiveData->setText(msg);  // 일반 메시지는 출력
    }
  });

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

void JetsonWindow::updateConnectionStatus(bool connected) {
  if (connected) {
    ui->labelTargetDeviceCondition->setText("Connected!");
    ui->labelTargetDeviceCondition->setStyleSheet("color: green;");
  } else {
    ui->labelTargetDeviceCondition->setText("Disconnected");
    ui->labelTargetDeviceCondition->setStyleSheet("color: red;");
  }
}

// Connect 버튼 클릭 시 실행되는 메서드
void JetsonWindow::onConnectToTargetDeviceClicked() {
  QString ipAddress = ui->lineEditTargetDeviceIPAddress->text();
  if (!ipAddress.isEmpty()) {
    // Ping 명령어로 IP 주소 연결 상태 확인
    qnode->checkIPAddressReachability(ipAddress); // Ping 실행

    connect(qnode, &QNode::pingResult, this, [this, ipAddress](bool reachable) {
      if (reachable) {
        isReceiveAddress_ = true;
        updateConnectionStatus(true); // 연결 성공

        // Ping 성공 후 Receiver IP 설정 및 메시지 전송
        qnode->setReceiverIPAddress(ipAddress.toStdString());
        qnode->sendUDPMessage("Connected Message");
        
      } else {
        isReceiveAddress_ = false;
        updateConnectionStatus(false); // 연결 실패
        qnode->sendUDPMessage("Disconnected Message");
      }
    });

  } else {
    updateConnectionStatus(false);
    qnode->sendUDPMessage("No IP address provided!");
  }
}


void JetsonWindow::onSendButtonClicked() {
  QString message = ui->lineEditConnectCheckSendData->text();
  if (!message.isEmpty()) {
    qnode->sendUDPMessage(message.toStdString());
  }
}
