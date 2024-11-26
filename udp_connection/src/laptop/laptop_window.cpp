#include "../include/udp_connection/laptop/laptop_window.hpp"

LaptopWindow::LaptopWindow(std::shared_ptr<MasterNode> masterNode, std::shared_ptr<VisionNode> visionNode, std::shared_ptr<PsdManagerNode> psdManagerNode, std::shared_ptr<ImuNode> imuNode, std::shared_ptr<DxlLeftNode> dxlLeftNode, std::shared_ptr<DxlRightNode> dxlRightNode, QWidget* parent) : QMainWindow(parent), ui(new Ui::LaptopWindowDesign), masterNode_(masterNode), visionNode_(visionNode), psdManagerNode_(psdManagerNode), imuNode_(imuNode), dxlLeftNode_(dxlLeftNode), dxlRightNode_(dxlRightNode), isReceiveAddress_(false)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode("laptop_qnode");

  // ========== [Now Device의 IP 주소 설정] ==========
  QString ipAddress = getLocalIPAddress();
  ui->labelNowDeviceIPAddress->setText("IP Address: " + ipAddress);

  connect(ui->btnConnectToTargetDevice, &QPushButton::clicked, this, &LaptopWindow::onConnectToTargetDeviceClicked);
  connect(ui->btnConnectCheckSend, &QPushButton::clicked, this, &LaptopWindow::onSendButtonClicked);
  connect(ui->btnRobotStart, &QPushButton::clicked, this, &LaptopWindow::onStopButtonClicked);
  connect(ui->btnDeployDxl, &QPushButton::clicked, this, &LaptopWindow::onDeployButtonClicked);

  // ========== [Node 초기화 상태 확인] ==========
  checkAllNodeInitializeCondition();

  connect(qnode, &QNode::messageReceived, this, [this](const QString& msg) {
    if (msg == "PONG") {
      updateConnectionStatus(true);  // 연결 성공 시 업데이트
    } else {
      ui->labelConnectCheckReceiveData->setText(msg);  // 일반 메시지는 출력
    }
  });

  // ========== [VisionNode의 시그널과 LaptopWindow의 슬롯 연결] ==========
  connect(visionNode_.get(), &VisionNode::imageReceived, this, [this](const QPixmap &pixmapOriginal, const QPixmap &pixmapDetected, const QPixmap &pixmapYellowMask, const QPixmap &pixmapWhiteMask) {
    ui->labelImageOriginal->setPixmap(pixmapOriginal.scaled(ui->labelImageOriginal->size(), Qt::KeepAspectRatio));
    ui->labelImageProcessed->setPixmap(pixmapDetected.scaled(ui->labelImageProcessed->size(), Qt::KeepAspectRatio));
    ui->labelImageYelloLine->setPixmap(pixmapYellowMask.scaled(ui->labelImageYelloLine->size(), Qt::KeepAspectRatio));
    ui->labelImageWhiteLine->setPixmap(pixmapWhiteMask.scaled(ui->labelImageWhiteLine->size(), Qt::KeepAspectRatio));
  });

  // ========== [PsdManagerNode의 시그널과 LaptopWindow의 슬롯 연결] ==========
  connect(masterNode_.get(), &MasterNode::stmPsdRightReceived, this, [this](const int &psdRight) {
    ui->lcdNumberPSDDataRight->setDigitCount(4); // 표시할 자리수 설정
    ui->lcdNumberPSDDataRight->display(psdRight); // 값 출력
  });

  connect(masterNode_.get(), &MasterNode::stmPsdFrontReceived, this, [this](const int &psdFront) {
    ui->lcdNumberPSDDataFront->setDigitCount(4);
    ui->lcdNumberPSDDataFront->display(psdFront);
  });

  connect(masterNode_.get(), &MasterNode::stmPsdLeftReceived, this, [this](const int &psdLeft) {
    ui->lcdNumberPSDDataLeft->setDigitCount(4);
    ui->lcdNumberPSDDataLeft->display(psdLeft);
  });

  // ========== [로봇 원격 제어 버튼 슬롯 연결] ==========
  connect(ui->btnMoveFront, &QPushButton::clicked, this, &LaptopWindow::onMoveFrontButtonClicked);
  connect(ui->btnMoveBack, &QPushButton::clicked, this, &LaptopWindow::onMoveBackButtonClicked);
  connect(ui->btnMoveLeft, &QPushButton::clicked, this, &LaptopWindow::onMoveLeftButtonClicked);
  connect(ui->btnMoveRight, &QPushButton::clicked, this, &LaptopWindow::onMoveRightButtonClicked);
  connect(ui->btnMoveStop, &QPushButton::clicked, this, &LaptopWindow::onMoveStopButtonClicked);

  // ========== [IMU 버튼 슬롯 연결] ==========
  connect(ui->btnIMUReset, &QPushButton::clicked, this, &LaptopWindow::onImuResetButtonClicked);

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

// Connect 버튼 클릭 시 실행되는 메서드
void LaptopWindow::onConnectToTargetDeviceClicked() {
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



void LaptopWindow::onSendButtonClicked() {
  QString message = ui->lineEditConnectCheckSendData->text();
  if (!message.isEmpty() && isReceiveAddress_) {
    qnode->sendUDPMessage(message.toStdString());
  }
}

// ========== [Node 초기화 상태 확인 메서드] ==========
void LaptopWindow::checkAllNodeInitializeCondition() {
  checkMasterNodeInitializeCondition();
  checkVisionNodeInitializeCondition();
  checkPsdManagerNodeInitializeCondition();
  checkImuNodeInitializeCondition();
  checkDxlLeftNodeInitializeCondition();
  checkDxlRightNodeInitializeCondition();
}

void LaptopWindow::checkMasterNodeInitializeCondition() {
  if (masterNode_->isInitialized()) {
    ui->labelConditionMasterNode->setText("Enable");
    ui->labelConditionMasterNode->setStyleSheet("color: green;");
  } else {
    ui->labelConditionMasterNode->setText("Disable");
    ui->labelConditionMasterNode->setStyleSheet("color: red;");
  }
}

void LaptopWindow::checkVisionNodeInitializeCondition() {
  if (visionNode_->isInitialized()) {
    ui->labelConditionVisionNode->setText("Enable");
    ui->labelConditionVisionNode->setStyleSheet("color: green;");
  } else {
    ui->labelConditionVisionNode->setText("Disable");
    ui->labelConditionVisionNode->setStyleSheet("color: red;");
  }
}

void LaptopWindow::checkPsdManagerNodeInitializeCondition() {
  if (psdManagerNode_->isInitialized()) {
    ui->labelConditionPsdManagerNode->setText("Enable");
    ui->labelConditionPsdManagerNode->setStyleSheet("color: green;");
  } else {
    ui->labelConditionPsdManagerNode->setText("Disable");
    ui->labelConditionPsdManagerNode->setStyleSheet("color: red;");
  }
}

void LaptopWindow::checkImuNodeInitializeCondition() {
  if (imuNode_->isInitialized()) {
    ui->labelConditionImuNode->setText("Enable");
    ui->labelConditionImuNode->setStyleSheet("color: green;");
  } else {
    ui->labelConditionImuNode->setText("Disable");
    ui->labelConditionImuNode->setStyleSheet("color: red;");
  }
}

void LaptopWindow::checkDxlLeftNodeInitializeCondition() {
  if (dxlLeftNode_->isInitialized()) {
    ui->labelConditionDxlLeftNode->setText("Enable");
    ui->labelConditionDxlLeftNode->setStyleSheet("color: green;");
  } else {
    ui->labelConditionDxlLeftNode->setText("Disable");
    ui->labelConditionDxlLeftNode->setStyleSheet("color: red;");
  }
}

void LaptopWindow::checkDxlRightNodeInitializeCondition() {
  if (dxlRightNode_->isInitialized()) {
    ui->labelConditionDxlRightNode->setText("Enable");
    ui->labelConditionDxlRightNode->setStyleSheet("color: green;");
  } else {
    ui->labelConditionDxlRightNode->setText("Disable");
    ui->labelConditionDxlRightNode->setStyleSheet("color: red;");
  }
}

void LaptopWindow::onStopButtonClicked() {
  if (ui->btnRobotStart->text() == "Robot Stop") {
    ui->btnRobotStart->setText("Robot Start");
  } else {
    ui->btnRobotStart->setText("Robot Stop");
  }
  masterNode_->stopDxl();
}

void LaptopWindow::onDeployButtonClicked() {
  int linear_vel_ = ui->spinBoxLinearVel->value();
  int angular_vel_ = ui->spinBoxAngularVel->value();
  masterNode_->updateDxlData(linear_vel_, angular_vel_);
}


// ========== [로봇 원격 제어 메서드] ==========
void LaptopWindow::onMoveFrontButtonClicked() {
  masterNode_->runDxl(5, 0);
}

void LaptopWindow::onMoveBackButtonClicked() {
  masterNode_->runDxl(-5, 0);
}

void LaptopWindow::onMoveLeftButtonClicked() {
  masterNode_->runDxl(2, 5);
}

void LaptopWindow::onMoveRightButtonClicked() {
  masterNode_->runDxl(2, -5);
}

void LaptopWindow::onMoveStopButtonClicked() {
  // masterNode_->runDxl(0, 0);
}


// ========== [IMU 초기화 메서드] ==========
void LaptopWindow::onImuResetButtonClicked() {
  imuNode_->resetAngles();
}