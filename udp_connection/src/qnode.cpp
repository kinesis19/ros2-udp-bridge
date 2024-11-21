#include "../include/udp_connection/qnode.hpp"

QNode::QNode(const std::string& nodeName) : udpSocket(new QUdpSocket(this)), receiverPort(12345)
{
  node = rclcpp::Node::make_shared(nodeName);
  this->start();

  setupSocket();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  // rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

// Socket 설정 메서드
void QNode::setupSocket() {
  udpSocket->bind(QHostAddress::Any, receiverPort);

  connect(udpSocket, &QUdpSocket::readyRead, this, [this]() {
    while (udpSocket->hasPendingDatagrams()) {
      QByteArray buffer;
      buffer.resize(udpSocket->pendingDatagramSize());
      QHostAddress sender;
      quint16 senderPort;

      udpSocket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);
      QString receivedMessage = QString::fromUtf8(buffer);
      emit messageReceived(receivedMessage);
    }
  });
}

// Receiver IP Address 설정 메서드
void QNode::setReceiverIPAddress(const std::string& ipAddress) {
  receiverAddress = QHostAddress(QString::fromStdString(ipAddress));
}

// UDP Message 전송 메서드
void QNode::sendUDPMessage(const std::string& message) {
  QByteArray data = QByteArray::fromStdString(message);
  udpSocket->writeDatagram(data, receiverAddress, receiverPort);
}

// 실제 주소인지 확인하는 메서드
void QNode::checkIPAddressReachability(const QString& ipAddress) {
  // QProcess를 사용하여 ping 실행
  QProcess* process = new QProcess(this);

  // ping 명령어하고 옵션 설정
  QString command = "ping";
  QStringList arguments = {"-c", "1", ipAddress};

  // ping 실행 완료 후 결과 처리
  connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this, [this, process](int exitCode, QProcess::ExitStatus) {
    // ping 명령어 결과 확인
    if (exitCode == 0) {
      qDebug() << "Ping successful!";
      emit pingResult(true); // 연결 성공
    } else {
      qDebug() << "Ping failed!";
      emit pingResult(false); // 연결 실패
    }
    process->deleteLater(); // QProcess 객체 삭제
  });

  // ping 명령 실행
  process->start(command, arguments);
}
