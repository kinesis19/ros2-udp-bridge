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