#include <QApplication>
#include <iostream>

#include "../include/udp_connection/laptop/laptop_window.hpp"
// #include "../include/udp_connection/jetson/jetson_window.hpp"

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);

  // 실행 인자를 통해 디바이스를 구분.
  if (argc < 2) {
    qFatal("Usage: %s [laptop|jetson]", argv[0]);
  }

  QString deviceType = argv[1]; // 실행 인자: "laptop" 또는 "jetson"

  LaptopWindow lw;
  // JetsonUI jw;

  if (deviceType == "laptop") {
    lw.show();
  } else if (deviceType == "jetson") {
    // jw.show();
  } else {
    qFatal("Invalid device type! Use 'laptop' or 'jetson'.");
  }

  return a.exec();
}
