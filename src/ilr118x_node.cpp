#include <boost/asio/io_service.hpp>

#include <ilr118x/ilr118x.hpp>

int main(int argc, char *argv[]) {

  boost::asio::io_service service;

  ilr118x::ILR118x device(service, "/dev/ttyUSB0");
  device.start();

  service.run();

  return 0;
}