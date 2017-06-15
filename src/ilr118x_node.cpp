#include <csignal>

#include <boost/asio/io_service.hpp>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <ilr118x/ilr118x.hpp>

boost::asio::io_service service;

void handleSigint(int) {
  service.stop();
  std::signal(SIGINT, handleSigint);
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "ilr118x_node", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, handleSigint);

  ros::NodeHandle handle;
  ilr118x::ILR118x device(service, handle);

  service.run();

  return 0;
}