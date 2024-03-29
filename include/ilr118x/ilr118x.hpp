#ifndef ILR118X_ILR118X
#define ILR118X_ILR118X

#include <limits>
#include <string>

#include <boost/asio/buffer.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include <ilr118x_msgs/Output.h>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

namespace ilr118x {

namespace ba = boost::asio;
namespace bs = boost::system;
namespace rn = ros::names;
namespace rp = ros::param;

class ILR118x {
public:
  ILR118x(ba::io_service &service, ros::NodeHandle &handle, const std::string &ns = "~")
      : serial_(service), timer_(service) {
    // load parameters
    device_ = rp::param< std::string >(rn::append(ns, "device"), "/dev/ttyUSB0");
    timeout_ = ros::Duration(rp::param(rn::append(ns, "timeout"), 3.)).toBoost();
    print_response_ = rp::param(rn::append(ns, "print_response"), false);
    frame_id_ = rp::param< std::string >(rn::append(ns, "frame_id"), "ilr118x");

    // create a ros topic
    output_publisher_ = handle.advertise< ilr118x_msgs::Output >("output", 1);
    range_publisher_ = handle.advertise< sensor_msgs::Range >("range", 1);

    // start the operation
    start();
  }

  virtual ~ILR118x() {}

private:
  void start() {
    try {
      // close the device if already opened
      if (serial_.is_open()) {
        serial_.close();
      }

      // open the device
      serial_.open(device_);

      // set options
      serial_.set_option(ba::serial_port::baud_rate(9600));
      serial_.set_option(ba::serial_port::flow_control(ba::serial_port::flow_control::none));
      serial_.set_option(ba::serial_port::parity(ba::serial_port::parity::none));
      serial_.set_option(ba::serial_port::stop_bits(ba::serial_port::stop_bits::one));
      serial_.set_option(ba::serial_port::character_size(8));
    } catch (const bs::system_error &error) {
      // restart on error
      ROS_ERROR_STREAM("On starting operation: " << error.what());
      restart();
      return;
    }

    // clear the read buffer
    buffer_.consume(buffer_.size());

    // start writing the first request
    startWriteFlushRequest();
  }

private:
  void startWriteFlushRequest() {
    // write an invalied command to flush the buffer on the device
    timer_.expires_from_now(timeout_);
    timer_.async_wait(boost::bind(&ILR118x::handleTimeout, this, _1));
    ba::async_write(serial_, ba::buffer("XX\r", 3),
                    boost::bind(&ILR118x::handleWriteFlushRequest, this, _1));
  }

  void handleWriteFlushRequest(const bs::error_code &error) {
    timer_.cancel();

    if (error) {
      ROS_ERROR_STREAM("On writing flush request: " << error.message());
      restart();
      return;
    }

    startReadFlushResponse();
  }

  void startReadFlushResponse() {
    // receive a response indicating an invalid command
    timer_.expires_from_now(timeout_);
    timer_.async_wait(boost::bind(&ILR118x::handleTimeout, this, _1));
    ba::async_read_until(serial_, buffer_, "E61\r\n",
                         boost::bind(&ILR118x::handleReadFlushResponse, this, _1, _2));
  }

  void handleReadFlushResponse(const bs::error_code &error, const std::size_t bytes) {
    timer_.cancel();

    if (error) {
      ROS_ERROR_STREAM("On reading flush response: " << error.message());
      restart();
      return;
    }

    if (print_response_) {
      ROS_INFO_STREAM(
          " > " << std::string(ba::buffer_cast< const char * >(buffer_.data()), bytes - 2));
    }

    buffer_.consume(bytes);

    startWriteResetRequest();
  }

  void startWriteResetRequest() {
    timer_.expires_from_now(timeout_);
    timer_.async_wait(boost::bind(&ILR118x::handleTimeout, this, _1));
    ba::async_write(serial_, ba::buffer("PR\r", 3),
                    boost::bind(&ILR118x::handleWriteResetRequest, this, _1));
  }

  void handleWriteResetRequest(const bs::error_code &error) {
    timer_.cancel();

    if (error) {
      ROS_ERROR_STREAM("On writing reset request: " << error.message());
      restart();
      return;
    }

    startReadResetResponse();
  }

  void startReadResetResponse(const std::size_t count = 16) {
    timer_.expires_from_now(timeout_);
    timer_.async_wait(boost::bind(&ILR118x::handleTimeout, this, _1));
    ba::async_read_until(serial_, buffer_, "\r\n",
                         boost::bind(&ILR118x::handleReadResetResponse, this, count, _1, _2));
  }

  void handleReadResetResponse(const std::size_t count, const bs::error_code &error,
                               const std::size_t bytes) {
    timer_.cancel();

    if (error) {
      ROS_ERROR_STREAM("On reading reset response: " << error.message());
      restart();
      return;
    }

    if (print_response_) {
      ROS_INFO_STREAM(
          " > " << std::string(ba::buffer_cast< const char * >(buffer_.data()), bytes - 2));
    }

    buffer_.consume(bytes);

    const std::size_t next_count(count - 1);
    if (next_count > 0) {
      startReadResetResponse(next_count);
    } else {
      startWriteStreamRequest();
    }
  }

  void startWriteStreamRequest() {
    timer_.expires_from_now(timeout_);
    timer_.async_wait(boost::bind(&ILR118x::handleTimeout, this, _1));
    ba::async_write(serial_, ba::buffer("DX\r", 3),
                    boost::bind(&ILR118x::handleWriteStreamRequest, this, _1));
  }

  void handleWriteStreamRequest(const bs::error_code &error) {
    timer_.cancel();

    if (error) {
      ROS_ERROR_STREAM("On writing stream request: " << error.message());
      restart();
      return;
    }

    startReadStreamResponse();
  }

  void startReadStreamResponse() {
    timer_.expires_from_now(timeout_);
    timer_.async_wait(boost::bind(&ILR118x::handleTimeout, this, _1));
    ba::async_read_until(serial_, buffer_, "\r\n",
                         boost::bind(&ILR118x::handleReadStreamResponse, this, _1, _2));
  }

  void handleReadStreamResponse(const bs::error_code &error, const std::size_t bytes) {
    timer_.cancel();

    if (error) {
      ROS_ERROR_STREAM("On reading stream response: " << error.message());
      restart();
      return;
    }

    // parse the response
    {
      // copy the response from the buffer except the delimiters
      const std::string response(ba::buffer_cast< const char * >(buffer_.data()), bytes - 2);

      // print the response string if needed
      if (print_response_) {
        ROS_INFO_STREAM(" > " << response);
      }

      // publish the response in a raw format if needed
      if (output_publisher_.getNumSubscribers() > 0) {
        // convert the reponse to a ros message
        ilr118x_msgs::Output output;
        output.header.stamp = ros::Time::now();
        output.header.frame_id = frame_id_;
        output.distance = std::numeric_limits< float >::quiet_NaN();
        output.error_code = ilr118x_msgs::Output::UNKNOWN_ERROR;
        if (boost::conversion::try_lexical_convert(response, output.distance)) {
          output.error_code = ilr118x_msgs::Output::SUCCESS;
        } else if (response == "E15") {
          output.error_code = ilr118x_msgs::Output::POOR_REFLEXES;
        } else if (response == "E16") {
          output.error_code = ilr118x_msgs::Output::STRONG_REFLEXES;
        } else if (response == "E17") {
          output.error_code = ilr118x_msgs::Output::STEADY_LIGHT;
        } else if (response == "E18") {
          output.error_code = ilr118x_msgs::Output::DISTANCE_DIFFERENCE;
        } else if (response == "E19") {
          output.error_code = ilr118x_msgs::Output::HIGH_TARGET_SPEED;
        } else if (response == "E23") {
          output.error_code = ilr118x_msgs::Output::LOW_TEMPERATURE;
        } else if (response == "E24") {
          output.error_code = ilr118x_msgs::Output::HIGH_TEMPERATURE;
        } else if (response == "E31") {
          output.error_code = ilr118x_msgs::Output::EEPROM_CHECKSUM;
        } else if (response == "E51") {
          output.error_code = ilr118x_msgs::Output::SET_LASER_VOLTAGE;
        } else if (response == "E52") {
          output.error_code = ilr118x_msgs::Output::LASER_CURRENT;
        } else if (response == "E53") {
          output.error_code = ilr118x_msgs::Output::BAD_PARAMETERS;
        } else if (response == "E54") {
          output.error_code = ilr118x_msgs::Output::HARDWARE_54;
        } else if (response == "E55") {
          output.error_code = ilr118x_msgs::Output::HARDWARE_55;
        } else if (response == "E61") {
          output.error_code = ilr118x_msgs::Output::INVALID_COMMAND;
        } else if (response == "E62") {
          output.error_code = ilr118x_msgs::Output::HARDWARE_OR_PARITY;
        } else if (response == "E63") {
          output.error_code = ilr118x_msgs::Output::BUFFER_OVERFLOW;
        } else if (response == "E64") {
          output.error_code = ilr118x_msgs::Output::FRAMING_ERROR;
        }

        // publish the message
        output_publisher_.publish(output);
      }

      // publish the response in common format if needed
      if (range_publisher_.getNumSubscribers() > 0) {
        sensor_msgs::Range range;
        // headers
        range.header.stamp = ros::Time::now();
        range.header.frame_id = frame_id_;
        // specs of the device from
        // http://www.micro-epsilon.com/download/manuals/man--optoNCDT-ILR-1181-1182--en.pdf
        range.radiation_type = sensor_msgs::Range::INFRARED;
        range.field_of_view = 0.0006; // 0.6 mrad
        range.min_range = 0.;         // 0 m
        range.max_range = 150.;       // 150 m
        // measurement
        range.range = std::numeric_limits< float >::quiet_NaN();
        boost::conversion::try_lexical_convert(response, range.range);

        range_publisher_.publish(range);
      }
    }

    // remove the parsed response from the buffer
    buffer_.consume(bytes);

    // receive the next response
    startReadStreamResponse();
  }

  void handleTimeout(const bs::error_code &error) {
    if (error == ba::error::operation_aborted) {
      return;
    }

    if (error) {
      ROS_ERROR_STREAM("On waiting timeout: " << error.message());
      return;
    }

    serial_.cancel();
  }

  void restart() {
    timer_.expires_from_now(timeout_);
    timer_.async_wait(boost::bind(&ILR118x::handleRestart, this, _1));
  }

  void handleRestart(const bs::error_code &error) {
    if (error == ba::error::operation_aborted) {
      return;
    }

    if (error) {
      ROS_ERROR_STREAM("On waiting restart: " << error.message());
      return;
    }

    start();
  }

private:
  // parameters
  std::string device_;
  ba::deadline_timer::duration_type timeout_;
  bool print_response_;
  std::string frame_id_;

  // sensor drivers
  ba::serial_port serial_;
  ba::streambuf buffer_;
  ba::deadline_timer timer_;

  // ros publishers
  ros::Publisher output_publisher_;
  ros::Publisher range_publisher_;
};
}

#endif // ILR118X_ILR118X