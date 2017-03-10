#ifndef ILR118X_ILR118X
#define ILR118X_ILR118X

#include <iostream>
#include <stdexcept>
#include <string>

#include <boost/asio/buffer.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include <ros/console.h>

namespace ilr118x {

class ILR118x {
public:
  ILR118x(boost::asio::io_service &service, const std::string &device)
      : serial_(service), timer_(service), device_(device) {}

  virtual ~ILR118x() {}

  void start() {
    try {
      // close the device if already opened
      if (serial_.is_open()) {
        serial_.close();
      }

      // open the device
      serial_.open(device_);

      // set options
      typedef boost::asio::serial_port Serial;
      serial_.set_option(Serial::baud_rate(9600));
      serial_.set_option(Serial::flow_control(Serial::flow_control::none));
      serial_.set_option(Serial::parity(Serial::parity::none));
      serial_.set_option(Serial::stop_bits(Serial::stop_bits::one));
      serial_.set_option(Serial::character_size(8));
    } catch (const boost::system::system_error &error) {
      // restart on error
      ROS_ERROR_STREAM("On starting operation: " << error.what());
      start();
      return;
    }

    // clear the read buffer
    read_buffer_.consume(read_buffer_.size());

    // enable timeout
    enableTimer();

    // start writing the first request
    startWriteFlushRequest();
  }

private:
  void startWriteFlushRequest() {
    // write an invalied command to flush the buffer on the device
    startTimer();
    boost::asio::async_write(serial_, boost::asio::buffer("XX\r", 3),
                             boost::bind(&ILR118x::handleWriteFlushRequest, this, _1));
  }

  void handleWriteFlushRequest(const boost::system::error_code &error) {
    if (error) {
      ROS_ERROR_STREAM("On writing flush request: " << error.message());
      start();
      return;
    }

    startReadFlushResponse();
  }

  void startReadFlushResponse() {
    // receive a response indicating an invalid command
    startTimer();
    boost::asio::async_read_until(serial_, read_buffer_, "E61\r\n",
                                  boost::bind(&ILR118x::handleReadFlushResponse, this, _1, _2));
  }

  void handleReadFlushResponse(const boost::system::error_code &error, const std::size_t bytes) {
    if (error) {
      std::cerr << "On reading flush response: " << error.message() << std::endl;
      start();
      return;
    }

    /*
    {
      const char *const data(boost::asio::buffer_cast< const char * >(read_buffer_.data()));
      const std::size_t size(bytes - 2);
      ROS_INFO_STREAM(" > " << std::string(data, size));
    }
    */

    read_buffer_.consume(bytes);

    startWriteResetRequest();
  }

  void startWriteResetRequest() {
    startTimer();
    boost::asio::async_write(serial_, boost::asio::buffer("PR\r", 3),
                             boost::bind(&ILR118x::handleWriteResetRequest, this, _1));
  }

  void handleWriteResetRequest(const boost::system::error_code &error) {
    if (error) {
      ROS_ERROR_STREAM("On writing reset request: " << error.message());
      start();
      return;
    }

    startReadResetResponse();
  }

  void startReadResetResponse(const std::size_t count = 16) {
    startTimer();
    boost::asio::async_read_until(
        serial_, read_buffer_, "\r\n",
        boost::bind(&ILR118x::handleReadResetResponse, this, count, _1, _2));
  }

  void handleReadResetResponse(const std::size_t count, const boost::system::error_code &error,
                               const std::size_t bytes) {
    if (error) {
      std::cerr << "On reading reset response: " << error.message() << std::endl;
      start();
      return;
    }

    {
      const char *const data(boost::asio::buffer_cast< const char * >(read_buffer_.data()));
      const std::size_t size(bytes - 2);
      ROS_INFO_STREAM(" > " << std::string(data, size));
    }

    read_buffer_.consume(bytes);

    const std::size_t next_count(count - 1);
    if (next_count > 0) {
      startReadResetResponse(next_count);
    } else {
      startWriteStreamRequest();
    }
  }

  void startWriteStreamRequest() {
    startTimer();
    boost::asio::async_write(serial_, boost::asio::buffer("DX\r", 3),
                             boost::bind(&ILR118x::handleWriteStreamRequest, this, _1));
  }

  void handleWriteStreamRequest(const boost::system::error_code &error) {
    if (error) {
      std::cerr << "On writing stream request: " << error.message() << std::endl;
      start();
      return;
    }

    startReadStreamResponse();
  }

  void startReadStreamResponse() {
    startTimer();
    boost::asio::async_read_until(serial_, read_buffer_, "\r\n",
                                  boost::bind(&ILR118x::handleReadStreamResponse, this, _1, _2));
  }

  void handleReadStreamResponse(const boost::system::error_code &error, const std::size_t bytes) {
    if (error) {
      std::cerr << "On reading stream response: " << error.message() << std::endl;
      start();
      return;
    }

    // parse the response
    {
      // access the response string except the delimiter
      const char *const data(boost::asio::buffer_cast< const char * >(read_buffer_.data()));
      const std::size_t size(bytes - 2);
      // print the response string for debug
      ROS_INFO_STREAM(" > " << std::string(data, size));
      // convert the reponse string to a number
      double value;
      if (boost::conversion::try_lexical_convert(data, size, value)) {
        // TODO: publish the value
      } else {
        ROS_WARN_STREAM("On parsing stream response: "
                        << "bad conversion: " << std::string(data, size));
      }
    }

    // remove the parsed response from the buffer
    read_buffer_.consume(bytes);

    // receive the next response
    startReadStreamResponse();
  }

  void enableTimer() { timer_.async_wait(boost::bind(&ILR118x::handleTimer, this)); }

  void startTimer() { timer_.expires_from_now(boost::posix_time::seconds(10)); }

  void handleTimer() {
    if (timer_.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
      serial_.close();
      timer_.expires_at(boost::posix_time::pos_infin);
    }
    enableTimer();
  }

private:
  boost::asio::serial_port serial_;
  boost::asio::deadline_timer timer_;
  const std::string device_;

  boost::asio::streambuf read_buffer_;
};
}

#endif // ILR118X_ILR118X