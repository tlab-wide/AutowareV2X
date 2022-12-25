#include <iostream>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <string>

#include "autoware_v2x/v2x_node.hpp"
#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/tcpip_application.hpp"

namespace v2x {
  TcpIpApplication::TcpIpApplication(V2XNode *node) : 
    node_(node), CPM_LTE_INTERVAL_(500)
  {

  }

  void TcpIpApplication::start() {
    boost::asio::io_service io_service;

    boost::asio::ip::tcp::socket socket(io_service);

    boost::asio::steady_timer t(io_service, boost::asio::chrono::milliseconds(CPM_LTE_INTERVAL_));

    t.async_wait(boost::bind(&TcpIpApplication::sendViaLTE, this, &socket, &t));

    // sendViaLTE(&socket);

    io_service.run();

  }

  void TcpIpApplication::startReceiver() {
    boost::asio::io_service io_service;

    boost::asio::ip::tcp::socket socket(io_service);

    boost::asio::ip::tcp::acceptor acceptor(io_service,
      boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 6000));

    // t.async_wait(boost::bind(&TcpIpApplication::sendViaLTE, this, &socket, &t));

    // sendViaLTE(&socket);

    acceptViaLTE(&socket, &acceptor);

    io_service.run();
  }

  void TcpIpApplication::acceptViaLTE(boost::asio::ip::tcp::socket *socket, boost::asio::ip::tcp::acceptor *acceptor) {
    acceptor->async_accept(
      *socket,
      boost::bind(&TcpIpApplication::on_accept, 
                  this,
                  boost::asio::placeholders::error,
                  socket,
                  acceptor));
  }

  void TcpIpApplication::on_accept(const boost::system::error_code& error, boost::asio::ip::tcp::socket *socket, boost::asio::ip::tcp::acceptor *acceptor) {
    if (error) {
      std::cout << "accept failed: " << error.message() << std::endl;
      return;
    }

    start_receive(socket, acceptor);
  }

  void TcpIpApplication::start_receive(boost::asio::ip::tcp::socket *socket, boost::asio::ip::tcp::acceptor *acceptor) {
    boost::asio::async_read(
      *socket,
      receive_buff_,
      boost::asio::transfer_all(),
      boost::bind(&TcpIpApplication::on_receive, this,
                  boost::asio::placeholders::error, 
                  boost::asio::placeholders::bytes_transferred,
                  socket,
                  acceptor));
  }

  void TcpIpApplication::on_receive(const boost::system::error_code& error, size_t bytes_transferred,
    boost::asio::ip::tcp::socket *socket, boost::asio::ip::tcp::acceptor *acceptor) {
    if (error && error != boost::asio::error::eof) {
      std::cout << "receive failed: " << error.message() << std::endl;
    }
    else {
      const char* data = boost::asio::buffer_cast<const char*>(receive_buff_.data());
      std::cout << data << std::endl;

      // vanetza::ByteBuffer bbuffer{receive_buff_.data()};
      node_->cpm_received_lte_.decode(receive_buff_.data().data(), receive_buff_.data().size());
      node_->cpm_app->indicateLTE();

      receive_buff_.consume(receive_buff_.size());
    }

    socket->close();

    acceptor->async_accept(
      *socket,
      boost::bind(&TcpIpApplication::on_accept, 
                  this,
                  boost::asio::placeholders::error,
                  socket,
                  acceptor));

  }

  void TcpIpApplication::sendViaLTE(boost::asio::ip::tcp::socket *socket, boost::asio::steady_timer *t) {

    RCLCPP_INFO(node_->get_logger(), "[sendViaLTE]");
  
    // boost::asio::tcp::socket socket(io_service_);
    boost::system::error_code ec;

    // std::thread thrContext = std::thread([&]() { context_.run(); });

    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::make_address("10.0.1.3", ec), 6000);

    // boost::asio::ip::tcp::socket socket(context);

    // ASYNC: Connect to receiver_ip and run on_connect as callback
    // socket.async_connect(
    //   boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address("60.56.123.234", ec), 6000),
    //   boost::bind(&CpamSender::on_connect, this, boost::asio::placeholders::error)
    // );

    // socket.connect(endpoint, ec);

    // socket_ = socket_(context_);

    socket->async_connect(
      endpoint,
      boost::bind(&TcpIpApplication::on_connect, this, boost::asio::placeholders::error, socket));

    // context_.stop();

    // if (thrContext.joinable()) thrContext.join();

    t->expires_at(t->expiry() + boost::asio::chrono::milliseconds(CPM_LTE_INTERVAL_));
    t->async_wait(boost::bind(&TcpIpApplication::sendViaLTE, this, socket, t));

  }

  void TcpIpApplication::on_connect(const boost::system::error_code& error, boost::asio::ip::tcp::socket *socket) {
    RCLCPP_INFO(node_->get_logger(), "[on_connect]");
    if (error) {
      RCLCPP_INFO(node_->get_logger(), "LTE Connection fail\n%s", error.message().c_str());
      // std::cout << "connect failed : " << error.message() << std::endl;
      socket->close();
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "LTE Connected");
    writeToLTE(socket);
  }

  void TcpIpApplication::writeToLTE(boost::asio::ip::tcp::socket *socket) {
    // std::string data = "Hello world";
    std::cout << "writeToLTE" << std::endl;
    // vanetza::ByteBuffer data;
    // std::cout << "writeToLTE: " << data.size() << std::endl;
    // memcpy(data, *(node_->cpm_)->encode(), (*(node_->cpm_)->encode()).size());
    if (node_->cpm_initialized) {
      boost::asio::async_write(
      *socket, 
      // boost::asio::buffer(&data[0], data.size()), 
      boost::asio::buffer(node_->cpm_.encode(), (node_->cpm_.encode()).size()), 
      boost::bind(&TcpIpApplication::onSendToLTE, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred,
                  socket));
    } else {
      socket->close();
    }
   
  }

  void TcpIpApplication::onSendToLTE(const boost::system::error_code& error, size_t bytes_transferred, boost::asio::ip::tcp::socket *socket) {
    if (error) {
      std::cout << "send failed: " << error.message() << std::endl;
    } else {
      std::cout << "send correct!" << std::endl;
    }
    socket->close();
  }

}