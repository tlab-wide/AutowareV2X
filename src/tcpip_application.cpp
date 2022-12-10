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
    node_(node),
    remote_endpoint_sender_(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address("127.0.0.1"), 2009))
  {}

  void TcpIpApplication::start() {
    boost::asio::io_service io_service;
    socket_ = std::make_shared<boost::asio::ip::udp::socket>(io_service, remote_endpoint_sender_);
    // socket_sender_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::make_address("127.0.0.1"), 2009));
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    boost::asio::steady_timer t(io_service, boost::asio::chrono::milliseconds(300));
    t.async_wait(boost::bind(&TcpIpApplication::sendViaLTE, this, &t));
    io_service.run();
  }

  void TcpIpApplication::startReceiver() {
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 2009));
    start_receive(&socket);
    io_service.run();
  }

  void TcpIpApplication::start_receive(boost::asio::ip::udp::socket *socket) {
    std::cout << "[start_receive]" << std::endl;
    socket->async_receive_from(
      boost::asio::buffer(receive_buff_),
      remote_endpoint_,
      boost::bind(&TcpIpApplication::handle_receive, this,
        socket,
        boost::asio::placeholders::error, _2));
  }

  void TcpIpApplication::handle_receive(boost::asio::ip::udp::socket *socket, const boost::system::error_code& error, size_t len) {
    if (!error || error == boost::asio::error::message_size){
      std::cout << remote_endpoint_.address() << ":" << remote_endpoint_.port() << std::endl ;
      // const char* data = boost::asio::buffer_cast<const char*>(receive_buff_.data());
      // const char* data = boost::asio::buffer_cast<const char*>(receive_buff_.data());
      // std::cout << receive_buff_ << std::endl;
      // node_->cpm_received_lte_.decode(receive_buff_.data().data(), receive_buff_.data().size());
      node_->cpm_received_lte_.decode(boost::asio::buffer(receive_buff_).data(), boost::asio::buffer(receive_buff_).size());
      node_->cpm_app->indicateLTE();
      // boost::asio::buffer(receive_buff_).consume(boost::asio::buffer(receive_buff_).size());
      start_receive(socket);
    }
  }
  

  void TcpIpApplication::sendViaLTE(boost::asio::steady_timer *t) {

    RCLCPP_INFO(node_->get_logger(), "[sendViaLTE]");
  
    socket_->async_send_to(
      boost::asio::buffer(node_->cpm_.encode(), (node_->cpm_.encode()).size()), 
      remote_endpoint_sender_,
      std::bind(&TcpIpApplication::on_sent, this, t, std::placeholders::_1));
  }

  void TcpIpApplication::on_sent(boost::asio::steady_timer *t, const boost::system::error_code& error) {
    if (error) {
      std::cout << "sending CPM(LTE) failed: " << error.message().c_str() << std::endl;
    }

    t->expires_at(t->expiry() + boost::asio::chrono::milliseconds(300));
    t->async_wait(boost::bind(&TcpIpApplication::sendViaLTE, this, t));

  }

}