#ifndef TCPIP_APPLICATION_HPP_EUIC2VFR
#define TCPIP_APPLICATION_HPP_EUIC2VFR

#include "autoware_v2x/application.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_v2x/positioning.hpp"
#include <vanetza/asn1/cpm.hpp>

namespace v2x {
  class TcpIpApplication {
    public:
      TcpIpApplication(V2XNode *node);
      void sendViaLTE(boost::asio::ip::tcp::socket *socket, boost::asio::steady_timer *t);
      void on_connect(const boost::system::error_code& error, boost::asio::ip::tcp::socket *socket);
      void writeToLTE(boost::asio::ip::tcp::socket *socket);
      void onSendToLTE(const boost::system::error_code& error, size_t bytes_transferred, boost::asio::ip::tcp::socket *socket);
      void start();

      void startReceiver();
      void acceptViaLTE(boost::asio::ip::tcp::socket *socket, boost::asio::ip::tcp::acceptor *acceptor);
      void on_accept(const boost::system::error_code& error, boost::asio::ip::tcp::socket *socket, boost::asio::ip::tcp::acceptor *acceptor);
      void start_receive(boost::asio::ip::tcp::socket *socket, boost::asio::ip::tcp::acceptor *acceptor);
      void on_receive(const boost::system::error_code& error, size_t bytes_transferred, boost::asio::ip::tcp::socket *socket, boost::asio::ip::tcp::acceptor *acceptor);

    private:
      V2XNode *node_;
      boost::asio::streambuf receive_buff_;

      int CPM_LTE_INTERVAL_;
  };
}

#endif /* TCPIP_APPLICATION_HPP_EUIC2VFR */