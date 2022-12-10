#ifndef TCPIP_APPLICATION_HPP_EUIC2VFR
#define TCPIP_APPLICATION_HPP_EUIC2VFR

#include "autoware_v2x/application.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_v2x/positioning.hpp"
#include <vanetza/asn1/cpm.hpp>

namespace v2x {
  class TcpIpApplication {
    public:
      TcpIpApplication(V2XNode *node);

      void start();
      void sendViaLTE(boost::asio::steady_timer *t);
      void on_sent(boost::asio::steady_timer *t, const boost::system::error_code& error);

      void startReceiver();
      void start_receive(boost::asio::ip::udp::socket *socket);
      void handle_receive(boost::asio::ip::udp::socket *socket, const boost::system::error_code& error, size_t len);

    private:
      V2XNode *node_;
      boost::array<char,2048> receive_buff_;
      boost::asio::ip::udp::endpoint remote_endpoint_;
      boost::asio::ip::udp::endpoint remote_endpoint_sender_;
      std::shared_ptr<boost::asio::ip::udp::socket> socket_;
  };
}

#endif /* TCPIP_APPLICATION_HPP_EUIC2VFR */