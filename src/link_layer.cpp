#include "autoware_v2x/link_layer.hpp"
#ifdef BUILD_COHDA
#include "autoware_v2x/cohda_link.hpp"
#endif
#include "autoware_v2x/cube_evk_link.hpp"
#include "autoware_v2x/raw_socket_link.hpp"

#include <vanetza/access/ethertype.hpp>

#include <boost/asio/generic/raw_protocol.hpp>

std::unique_ptr<LinkLayer> create_link_layer(boost::asio::io_service &io_service, const std::string &name, const EthernetDevice &target_device)
{
  std::unique_ptr<LinkLayer> link_layer;

  boost::asio::generic::raw_protocol raw_protocol(
    AF_PACKET, vanetza::access::ethertype::GeoNetworking.net());
  boost::asio::generic::raw_protocol::socket raw_socket(io_service, raw_protocol);
  raw_socket.bind(target_device.endpoint(AF_PACKET));
  if (name == "ethernet") {
    link_layer.reset(new RawSocketLink{std::move(raw_socket)});
  }
#ifdef BUILD_COHDA
  else if (name == "cohda") {
    link_layer.reset(new CohdaLink{std::move(raw_socket)});
  }
#endif
  else {
    throw std::runtime_error("Unknown link layer: " + name);
  }

  return link_layer;
}

std::unique_ptr<LinkLayer> create_link_layer(boost::asio::io_service &io_service, const std::string &name, const std::string &target_device)
{
  std::unique_ptr<LinkLayer> link_layer;

  if (name == "cube-evk") {
    link_layer.reset(new CubeEvkLink{ io_service, boost::asio::ip::address_v4::from_string(target_device) });
  } else {
    throw std::runtime_error("Unknown link layer: " + name);
  }

  return link_layer;
}