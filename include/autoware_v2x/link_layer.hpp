#ifndef LINK_LAYER_HPP_FGEK0QTH
#define LINK_LAYER_HPP_FGEK0QTH

#include "autoware_v2x/ethernet_device.hpp"
#include <vanetza/access/interface.hpp>
#include <vanetza/net/cohesive_packet.hpp>
#include <vanetza/net/ethernet_header.hpp>
#include <boost/asio/io_service.hpp>
#include <memory>
#include <string>

class LinkLayerIndication
{
public:
    using IndicationCallback = std::function<void(vanetza::CohesivePacket&&, const vanetza::EthernetHeader&)>;

    virtual void indicate(IndicationCallback) = 0;
    virtual ~LinkLayerIndication() = default;
};

class LinkLayer : public vanetza::access::Interface, public LinkLayerIndication
{
};

std::unique_ptr<LinkLayer> create_link_layer(boost::asio::io_service &io_service, const std::string &name, const EthernetDevice &target_device);
std::unique_ptr<LinkLayer> create_link_layer(boost::asio::io_service &io_service, const std::string &name, const std::string &target_device);

#endif /* LINK_LAYER_HPP_FGEK0QTH */
