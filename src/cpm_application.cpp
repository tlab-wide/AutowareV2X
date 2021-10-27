#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/link_layer.hpp"

#include "rclcpp/rclcpp.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/facilities/cpm_functions.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>
#include <exception>

// This is a very simple application that sends BTP-B messages with the content 0xc0ffee.

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;

CpmApplication::CpmApplication(rclcpp::Node *node) : node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "CpmApplication started...");
}

CpmApplication::PortType CpmApplication::port()
{
  return btp::ports::CPM;
}

void CpmApplication::indicate(const DataIndication &indication, UpPacketPtr packet)
{
  asn1::PacketVisitor<asn1::Cpm> visitor;
  std::shared_ptr<const asn1::Cpm> cpm = boost::apply_visitor(visitor, *packet);
  if (cpm) {
    RCLCPP_INFO(node_->get_logger(), "Received decodable CPM content");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Received broken content");
  }
  asn1::Cpm message = *cpm;
  ItsPduHeader_t &header = message->header;
  RCLCPP_INFO(node_->get_logger(), "cpm.ItsPduHeader.messageId = %d", header.messageID);
  RCLCPP_INFO(node_->get_logger(), "cpm.ItsPduHeader.stationId = %d", header.stationID);

  CpmManagementContainer_t &management = message->cpm.cpmParameters.managementContainer;
  double lat = management.referencePosition.latitude;
  double lon = management.referencePosition.longitude;
  RCLCPP_INFO(node_->get_logger(), "cpm.(reference position) = %f, %f", lat, lon);
}

void CpmApplication::send(const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr msg, rclcpp::Node *node, double pos_lat, double pos_lon)
{
  RCLCPP_INFO(node->get_logger(), "Sending CPM...");
  // Send all objects in 1 CPM message
  vanetza::asn1::Cpm message;

  // ITS PDU Header
  ItsPduHeader_t &header = message->header;
  header.protocolVersion = 1;
  header.messageID = 14;
  header.stationID = 1;

  const auto time_now = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  uint16_t gen_delta_time = time_now.count();

  CollectivePerceptionMessage_t &cpm = message->cpm;
  cpm.generationDeltaTime = gen_delta_time * GenerationDeltaTime_oneMilliSec;

  // auto position = positioning->position_specify(pos_lat, pos_lon);

  CpmManagementContainer_t& management = cpm.cpmParameters.managementContainer;
  management.stationType = StationType_passengerCar;
  // management.referencePosition.latitude = pos_lat;
  // management.referencePosition.longitude = pos_lon;
  // management.referencePosition.positionConfidenceEllipse.semiMajorConfidence = 1.0;
  // management.referencePosition.positionConfidenceEllipse.semiMinorConfidence = 1.0;
  PositionFix fix;
  // fix.timestamp = time_now;
  fix.latitude = pos_lat * units::degree;
  fix.longitude = pos_lon * units::degree;
  fix.confidence.semi_major = 1.0 * units::si::meter;
  fix.confidence.semi_minor = fix.confidence.semi_major;
  copy(fix, management.referencePosition);

  // cpm.cpmParameters.stationDataContainer = NULL;
  // cpm.cpmParameters.perceivedObjectContainer = NULL;
  cpm.cpmParameters.numberOfPerceivedObjects = msg->objects.size();

  StationDataContainer_t*& sdc = cpm.cpmParameters.stationDataContainer;
  sdc = vanetza::asn1::allocate<StationDataContainer_t>();
  // RCLCPP_INFO(node->get_logger(), "Allocated sdc");
  sdc->present = StationDataContainer_PR_originatingVehicleContainer;
  OriginatingVehicleContainer_t& ovc = sdc->choice.originatingVehicleContainer;
  ovc.speed.speedValue = 0;
  ovc.speed.speedConfidence = 1;
  ovc.heading.headingValue = 0;
  ovc.heading.headingConfidence = 1;

  PerceivedObjectContainer_t *&poc = cpm.cpmParameters.perceivedObjectContainer;
  poc = vanetza::asn1::allocate<PerceivedObjectContainer_t>();
  // cpm.cpmParameters.perceivedObjectContainer = poc;
  // PerceivedObjectContainer_t pocc = *poc;
  // RCLCPP_INFO(node->get_logger(), "Allocated poc");

  for (auto object : msg->objects) {
    PerceivedObject *pObj = vanetza::asn1::allocate<PerceivedObject>();
    pObj->objectID = 1;
    pObj->timeOfMeasurement = 10;
    pObj->xDistance.value = 100;
    pObj->xDistance.confidence = 1;
    pObj->yDistance.value = 200;
    pObj->yDistance.confidence = 1;
    pObj->xSpeed.value = 5;
    pObj->xSpeed.confidence = 1;
    pObj->ySpeed.value = 0;
    pObj->ySpeed.confidence = 1;
      
    ASN_SEQUENCE_ADD(poc, pObj);
    // RCLCPP_INFO(node->get_logger(), "Added one object to poc->list");
  }


  Application::DownPacketPtr packet{new DownPacket()};
  std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
  // std::shared_ptr<asn1::Cpm> message_p = std::make_shared<asn1::Cpm>(message);
  // std::unique_ptr<convertible::byte_buffer> buffer { new convertible::byte_buffer_impl<asn1::Cpm>(&message)};

  payload->layer(OsiLayer::Application) = std::move(message);

  Application::DataRequest request;
  request.its_aid = aid::CP;
  request.transport_type = geonet::TransportType::SHB;
  request.communication_profile = geonet::CommunicationProfile::ITS_G5;

  // RCLCPP_INFO(node->get_logger(), "Packet Size: %d", payload->size());

  // RCLCPP_INFO(node->get_logger(), "Going to Application::request...");
  Application::DataConfirm confirm = Application::request(request, std::move(payload), node);
  if (!confirm.accepted())
  {
    throw std::runtime_error("CPM application data request failed");
  }
  // RCLCPP_INFO(node->get_logger(), "Application::request END");
}

// void HelloApplication::schedule_timer()
// {
//     timer_.expires_from_now(interval_);
//     timer_.async_wait(std::bind(&HelloApplication::on_timer, this, std::placeholders::_1));
// }

// void HelloApplication::on_timer(const boost::system::error_code& ec)
// {
//     if (ec != boost::asio::error::operation_aborted) {
//         DownPacketPtr packet { new DownPacket() };
//         packet->layer(OsiLayer::Application) = ByteBuffer { 0xC0, 0xFF, 0xEE };
//         DataRequest request;
//         request.transport_type = geonet::TransportType::SHB;
//         request.communication_profile = geonet::CommunicationProfile::ITS_G5;
//         request.its_aid = aid::CA;
//         auto confirm = Application::request(request, std::move(packet));
//         if (!confirm.accepted()) {
//             throw std::runtime_error("Hello application data request failed");
//         }

//         schedule_timer();
//     }
// }
