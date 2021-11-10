#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/v2x_node.hpp"

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
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;

namespace v2x
{
  CpmApplication::CpmApplication(V2XNode *node, Runtime &rt) :     
    node_(node),
    runtime_(rt),
    ego_x_(0),
    ego_y_(0),
    ego_lat_(0),
    ego_lon_(0),
    ego_altitude_(0),
    ego_heading_(0),
    generationDeltaTime_(0),
    updating_objects_stack_(false),
    sending_(false)
  {
    RCLCPP_INFO(node_->get_logger(), "CpmApplication started...");
    set_interval(milliseconds(1000));
  }

  void CpmApplication::set_interval(Clock::duration interval)
  {
    cpm_interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
  }

  void CpmApplication::schedule_timer()
  {
    runtime_.schedule(cpm_interval_, std::bind(&CpmApplication::on_timer, this, std::placeholders::_1), this);
  }

  void CpmApplication::on_timer(Clock::time_point)
  {
    schedule_timer();
    send();
  }

  CpmApplication::PortType CpmApplication::port()
  {
    return btp::ports::CPM;
  }

  void CpmApplication::indicate(const DataIndication &indication, UpPacketPtr packet)
  {
    asn1::PacketVisitor<asn1::Cpm> visitor;
    std::shared_ptr<const asn1::Cpm> cpm = boost::apply_visitor(visitor, *packet);
    if (cpm)
    {
      RCLCPP_INFO(node_->get_logger(), "Received decodable CPM content");
      asn1::Cpm message = *cpm;
      ItsPduHeader_t &header = message->header;

      CpmManagementContainer_t &management = message->cpm.cpmParameters.managementContainer;
      double lat = management.referencePosition.latitude / 1.0e7;
      double lon = management.referencePosition.longitude / 1.0e7;
      RCLCPP_INFO(node_->get_logger(), "cpm.(reference position) = %f, %f", lat, lon);

      std::string mgrs;
      int zone;
      bool northp;
      double x, y;
      GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
      GeographicLib::MGRS::Forward(zone, northp, x, y, lat, 5, mgrs);

      int x_mgrs = std::stoi(mgrs.substr(5, 5));
      int y_mgrs = std::stoi(mgrs.substr(10, 5));
      RCLCPP_INFO(node_->get_logger(), "cpm.(RP).mgrs = %s, %d, %d", mgrs.c_str(), x_mgrs, y_mgrs);

      // Calculate orientation from Heading
      OriginatingVehicleContainer_t &ovc = message->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer;
      int heading = ovc.heading.headingValue;
      double orientation = 1.5708 - (M_PI * heading / 10) / 180;
      RCLCPP_INFO(node_->get_logger(), "cpm: heading = %d, orientation = %f", heading, orientation);

      // Get PerceivedObjects
      receivedObjectsStack.clear();
      PerceivedObjectContainer_t *&poc = message->cpm.cpmParameters.perceivedObjectContainer;
      for (int i = 0; i < poc->list.count; ++i)
      {
        // RCLCPP_INFO(node_->get_logger(), "cpm: %d", poc->list.array[i]->objectID);
        CpmApplication::Object object;
        double x1 = poc->list.array[i]->xDistance.value;
        double y1 = poc->list.array[i]->yDistance.value;
        object.position_x = x_mgrs - (cos(orientation) * x1 - sin(orientation) * y1);
        object.position_y = y_mgrs - (sin(orientation) * x1 + cos(orientation) * y1);
        receivedObjectsStack.push_back(object);
      }
      node_->publishObjects(&receivedObjectsStack);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Received broken content");
    }
  }

  void CpmApplication::updateMGRS(double *x, double *y)
  {
    // RCLCPP_INFO(node_->get_logger(), "Update MGRS");
    ego_x_ = *x;
    ego_y_ = *y;
  }

  void CpmApplication::updateRP(double *lat, double *lon, double *altitude)
  {
    // RCLCPP_INFO(node_->get_logger(), "Update RP");
    ego_lat_ = *lat;
    ego_lon_ = *lon;
    ego_altitude_ = *altitude;
  }

  void CpmApplication::updateGenerationDeltaTime(int *gdt)
  {
    // RCLCPP_INFO(node_->get_logger(), "Update GDT");
    generationDeltaTime_ = *gdt;
  }

  void CpmApplication::updateHeading(double *yaw)
  {
    // RCLCPP_INFO(node_->get_logger(), "Update Heading : %f", *yaw);
    ego_heading_ = *yaw;
  }

  void CpmApplication::updateObjectsStack(const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr msg)
  {
    RCLCPP_INFO(node_->get_logger(), "Update ObjectsStack");
    updating_objects_stack_ = true;

    if (sending_)
    {
      RCLCPP_INFO(node_->get_logger(), "updateObjectsStack Skipped...");
      return;
    }

    if (msg->objects.size() > 0)
    {
      // RCLCPP_INFO(node_->get_logger(), "At least 1 object detected");

      // Initialize ObjectsStack
      // std::vector<CpmApplication::Object> objectsStack;
      objectsStack.clear();

      // Create CpmApplication::Object per msg->object and add it to objectsStack
      int i = 0;
      for (auto obj : msg->objects)
      {
        CpmApplication::Object object;
        object.objectID = i;
        object.timestamp = msg->header.stamp;
        object.position_x = obj.state.pose_covariance.pose.position.x; // MGRS
        object.position_y = obj.state.pose_covariance.pose.position.y;
        object.position_z = obj.state.pose_covariance.pose.position.z;
        object.orientation_x = obj.state.pose_covariance.pose.orientation.x;
        object.orientation_y = obj.state.pose_covariance.pose.orientation.y;
        object.orientation_z = obj.state.pose_covariance.pose.orientation.z;
        object.orientation_w = obj.state.pose_covariance.pose.orientation.w;
        object.xDistance = (int)((object.position_x - ego_x_) * cos(-ego_heading_) - (object.position_y - ego_y_) * sin(-ego_heading_)) * 100;
        object.yDistance = (int)((object.position_x - ego_x_) * sin(-ego_heading_) + (object.position_y - ego_y_) * cos(-ego_heading_)) * 100;
        // RCLCPP_INFO(node_->get_logger(), "xDistance: %d, yDistance: %d", object.xDistance, object.yDistance);
        object.timeOfMeasurement = 100;
        objectsStack.push_back(object);
        ++i;
      }
    }
    RCLCPP_INFO(node_->get_logger(), "%d objects added to objectsStack", objectsStack.size());
    updating_objects_stack_ = false;
  }

  void CpmApplication::send()
  {
    sending_ = true;
    if (!updating_objects_stack_ && objectsStack.size() > 0)
    {
      RCLCPP_INFO(node_->get_logger(), "Sending CPM...");

      // Send all objects in 1 CPM message
      vanetza::asn1::Cpm message;

      // ITS PDU Header
      ItsPduHeader_t &header = message->header;
      header.protocolVersion = 1;
      header.messageID = 14;
      header.stationID = 1;

      // const auto time_now = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
      // uint16_t gen_delta_time = time_now.count();

      CollectivePerceptionMessage_t &cpm = message->cpm;
      cpm.generationDeltaTime = generationDeltaTime_ * GenerationDeltaTime_oneMilliSec;

      // auto position = positioning->position_specify(pos_lat, pos_lon);

      CpmManagementContainer_t &management = cpm.cpmParameters.managementContainer;
      management.stationType = StationType_passengerCar;
      // management.referencePosition.latitude = pos_lat;
      // management.referencePosition.longitude = pos_lon;
      // management.referencePosition.positionConfidenceEllipse.semiMajorConfidence = 1.0;
      // management.referencePosition.positionConfidenceEllipse.semiMinorConfidence = 1.0;
      PositionFix fix;
      // fix.timestamp = time_now;
      fix.latitude = ego_lat_ * units::degree;
      fix.longitude = ego_lon_ * units::degree;
      // fix.altitude = ego_altitude_;
      fix.confidence.semi_major = 1.0 * units::si::meter;
      fix.confidence.semi_minor = fix.confidence.semi_major;
      copy(fix, management.referencePosition);

      // cpm.cpmParameters.stationDataContainer = NULL;
      // cpm.cpmParameters.perceivedObjectContainer = NULL;
      cpm.cpmParameters.numberOfPerceivedObjects = objectsStack.size();

      StationDataContainer_t *&sdc = cpm.cpmParameters.stationDataContainer;
      sdc = vanetza::asn1::allocate<StationDataContainer_t>();
      // RCLCPP_INFO(node->get_logger(), "Allocated sdc");
      sdc->present = StationDataContainer_PR_originatingVehicleContainer;
      OriginatingVehicleContainer_t &ovc = sdc->choice.originatingVehicleContainer;
      ovc.speed.speedValue = 0;
      ovc.speed.speedConfidence = 1;
      // ovc.heading.headingValue = (int) (1.5708 - ego_heading_) * M_PI / 180;
      // RCLCPP_INFO(node_->get_logger(), "headingValue...");
      ovc.heading.headingValue = (int)std::fmod((1.5708 - ego_heading_) * 180 / M_PI, 360.0) * 10;
      ovc.heading.headingConfidence = 1;

      // RCLCPP_INFO(node_->get_logger(), "PerceviedObjectContainer...");
      PerceivedObjectContainer_t *&poc = cpm.cpmParameters.perceivedObjectContainer;
      poc = vanetza::asn1::allocate<PerceivedObjectContainer_t>();

      for (CpmApplication::Object object : objectsStack)
      {
        PerceivedObject *pObj = vanetza::asn1::allocate<PerceivedObject>();
        pObj->objectID = object.objectID;
        pObj->timeOfMeasurement = object.timeOfMeasurement;
        pObj->xDistance.value = object.xDistance;
        pObj->xDistance.confidence = 1;
        pObj->yDistance.value = object.yDistance;
        pObj->yDistance.confidence = 1;
        pObj->xSpeed.value = object.xSpeed;
        pObj->xSpeed.confidence = 1;
        pObj->ySpeed.value = object.ySpeed;
        pObj->ySpeed.confidence = 1;

        ASN_SEQUENCE_ADD(poc, pObj);
        // RCLCPP_INFO(node_->get_logger(), "ADD: %d, %d, %d, %d, %d, %d", pObj->objectID, pObj->timeOfMeasurement, pObj->xDistance.value, pObj->yDistance.value, pObj->xSpeed.value, pObj->ySpeed.value);
      }
      Application::DownPacketPtr packet{new DownPacket()};
      std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};
      // std::shared_ptr<asn1::Cpm> message_p = std::make_shared<asn1::Cpm>(message);
      // std::unique_ptr<convertible::byte_buffer> buffer { new convertible::byte_buffer_impl<asn1::Cpm>(&message)};

      payload->layer(OsiLayer::Application) = std::move(message);

      Application::DataRequest request;
      request.its_aid = aid::CP;
      request.transport_type = geonet::TransportType::SHB;
      request.communication_profile = geonet::CommunicationProfile::ITS_G5;
      // RCLCPP_INFO(node->get_logger(), "Packet Size: %d", payload->size());
      // RCLCPP_INFO(node->get_logger(), "Going to Application::request...");
      Application::DataConfirm confirm = Application::request(request, std::move(payload), node_);
      if (!confirm.accepted())
      {
        throw std::runtime_error("CPM application data request failed");
      }
    }
    sending_ = false;
    // RCLCPP_INFO(node->get_logger(), "Application::request END");
  }
}