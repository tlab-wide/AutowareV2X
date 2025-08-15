#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/v2x_node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rclcpp/rclcpp.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>
#include <exception>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <string>

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>

#include <sqlite3.h>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace vanetza;
using namespace std::chrono;

namespace v2x {
  CpmApplication::CpmApplication(V2XNode *node, Runtime &rt, bool is_sender) :     
    node_(node),
    runtime_(rt),
    ego_(),
    generationTime_(0),
    updating_objects_list_(false),
    sending_(false),
    is_sender_(is_sender),
    reflect_packet_(false),
    objectConfidenceThreshold_(0.0),
    include_all_persons_and_animals_(false),
    cpm_num_(0),
    received_cpm_num_(0),
    cpm_object_id_(0),
    use_dynamic_generation_rules_(false)
  {
    RCLCPP_INFO(node_->get_logger(), "CpmApplication started. is_sender: %d", is_sender_);
    set_interval(milliseconds(100));
    createTables();
  }

  void CpmApplication::set_interval(Clock::duration interval) {
    cpm_interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
  }

  void CpmApplication::schedule_timer() {
    runtime_.schedule(cpm_interval_, std::bind(&CpmApplication::on_timer, this, std::placeholders::_1), this);
  }

  void CpmApplication::on_timer(Clock::time_point) {
    schedule_timer();
    send();
  }

  CpmApplication::PortType CpmApplication::port() {
    return btp::ports::CPM;
  }

  std::string CpmApplication::uuidToHexString(const unique_identifier_msgs::msg::UUID &id) {
    std::stringstream ss;
    for (auto i = 0; i < 16; ++i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
    }
    return ss.str();
  }

  void CpmApplication::indicate(const DataIndication &indication, UpPacketPtr packet) {

    asn1::PacketVisitor<asn1::Cpm> visitor;
    std::shared_ptr<const asn1::Cpm> cpm = boost::apply_visitor(visitor, *packet);

    if (cpm) {
      RCLCPP_INFO(node_->get_logger(), "[INDICATE] Received CPM #%d", received_cpm_num_);

      rclcpp::Time current_time = node_->now();
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] T_receive_r1 %ld", current_time.nanoseconds());

      asn1::Cpm message = *cpm;
      ItsPduHeader_t &header = message->header;

       std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
        std::chrono::system_clock::now().time_since_epoch()
      );
      node_->latency_log_file << "T_received," << header.stationID << "," << ms.count() << std::endl;


      // Calculate GDT and get GDT from CPM and calculate the "Age of CPM"
      GenerationDeltaTime_t gt_cpm = message->cpm.generationDeltaTime;
      // const auto time_now = duration_cast<milliseconds> (runtime_.now().time_since_epoch());
      // uint16_t gdt = time_now.count();
      // int gdt_diff = (65536 + (gdt - gdt_cpm) % 65536) % 65536;
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] GDT_CPM: %ld", gdt_cpm);
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] GDT: %u", gdt);
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] [measure] T_R1R2: %d", gdt_diff);


      CpmManagementContainer_t &management = message->cpm.cpmParameters.managementContainer;
      double lat = management.referencePosition.latitude / 1.0e7;
      double lon = management.referencePosition.longitude / 1.0e7;

      int zone;
      int grid_num_x = 4;
      int grid_num_y = 39;
      int grid_size = 100000;
      bool northp;
      double x, y;

      GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
      double x_mgrs = x - grid_num_x * grid_size;
      double y_mgrs = y - grid_num_y * grid_size;

      OriginatingVehicleContainer_t &ovc = message->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer;

      // Calculate ego-vehicle orientation (radians) from heading (degree).
      // orientation: True-East, counter-clockwise angle. (0.1 degree accuracy)
      int heading = ovc.heading.headingValue;
      double orientation = (90.0 - (double) heading / 10.0) * M_PI / 180.0;
      if (orientation < 0.0) orientation += (2.0 * M_PI);
      // double orientation = heading / 10.0;
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] heading: %d", heading);
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] orientation: %f", orientation);


      // Publish CPM Sender info to /v2x/cpm/sender through V2XNode function
      node_->publishCpmSenderObject(x_mgrs, y_mgrs, orientation);


      // Get PerceivedObjects
      receivedObjectsStack.clear();

      PerceivedObjectContainer_t *&poc = message->cpm.cpmParameters.perceivedObjectContainer;

      if (poc != NULL) {
        for (int i = 0; i < poc->list.count; ++i) {
          // RCLCPP_INFO(node_->get_logger(), "[INDICATE] Object: #%d", poc->list.array[i]->objectID);

          CpmApplication::Object object;
          double x1 = poc->list.array[i]->xDistance.value;
          double y1 = poc->list.array[i]->yDistance.value;
          x1 = x1 / 100.0;
          y1 = y1 / 100.0;
          object.position_x = x_mgrs + (cos(orientation) * x1 - sin(orientation) * y1);
          object.position_y = y_mgrs + (sin(orientation) * x1 + cos(orientation) * y1);          
          object.shape_x = poc->list.array[i]->planarObjectDimension2->value;
          object.shape_y = poc->list.array[i]->planarObjectDimension1->value;
          object.shape_z = poc->list.array[i]->verticalObjectDimension->value;

          object.yawAngle = poc->list.array[i]->yawAngle->value;
          double yaw_radian = (M_PI * object.yawAngle / 10.0) / 180.0;

          tf2::Quaternion quat;
          quat.setRPY(0, 0, yaw_radian);
          object.orientation_x = quat.x();
          object.orientation_y = quat.y();
          object.orientation_z = quat.z();
          object.orientation_w = quat.w();
          // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::indicate] object.quat: %f, %f, %f, %f", object.orientation_x, object.orientation_y, object.orientation_z, object.orientation_w);

          receivedObjectsStack.push_back(object);
        }
        node_->publishObjects(&receivedObjectsStack, header.stationID);
      } else {
        // RCLCPP_INFO(node_->get_logger(), "[INDICATE] Empty POC");
      }

      insertCpmToCpmTable(message, (char*) "cpm_received");

      if (reflect_packet_) {
        Application::DownPacketPtr packet{new DownPacket()};
        std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};

        payload->layer(OsiLayer::Application) = std::move(message);

        Application::DataRequest request;
        request.its_aid = aid::CP;
        request.transport_type = geonet::TransportType::SHB;
        request.communication_profile = geonet::CommunicationProfile::ITS_G5;

        Application::DataConfirm confirm = Application::request(request, std::move(payload), node_);

        if (!confirm.accepted()) {
          throw std::runtime_error("[CpmApplication::indicate] Packet reflection failed");
        }
      }

      ++received_cpm_num_;

    } else {
      RCLCPP_INFO(node_->get_logger(), "[INDICATE] Received broken content");
    }
  }

  void CpmApplication::updateMGRS(double *x, double *y) {
    ego_.mgrs_x = *x;
    ego_.mgrs_y = *y;
    // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::updateMGRS] ego-vehicle.position: %.10f, %.10f", ego_.mgrs_x, ego_.mgrs_y);
  }

  void CpmApplication::updateRP(double *lat, double *lon, double *altitude) {
    ego_.latitude = *lat;
    ego_.longitude = *lon;
    ego_.altitude = *altitude;
  }

  void CpmApplication::updateGenerationTime(int *gdt, long *gdt_timestamp) {
    generationTime_ = *gdt;
    gdt_timestamp_ = *gdt_timestamp; // ETSI-epoch milliseconds timestamp
  }

  void CpmApplication::updateHeading(double *yaw) {
    ego_.heading = *yaw;
  }

  void CpmApplication::setAllObjectsOfPersonsAnimalsToSend(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) {
    if (msg->objects.size() > 0) {
      for (autoware_perception_msgs::msg::PredictedObject obj : msg->objects) {
        std::string object_uuid = uuidToHexString(obj.object_id);
        auto found_object = std::find_if(objectsList.begin(), objectsList.end(), [&](auto const &e) {
          return !strcmp(e.uuid.c_str(), object_uuid.c_str());
        });

        if (found_object == objectsList.end()) {
          
        } else {
          found_object->to_send = true;
        }
      }

    }
  }

  void CpmApplication::updateObjectsList(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) {
    updating_objects_list_ = true;

    // Flag all objects as NOT_SEND
    if (objectsList.size() > 0) {
      for (auto& object : objectsList) {
        object.to_send = false;
        object.to_send_trigger = -1;
      }
    }

    if (msg->objects.size() > 0) {
      for (autoware_perception_msgs::msg::PredictedObject obj : msg->objects) {

        // RCLCPP_INFO(node_->get_logger(), "%d", obj.classification.front().label);
        double existence_probability = obj.existence_probability;
        // RCLCPP_INFO(node_->get_logger(), "existence_probability: %f", existence_probability);

        std::string object_uuid = uuidToHexString(obj.object_id);
        // RCLCPP_INFO(node_->get_logger(), "received object_id: %s", object_uuid.c_str());

        // RCLCPP_INFO(node_->get_logger(), "ObjectsList count: %d", objectsList.size());

        if (existence_probability >= objectConfidenceThreshold_) {
          // ObjectConfidence > ObjectConfidenceThreshold

          // Object tracked in internal memory? (i.e. Is object included in ObjectsList?)
          auto found_object = std::find_if(objectsList.begin(), objectsList.end(), [&](auto const &e) {
            return !strcmp(e.uuid.c_str(), object_uuid.c_str());
          });

          if (found_object == objectsList.end()) {
            // Object is new to internal memory

            if (cpm_object_id_ > 255) {
              cpm_object_id_ = 0;
            }

            // Add new object to ObjectsList
            CpmApplication::Object object;
            object.objectID = cpm_object_id_;
            object.uuid = object_uuid;
            object.timestamp_ros = msg->header.stamp;
            object.position_x = obj.kinematics.initial_pose_with_covariance.pose.position.x;
            object.position_y = obj.kinematics.initial_pose_with_covariance.pose.position.y;
            object.position_z = obj.kinematics.initial_pose_with_covariance.pose.position.z;
            object.orientation_x = obj.kinematics.initial_pose_with_covariance.pose.orientation.x;
            object.orientation_y = obj.kinematics.initial_pose_with_covariance.pose.orientation.y;
            object.orientation_z = obj.kinematics.initial_pose_with_covariance.pose.orientation.z;
            object.orientation_w = obj.kinematics.initial_pose_with_covariance.pose.orientation.w;
            object.shape_x = std::lround(obj.shape.dimensions.x * 10.0);
            object.shape_y = std::lround(obj.shape.dimensions.y * 10.0);
            object.shape_z = std::lround(obj.shape.dimensions.z * 10.0);

            long long msg_timestamp_sec = msg->header.stamp.sec;
            long long msg_timestamp_nsec = msg->header.stamp.nanosec;
            msg_timestamp_sec -= 1072915200; // convert to etsi-epoch
            long long msg_timestamp_msec = msg_timestamp_sec * 1000 + msg_timestamp_nsec / 1000000;
            object.timeOfMeasurement = gdt_timestamp_ - msg_timestamp_msec;
            if (object.timeOfMeasurement < -1500 || object.timeOfMeasurement > 1500) {
              RCLCPP_INFO(node_->get_logger(), "[updateObjectsStack] timeOfMeasurement out of bounds: %d", object.timeOfMeasurement);
              continue;
            }

            object.to_send = true;
            object.to_send_trigger = 0;
            object.timestamp = runtime_.now();

            objectsList.push_back(object);
            ++cpm_object_id_;

          } else {
            
            // Object was already in internal memory

            // Object belongs to class person or animal
            if (obj.classification.front().label == autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN || obj.classification.front().label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN) {

              if (include_all_persons_and_animals_) {
                found_object->to_send = true;
                found_object->to_send_trigger = 5;
              }
              
              // Object has not been included in a CPM in the past 500 ms.
              if (runtime_.now().time_since_epoch().count() - found_object->timestamp.time_since_epoch().count() > 500000) {
                // Include all objects of class person or animal in the current CPM
                include_all_persons_and_animals_ = true;
                found_object->to_send = true;
                found_object->to_send_trigger = 5;
                setAllObjectsOfPersonsAnimalsToSend(msg);
                RCLCPP_INFO(node_->get_logger(), "Include all objects of person/animal class");
              }

            } else {
              // Object does not belong to class person or animal

              // Euclidean absolute distance has changed by more than 4m
              double dist = pow(obj.kinematics.initial_pose_with_covariance.pose.position.x - found_object->position_x, 2) + pow(obj.kinematics.initial_pose_with_covariance.pose.position.y - found_object->position_y, 2);
              dist = sqrt(dist);
              // RCLCPP_INFO(node_->get_logger(), "Distance changed: %f", dist);
              if (dist > 4) {
                found_object->to_send = true;
                found_object->to_send_trigger = 1;
              } else {
                
              }

              // Absolute speed changed by more than 0.5 m/s
              double speed = pow(obj.kinematics.initial_twist_with_covariance.twist.linear.x - found_object->twist_linear_x, 2) + pow(obj.kinematics.initial_twist_with_covariance.twist.linear.x- found_object->twist_linear_y, 2);
              speed = sqrt(speed);
              // RCLCPP_INFO(node_->get_logger(), "Speed changed: %f", dist);
              if (speed > 0.5) {
                found_object->to_send = true;
                found_object->to_send_trigger = 2;
              } 

              // Orientation of speed vector changed by more than 4 degrees
              double twist_angular_x_diff = (obj.kinematics.initial_twist_with_covariance.twist.angular.x - found_object->twist_angular_x) * 180 / M_PI;
              double twist_angular_y_diff = (obj.kinematics.initial_twist_with_covariance.twist.angular.y - found_object->twist_angular_y) * 180 / M_PI;
              // RCLCPP_INFO(node_->get_logger(), "Orientation speed vector changed x: %f", twist_angular_x_diff);
              // RCLCPP_INFO(node_->get_logger(), "Orientation speed vector changed y: %f", twist_angular_y_diff);
              if( twist_angular_x_diff > 4 || twist_angular_y_diff > 4 ) {
                found_object->to_send = true;
                found_object->to_send_trigger = 3;
              }


              // It has been more than 1 s since last transmission of this object
              if (runtime_.now().time_since_epoch().count() - found_object->timestamp.time_since_epoch().count() > 1000000) {
                found_object->to_send = true;
                found_object->to_send_trigger = 4;
                // RCLCPP_INFO(node_->get_logger(), "Been more than 1s: %ld", runtime_.now().time_since_epoch().count() - found_object->timestamp.time_since_epoch().count());
              }

            }

            // Update found_object
            found_object->timestamp_ros = msg->header.stamp;
            found_object->position_x = obj.kinematics.initial_pose_with_covariance.pose.position.x;
            found_object->position_y = obj.kinematics.initial_pose_with_covariance.pose.position.y;
            found_object->position_z = obj.kinematics.initial_pose_with_covariance.pose.position.z;
            found_object->orientation_x = obj.kinematics.initial_pose_with_covariance.pose.orientation.x;
            found_object->orientation_y = obj.kinematics.initial_pose_with_covariance.pose.orientation.y;
            found_object->orientation_z = obj.kinematics.initial_pose_with_covariance.pose.orientation.z;
            found_object->orientation_w = obj.kinematics.initial_pose_with_covariance.pose.orientation.w;
            found_object->shape_x = std::lround(obj.shape.dimensions.x * 10.0);
            found_object->shape_y = std::lround(obj.shape.dimensions.y * 10.0);
            found_object->shape_z = std::lround(obj.shape.dimensions.z * 10.0);

            long long msg_timestamp_sec = msg->header.stamp.sec;
            long long msg_timestamp_nsec = msg->header.stamp.nanosec;
            msg_timestamp_sec -= 1072915200; // convert to etsi-epoch
            long long msg_timestamp_msec = msg_timestamp_sec * 1000 + msg_timestamp_nsec / 1000000;
            found_object->timeOfMeasurement = gdt_timestamp_ - msg_timestamp_msec;
            if (found_object->timeOfMeasurement < -1500 || found_object->timeOfMeasurement > 1500) {
              RCLCPP_INFO(node_->get_logger(), "[updateObjectsStack] timeOfMeasurement out of bounds: %d", found_object->timeOfMeasurement);
              continue;
            }

            found_object->timestamp = runtime_.now();

            // if use_dymanic_generation_rules_ == false, then always include object in CPM
            if (!use_dynamic_generation_rules_) {
              found_object->to_send = true;
              found_object->to_send_trigger = 0;
            }

          }
        }
      }
    } else {
      // No objects detected
    }

    // RCLCPP_INFO(node_->get_logger(), "ObjectsStack: %d objects", objectsStack.size());
    rclcpp::Time current_time = node_->now();
    // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::updateObjectsStack] [measure] T_objstack_updated %ld", current_time.nanoseconds());
    updating_objects_list_ = false;
  }

  void CpmApplication::printObjectsList(int cpm_num) {
    // RCLCPP_INFO(node_->get_logger(), "------------------------");
    if (objectsList.size() > 0) {
      for (auto& object : objectsList) {
        RCLCPP_INFO(node_->get_logger(), "[objectsList] %d,%d,%s,%d,%d", cpm_num, object.objectID, object.uuid.c_str(), object.to_send, object.to_send_trigger);
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "[objectsList] %d,,,,", cpm_num);
    }
    
    // RCLCPP_INFO(node_->get_logger(), "------------------------");
  }

  void CpmApplication::send() {

    if (is_sender_) {

      sending_ = true;

      printObjectsList(cpm_num_);
      
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] Sending CPM...");

      vanetza::asn1::Cpm message;

      // ITS PDU Header
      ItsPduHeader_t &header = message->header;
      header.protocolVersion = 1;
      header.messageID = 14;
      header.stationID = cpm_num_;

      CollectivePerceptionMessage_t &cpm = message->cpm;

      // Set GenerationTime
      RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] %ld", gdt_timestamp_);
      cpm.generationDeltaTime = generationTime_;

      CpmManagementContainer_t &management = cpm.cpmParameters.managementContainer;
      management.stationType = StationType_passengerCar;
      management.referencePosition.latitude = ego_.latitude * 1e7;
      management.referencePosition.longitude = ego_.longitude * 1e7;

      StationDataContainer_t *&sdc = cpm.cpmParameters.stationDataContainer;
      sdc = vanetza::asn1::allocate<StationDataContainer_t>();
      sdc->present = StationDataContainer_PR_originatingVehicleContainer;

      OriginatingVehicleContainer_t &ovc = sdc->choice.originatingVehicleContainer;
      ovc.speed.speedValue = 0;
      ovc.speed.speedConfidence = 1;

      // Calculate headingValue of ego-vehicle.
      // Convert ego-vehicle yaw to True-North clockwise angle (heading). 0.1 degree accuracy.
      int heading = std::lround(((-ego_.heading * 180.0 / M_PI) + 90.0) * 10.0);
      if (heading < 0) heading += 3600;
      ovc.heading.headingValue = heading;
      ovc.heading.headingConfidence = 1;

      int perceivedObjectsCount = 0;

      if (objectsList.size() > 0) {
        PerceivedObjectContainer_t *&poc = cpm.cpmParameters.perceivedObjectContainer;
        poc = vanetza::asn1::allocate<PerceivedObjectContainer_t>();

        for (auto& object : objectsList) {

          // RCLCPP_INFO(node_->get_logger(), "object.to_send: %d", object.to_send);

          if (object.to_send) {

            PerceivedObject *pObj = vanetza::asn1::allocate<PerceivedObject>();

            // Update CPM-specific values for Object
            object.xDistance = std::lround((
              (object.position_x - ego_.mgrs_x) * cos(-ego_.heading) - (object.position_y - ego_.mgrs_y) * sin(-ego_.heading)
              ) * 100.0);
            object.yDistance = std::lround((
              (object.position_x - ego_.mgrs_x) * sin(-ego_.heading) + (object.position_y - ego_.mgrs_y) * cos(-ego_.heading)
              ) * 100.0);
            if (object.xDistance < -132768 || object.xDistance > 132767) {
              RCLCPP_WARN(node_->get_logger(), "xDistance out of bounds. objectID: #%d", object.objectID);
              continue;
            }
            if (object.yDistance < -132768 || object.yDistance > 132767) {
              RCLCPP_WARN(node_->get_logger(), "yDistance out of bounds. objectID: #%d", object.objectID);
              continue;
            }

            // Calculate orientation of detected object
            tf2::Quaternion quat(object.orientation_x, object.orientation_y, object.orientation_z, object.orientation_w);
            tf2::Matrix3x3 matrix(quat);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            if (yaw < 0) {
              object.yawAngle = std::lround(((yaw + 2*M_PI) * 180.0 / M_PI) * 10.0); // 0 - 3600
            } else {
              object.yawAngle = std::lround((yaw * 180.0 / M_PI) * 10.0); // 0 - 3600
            }
            object.xSpeed = 0;
            object.ySpeed = 0;
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

            pObj->planarObjectDimension1 = vanetza::asn1::allocate<ObjectDimension_t>();
            pObj->planarObjectDimension2 = vanetza::asn1::allocate<ObjectDimension_t>();
            pObj->verticalObjectDimension = vanetza::asn1::allocate<ObjectDimension_t>();

            (*(pObj->planarObjectDimension1)).value = object.shape_y;
            (*(pObj->planarObjectDimension1)).confidence = 1;
            (*(pObj->planarObjectDimension2)).value = object.shape_x;
            (*(pObj->planarObjectDimension2)).confidence = 1;
            (*(pObj->verticalObjectDimension)).value = object.shape_z;
            (*(pObj->verticalObjectDimension)).confidence = 1;

            pObj->yawAngle = vanetza::asn1::allocate<CartesianAngle>();
            (*(pObj->yawAngle)).value = object.yawAngle;
            (*(pObj->yawAngle)).confidence = 1;

            ASN_SEQUENCE_ADD(poc, pObj);

            // object.to_send = false;
            // object.to_send_trigger = -1;
            // RCLCPP_INFO(node_->get_logger(), "Sending object: %s", object.uuid.c_str());

            ++perceivedObjectsCount;

          } else {
            // Object.to_send is set to False
            // RCLCPP_INFO(node_->get_logger(), "Object: %s not being sent.", object.uuid.c_str());
          }
        }

        cpm.cpmParameters.numberOfPerceivedObjects = perceivedObjectsCount;

        if (perceivedObjectsCount == 0) {
          cpm.cpmParameters.perceivedObjectContainer = NULL;
        }

      } else {
        cpm.cpmParameters.perceivedObjectContainer = NULL;
        RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] Empty POC");
      }

      RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] Sending CPM with %d objects", perceivedObjectsCount);

      insertCpmToCpmTable(message, (char*) "cpm_sent");
      
      std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};

      payload->layer(OsiLayer::Application) = std::move(message);

      Application::DataRequest request;
      request.its_aid = aid::CP;
      request.transport_type = geonet::TransportType::SHB;
      request.communication_profile = geonet::CommunicationProfile::ITS_G5;

      Application::DataConfirm confirm = Application::request(request, std::move(payload), node_);

      if (!confirm.accepted()) {
        throw std::runtime_error("[CpmApplication::send] CPM application data request failed");
      }

      sending_ = false;
      // rclcpp::Time current_time = node_->now();
      // RCLCPP_INFO(node_->get_logger(), "[CpmApplication::send] [measure] T_depart_r1 %ld", current_time.nanoseconds());

      std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
        std::chrono::system_clock::now().time_since_epoch()
      );
      node_->latency_log_file << "T_depart," << cpm_num_ << "," << ms.count() << std::endl;

      ++cpm_num_;
    }
  }

  void CpmApplication::createTables() {
    sqlite3 *db = NULL;
    char* err = NULL;

    int ret = sqlite3_open("autoware_v2x.db", &db);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB File Open Error");
      return;
    }

    char* sql_command;
    
    sql_command = (char*) "create table if not exists cpm_sent(id INTEGER PRIMARY KEY, timestamp BIGINT, perceivedObjectCount INTEGER);";

    ret = sqlite3_exec(db, sql_command, NULL, NULL, &err);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB Execution Error (create table cpm_sent)");
      sqlite3_close(db);
      sqlite3_free(err);
      return;
    }

    sql_command = (char*) "create table if not exists cpm_received(id INTEGER PRIMARY KEY, timestamp BIGINT, perceivedObjectCount INTEGER);";

    ret = sqlite3_exec(db, sql_command, NULL, NULL, &err);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB Execution Error (create table cpm_received)");
      sqlite3_close(db);
      sqlite3_free(err);
      return;
    }

    sqlite3_close(db);
    RCLCPP_INFO(node_->get_logger(), "CpmApplication::createTables Finished");
  }

  void CpmApplication::insertCpmToCpmTable(vanetza::asn1::Cpm cpm, char* table_name) {
    sqlite3 *db = NULL;
    char* err = NULL;

    int ret = sqlite3_open("autoware_v2x.db", &db);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB File Open Error");
      return;
    }

    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    int perceivedObjectCount = 0;
    if (cpm->cpm.cpmParameters.numberOfPerceivedObjects) {
      perceivedObjectCount = cpm->cpm.cpmParameters.numberOfPerceivedObjects;
    }

    std::stringstream sql_command;
    
    sql_command << "insert into " << table_name << " (timestamp, perceivedObjectCount) values (" << timestamp << ", " << perceivedObjectCount << ");";

    ret = sqlite3_exec(db, sql_command.str().c_str(), NULL, NULL, &err);
    if (ret != SQLITE_OK) {
      RCLCPP_INFO(node_->get_logger(), "DB Execution Error (insertCpmToCpmTable)");
      RCLCPP_INFO(node_->get_logger(), sql_command.str().c_str());
      RCLCPP_INFO(node_->get_logger(), err);
      sqlite3_close(db);
      sqlite3_free(err);
      return;
    }

    sqlite3_close(db);
    // RCLCPP_INFO(node_->get_logger(), "CpmApplication::insertCpmToCpmTable Finished");
  }
}