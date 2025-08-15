#include "autoware_v2x/v2x_node.hpp"
#include "autoware_v2x/v2x_app.hpp"
#include "autoware_v2x/time_trigger.hpp"
#include "autoware_v2x/router_context.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/cpm_application.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <vanetza/asn1/cpm.hpp>
#include <sstream>
#include <memory>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include <chrono>
#include <iostream>
#include <random>

namespace gn = vanetza::geonet;

using namespace vanetza;
using namespace std::chrono;

namespace v2x
{
  V2XNode::V2XNode(const rclcpp::NodeOptions &node_options) : rclcpp::Node("autoware_v2x_node", node_options) {
    using std::placeholders::_1;

    objects_sub_ = this->create_subscription<autoware_perception_msgs::msg::PredictedObjects>("/perception/object_recognition/objects", 10, std::bind(&V2XNode::objectsCallback, this, _1));
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&V2XNode::tfCallback, this, _1));

    cpm_objects_pub_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>("/v2x/cpm/objects", rclcpp::QoS{10});
    // cpm_sender_pub_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>("/v2x/cpm/sender", rclcpp::QoS{10});

    // Declare Parameters
    this->declare_parameter<std::string>("network_interface", "vmnet1");
    this->declare_parameter<bool>("is_sender", true);

    // Launch V2XApp in a new thread
    app = new V2XApp(this);
    boost::thread v2xApp(boost::bind(&V2XApp::start, app));

    RCLCPP_INFO(get_logger(), "V2X Node Launched");

    // Make latency_log file from current timestamp
    time_t t = time(nullptr);
    const tm* lt = localtime(&t);
    std::stringstream s;
    s << "20" << lt->tm_year-100 <<"-" << lt->tm_mon+1 << "-" << lt->tm_mday << "_" << lt->tm_hour << ":" << lt->tm_min << ":" << lt->tm_sec;
    std::string timestamp = s.str();
    char cur_dir[1024];
    getcwd(cur_dir, 1024);
    std::string latency_log_filename = std::string(cur_dir) + "/latency_logs/latency_log_file_" + timestamp + ".csv";
    latency_log_file.open(latency_log_filename, std::ios::out);
  }

  void V2XNode::publishCpmSenderObject(double x_mgrs, double y_mgrs, double orientation) {
    autoware_perception_msgs::msg::PredictedObjects cpm_sender_object_msg;
    std_msgs::msg::Header header;
    rclcpp::Time current_time = this->now();
    cpm_sender_object_msg.header.frame_id = "map";
    cpm_sender_object_msg.header.stamp = current_time;

    autoware_perception_msgs::msg::PredictedObject object;
    autoware_perception_msgs::msg::ObjectClassification classification;
    autoware_perception_msgs::msg::Shape shape;
    autoware_perception_msgs::msg::PredictedObjectKinematics kinematics;

    classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
    classification.probability = 0.99;

    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    shape.dimensions.x = 5.0;
    shape.dimensions.y = 2.0;
    shape.dimensions.z = 1.7;

    kinematics.initial_pose_with_covariance.pose.position.x = x_mgrs;
    kinematics.initial_pose_with_covariance.pose.position.y = y_mgrs;
    kinematics.initial_pose_with_covariance.pose.position.z = 0.1;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, orientation);

    kinematics.initial_pose_with_covariance.pose.orientation.x = quat.x();
    kinematics.initial_pose_with_covariance.pose.orientation.y = quat.y();
    kinematics.initial_pose_with_covariance.pose.orientation.z = quat.z();
    kinematics.initial_pose_with_covariance.pose.orientation.w = quat.w();

    object.classification.emplace_back(classification);
    object.shape = shape;
    object.kinematics = kinematics;

    cpm_sender_object_msg.objects.push_back(object);

    // publisher_v2x_cpm_sender_->publish(cpm_sender_object_msg);

  }

  void V2XNode::publishObjects(std::vector<CpmApplication::Object> *objectsStack, int cpm_num) {
    autoware_perception_msgs::msg::PredictedObjects output_dynamic_object_msg;
    std_msgs::msg::Header header;
    rclcpp::Time current_time = this->now();
    output_dynamic_object_msg.header.frame_id = "map";
    output_dynamic_object_msg.header.stamp = current_time;

    for (CpmApplication::Object obj : *objectsStack) {
      autoware_perception_msgs::msg::PredictedObject object;
      autoware_perception_msgs::msg::ObjectClassification classification;
      autoware_perception_msgs::msg::Shape shape;
      autoware_perception_msgs::msg::PredictedObjectKinematics kinematics;

      classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
      classification.probability = 0.99;

      shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
      shape.dimensions.x = obj.shape_x / 10.0;
      shape.dimensions.y = obj.shape_y / 10.0;
      shape.dimensions.z = obj.shape_z / 10.0;

      kinematics.initial_pose_with_covariance.pose.position.x = obj.position_x;
      kinematics.initial_pose_with_covariance.pose.position.y = obj.position_y;
      kinematics.initial_pose_with_covariance.pose.position.z = 0.1;

      kinematics.initial_pose_with_covariance.pose.orientation.x = obj.orientation_x;
      kinematics.initial_pose_with_covariance.pose.orientation.y = obj.orientation_y;
      kinematics.initial_pose_with_covariance.pose.orientation.z = obj.orientation_z;
      kinematics.initial_pose_with_covariance.pose.orientation.w = obj.orientation_w;

      object.classification.emplace_back(classification);
      object.shape = shape;
      object.kinematics = kinematics;

      std::mt19937 gen(std::random_device{}());
      std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
      std::generate(object.object_id.uuid.begin(), object.object_id.uuid.end(), bit_eng);

      output_dynamic_object_msg.objects.push_back(object);
    }

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
      std::chrono::system_clock::now().time_since_epoch()
    );
    latency_log_file << "T_publish," << cpm_num << "," << ms.count() << std::endl;

    cpm_objects_pub_->publish(output_dynamic_object_msg);
  }

  void V2XNode::objectsCallback(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) {
    rclcpp::Time msg_time = msg->header.stamp; // timestamp included in the Autoware Perception Msg.

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
      std::chrono::system_clock::now().time_since_epoch()
    );
    latency_log_file << "T_rosmsg,," << ms.count() << std::endl;

    app->objectsCallback(msg);
  }

  void V2XNode::tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
    app->tfCallback(msg);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(v2x::V2XNode)