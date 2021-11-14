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
#include <vanetza/facilities/cpm_functions.hpp>
#include <sstream>
#include <memory>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace gn = vanetza::geonet;

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;

namespace v2x
{
  V2XNode::V2XNode(const rclcpp::NodeOptions &node_options) : rclcpp::Node("autoware_v2x_node", node_options), received_time(this->now()), cpm_received(false)
  {
    using std::placeholders::_1;
    subscription_ = this->create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>("/perception/object_recognition/objects", 10, std::bind(&V2XNode::objectsCallback, this, _1));

    subscription_pos_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&V2XNode::tfCallback, this, _1));

    publisher_ = create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>("/perception/object_recognition/objects", rclcpp::QoS{10});

    app = new V2XApp(this);
    boost::thread v2xApp(boost::bind(&V2XApp::start, app));

    RCLCPP_INFO(get_logger(), "V2X Node Launched");
  }

  void V2XNode::publishObjects(std::vector<CpmApplication::Object> *objectsStack) {
    autoware_perception_msgs::msg::DynamicObjectArray output_dynamic_object_msg;
    std_msgs::msg::Header header;
    rclcpp::Time current_time = this->now();
    received_time = current_time;
    output_dynamic_object_msg.header.frame_id = "map";
    output_dynamic_object_msg.header.stamp = current_time;

    for (CpmApplication::Object obj : *objectsStack) {
      autoware_perception_msgs::msg::DynamicObject object;
      autoware_perception_msgs::msg::Semantic semantic;
      autoware_perception_msgs::msg::Shape shape;
      autoware_perception_msgs::msg::State state;

      semantic.type = autoware_perception_msgs::msg::Semantic::CAR;
      semantic.confidence = 0.99;

      shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
      shape.dimensions.x = obj.shape_x / 10;
      shape.dimensions.y = obj.shape_y / 10;
      shape.dimensions.z = obj.shape_z / 10;

      state.pose_covariance.pose.position.x = obj.position_x;
      state.pose_covariance.pose.position.y = obj.position_y;
      state.pose_covariance.pose.position.z = 0.1;

      state.pose_covariance.pose.orientation.x = obj.orientation_x;
      state.pose_covariance.pose.orientation.y = obj.orientation_y;
      state.pose_covariance.pose.orientation.z = obj.orientation_z;
      state.pose_covariance.pose.orientation.w = obj.orientation_w;

      object.semantic = semantic;
      object.shape = shape;
      object.state = state;

      output_dynamic_object_msg.objects.push_back(object);
    }
    cpm_received = true;
    publisher_->publish(output_dynamic_object_msg);
  }

  void V2XNode::objectsCallback(const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr msg)
  {
    rclcpp::Time current_time = this->now();
    // RCLCPP_INFO(get_logger(), "%ld", current_time.nanoseconds() - received_time.nanoseconds());
    if (cpm_received) {
      // if time difference is more than 10ms
      if (current_time - received_time > rclcpp::Duration(std::chrono::nanoseconds(10000000))) {
        app->objectsCallback(msg);
      }
      cpm_received = false;
    } else {
      app->objectsCallback(msg);
    }
  }

  void V2XNode::tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg)
  {
    app->tfCallback(msg);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(v2x::V2XNode)