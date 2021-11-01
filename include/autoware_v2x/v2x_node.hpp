#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <boost/asio/io_service.hpp>
#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/time_trigger.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/ethernet_device.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/router_context.hpp"

namespace v2x
{
  class V2XNode : public rclcpp::Node
  {
  public:
    explicit V2XNode(const rclcpp::NodeOptions &node_options);

  private:
    void objectsCallback(
        const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr msg);
    void tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg);

    friend class CpmApplication;
    friend class Application;
    rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr subscription_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_pos_;

    std::unique_ptr<CpmApplication> cp_;
    std::unique_ptr<Application> app_;
    boost::asio::io_service io_service_;
    TimeTrigger trigger_;
    const char *device_name_;
    EthernetDevice device_;
    vanetza::MacAddress mac_address_;
    std::unique_ptr<LinkLayer> link_layer_;
    vanetza::geonet::MIB mib_;
    std::unique_ptr<vanetza::PositionProvider> positioning_;
    std::unique_ptr<vanetza::security::SecurityEntity> security_;
    RouterContext context_;

    double pos_lat_;
    double pos_lon_;

  };
}