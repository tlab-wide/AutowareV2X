#ifndef V2X_APP_HPP_EUIC2VFR
#define V2X_APP_HPP_EUIC2VFR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <boost/asio/io_service.hpp>
#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/time_trigger.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/ethernet_device.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/router_context.hpp"
// #include "autoware_v2x/v2x_node.hpp"

namespace v2x
{
  class V2XNode;
  class V2XApp
  {
  public:
    V2XApp(V2XNode *);
    void start();
    void objectsCallback(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr);
    void tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr);

    CpmApplication *cp;
    // V2XNode *v2x_node;

  private:
    friend class CpmApplication;
    friend class Application;
    V2XNode* node_;
    bool tf_received_;
    int tf_interval_;
    bool cp_started_;
  };
}

#endif