#ifndef V2X_NODE_HPP_EUIC2VFR
#define V2X_NODE_HPP_EUIC2VFR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_adapi_v1_msgs/srv/get_vehicle_dimensions.hpp"
#include "autoware_adapi_v1_msgs/msg/vehicle_dimensions.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <boost/asio/io_service.hpp>
#include "autoware_v2x/v2x_app.hpp"
#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/cam_application.hpp"
#include "autoware_v2x/time_trigger.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/ethernet_device.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/router_context.hpp"
#include <fstream>

#include <etsi_its_cam_ts_msgs/msg/cam.hpp>

namespace v2x
{
  class V2XNode : public rclcpp::Node
  {
  public:
    explicit V2XNode(const rclcpp::NodeOptions &node_options);
    V2XApp *app;
    void publishObjects(std::vector<CpmApplication::Object> *, int cpm_num);
    void publishCpmSenderObject(double, double, double);
    void publishReceivedCam(etsi_its_cam_ts_msgs::msg::CAM &);
    void getVehicleDimensions();

    std::ofstream latency_log_file;

  private:
    void objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);
    void velocityReportCallback(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg);
    void gearReportCallback(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg);
    void steeringReportCallback(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg);
    void tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg);

    rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_sub_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_sub_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::GetVehicleDimensions>::SharedPtr get_vehicle_dimensions_;
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr cpm_objects_pub_;
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr cpm_sender_pub_;
    rclcpp::Publisher<etsi_its_cam_ts_msgs::msg::CAM>::SharedPtr cam_rec_pub_;

    double pos_lat_;
    double pos_lon_;

    bool vehicle_dimensions_available_;
  };
}

#endif
