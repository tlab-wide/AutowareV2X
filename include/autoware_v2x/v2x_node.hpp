#ifndef V2X_NODE_HPP_EUIC2VFR
#define V2X_NODE_HPP_EUIC2VFR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <boost/asio/io_service.hpp>
#include "autoware_v2x/v2x_app.hpp"
#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/tcpip_application.hpp"
#include "autoware_v2x/time_trigger.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/ethernet_device.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/router_context.hpp"
#include <fstream>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <random>

#include <vanetza/asn1/cpm.hpp>

namespace v2x
{
  class V2XNode : public rclcpp::Node
  {
  public:
    explicit V2XNode(const rclcpp::NodeOptions &node_options);
    V2XApp *app;
    void publishObjects(std::vector<CpmApplication::Object> *, int cpm_num);
    void publishCpmSenderObject(double, double, double);

    TcpIpApplication *tcpip_app;
    CpmApplication *cpm_app;
    
    std::ofstream latency_log_file;

    vanetza::asn1::Cpm cpm_;
    vanetza::asn1::Cpm cpm_received_lte_;

    bool cpm_initialized;

  private:
    void objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);
    void tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg);
    // void createObjectPointcloud(
    //   const double length, const double width, const double height, const double std_dev_x,
    //   const double std_dev_y, const double std_dev_z, const tf2::Transform & tf_base_link2moved_object,
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr & pointcloud_ptr);

    rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr cpm_objects_pub_;
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr cpm_sender_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;
    // std::mt19937 random_generator_;

    double pos_lat_;
    double pos_lon_;

    bool is_sender_;
  };
}

#endif