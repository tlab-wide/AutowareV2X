#include "autoware_v2x/v2x_node.hpp"
#include "autoware_v2x/v2x_app.hpp"
#include "autoware_v2x/time_trigger.hpp"
#include "autoware_v2x/router_context.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/tcpip_application.hpp"

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

    objects_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>("/perception/object_recognition/objects", 10, std::bind(&V2XNode::objectsCallback, this, _1));
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&V2XNode::tfCallback, this, _1));

    cpm_objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>("/perception/object_recognition/objects", rclcpp::QoS{10});
    // cpm_sender_pub_ = create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>("/v2x/cpm/sender", rclcpp::QoS{10});

    // pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/obstacle_segmentation/pointcloud_test", rclcpp::QoS{1});

    // Declare Parameters
    this->declare_parameter<std::string>("network_interface", "vmnet1");
    this->declare_parameter<bool>("is_sender", true);

    this->get_parameter("is_sender", is_sender_);

    // Launch V2XApp in a new thread
    app = new V2XApp(this);
    boost::thread v2xApp(boost::bind(&V2XApp::start, app));

    if (is_sender_) {
      // Launch TCPIP App in a new thread
      tcpip_app = new TcpIpApplication(this);
      boost::thread tcpipApp(boost::bind(&TcpIpApplication::start, tcpip_app));
    } else {
      // Launch TCPIP App in a new thread
      tcpip_app = new TcpIpApplication(this);
      boost::thread tcpipApp(boost::bind(&TcpIpApplication::startReceiver, tcpip_app));
    }
  

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

    cpm_initialized = false;
  }

  void V2XNode::publishCpmSenderObject(double x_mgrs, double y_mgrs, double orientation) {
    autoware_auto_perception_msgs::msg::PredictedObjects cpm_sender_object_msg;
    std_msgs::msg::Header header;
    rclcpp::Time current_time = this->now();
    cpm_sender_object_msg.header.frame_id = "map";
    cpm_sender_object_msg.header.stamp = current_time;

    autoware_auto_perception_msgs::msg::PredictedObject object;
    autoware_auto_perception_msgs::msg::ObjectClassification classification;
    autoware_auto_perception_msgs::msg::Shape shape;
    autoware_auto_perception_msgs::msg::PredictedObjectKinematics kinematics;

    classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
    classification.probability = 0.99;

    shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
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
    autoware_auto_perception_msgs::msg::PredictedObjects output_dynamic_object_msg;
    std_msgs::msg::Header header;
    rclcpp::Time current_time = this->now();
    output_dynamic_object_msg.header.frame_id = "map";
    output_dynamic_object_msg.header.stamp = current_time;

    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> v_pointcloud;

    for (CpmApplication::Object obj : *objectsStack) {
      autoware_auto_perception_msgs::msg::PredictedObject object;
      autoware_auto_perception_msgs::msg::ObjectClassification classification;
      autoware_auto_perception_msgs::msg::Shape shape;
      autoware_auto_perception_msgs::msg::PredictedObjectKinematics kinematics;

      classification.label = obj.classification.label;
      classification.probability = obj.classification.probability;

      shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
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

    //   rclcpp::Time current_time_tf = this->now();
    //   tf2::Transform tf_base_link2map;
    //   try {
    //     geometry_msgs::msg::TransformStamped ros_base_link2map;
    //     ros_base_link2map = tf_buffer_.lookupTransform(
    //       /*target*/ "base_link", /*src*/ "map", current_time_tf, rclcpp::Duration::from_seconds(0.5));
    //     tf2::fromMsg(ros_base_link2map.transform, tf_base_link2map);
    //   } catch (tf2::TransformException & ex) {
    //     RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    //     return;
    //   }

    //   // pointcloud
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //   createObjectPointcloud(
    //     10, 10, 10, 1.0, 1.0, 1.0,
    //     tf_base_link2map, pointcloud_ptr);
    //   v_pointcloud.push_back(pointcloud_ptr);
    }

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
      std::chrono::system_clock::now().time_since_epoch()
    );
    latency_log_file << "T_publish," << cpm_num << "," << ms.count() << std::endl;

    cpm_objects_pub_->publish(output_dynamic_object_msg);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // for (size_t i = 0; i < v_pointcloud.size(); ++i) {
    //   for (size_t j = 0; j < v_pointcloud.at(i)->size(); ++j) {
    //     merged_pointcloud_ptr->push_back(v_pointcloud.at(i)->at(j));
    //   }
    // }

    // sensor_msgs::msg::PointCloud2 output_pointcloud_msg;

    // // no ground
    // pcl::toROSMsg(*merged_pointcloud_ptr, output_pointcloud_msg);

    // output_pointcloud_msg.header.frame_id = "base_link";
    // output_pointcloud_msg.header.stamp = current_time;

    // // publish
    // pointcloud_pub_->publish(output_pointcloud_msg);
  }

  void V2XNode::objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) {
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

  // void V2XNode::createObjectPointcloud(
  //   const double length, const double width, const double height, const double std_dev_x,
  //   const double std_dev_y, const double std_dev_z, const tf2::Transform & tf_base_link2moved_object,
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr & pointcloud_ptr)
  // {
  //   std::normal_distribution<> x_random(0.0, std_dev_x);
  //   std::normal_distribution<> y_random(0.0, std_dev_y);
  //   std::normal_distribution<> z_random(0.0, std_dev_z);
  //   auto getBaseLinkTo2DPoint = [tf_base_link2moved_object](double x, double y) -> pcl::PointXYZ {
  //     tf2::Transform tf_moved_object2point;
  //     tf2::Transform tf_base_link2point;
  //     geometry_msgs::msg::Transform ros_moved_object2point;
  //     ros_moved_object2point.translation.x = x;
  //     ros_moved_object2point.translation.y = y;
  //     ros_moved_object2point.translation.z = 0.0;
  //     ros_moved_object2point.rotation.x = 0;
  //     ros_moved_object2point.rotation.y = 0;
  //     ros_moved_object2point.rotation.z = 0;
  //     ros_moved_object2point.rotation.w = 1;
  //     tf2::fromMsg(ros_moved_object2point, tf_moved_object2point);
  //     tf_base_link2point = tf_base_link2moved_object * tf_moved_object2point;
  //     pcl::PointXYZ point;
  //     point.x = tf_base_link2point.getOrigin().x();
  //     point.y = tf_base_link2point.getOrigin().y();
  //     point.z = tf_base_link2point.getOrigin().z();
  //     return point;
  //   };
  //   const double epsilon = 0.001;
  //   const double step = 0.05;
  //   const double vertical_theta_step = (1.0 / 180.0) * M_PI;
  //   const double vertical_min_theta = (-15.0 / 180.0) * M_PI;
  //   const double vertical_max_theta = (15.0 / 180.0) * M_PI;
  //   const double horizontal_theta_step = (0.1 / 180.0) * M_PI;
  //   const double horizontal_min_theta = (-180.0 / 180.0) * M_PI;
  //   const double horizontal_max_theta = (180.0 / 180.0) * M_PI;

  //   const double min_z = -1.0 * (height / 2.0) + tf_base_link2moved_object.getOrigin().z();
  //   const double max_z = 1.0 * (height / 2.0) + tf_base_link2moved_object.getOrigin().z();
  //   pcl::PointCloud<pcl::PointXYZ> horizontal_candidate_pointcloud;
  //   pcl::PointCloud<pcl::PointXYZ> horizontal_pointcloud;
  //   {
  //     const double y = -1.0 * (width / 2.0);
  //     for (double x = -1.0 * (length / 2.0); x <= ((length / 2.0) + epsilon); x += step) {
  //       horizontal_candidate_pointcloud.push_back(getBaseLinkTo2DPoint(x, y));
  //     }
  //   }
  //   {
  //     const double y = 1.0 * (width / 2.0);
  //     for (double x = -1.0 * (length / 2.0); x <= ((length / 2.0) + epsilon); x += step) {
  //       horizontal_candidate_pointcloud.push_back(getBaseLinkTo2DPoint(x, y));
  //     }
  //   }
  //   {
  //     const double x = -1.0 * (length / 2.0);
  //     for (double y = -1.0 * (width / 2.0); y <= ((width / 2.0) + epsilon); y += step) {
  //       horizontal_candidate_pointcloud.push_back(getBaseLinkTo2DPoint(x, y));
  //     }
  //   }
  //   {
  //     const double x = 1.0 * (length / 2.0);
  //     for (double y = -1.0 * (width / 2.0); y <= ((width / 2.0) + epsilon); y += step) {
  //       horizontal_candidate_pointcloud.push_back(getBaseLinkTo2DPoint(x, y));
  //     }
  //   }
  //   // 2D ray tracing
  //   size_t ranges_size =
  //     std::ceil((horizontal_max_theta - horizontal_min_theta) / horizontal_theta_step);
  //   std::vector<double> horizontal_ray_traced_2d_pointcloud;
  //   horizontal_ray_traced_2d_pointcloud.assign(ranges_size, std::numeric_limits<double>::infinity());
  //   const int no_data = -1;
  //   std::vector<int> horizontal_ray_traced_pointcloud_indices;
  //   horizontal_ray_traced_pointcloud_indices.assign(ranges_size, no_data);
  //   for (size_t i = 0; i < horizontal_candidate_pointcloud.points.size(); ++i) {
  //     double angle =
  //       std::atan2(horizontal_candidate_pointcloud.at(i).y, horizontal_candidate_pointcloud.at(i).x);
  //     double range =
  //       std::hypot(horizontal_candidate_pointcloud.at(i).y, horizontal_candidate_pointcloud.at(i).x);
  //     if (angle < horizontal_min_theta || angle > horizontal_max_theta) {
  //       continue;
  //     }
  //     int index = (angle - horizontal_min_theta) / horizontal_theta_step;
  //     if (range < horizontal_ray_traced_2d_pointcloud[index]) {
  //       horizontal_ray_traced_2d_pointcloud[index] = range;
  //       horizontal_ray_traced_pointcloud_indices.at(index) = i;
  //     }
  //   }
  //   for (const auto & pointcloud_index : horizontal_ray_traced_pointcloud_indices) {
  //     if (pointcloud_index != no_data) {
  //       // generate vertical point
  //       horizontal_pointcloud.push_back(horizontal_candidate_pointcloud.at(pointcloud_index));
  //       const double distance = std::hypot(
  //         horizontal_candidate_pointcloud.at(pointcloud_index).x,
  //         horizontal_candidate_pointcloud.at(pointcloud_index).y);
  //       for (double vertical_theta = vertical_min_theta;
  //           vertical_theta <= vertical_max_theta + epsilon; vertical_theta += vertical_theta_step) {
  //         const double z = distance * std::tan(vertical_theta);
  //         if (min_z <= z && z <= max_z + epsilon) {
  //           pcl::PointXYZ point;
  //           point.x =
  //             horizontal_candidate_pointcloud.at(pointcloud_index).x + x_random(random_generator_);
  //           point.y =
  //             horizontal_candidate_pointcloud.at(pointcloud_index).y + y_random(random_generator_);
  //           point.z = z + z_random(random_generator_);
  //           pointcloud_ptr->push_back(point);
  //         }
  //       }
  //     }
  //   }
  // }
}



#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(v2x::V2XNode)