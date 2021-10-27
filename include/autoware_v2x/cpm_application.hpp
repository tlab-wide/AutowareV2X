#ifndef CPM_APPLICATION_HPP_EUIC2VFR
#define CPM_APPLICATION_HPP_EUIC2VFR

#include "autoware_v2x/application.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_v2x/positioning.hpp"

class CpmApplication : public Application
{
public:
    CpmApplication(rclcpp::Node *node);
    PortType port() override;
    void indicate(const DataIndication &, UpPacketPtr) override;
    void send(const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr, rclcpp::Node *, double, double);

private:
    rclcpp::Node* node_;
};

#endif /* CPM_APPLICATION_HPP_EUIC2VFR */
