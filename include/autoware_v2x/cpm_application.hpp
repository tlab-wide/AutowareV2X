#ifndef CPM_APPLICATION_HPP_EUIC2VFR
#define CPM_APPLICATION_HPP_EUIC2VFR

#include "autoware_v2x/application.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_v2x/positioning.hpp"

namespace v2x
{
    class V2XNode;
    class CpmApplication : public Application
    {
    public:
        CpmApplication(V2XNode *node, vanetza::Runtime &);
        PortType port() override;
        void indicate(const DataIndication &, UpPacketPtr) override;
        void set_interval(vanetza::Clock::duration);
        void updateObjectsStack(const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr);
        void updateMGRS(double *, double *);
        void updateRP(double *, double *, double *);
        void updateGenerationDeltaTime(int *);
        void updateHeading(double *);
        void send();

        struct Object
        {
            int objectID; // 0-255
            rclcpp::Time timestamp;
            double position_x;
            double position_y;
            double position_z;
            double orientation_x;
            double orientation_y;
            double orientation_z;
            double orientation_w;
            int shape_x;
            int shape_y;
            int shape_z;
            int xDistance;
            int yDistance;
            double xSpeed;
            double ySpeed;
            int yawAngle;
            vanetza::PositionFix position;
            int timeOfMeasurement;
        };
        std::vector<CpmApplication::Object> objectsStack;
        std::vector<CpmApplication::Object> receivedObjectsStack;

    private:
        void schedule_timer();
        void on_timer(vanetza::Clock::time_point);

        V2XNode *node_;
        vanetza::Runtime &runtime_;
        vanetza::Clock::duration cpm_interval_;

        double ego_x_;
        double ego_y_;
        double ego_lat_;
        double ego_lon_;
        double ego_altitude_;
        double ego_heading_;
        int generationDeltaTime_;

        bool updating_objects_stack_;
        bool sending_;
    };
}

#endif /* CPM_APPLICATION_HPP_EUIC2VFR */
