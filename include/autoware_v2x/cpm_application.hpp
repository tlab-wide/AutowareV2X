#ifndef CPM_APPLICATION_HPP_EUIC2VFR
#define CPM_APPLICATION_HPP_EUIC2VFR

#include "autoware_v2x/application.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_v2x/positioning.hpp"
#include <vanetza/asn1/cpm.hpp>

namespace v2x
{
    class V2XNode;
    class CpmApplication : public Application
    {
    public:
        CpmApplication(V2XNode *node, vanetza::Runtime &, bool is_sender);
        PortType port() override;
        std::string uuidToHexString(const unique_identifier_msgs::msg::UUID&);
        void indicate(const DataIndication &, UpPacketPtr) override;
        void set_interval(vanetza::Clock::duration);
        void setAllObjectsOfPersonsAnimalsToSend(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr);
        void updateObjectsList(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr);
        void updateMGRS(double *, double *);
        void updateRP(double *, double *, double *);
        void updateGenerationTime(int *, long *);
        void updateHeading(double *);
        void printObjectsList(int);
        void send();

        /** Creates the database schema.
         *  Creates tables for cpm_sent, cpm_received
        */
        void createTables();

        /** Inserts CPM into the database.
         *  Constructs and executes queries for inserting the specified CPM data.
         *  @param cpm The CPM to be inserted 
         *  @param table_name The table to insert the CPM into (cpm_sent or cpm_received)
        */
        void insertCpmToCpmTable(vanetza::asn1::Cpm, char*);

        struct Object {
            std::string uuid;
            int objectID; // 0-255 for CPM
            vanetza::Clock::time_point timestamp;
            rclcpp::Time timestamp_ros;
            double position_x;
            double position_y;
            double position_z;
            double orientation_x;
            double orientation_y;
            double orientation_z;
            double orientation_w;
            double twist_linear_x;
            double twist_linear_y;
            double twist_angular_x;
            double twist_angular_y;
            int shape_x;
            int shape_y;
            int shape_z;
            int xDistance;
            int yDistance;
            int xSpeed;
            int ySpeed;
            int yawAngle;
            vanetza::PositionFix position;
            int timeOfMeasurement;
            bool to_send;
            int to_send_trigger;
        };
        std::vector<CpmApplication::Object> objectsList;
        std::vector<CpmApplication::Object> receivedObjectsStack;

    private:
        void schedule_timer();
        void on_timer(vanetza::Clock::time_point);

        V2XNode *node_;
        vanetza::Runtime &runtime_;
        vanetza::Clock::duration cpm_interval_;

        struct Ego_station {
            double mgrs_x;
            double mgrs_y;
            double latitude;
            double longitude;
            double altitude;
            double heading;
        };

        Ego_station ego_;
        
        int generationTime_;
        long gdt_timestamp_;

        double objectConfidenceThreshold_;

        bool updating_objects_list_;
        bool sending_;
        bool is_sender_;
        bool reflect_packet_;
        bool include_all_persons_and_animals_;

        int cpm_num_;
        int received_cpm_num_;

        int cpm_object_id_;

        bool use_dynamic_generation_rules_;
    };
}

#endif /* CPM_APPLICATION_HPP_EUIC2VFR */
