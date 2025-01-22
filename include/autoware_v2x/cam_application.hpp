#ifndef CAM_APPLICATION_HPP_EUIC2VFR
#define CAM_APPLICATION_HPP_EUIC2VFR

#include "autoware_v2x/application.hpp"
#include "autoware_v2x/positioning.hpp"
#include "rclcpp/rclcpp.hpp"

#include <vanetza/asn1/cam.hpp>

#include "autoware_adapi_v1_msgs/msg/vehicle_dimensions.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>

#include <etsi_its_cam_ts_coding/cam_ts_CAM.h>

namespace v2x
{
class V2XNode;
class CamApplication : public Application
{
public:
  CamApplication(V2XNode *node, vanetza::Runtime &, bool is_sender, bool publish_own_cams);
  PortType port() override;
  void indicate(const DataIndication &, UpPacketPtr) override;
  void set_interval(vanetza::Clock::duration);
  void updateMGRS(double *, double *);
  void updateRP(const double *, const double *, const double *);
  void updateHeading(const double *);
  void setVehicleDimensions(const autoware_adapi_v1_msgs::msg::VehicleDimensions &);
  void updateVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr);
  void updateGearReport(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr);
  void updateSteeringReport(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr);
  void send();

private:
  void calc_interval();
  void schedule_timer();
  void on_timer(vanetza::Clock::time_point);
  static void build_etsi_its_cam_ts_from_vanetza(vanetza::asn1::Cam &, cam_ts_CAM_t &);

  V2XNode *node_;
  vanetza::Runtime &runtime_;
  vanetza::Clock::duration cam_interval_{};

  struct VehicleDimensions {
    float wheel_radius = 0.0;
    float wheel_width = 0.0;
    float wheel_base = 0.0;
    float wheel_tread = 0.0;
    float front_overhang = 0.0;
    float rear_overhang = 0.0;
    float left_overhang = 0.0;
    float right_overhang = 0.0;
    float height = 0.0;
  };
  VehicleDimensions vehicleDimensions_;

  struct Ego_station {
    double mgrs_x;
    double mgrs_y;
    double latitude;
    double longitude;
    double altitude;
    double heading;
  };
  Ego_station ego_;

  class PositionsDeque {
  public:
    void insert(double value) {
      if (deque.size() >= maxSize) {
        total -= deque.front();
        deque.pop_front();
      }
      total += value;
      mean = total / deque.size();
      deque.push_back(value);
    }

    int getSize() {
      return deque.size();
    }

    [[nodiscard]] double getMean() const {
      return this->mean;
    }

    using iterator = std::deque<double>::const_iterator;

    [[nodiscard]] iterator begin() const {
      return deque.begin();
    }

    [[nodiscard]] iterator end() const {
      return deque.end();
    }

    double operator[](std::size_t index) const {
      if (index >= deque.size())
        throw std::out_of_range("[PositionDeque] Index out of range");
      return deque[index];
    }

  private:
    static const std::size_t maxSize = 5;
    std::deque<double> deque;

    double total = 0;
    double mean = 0;
  };

  struct PositionConfidenceEllipse {
    PositionsDeque x;
    PositionsDeque y;

    double semiMajorConfidence{};
    double semiMinorConfidence{};
    double semiMajorOrientation{};
  };
  PositionConfidenceEllipse positionConfidenceEllipse_;

  struct VelocityReport {
    rclcpp::Time stamp;
    float heading_rate;
    float lateral_velocity;
    float longitudinal_velocity;
    float speed;
    float longitudinal_acceleration;
  };
  VelocityReport velocityReport_;

  struct VehicleStatus {
    uint8_t gear;
    float steering_tire_angle;
  };
  VehicleStatus vehicleStatus_;

  bool sending_;
  bool is_sender_;
  bool publish_own_cams_;

  unsigned long stationId_;

  bool use_dynamic_generation_rules_;
};
}

#endif /* CAM_APPLICATION_HPP_EUIC2VFR */
