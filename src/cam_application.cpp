#include "autoware_v2x/cam_application.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/v2x_node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rclcpp/rclcpp.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>
#include <exception>
#include <random>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <string>

#include <etsi_its_cam_ts_msgs/msg/cam.hpp>
#include <etsi_its_cam_ts_coding/cam_ts_CAM.h>
#include <etsi_its_cam_ts_conversion/convertCAM.h>
#include <etsi_its_msgs_utils/cam_ts_access.hpp>

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>

#include <sqlite3.h>

using namespace vanetza;
using namespace std::chrono;

namespace v2x
{
  CamApplication::CamApplication(V2XNode * node, Runtime & rt, bool is_sender, bool publish_own_cams)
  : node_(node),
    runtime_(rt),
    cam_interval_(milliseconds(1000)),
    vehicleDimensions_(),
    ego_(),
    positionConfidenceEllipse_(),
    velocityReport_(),
    vehicleStatus_(),
    sending_(false),
    is_sender_(is_sender),
    publish_own_cams_(publish_own_cams),
    use_dynamic_generation_rules_(true)
  {
    RCLCPP_INFO(node_->get_logger(), "CamApplication started. is_sender: %d, publish_own_cams: %d", is_sender_, publish_own_cams_);
    set_interval(cam_interval_);
    //createTables();

    // Generate ID for this station
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<unsigned long> dis(0, 4294967295);
    stationId_ = dis(gen);

    node_->getVehicleDimensions();
  }

  void CamApplication::set_interval(Clock::duration interval) {
    cam_interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
  }

  static double calc_haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371e3;
    double dLat = (lat2 - lat1) * M_PI / 180;
    double dLon = (lon2 - lon1) * M_PI / 180;

    lat1 = lat1 * (M_PI / 180);
    lat2 = lat2 * (M_PI / 180);

    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
               std::sin(dLon / 2) * std::sin(dLon / 2) * std::cos(lat1) * std::cos(lat2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    return R * c;
  }

  void CamApplication::calc_interval() {
    if (use_dynamic_generation_rules_) {
      static struct {
        double speed;
        double heading;
        double latitude;
        double longitude;
      } lastCam = {0.0, 0.0, 0.0, 0.0};

      double latitude = ego_.latitude;
      double longitude = ego_.longitude;
      double heading = ego_.heading;
      double speed = velocityReport_.speed;

      double deltaSpeed = std::fabs(speed - lastCam.speed);
      double deltaHeading = std::fabs(heading - lastCam.heading);
      if (deltaHeading > 180.0) deltaHeading = 360.0 - deltaHeading;

      double distance = calc_haversine(lastCam.latitude, lastCam.longitude, latitude, longitude);

      const double distance_threshold = 4.0;
      const double speed_threshold = 0.5;
      const double heading_threshold = 4.0;

      if (distance > distance_threshold || deltaSpeed > speed_threshold || deltaHeading > heading_threshold) {
        int interval = static_cast<int>((distance / speed) * 1000);

        if (interval < 100) {
          cam_interval_ = milliseconds(100);
        } else if (interval > 1000) {
          cam_interval_ = milliseconds(1000);
        } else {
          cam_interval_ = milliseconds(interval);
        }

        lastCam.speed = speed;
        lastCam.heading = heading;
        lastCam.latitude = latitude;
        lastCam.longitude = longitude;
      } else {
        cam_interval_ = milliseconds(1000);
      }
    }
  }

  void CamApplication::schedule_timer() {
    runtime_.schedule(cam_interval_, std::bind(&CamApplication::on_timer, this, std::placeholders::_1));
  }

  void CamApplication::on_timer(vanetza::Clock::time_point) {
    calc_interval();
    schedule_timer();
    send();
  }

  CamApplication::PortType CamApplication::port() {
    return btp::ports::CAM;
  }

  void CamApplication::indicate(const Application::DataIndication &indication, Application::UpPacketPtr packet)
  {
    asn1::PacketVisitor<asn1::r2::Cam> visitor;
    std::shared_ptr<const asn1::r2::Cam> rec_cam_ptr = boost::apply_visitor(visitor, *packet);

    if (!rec_cam_ptr) {
      RCLCPP_INFO(node_->get_logger(), "[CamApplication::indicate] Received invalid CAM");
      return;
    }

    asn1::r2::Cam rec_cam = *rec_cam_ptr;
    auto now = std::chrono::system_clock::now();
    std::chrono::milliseconds now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    RCLCPP_INFO(node_->get_logger(), "[CamApplication::indicate] Received CAM from station with ID #%ld at %ld epoch time", rec_cam->header.stationId, now_ms.count());
    vanetza::facilities::print_indented(std::cout, rec_cam, "  ", 0);

    namespace access = etsi_its_cam_ts_msgs::access;
    cam_ts_msgs::CAM ros_cam;

    Vanetza_ITS2_ItsPduHeader_t &header = rec_cam->header;
    Vanetza_ITS2_CamPayload_t &cam = rec_cam->cam;
    Vanetza_ITS2_BasicContainer_t &basic_container = cam.camParameters.basicContainer;
    Vanetza_ITS2_BasicVehicleContainerHighFrequency_t &bvc = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;

    cam_ts_msgs::GenerationDeltaTime gdt;
    gdt.value = cam.generationDeltaTime;

    access::setItsPduHeader(ros_cam, header.stationId, header.protocolVersion);
    access::setGenerationDeltaTime(ros_cam, access::getUnixNanosecondsFromGenerationDeltaTime(gdt, std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()));
    access::setStationType(ros_cam, basic_container.stationType);
    access::setReferencePosition(ros_cam,
                                 basic_container.referencePosition.latitude / 1e7,
                                 basic_container.referencePosition.longitude / 1e7,
                                 basic_container.referencePosition.altitude.altitudeValue / 1e2,
                                 basic_container.referencePosition.positionConfidenceEllipse.semiMajorAxisLength / 1e1,
                                 basic_container.referencePosition.positionConfidenceEllipse.semiMinorAxisLength / 1e1,
                                 basic_container.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation / 1e2);
    access::setHeading(ros_cam, bvc.heading.headingValue / 10);
    access::setSpeed(ros_cam, bvc.speed.speedValue / 100);
    access::setVehicleDimensions(ros_cam, bvc.vehicleLength.vehicleLengthValue / 10, bvc.vehicleWidth / 10);
    access::setLongitudinalAcceleration(ros_cam, bvc.longitudinalAcceleration.value / 10);
    access::setDriveDirection(ros_cam, bvc.driveDirection);
    access::setCurvatureValue(ros_cam, bvc.curvature.curvatureValue / 10, bvc.curvatureCalculationMode);
    access::setYawRateValue(ros_cam, bvc.yawRate.yawRateValue / 10);

    node_->publishReceivedCam(ros_cam);
  }

  void CamApplication::updateMGRS(double *x, double *y) {
    ego_.mgrs_x = *x;
    ego_.mgrs_y = *y;
  }

  void CamApplication::updateRP(const double *lat, const double *lon, const double *altitude) {
    ego_.latitude = *lat;
    ego_.longitude = *lon;
    ego_.altitude = *altitude;

    positionConfidenceEllipse_.x.insert(*lat);
    positionConfidenceEllipse_.y.insert(*lon);
  }

  void CamApplication::updateHeading(const double *yaw) {
    ego_.heading = *yaw;
  }

  void CamApplication::setVehicleDimensions(const autoware_adapi_v1_msgs::msg::VehicleDimensions &msg) {
    if (msg == autoware_adapi_v1_msgs::msg::VehicleDimensions{}) {
      RCLCPP_WARN(node_->get_logger(), "[CamApplication::getVehicleDimensions] Vehicle dimensions not available");
      return;
    }

    vehicleDimensions_.wheel_radius = msg.wheel_radius;
    vehicleDimensions_.wheel_width = msg.wheel_width;
    vehicleDimensions_.wheel_base = msg.wheel_base;
    vehicleDimensions_.wheel_tread = msg.wheel_tread;
    vehicleDimensions_.front_overhang = msg.front_overhang;
    vehicleDimensions_.rear_overhang = msg.rear_overhang;
    vehicleDimensions_.left_overhang = msg.left_overhang;
    vehicleDimensions_.right_overhang = msg.right_overhang;
    vehicleDimensions_.height = msg.height;
  }

  void CamApplication::updateVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg) {
    rclcpp::Time msg_stamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
    double dt = msg_stamp.seconds() - velocityReport_.stamp.seconds();
    if (dt == 0) {
      RCLCPP_WARN(node_->get_logger(), "[CamApplication::updateVelocityReport] deltaTime is 0");
      return;
    }
    float longitudinal_acceleration = (msg->longitudinal_velocity - velocityReport_.longitudinal_velocity) / dt;
    float speed = std::sqrt(std::pow(msg->longitudinal_velocity, 2) + std::pow(msg->lateral_velocity, 2));

    velocityReport_.stamp = msg->header.stamp;
    velocityReport_.heading_rate = msg->heading_rate;
    velocityReport_.lateral_velocity = msg->lateral_velocity;
    velocityReport_.longitudinal_velocity = msg->longitudinal_velocity;
    velocityReport_.speed = speed;
    velocityReport_.longitudinal_acceleration = longitudinal_acceleration;
  }

  void CamApplication::updateGearReport(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg) {
    rclcpp::Time msg_stamp(msg->stamp.sec, msg->stamp.nanosec);
    vehicleStatus_.gear = msg->report;
  }

  void CamApplication::updateSteeringReport(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg) {
    vehicleStatus_.steering_tire_angle = msg->steering_tire_angle;
  }

  void CamApplication::send() {
    if (!is_sender_) return;

    if (sending_) {
      RCLCPP_WARN(node_->get_logger(), "[CamApplication::send] already sending");
      return;
    }

    sending_ = true;

    auto now = std::chrono::system_clock::now();
    std::chrono::nanoseconds now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());

    vanetza::asn1::r2::Cam message;

    Vanetza_ITS2_ItsPduHeader_t &header = message->header;
    header.protocolVersion = 2;
    header.messageId = Vanetza_ITS2_MessageId_cam;
    header.stationId = stationId_;

    Vanetza_ITS2_CamPayload_t &cam = message->cam;

    // Convert Unix timestamp to ETSI epoch (2004-01-01 00:00:00)
    cam.generationDeltaTime = cam.generationDeltaTime = static_cast<uint16_t>((now_ns.count() / 1000000 - 1072915200000ULL) % 65536);

    Vanetza_ITS2_BasicContainer_t &basic_container = cam.camParameters.basicContainer;
    basic_container.stationType = cam_ts_TrafficParticipantType_passengerCar;
    float latitude = ego_.latitude * 1e7;
    float longitude = ego_.longitude * 1e7;
    float altitude = ego_.altitude * 100;

    if (-900000000 <= latitude && latitude <= 900000000) basic_container.referencePosition.latitude = latitude;
    else basic_container.referencePosition.latitude = Vanetza_ITS2_Latitude_unavailable;
    if (-1800000000 <= longitude && longitude <= 1800000000) basic_container.referencePosition.longitude = longitude;
    else basic_container.referencePosition.longitude = Vanetza_ITS2_Longitude_unavailable;
    if (-100000 <= altitude && altitude <= 800000) basic_container.referencePosition.altitude.altitudeValue = altitude;
    else basic_container.referencePosition.altitude.altitudeValue = Vanetza_ITS2_AltitudeValue_unavailable;

    // Articles consulted for the positionConficenceEllipse
    // https://users.cs.utah.edu/~tch/CS4640F2019/resources/A%20geometric%20interpretation%20of%20the%20covariance%20matrix.pdf
    // https://users.cs.utah.edu/~tch/CS6640F2020/resources/How%20to%20draw%20a%20covariance%20error%20ellipse.pdf
    if (positionConfidenceEllipse_.x.getSize() == positionConfidenceEllipse_.y.getSize()) {
      double xx_sum = 0;
      double yy_sum = 0;
      double xy_sum = 0;
      for (double x : positionConfidenceEllipse_.x)
        xx_sum += std::pow(x - positionConfidenceEllipse_.x.getMean(), 2);
      for (double y : positionConfidenceEllipse_.y)
        yy_sum += std::pow(y - positionConfidenceEllipse_.y.getMean(), 2);
      for (int i = 0; i < positionConfidenceEllipse_.x.getSize(); i++)
        xy_sum += (positionConfidenceEllipse_.x[i] - positionConfidenceEllipse_.x.getMean()) *
                  (positionConfidenceEllipse_.y[i] - positionConfidenceEllipse_.y.getMean());

      double sigma_xx = xx_sum / (positionConfidenceEllipse_.x.getSize() - 1);
      double sigma_yy = yy_sum / (positionConfidenceEllipse_.y.getSize() - 1);
      double sigma_xy = xy_sum / (positionConfidenceEllipse_.x.getSize() - 1);

      double lambda1 = (sigma_xx + sigma_yy) - std::sqrt(std::pow(sigma_xx + sigma_yy, 2) - 4 * (sigma_xx * sigma_yy - sigma_xy * sigma_xy)) / 2;
      double lambda2 = (sigma_xx + sigma_yy) + std::sqrt(std::pow(sigma_xx + sigma_yy, 2) - 4 * (sigma_xx * sigma_yy - sigma_xy * sigma_xy)) / 2;

      double lambda_max = std::max(lambda1, lambda2);
      double lambda_min = std::min(lambda1, lambda2);

      // For 95% confidence level, must use 2.4477
      double majorAxisLength = std::lround(2.4477 * std::sqrt(lambda_max));
      double minorAxisLength = std::lround(2.4477 * std::sqrt(lambda_min));
      double majorAxisOrientation = - (sigma_xy != 0
                                   ? std::lround(std::atan(- (sigma_xx - lambda_max) / sigma_xy) * 180 / M_PI)
                                   : sigma_xx != 0 ? 0 : -90) * 10;
      if (majorAxisOrientation < 0) majorAxisOrientation += 3600;

      if (0 <= majorAxisLength && majorAxisLength <= 4094) basic_container.referencePosition.positionConfidenceEllipse.semiMajorAxisLength = majorAxisLength;
      else basic_container.referencePosition.positionConfidenceEllipse.semiMajorAxisLength = cam_ts_SemiAxisLength_unavailable;
      if (0 <= minorAxisLength && minorAxisLength <= 4094) basic_container.referencePosition.positionConfidenceEllipse.semiMinorAxisLength = minorAxisLength;
      else basic_container.referencePosition.positionConfidenceEllipse.semiMinorAxisLength = cam_ts_SemiAxisLength_unavailable;
      if (0 <= majorAxisOrientation && majorAxisOrientation <= 3600) basic_container.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation = majorAxisOrientation;
      else basic_container.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation = cam_ts_Wgs84AngleValue_unavailable;
    } else {
      basic_container.referencePosition.positionConfidenceEllipse.semiMajorAxisLength = cam_ts_SemiAxisLength_unavailable;
      basic_container.referencePosition.positionConfidenceEllipse.semiMinorAxisLength = cam_ts_SemiAxisLength_unavailable;
      basic_container.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation = cam_ts_Wgs84AngleValue_unavailable;
    }

    Vanetza_ITS2_BasicVehicleContainerHighFrequency_t &bvc = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
    cam.camParameters.highFrequencyContainer.present = Vanetza_ITS2_HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    int heading = std::lround(((-ego_.heading * 180.0 / M_PI) + 90.0) * 10.0);
    if (heading < 0) heading += 3600;
    if (0 <= heading && heading <= 3600) bvc.heading.headingValue = heading;
    else bvc.heading.headingValue = Vanetza_ITS2_HeadingValue_unavailable;

    float heading_rate = velocityReport_.heading_rate;
    float lateral_velocity = velocityReport_.lateral_velocity;
    float longitudinal_velocity = velocityReport_.longitudinal_velocity;
    float longitudinal_acceleration = std::lround(velocityReport_.longitudinal_acceleration * 100);
    uint8_t gearStatus = vehicleStatus_.gear;
    float steering_tire_angle = vehicleStatus_.steering_tire_angle;

    long speed = std::lround(velocityReport_.speed * 100);
    if (0 <= speed && speed <= 16382) bvc.speed.speedValue = speed;
    else bvc.speed.speedValue = Vanetza_ITS2_SpeedValue_unavailable;

    if ((gearStatus >= 2 && gearStatus <= 19) || (gearStatus == 23 || gearStatus == 24))
      bvc.driveDirection = Vanetza_ITS2_DriveDirection_forward;
    else if (gearStatus == 20 || gearStatus == 21)
      bvc.driveDirection = Vanetza_ITS2_DriveDirection_backward;
    else
      bvc.driveDirection = Vanetza_ITS2_DriveDirection_unavailable;

    long vehicleLength = std::lround((vehicleDimensions_.front_overhang + vehicleDimensions_.wheel_base + vehicleDimensions_.rear_overhang) * 10);
    if (1 <= vehicleLength && vehicleLength <= 1022) bvc.vehicleLength.vehicleLengthValue = vehicleLength;
    else bvc.vehicleLength.vehicleLengthValue = Vanetza_ITS2_VehicleLengthValue_unavailable;

    long vehicleWidth = std::lround((vehicleDimensions_.left_overhang + vehicleDimensions_.wheel_tread + vehicleDimensions_.right_overhang) * 10);
    if (1 <= vehicleWidth && vehicleWidth <= 61) bvc.vehicleWidth = vehicleWidth;
    else bvc.vehicleWidth = Vanetza_ITS2_VehicleWidth_unavailable;

    if (-160 <= longitudinal_acceleration && longitudinal_acceleration <= 160) bvc.longitudinalAcceleration.value = longitudinal_acceleration;
    else bvc.longitudinalAcceleration.value = cam_ts_AccelerationValue_unavailable;

    long curvature = longitudinal_velocity != 0 ? std::abs(std::lround(lateral_velocity / std::pow(longitudinal_velocity, 2) * 100)) * (steering_tire_angle < 0 ? -1 : 1)
                                                : std::numeric_limits<long>::infinity();
    if (-1023 <= curvature && curvature <= 1022) bvc.curvature.curvatureValue = curvature;
    else bvc.curvature.curvatureValue = Vanetza_ITS2_CurvatureValue_unavailable;
    bvc.curvatureCalculationMode = Vanetza_ITS2_CurvatureCalculationMode_yawRateNotUsed;

    long heading_rate_deg = std::abs(std::lround(heading_rate * (180.0 / M_PI))) * (steering_tire_angle < 0 ? -1 : 1);
    if (-32766 <= heading_rate_deg && heading_rate_deg <= 32766) bvc.yawRate.yawRateValue = heading_rate_deg;
    else bvc.yawRate.yawRateValue = YawRateValue_unavailable;

    // UNAVAILABLE VALUES
    basic_container.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    // ------------------------------
    bvc.heading.headingConfidence = HeadingConfidence_unavailable;
    bvc.speed.speedConfidence = SpeedConfidence_unavailable;
    bvc.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;
    bvc.longitudinalAcceleration.confidence = cam_ts_AccelerationConfidence_unavailable;
    bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvc.yawRate.yawRateConfidence = YawRateConfidence_unavailable;
    // ------------------------------

    RCLCPP_INFO(node_->get_logger(), "[CamApplication::send] Sending CAM from station with ID %ld with generationDeltaTime %ld, latitude %f, longitude %f, altitude %f",
            stationId_, cam.generationDeltaTime, ego_.latitude, ego_.longitude, ego_.altitude);
    std::unique_ptr<geonet::DownPacket> payload{new geonet::DownPacket()};
    payload->layer(OsiLayer::Application) = std::move(message);

    Application::DataRequest request;
    request.its_aid = aid::CP;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    Application::DataConfirm confirm = Application::request(request, std::move(payload), node_);

    if (!confirm.accepted()) {
      throw std::runtime_error("[CamApplication::send] CAM application data request failed");
    }

    if (publish_own_cams_) {
      namespace access = etsi_its_cam_ts_msgs::access;
      cam_ts_msgs::CAM ros_cam;

      access::setItsPduHeader(ros_cam, header.stationId, header.protocolVersion);
      access::setGenerationDeltaTime(ros_cam, now_ns.count(), 0);
      access::setStationType(ros_cam, basic_container.stationType);
      access::setReferencePosition(ros_cam,
                                   basic_container.referencePosition.latitude / 1e7,
                                   basic_container.referencePosition.longitude / 1e7,
                                   basic_container.referencePosition.altitude.altitudeValue / 1e2,
                                   basic_container.referencePosition.positionConfidenceEllipse.semiMajorAxisLength / 1e1,
                                   basic_container.referencePosition.positionConfidenceEllipse.semiMinorAxisLength / 1e1,
                                   basic_container.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation / 1e2);
      access::setHeading(ros_cam, bvc.heading.headingValue / 10);
      access::setSpeed(ros_cam, bvc.speed.speedValue / 100);
      access::setVehicleDimensions(ros_cam, bvc.vehicleLength.vehicleLengthValue / 10, bvc.vehicleWidth / 10);
      access::setLongitudinalAcceleration(ros_cam, bvc.longitudinalAcceleration.value / 10);
      access::setDriveDirection(ros_cam, bvc.driveDirection);
      access::setCurvatureValue(ros_cam, bvc.curvature.curvatureValue / 10, bvc.curvatureCalculationMode);
      access::setYawRateValue(ros_cam, bvc.yawRate.yawRateValue / 10);

      node_->publishReceivedCam(ros_cam);
      RCLCPP_INFO(node_->get_logger(), "[CamApplication::send] Published own CAM to ROS network");
    }

    RCLCPP_INFO(node_->get_logger(), "[CamApplication::send] Successfully sent");

    sending_ = false;

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds> (
      std::chrono::system_clock::now().time_since_epoch()
    );
  }

}
