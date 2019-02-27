/*
 * Copyright 2017 Flyability SA, Jiadong Guo
 */

#ifndef ROTORS_AUTOMATEST_CONTROLLER_H
#define ROTORS_AUTOMATEST_CONTROLLER_H
#endif

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <rotors_control/common.h>

namespace flya_control {

static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(2, 2, 2);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(0.4, 0.4, 30);
static const Eigen::Vector3d kDefaultVelocityIntegralGain = Eigen::Vector3d(0.2, 0.2, 30);
static const Eigen::Vector3d kDefaultPositionSaturation = Eigen::Vector3d(0.2, 0.2, 30);
static const Eigen::Vector3d kDefaultVelocitySaturation = Eigen::Vector3d(0.4, 0.4, 60);
static const Eigen::Vector3d kDefaultVelocityIntegralSaturation =
    Eigen::Vector3d(0.2, 0.2, 30);
static const double kDefaultYawGain = 5.0;
static const double kDefaultYawRateGain = 0.0;
static const double kDefaultYawSaturation = 1.0;
static const double baseCommand = 50.0;

class WaypointWithSpeedTime {
 public:
  // WaypointWithSpeedTime()
  //     : waiting_time(0) {
  // }

  WaypointWithSpeedTime(double x, double y, double z, double _yaw)
      : position(x, y, z), yaw(_yaw)
  {
  }

  Eigen::Vector3d position, velocity;
  double yaw;  //
  double waiting_time;
};

class WaypointControllerParameters {
 public:
  WaypointControllerParameters()
      : position_gain_(kDefaultPositionGain),
        velocity_gain_(kDefaultVelocityGain),
        velocity_integral_gain_(kDefaultVelocityIntegralGain),
        position_saturation_(kDefaultPositionSaturation),
        velocity_saturation_(kDefaultVelocitySaturation),
        velocity_integral_saturation_(kDefaultVelocityIntegralSaturation),
        yaw_gain_(kDefaultYawGain),
        yaw_rate_gain_(kDefaultYawRateGain),
        yaw_saturation_(kDefaultYawSaturation)
  {
  }
  Eigen::Vector3d position_gain_, velocity_gain_, velocity_integral_gain_;
  Eigen::Vector3d position_saturation_, velocity_saturation_,
      velocity_integral_saturation_;
  double yaw_gain_, yaw_rate_gain_, yaw_saturation_;
};

class WaypointController {
 public:
  WaypointController();
  ~WaypointController();
  void InitializeParameters();
  void SetOdometry(const rotors_control::EigenOdometry& odometry);
  void SetDesiredWaypoint(const WaypointWithSpeedTime& waypoint);
  void CalculateJoystickCommands(mav_msgs::RollPitchYawrateThrust* joystick_cmd);
  WaypointControllerParameters controller_parameters_;

 private:
  void inBound(double& value, const double lower_bound, const double upper_bound) const;
  bool initialized_params_;
  rotors_control::EigenOdometry odometry_;
  Eigen::Vector3d position_des_;
  Eigen::Vector3d velocity_des_;
  double yaw_des_, previous_yaw_err_;
  double sample_dt_;
  Eigen::Vector3d previous_ui_;
};

}  // namespace flya_control
