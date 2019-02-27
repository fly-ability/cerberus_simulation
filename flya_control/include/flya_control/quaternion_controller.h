/*
 * Copyright 2017 Flyability SA, Jiadong Guo
 */

#ifndef ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_H
#define ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_H

#include <std_msgs/Bool.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace flya_control {

// Default values for the quaternion controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(12, 12, 9);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(6, 6, 4);
static const Eigen::Vector3d kDefaultAttitudeGainSaturation = Eigen::Vector3d(6, 6, 3);
static const Eigen::Vector3d kDefaultAngularRateGainSaturation =
    Eigen::Vector3d(40, 40, 40);
static const double DefaultMaxESC = 100.0;
static const double DefaultMinESC = 10.0;
static const double DefaultMaxRotVel = 2000.0;
static const double kDefaultYawNeglectThreshold = 40.0;
static const double kDefaultAltMin = -2.0;
static const double kDefaultAltMax = 2.5;
static const double kDefaultAltitudeGain = 5.0;
static const double kDefaultAltitudeRateGain = 40.0;
static const double kDefaultMinThrust = 30.0;
static const double kDefaultThrustApriori = 70.0;
static const double kDefaultAltStickGain = 0.015;
static const double kDefaultAltSaturation = 2.0;
class QuaternionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuaternionControllerParameters()
      : attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain),
        attitude_saturation_(kDefaultAttitudeGainSaturation),
        angular_rate_saturation_(kDefaultAngularRateGainSaturation),
        neglect_yaw_threshold_(kDefaultYawNeglectThreshold),
        max_rotor_velocity_(DefaultMaxRotVel),
        MAX_ESC_(DefaultMaxESC),
        MIN_ESC_(DefaultMinESC),
        altitude_min_(kDefaultAltMin),
        altitude_max_(kDefaultAltMax),
        altitude_gain_(kDefaultAltitudeGain),
        altitude_rate_gain_(kDefaultAltitudeRateGain),
        min_thrust_(kDefaultMinThrust),
        thrust_apriori_(kDefaultThrustApriori),
        alt_stick_gain_(kDefaultAltStickGain),
        delta_alt_saturation_(kDefaultAltSaturation)
  {
    allocation_matrix_ << -1, -1, -1, 1, 1, 1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1;
  }

  Eigen::Matrix4d allocation_matrix_;  // Relationship PID output to motor PWM
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d angular_rate_gain_;
  Eigen::Vector3d attitude_saturation_;
  Eigen::Vector3d angular_rate_saturation_;
  rotors_control::RotorConfiguration rotor_configuration_;
  double neglect_yaw_threshold_;
  double max_rotor_velocity_;
  double MAX_ESC_, MIN_ESC_;
  double altitude_min_, altitude_max_;
  double altitude_gain_, altitude_rate_gain_;
  double min_thrust_, thrust_apriori_;
  double alt_stick_gain_, delta_alt_saturation_;
};

class QuaternionController {
 public:
  QuaternionController();
  ~QuaternionController();
  // void InitializeParameters();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

  void SetOdometry(const rotors_control::EigenOdometry& odometry);
  void SetRollPitchYawrateThrust(
      const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust);

  QuaternionControllerParameters controller_parameters_;
  rotors_control::VehicleParameters vehicle_parameters_;
  bool alt_hold_mode_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  void InBound(double& value, const double lower_bound, const double upper_bound) const;
  bool initialized_params_;
  bool controller_active_;
  double yaw_des_, alt_des_;  // Keep track of desired yaw and altitude
  ros::Time last_command_time_;

  mav_msgs::EigenRollPitchYawrateThrust roll_pitch_yawrate_thrust_;
  rotors_control::EigenOdometry odometry_;

  void ComputePPoutput(Eigen::Vector3d* pp_output) const;
};
}  // namespace flya_control

#endif  // ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_H
