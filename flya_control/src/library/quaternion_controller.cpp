/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <math.h>

#include "flya_control/quaternion_controller.h"

namespace flya_control {

QuaternionController::QuaternionController()
    : initialized_params_(false), controller_active_(false), yaw_des_(0.0), alt_des_(0.0)
{
  // InitializeParameters();
}

QuaternionController::~QuaternionController() {}

void QuaternionController::CalculateRotorVelocities(
    Eigen::VectorXd* rotor_velocities) const
{
  assert(rotor_velocities);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    // No action if collision is happening
    ROS_INFO("Controller turned off, set velocity all to zero");
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d pp_output = Eigen::Vector3d::Zero();

  ComputePPoutput(&pp_output);

  // Check inner loop saturation
  for (int i = 0; i < 3; i++) {
    if (pp_output(i) > controller_parameters_.angular_rate_saturation_(i)) {
      pp_output(i) = controller_parameters_.angular_rate_saturation_(i);
    }
    else if (pp_output(i) < -controller_parameters_.angular_rate_saturation_(i)) {
      pp_output(i) = -controller_parameters_.angular_rate_saturation_(i);
    }
  }

  // Check Roll Pitch saturation
  if ((abs(pp_output(0) + abs(pp_output(1)))) >
      controller_parameters_.neglect_yaw_threshold_) {
    pp_output(2) = 0;  // Set yaw output as zero if the raw and pitch too big already
  }

  // All outputs in ESC command
  Eigen::Vector4d pp_output_thrust;
  pp_output_thrust.block<3, 1>(0, 0) = pp_output;

  if (alt_hold_mode_) {  // PP Controller for altitude
    double alt_err = (alt_des_ - odometry_.position.z());
    // std::cerr << "desired alt" << alt_des_ << "real " <<
    // odometry_.position.z()<<std::endl;

    InBound(alt_err, -controller_parameters_.delta_alt_saturation_,
            controller_parameters_.delta_alt_saturation_);

    double outer_alt_output = alt_err * controller_parameters_.altitude_gain_;

    InBound(outer_alt_output, controller_parameters_.altitude_min_,
            controller_parameters_.altitude_max_);
    // std::cerr << "desired vspeed" << outer_alt_output << std::endl;

    const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
    Eigen::Vector3d velocity_W = R_W_I * odometry_.velocity;
    double inner_alt_output =
        (outer_alt_output - velocity_W.z()) * controller_parameters_.altitude_rate_gain_;

    double raw_thrust = controller_parameters_.thrust_apriori_ + inner_alt_output;
    InBound(raw_thrust, controller_parameters_.min_thrust_,
            controller_parameters_.MAX_ESC_);
    // std::cerr << "raw thrust" << raw_thrust <<std::endl;

    pp_output_thrust(3) = raw_thrust;
  }
  else {
    pp_output_thrust(3) = roll_pitch_yawrate_thrust_.thrust.z();
  }

  // Translate into ESC command
  Eigen::Vector4d ESC_commands =
      controller_parameters_.allocation_matrix_ * pp_output_thrust;
  // std::cerr << "ESC commands read" << ESC_commands << std::endl;

  // If out of bound, apply scaling and offset
  Eigen::Vector4d::Index maxIndex, minIndex;
  double scale = 1.0, offset = 0;
  double max = ESC_commands.maxCoeff(&maxIndex);
  double min = ESC_commands.minCoeff(&minIndex);
  if ((min < controller_parameters_.MIN_ESC_) ||
      (max > controller_parameters_.MAX_ESC_)) {
    double min_max_delta = max - min;
    if (min_max_delta >
        (controller_parameters_.MAX_ESC_ - controller_parameters_.MIN_ESC_)) {
      scale = (controller_parameters_.MAX_ESC_ - controller_parameters_.MIN_ESC_) /
              min_max_delta;
      offset = controller_parameters_.MAX_ESC_ - scale * max;
    }
    else {
      if (min < controller_parameters_.MIN_ESC_) {
        offset = controller_parameters_.MIN_ESC_ - min;
      }
      else {
        offset = controller_parameters_.MAX_ESC_ - max;
      }
    }
  }
  Eigen::Vector4d offset_vec(offset, offset, offset, offset);
  ESC_commands *= scale;
  ESC_commands += offset_vec;

  // Transform ESC command into rotor velocities
  *rotor_velocities = ESC_commands * (0.01 * controller_parameters_.max_rotor_velocity_);
}

void QuaternionController::SetOdometry(const rotors_control::EigenOdometry& odometry)
{
  odometry_ = odometry;
}

void QuaternionController::InBound(double& value, const double lower_bound,
                                   const double upper_bound) const
{
  if (value > upper_bound) value = upper_bound;
  if (value < lower_bound) value = lower_bound;
}

void QuaternionController::SetRollPitchYawrateThrust(
    const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust)
{
  roll_pitch_yawrate_thrust_ = roll_pitch_yawrate_thrust;
  if (!controller_active_) {  // First time calling
    last_command_time_ = ros::Time::now();
    controller_active_ = true;
  }
  else {
    yaw_des_ += ((ros::Time::now() - last_command_time_).toSec() *
                 roll_pitch_yawrate_thrust_.yaw_rate);
    if (alt_hold_mode_) {
      alt_des_ += ((ros::Time::now() - last_command_time_).toSec() * 2.0 *
                   (roll_pitch_yawrate_thrust_.thrust.z() - 50.0) *
                   controller_parameters_.alt_stick_gain_);
      // std::cerr << "read joystick " <<  roll_pitch_yawrate_thrust_.thrust.z() << "now
      // desired" << alt_des_ <<std::endl;
    }
    last_command_time_ = ros::Time::now();
    while (yaw_des_ > M_PI) {
      yaw_des_ -= (2 * M_PI);
    }
    while (yaw_des_ < -M_PI) {
      yaw_des_ += (2 * M_PI);
    }
  }
}

// Control of for a quadrotor UAV using quaternions
void QuaternionController::ComputePPoutput(Eigen::Vector3d* pp_output) const
{
  assert(pp_output);
  // Get the desired quaternion

  Eigen::Matrix3d R_des;

  // mimicing yaw-pitch-roll
  R_des = Eigen::AngleAxisd(yaw_des_, Eigen::Vector3d::UnitZ())  // yaw
          * Eigen::AngleAxisd(roll_pitch_yawrate_thrust_.pitch,
                              Eigen::Vector3d::UnitY())  // pitch
          * Eigen::AngleAxisd(roll_pitch_yawrate_thrust_.roll,
                              Eigen::Vector3d::UnitX());  // roll

  Eigen::Quaterniond q_des(R_des);  // Initialize with the desired rotation matrix

  Eigen::Quaterniond q_err_W = q_des * (odometry_.orientation.conjugate());
  if (q_err_W.w() < 0) {
    q_err_W = q_err_W.conjugate();
  }

  Eigen::Vector3d outer_p_output = controller_parameters_.attitude_gain_.cwiseProduct(
      odometry_.orientation.conjugate()._transformVector(q_err_W.vec()));
  //

  // Check saturation
  for (int i = 0; i < 3; i++) {
    if (outer_p_output(i) > controller_parameters_.attitude_saturation_(i)) {
      outer_p_output(i) = controller_parameters_.attitude_saturation_(i);
    }
    else if (outer_p_output(i) < -controller_parameters_.attitude_saturation_(i)) {
      outer_p_output(i) = -controller_parameters_.attitude_saturation_(i);
    }
  }

  // Outer loop gives desired angular velocity
  *pp_output = (outer_p_output - odometry_.angular_velocity)
                   .cwiseProduct(controller_parameters_.angular_rate_gain_);
}
}  // namespace flya_control
