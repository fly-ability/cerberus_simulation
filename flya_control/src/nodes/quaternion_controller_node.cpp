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

#include "quaternion_controller_node.h"

#include "rotors_control/parameters_ros.h"

using rotors_control::GetRosParameter;

QuaternionControllerNode::QuaternionControllerNode()
{

  InitializeParams();

  ros::NodeHandle nh;

  cmd_roll_pitch_yawrate_thrust_sub_ =
      nh.subscribe(rotors_control::kDefaultCommandRollPitchYawrateThrustTopic, 1,
                   &QuaternionControllerNode::RollPitchYawrateThrustCallback, this);
  odometry_sub_ = nh.subscribe(rotors_control::kDefaultOdometryTopic, 1,
                               &QuaternionControllerNode::OdometryCallback, this,
                               ros::TransportHints().tcpNoDelay());

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      rotors_control::kDefaultCommandMotorSpeedTopic, 1);
  controller_timer_ = nh.createTimer(
      ros::Duration(0.001), &QuaternionControllerNode::PublishMotorVelocity, this);
  controller_timer_.start();
}

QuaternionControllerNode::~QuaternionControllerNode() {}

void QuaternionControllerNode::InitializeParams()
{
  ros::NodeHandle pnh("~");

  if (pnh.getParam(pnh.getNamespace() + "/alt_hold_mode",
                   quaternion_controller_.alt_hold_mode_)) {
    std::cerr << "Successfully loaded motor invertion status \n";
  }

  // Read parameters from rosparam.
  GetRosParameter(pnh, "attitude_gain/x",
                  quaternion_controller_.controller_parameters_.attitude_gain_.x(),
                  &quaternion_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  quaternion_controller_.controller_parameters_.attitude_gain_.y(),
                  &quaternion_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  quaternion_controller_.controller_parameters_.attitude_gain_.z(),
                  &quaternion_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  quaternion_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &quaternion_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  quaternion_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &quaternion_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  quaternion_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &quaternion_controller_.controller_parameters_.angular_rate_gain_.z());

  GetVehicleParameters(pnh, &quaternion_controller_.vehicle_parameters_);
  GetRosParameter(
      pnh, "attitude_saturation/x",
      quaternion_controller_.controller_parameters_.attitude_saturation_.x(),
      &quaternion_controller_.controller_parameters_.attitude_saturation_.x());

  GetRosParameter(
      pnh, "attitude_saturation/y",
      quaternion_controller_.controller_parameters_.attitude_saturation_.y(),
      &quaternion_controller_.controller_parameters_.attitude_saturation_.y());
  GetRosParameter(
      pnh, "attitude_saturation/z",
      quaternion_controller_.controller_parameters_.attitude_saturation_.z(),
      &quaternion_controller_.controller_parameters_.attitude_saturation_.z());
  GetRosParameter(
      pnh, "angular_rate_saturation/x",
      quaternion_controller_.controller_parameters_.angular_rate_saturation_.x(),
      &quaternion_controller_.controller_parameters_.angular_rate_saturation_.x());
  GetRosParameter(
      pnh, "angular_rate_saturation/y",
      quaternion_controller_.controller_parameters_.angular_rate_saturation_.y(),
      &quaternion_controller_.controller_parameters_.angular_rate_saturation_.y());
  GetRosParameter(
      pnh, "angular_rate_saturation/z",
      quaternion_controller_.controller_parameters_.angular_rate_saturation_.z(),
      &quaternion_controller_.controller_parameters_.angular_rate_saturation_.z());
  GetRosParameter(pnh, "neglect_yaw",
                  quaternion_controller_.controller_parameters_.neglect_yaw_threshold_,
                  &quaternion_controller_.controller_parameters_.neglect_yaw_threshold_);
  GetRosParameter(pnh, "max_esc", quaternion_controller_.controller_parameters_.MAX_ESC_,
                  &quaternion_controller_.controller_parameters_.MAX_ESC_);
  GetRosParameter(pnh, "min_esc", quaternion_controller_.controller_parameters_.MIN_ESC_,
                  &quaternion_controller_.controller_parameters_.MIN_ESC_);
  GetRosParameter(pnh, "max_rotor_velocity",
                  quaternion_controller_.controller_parameters_.max_rotor_velocity_,
                  &quaternion_controller_.controller_parameters_.max_rotor_velocity_);
  GetRosParameter(pnh, "alt_gain/z",
                  quaternion_controller_.controller_parameters_.altitude_gain_,
                  &quaternion_controller_.controller_parameters_.altitude_gain_);
  GetRosParameter(pnh, "alt_gain/vz",
                  quaternion_controller_.controller_parameters_.altitude_rate_gain_,
                  &quaternion_controller_.controller_parameters_.altitude_rate_gain_);
  GetRosParameter(pnh, "alt_min",
                  quaternion_controller_.controller_parameters_.altitude_min_,
                  &quaternion_controller_.controller_parameters_.altitude_min_);
  GetRosParameter(pnh, "alt_max",
                  quaternion_controller_.controller_parameters_.altitude_max_,
                  &quaternion_controller_.controller_parameters_.altitude_max_);
  GetRosParameter(pnh, "min_thrust",
                  quaternion_controller_.controller_parameters_.min_thrust_,
                  &quaternion_controller_.controller_parameters_.min_thrust_);
  GetRosParameter(pnh, "thrust_apriori",
                  quaternion_controller_.controller_parameters_.thrust_apriori_,
                  &quaternion_controller_.controller_parameters_.thrust_apriori_);
  GetRosParameter(pnh, "alt_stick_gain",
                  quaternion_controller_.controller_parameters_.alt_stick_gain_,
                  &quaternion_controller_.controller_parameters_.alt_stick_gain_);
  GetRosParameter(pnh, "delta_alt_saturation",
                  quaternion_controller_.controller_parameters_.delta_alt_saturation_,
                  &quaternion_controller_.controller_parameters_.delta_alt_saturation_);
}
void QuaternionControllerNode::Publish() {}

void QuaternionControllerNode::RollPitchYawrateThrustCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr&
        roll_pitch_yawrate_thrust_reference_msg)
{
  mav_msgs::EigenRollPitchYawrateThrust roll_pitch_yawrate_thrust;
  mav_msgs::eigenRollPitchYawrateThrustFromMsg(*roll_pitch_yawrate_thrust_reference_msg,
                                               &roll_pitch_yawrate_thrust);
  quaternion_controller_.SetRollPitchYawrateThrust(roll_pitch_yawrate_thrust);
}

void QuaternionControllerNode::OdometryCallback(
    const nav_msgs::OdometryConstPtr& odometry_msg)
{

  ROS_INFO_ONCE("QuaternionControllerNode got first odometry message.");

  rotors_control::EigenOdometry odometry;
  rotors_control::eigenOdometryFromMsg(odometry_msg, &odometry);
  quaternion_controller_.SetOdometry(odometry);
  // std::cerr << "Odometry read: "<< odometry.orientation.vec() << " At current time " <<
  // ros::Time::now().toSec() << std::endl; Todo(ffurrer): Do this in the conversions
  // header.
}
void QuaternionControllerNode::PublishMotorVelocity(const ros::TimerEvent& e)
{
  Eigen::VectorXd ref_rotor_velocities;
  quaternion_controller_.CalculateRotorVelocities(&ref_rotor_velocities);
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = ros::Time::now();
  motor_velocity_reference_pub_.publish(actuator_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quaternion_controller_node");

  QuaternionControllerNode QuaternionControllerNode;

  ros::spin();

  return 0;
}
