#include <math.h>
#include <algorithm>

#include <ros/ros.h>
#include <Eigen/Eigen>  // FIXME: this has to be before including rotors_control/parameters_ros.h

#include <rotors_control/parameters_ros.h>

#include <flya_control/waypoint_controller_node.h>

using namespace flya_control;

WaypointControllerNode::WaypointControllerNode() : repetition_(0), current_ind_(0)
{
  InitializeParams();
  WaypointWithSpeedTime init_waypoint = WaypointWithSpeedTime(0, 0, 1, 90 * M_PI / 180);
  waypoint_controller_.SetDesiredWaypoint(init_waypoint);

  ros::NodeHandle nh;

  waypoint_subscriber_ = nh.subscribe<flya_control::Waypoint>(
      "/elios/waypoint", 1, &WaypointControllerNode::LiveWaypointUpdateCallback, this);

  odometry_sub_ = nh.subscribe(rotors_control::kDefaultOdometryTopic, 1,
                               &WaypointControllerNode::OdometryCallback, this,
                               ros::TransportHints().tcpNoDelay());

  waypoint_pub_ = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
      rotors_control::kDefaultCommandRollPitchYawrateThrustTopic, 1);
  publish_timer_ = nh.createTimer(ros::Duration(0.08333),
                                  &WaypointControllerNode::PublishCommand, this);
  publish_timer_.start();
}

WaypointControllerNode::~WaypointControllerNode() {}

void WaypointControllerNode::PublishCommand(const ros::TimerEvent& e_pub)
{
  mav_msgs::RollPitchYawrateThrust joystick_cmd;
  waypoint_controller_.CalculateJoystickCommands(&joystick_cmd);
  ros::Time update_time = ros::Time::now();
  joystick_cmd.header.stamp = update_time;
  joystick_cmd.header.frame_id = "rotors_joy_frame";
  waypoint_pub_.publish(joystick_cmd);
}

void WaypointControllerNode::LiveWaypointUpdateCallback(
    const flya_control::Waypoint::ConstPtr& msg)
{
  WaypointWithSpeedTime cur_waypoint =
      WaypointWithSpeedTime(msg->x, msg->y, msg->z, msg->yaw);
  waypoint_controller_.SetDesiredWaypoint(cur_waypoint);
}

void WaypointControllerNode::OdometryCallback(
    const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("WaypointControllerNode got first odometry message.");
  if (!simulation_ready_) {
    simulation_ready_ = true;
    std::cerr << "moving to initial waypoint "
              << "\n";
  }
  rotors_control::EigenOdometry odometry;
  rotors_control::eigenOdometryFromMsg(odometry_msg, &odometry);
  waypoint_controller_.SetOdometry(odometry);
}

void WaypointControllerNode::InitializeParams()
{
  ros::NodeHandle pnh("~");
  // Read parameters from rosparam.
  rotors_control::GetRosParameter(pnh, "repetition", repetition_, &repetition_);
  rotors_control::GetRosParameter(pnh, "file_name", input_file_name_, &input_file_name_);
  // Controller parameters
  rotors_control::GetRosParameter(
      pnh, "position_gain/x",
      waypoint_controller_.controller_parameters_.position_gain_.x(),
      &waypoint_controller_.controller_parameters_.position_gain_.x());
  rotors_control::GetRosParameter(
      pnh, "position_gain/y",
      waypoint_controller_.controller_parameters_.position_gain_.y(),
      &waypoint_controller_.controller_parameters_.position_gain_.y());
  rotors_control::GetRosParameter(
      pnh, "position_gain/z",
      waypoint_controller_.controller_parameters_.position_gain_.z(),
      &waypoint_controller_.controller_parameters_.position_gain_.z());
  rotors_control::GetRosParameter(
      pnh, "velocity_gain/x",
      waypoint_controller_.controller_parameters_.velocity_gain_.x(),
      &waypoint_controller_.controller_parameters_.velocity_gain_.x());
  rotors_control::GetRosParameter(
      pnh, "velocity_gain/y",
      waypoint_controller_.controller_parameters_.velocity_gain_.y(),
      &waypoint_controller_.controller_parameters_.velocity_gain_.y());
  rotors_control::GetRosParameter(
      pnh, "velocity_gain/z",
      waypoint_controller_.controller_parameters_.velocity_gain_.z(),
      &waypoint_controller_.controller_parameters_.velocity_gain_.z());
  rotors_control::GetRosParameter(
      pnh, "velocity_integral_gain/x",
      waypoint_controller_.controller_parameters_.velocity_integral_gain_.x(),
      &waypoint_controller_.controller_parameters_.velocity_integral_gain_.x());
  rotors_control::GetRosParameter(
      pnh, "velocity_integral_gain/y",
      waypoint_controller_.controller_parameters_.velocity_integral_gain_.y(),
      &waypoint_controller_.controller_parameters_.velocity_integral_gain_.y());
  rotors_control::GetRosParameter(
      pnh, "velocity_integral_gain/z",
      waypoint_controller_.controller_parameters_.velocity_integral_gain_.z(),
      &waypoint_controller_.controller_parameters_.velocity_integral_gain_.z());
  rotors_control::GetRosParameter(
      pnh, "position_saturation/x",
      waypoint_controller_.controller_parameters_.position_saturation_.x(),
      &waypoint_controller_.controller_parameters_.position_saturation_.x());
  rotors_control::GetRosParameter(
      pnh, "position_saturation/y",
      waypoint_controller_.controller_parameters_.position_saturation_.y(),
      &waypoint_controller_.controller_parameters_.position_saturation_.y());
  rotors_control::GetRosParameter(
      pnh, "position_saturation/z",
      waypoint_controller_.controller_parameters_.position_saturation_.z(),
      &waypoint_controller_.controller_parameters_.position_saturation_.z());
  rotors_control::GetRosParameter(
      pnh, "velocity_saturation/x",
      waypoint_controller_.controller_parameters_.velocity_saturation_.x(),
      &waypoint_controller_.controller_parameters_.velocity_saturation_.x());
  rotors_control::GetRosParameter(
      pnh, "velocity_saturation/y",
      waypoint_controller_.controller_parameters_.velocity_saturation_.y(),
      &waypoint_controller_.controller_parameters_.velocity_saturation_.y());
  rotors_control::GetRosParameter(
      pnh, "velocity_saturation/z",
      waypoint_controller_.controller_parameters_.velocity_saturation_.z(),
      &waypoint_controller_.controller_parameters_.velocity_saturation_.z());
  rotors_control::GetRosParameter(
      pnh, "velocity_integral_saturation/x",
      waypoint_controller_.controller_parameters_.velocity_integral_saturation_.x(),
      &waypoint_controller_.controller_parameters_.velocity_integral_saturation_.x());
  rotors_control::GetRosParameter(
      pnh, "velocity_integral_saturation/y",
      waypoint_controller_.controller_parameters_.velocity_integral_saturation_.y(),
      &waypoint_controller_.controller_parameters_.velocity_integral_saturation_.y());
  rotors_control::GetRosParameter(
      pnh, "velocity_integral_saturation/z",
      waypoint_controller_.controller_parameters_.velocity_integral_saturation_.z(),
      &waypoint_controller_.controller_parameters_.velocity_integral_saturation_.z());
  rotors_control::GetRosParameter(pnh, "yaw_gain",
                                  waypoint_controller_.controller_parameters_.yaw_gain_,
                                  &waypoint_controller_.controller_parameters_.yaw_gain_);
  rotors_control::GetRosParameter(
      pnh, "yaw_rate_gain", waypoint_controller_.controller_parameters_.yaw_rate_gain_,
      &waypoint_controller_.controller_parameters_.yaw_rate_gain_);
  rotors_control::GetRosParameter(
      pnh, "yaw_saturation", waypoint_controller_.controller_parameters_.yaw_saturation_,
      &waypoint_controller_.controller_parameters_.yaw_saturation_);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_controller_node");
  WaypointControllerNode node;
  ros::spin();

  return 0;
}
