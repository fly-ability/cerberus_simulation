#ifndef AUTOMATEST_CONTROLLER_NODE_H
#define AUTOMATEST_CONTROLLER_NODE_H

#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <Eigen/Eigen>

#include <flya_control/Waypoint.h>
#include <flya_control/waypoint_controller.h>
#include <rotors_control/common.h>

class WaypointControllerNode {
 public:
  WaypointControllerNode();
  ~WaypointControllerNode();

  void LoadWaypoints();
  void PublishCommand(const ros::TimerEvent& e_pub);
  void InitializeParams();

 private:
  void LiveWaypointUpdateCallback(const flya_control::Waypoint::ConstPtr& waypoint_msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  double task_time_;
  bool simulation_ready_;  // Start when receiving odometry_msg
  int repetition_;
  int current_ind_;  // Debug purpose
  flya_control::WaypointController waypoint_controller_;
  std::string namespace_;
  std::string input_file_name_;
  ros::Subscriber waypoint_subscriber_;
  ros::Subscriber odometry_sub_;
  ros::Timer publish_timer_;  // 120Hz
  ros::Publisher waypoint_pub_;
};

#endif
