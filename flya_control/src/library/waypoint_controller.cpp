#include <math.h>

#include <rotors_control/common.h>
#include "flya_control/waypoint_controller.h"

namespace flya_control {

WaypointController::WaypointController()
    : initialized_params_(false),
      yaw_des_(0.0),
      previous_yaw_err_(0.0),
      sample_dt_(0.008333)
{  // Correspond to 120 Hz
  InitializeParameters();
  previous_ui_ = Eigen::Vector3d::Zero();
}
WaypointController::~WaypointController() {}

void WaypointController::InitializeParameters() { initialized_params_ = true; }

void WaypointController::SetOdometry(const rotors_control::EigenOdometry& odometry)
{
  odometry_ = odometry;
}
void WaypointController::SetDesiredWaypoint(const WaypointWithSpeedTime& waypoint)
{
  position_des_ = waypoint.position;
  yaw_des_ = waypoint.yaw;
  std::cerr << "waypoint position: " << position_des_.x() << " " << position_des_.y()
            << " " << position_des_.z();
  std::cerr << "  waypoint yaw: " << yaw_des_ << std::endl;
}
void WaypointController::inBound(double& value, const double lower_bound,
                                 const double upper_bound) const
{
  value = (value > upper_bound ? upper_bound : value);
  value = (value < lower_bound ? lower_bound : value);
}
void WaypointController::CalculateJoystickCommands(
    mav_msgs::RollPitchYawrateThrust* joystick_cmd)
{
  Eigen::Vector3d speed_sp;
  if (position_des_ != Eigen::Vector3d::Zero()) {
    // Part 1 controller for R/P/Thrust Command
    speed_sp = (position_des_ - odometry_.position)
                   .cwiseProduct(controller_parameters_.position_gain_);
  }
  else {
    // Speed controller
    speed_sp = velocity_des_;
  }

  // Saturation at outer P
  for (int i = 0; i < 3; i++) {
    inBound(speed_sp(i), -controller_parameters_.position_saturation_(i),
            controller_parameters_.position_saturation_(i));
  }

  // std::cerr << "Speed outer loop setpoint " << speed_sp << std::endl;

  // Speed in World frame
  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W = R_W_I * odometry_.velocity;
  Eigen::Vector3d PPIOut;
  if (speed_sp != Eigen::Vector3d::Zero()) {
    // Proportional Gain
    PPIOut = (speed_sp - velocity_W).cwiseProduct(controller_parameters_.velocity_gain_);

    // Integrator
    previous_ui_ +=
        sample_dt_ * (speed_sp - velocity_W)
                         .cwiseProduct(controller_parameters_.velocity_integral_gain_);

    // Saturation
    for (int i = 0; i < 3; i++) {
      inBound(previous_ui_(i), -controller_parameters_.velocity_integral_saturation_(i),
              controller_parameters_.velocity_integral_saturation_(i));
    }
    PPIOut += previous_ui_;
  }
  else {
    PPIOut = Eigen::Vector3d::Zero();
  }
  // Last Saturation
  for (int i = 0; i < 3; i++) {
    inBound(PPIOut(i), -controller_parameters_.velocity_saturation_(i),
            controller_parameters_.velocity_saturation_(i));
  }

  // std::cerr << "PPI output " << PPIOut << std::endl;

  // Assign Values, this ONLY WORKS IN ALT HOLD
  // Robust yaw calculation from quaternion . TODO make it a help function
  double t3 = 2.0 * (odometry_.orientation.w() * odometry_.orientation.z() +
                     odometry_.orientation.x() * odometry_.orientation.y());
  double t4 = 1.0 - 2.0 * (odometry_.orientation.y() * odometry_.orientation.y() +
                           odometry_.orientation.z() * odometry_.orientation.z());
  double cur_yaw = std::atan2(t3, t4);
  // std::cerr << "Yaw angle " << cur_yaw << std::endl;
  // The joystick cmd here is from 0 to 100, rescale
  joystick_cmd->pitch = (PPIOut(0) * cos(cur_yaw) + PPIOut(1) * sin(cur_yaw)) / 50 * 0.2;
  joystick_cmd->roll = (PPIOut(0) * sin(cur_yaw) - PPIOut(1) * cos(cur_yaw)) / 50 * 0.2;
  joystick_cmd->thrust.z = PPIOut(2) + baseCommand;

  // Now Yaw TODO For now only proportional is used

  double yaw_err = yaw_des_ - cur_yaw;
  while (yaw_err > M_PI) yaw_err -= (2 * M_PI);
  while (yaw_err < -M_PI) yaw_err += (2 * M_PI);
  double yaw_output = (yaw_err)*controller_parameters_.yaw_gain_;
  inBound(yaw_output, -controller_parameters_.yaw_saturation_,
          controller_parameters_.yaw_saturation_);
  joystick_cmd->yaw_rate = yaw_output / 100 * M_PI;
}

}  // namespace flya_control
