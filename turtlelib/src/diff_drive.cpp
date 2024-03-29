#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
DiffDrive::DiffDrive(double track_width, double wheel_radius)
:  wheel_r(wheel_radius), track_w(track_width) {}

DiffDrive::DiffDrive(double track_width, double wheel_radius, state location)
: config(location), wheel_r(wheel_radius), track_w(track_width) {}


DiffDrive::DiffDrive(
  double track_width, double wheel_radius, state location, double left,
  double right)
: config(location), wheelPos({left, right}), wheel_r(wheel_radius), track_w(track_width) {}


DiffDrive::DiffDrive(double track_width, double wheel_radius, double left, double right)
: wheelPos({left, right}), wheel_r(wheel_radius), track_w(track_width) {}


// ############################ Begin_Citation [4] ############################
Twist2D DiffDrive::get_body_twist(const double new_left, const double new_right)
{
  // Velocity from new positions
  // Refer to the second page of doc/Kinematics.pdf, step 1
  wheels vel = {new_left - wheelPos.l, new_right - wheelPos.r};

  // Twist for calculations
  // Refer to the second page of doc/Kinematics.pdf, step 2
  return {
    wheel_r / (track_w * 2.0) * (vel.r - vel.l),
    wheel_r / 2.0 * (vel.r + vel.l),
    0.0
  };
}

state DiffDrive::fk(const double new_left, const double new_right)
{
  Twist2D tw = get_body_twist(new_left, new_right);

  // Update wheel angles
  wheelPos = {new_left, new_right};

  // Transform needed to get to new wheel positions
  // Refer to the second page of doc/Kinematics.pdf, step 3
  Transform2D t = integrate_twist(tw);

  // Transform from world to base
  Transform2D t_wb = {{config.x, config.y}, config.th};

  // Calculate transform to new location of the robot
  // Refer to the second page of doc/Kinematics.pdf, step 4
  Transform2D t_wb_new = t_wb * t;

  // Update config based on calculations
  config = {
    t_wb_new.translation().x,
    t_wb_new.translation().y,
    t_wb_new.rotation()
  };
  return config;
}

wheels DiffDrive::ik(const Twist2D & tw)
{
  // diff drive robots cannot move in the y direction
  if (!almost_equal(tw.y, 0.0)) {
    throw std::logic_error("Twist is invaled for a differential drive robot");
  }
  return wheels{       // Refer to the first page of doc/Kinematics.pdf
    (tw.x - tw.omega * track_w) / wheel_r,
    (tw.x + tw.omega * track_w) / wheel_r,
  };
}
// ############################ End_Citation [4]  #############################

state DiffDrive::get_config()
{
  return config;
}

wheels DiffDrive::set_wheels(wheels pos)
{
  wheelPos = pos;
  return pos;
}

wheels DiffDrive::get_wheels()
{
  return wheelPos;
}

void DiffDrive::set_config(state location)
{
  config = location;
}

}
