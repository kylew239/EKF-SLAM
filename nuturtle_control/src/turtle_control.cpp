#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

// message includes
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"


using namespace std::chrono_literals;

class Turtle_control : public rclcpp::Node
{
public:
  Turtle_control()
  : Node("turtle_control")
  {
    // Parameters
    declare_parameter("rate", 200.0);
    declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("track_width", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("motor_cmd_max", rclcpp::PARAMETER_INTEGER);
    declare_parameter("motor_cmd_per_rad_sec", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("encoder_ticks_per_rad", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("collision_radius", rclcpp::PARAMETER_DOUBLE);

    // Get values for params
    rate_ = get_parameter("rate").as_double();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_int();
    motor_cmd_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    enc_ticks_ = get_parameter("encoder_ticks_per_rad").as_double();
    coll_radius_ = get_parameter("collision_radius").as_double();

    // Check if parameters are defined
    if (get_parameter("wheel_radius").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("track_width").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("motor_cmd_max").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("motor_cmd_per_rad_sec").get_type() ==
      rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("encoder_ticks_per_rad").get_type() ==
      rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("collision_radius").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      rclcpp::shutdown();
    }

    // Vars
    js_.header.frame_id = "red/base_link";
    js_.name = {"wheel_left_link", "wheel_right_link"};
    js_.position = {0.0, 0.0};
    js_.velocity = {0.0, 0.0};
    diff_drive = {track_width_, wheel_radius_};
    sensor_step_ = 0.0;
    prev_sensor_ = this->now();

    // Pubishers
    wheel_cmd_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscribers
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(
        &Turtle_control::cmd_vel_cb, this,
        std::placeholders::_1));

    sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(
        &Turtle_control::sensor_data_sub_cb_, this,
        std::placeholders::_1));

    // Timer
    timer_ = create_wall_timer(1s / rate_, std::bind(&Turtle_control::timer_callback, this));
  }

private:
  // Parameters
  double wheel_radius_, track_width_, motor_cmd_max_;
  double motor_cmd_rad_sec_, enc_ticks_, coll_radius_;
  double rate_;

  // Vars
  turtlelib::DiffDrive diff_drive = {track_width_, wheel_radius_};
  turtlelib::Twist2D tw_;
  turtlelib::wheels wheel_vels_;
  nuturtlebot_msgs::msg::WheelCommands wheel_cmds_;
  sensor_msgs::msg::JointState js_;
  double sensor_step_;
  rclcpp::Time prev_sensor_;

  // Publishers
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void timer_callback()
  {
    // Publish wheel command message
    wheel_cmd_pub_->publish(wheel_cmds_);

    // Publish joint states message
    js_.header.stamp = this->now();
    joint_state_pub_->publish(js_);
  }

  void cmd_vel_cb(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
  {
    // Get 2D twist
    tw_ = {
      static_cast<double>(msg->angular.z),
      static_cast<double>(msg->linear.x),
      static_cast<double>(msg->linear.y)
    };

    // Calculate velocities needed
    wheel_vels_ = diff_drive.ik(tw_);

    // NOTE: divide angular velocity by motor commands per radian-second
    // to get the motor command units
    wheel_vels_.l = static_cast<int>(wheel_vels_.l / motor_cmd_rad_sec_);
    wheel_vels_.r = static_cast<int>(wheel_vels_.r / motor_cmd_rad_sec_);

    // Clip motor command units to motor_cmd_max
    if (wheel_vels_.l > motor_cmd_max_) {
      wheel_vels_.l = motor_cmd_max_;
    } else if (wheel_vels_.l < -motor_cmd_max_) {
      wheel_vels_.l = -motor_cmd_max_;
    }

    if (wheel_vels_.r > motor_cmd_max_) {
      wheel_vels_.r = motor_cmd_max_;
    } else if (wheel_vels_.r < -motor_cmd_max_) {
      wheel_vels_.r = -motor_cmd_max_;
    }

    // Update message variable for publishing in main timer
    wheel_cmds_.left_velocity = wheel_vels_.l;
    wheel_cmds_.right_velocity = wheel_vels_.r;
  }

  void sensor_data_sub_cb_(const std::shared_ptr<nuturtlebot_msgs::msg::SensorData> msg)
  {
    // Calculate time since last callback
    sensor_step_ = (this->now() - prev_sensor_).seconds();

    // Calculate angular velocity using the previous position
    js_.velocity = {
      (msg->left_encoder / enc_ticks_ - js_.position[0]) / sensor_step_,
      (msg->right_encoder / enc_ticks_ - js_.position[1]) / sensor_step_
    };

    js_.position = {
      msg->left_encoder / enc_ticks_,
      msg->right_encoder / enc_ticks_
    };

    // Update time
    prev_sensor_ = this->now();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtle_control>());
  rclcpp::shutdown();
  return 0;
}
