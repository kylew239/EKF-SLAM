#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odom")
  {
    // Parameters
    declare_parameter("rate", 200.0);
    declare_parameter("body_id", rclcpp::PARAMETER_STRING);
    declare_parameter("odom_id", rclcpp::PARAMETER_STRING);
    declare_parameter("wheel_left", rclcpp::PARAMETER_STRING);
    declare_parameter("wheel_right", rclcpp::PARAMETER_STRING);
    declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("track_width", rclcpp::PARAMETER_DOUBLE);

    // Get values for params
    rate_ = get_parameter("rate").as_double();
    body_id_ = get_parameter("body_id").as_string();
    odom_id_ = get_parameter("odom_id").as_string();
    wheel_left_ = get_parameter("wheel_left").as_string();
    wheel_right_ = get_parameter("wheel_right").as_string();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();


    // Check if parameters are defined
    if (get_parameter("body_id").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("wheel_left").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("wheel_right").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("wheel_radius").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
      get_parameter("track_width").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      RCLCPP_ERROR(this->get_logger(), "Parameters are not defined");
      rclcpp::shutdown();
    }

    // Vars
    odom_.header.frame_id = odom_id_;
    odom_.child_frame_id = body_id_;
    diff_drive = {track_width_, wheel_radius_};
    config = {0.0, 0.0, 0.0};

    // Pubishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Subscribers
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(
        &Odometry::js_cb, this,
        std::placeholders::_1));

    // Services
    initial_pose_server_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(
        &Odometry::inital_pose_cb_, this,
        std::placeholders::_1,
        std::placeholders::_2));

    // Timer
    timer_ = create_wall_timer(1s / rate_, std::bind(&Odometry::timer_callback, this));

    // TF Broadcaster
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_stamped_.header.frame_id = odom_id_;
    tf_stamped_.child_frame_id = body_id_;
  }

private:
  // Parameters
  double rate_, wheel_radius_, track_width_;
  std::string body_id_, odom_id_, wheel_left_, wheel_right_;

  // Vars
  nav_msgs::msg::Odometry odom_;
  double left_pos, right_pos, left_vel, right_vel;
  turtlelib::DiffDrive diff_drive = {track_width_, wheel_radius_};
  turtlelib::state config;
  tf2::Quaternion q;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Services
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_server_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // TF Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::msg::TransformStamped tf_stamped_;

  // Callbacks
  void timer_callback()
  {
    // Publish odom message
    odom_.header.stamp = this->now();
    odom_pub_->publish(odom_);

    // Update and publish TF
    tf_stamped_.transform.translation.x = odom_.pose.pose.position.x;
    tf_stamped_.transform.translation.y = odom_.pose.pose.position.y;
    tf_stamped_.transform.rotation.x = odom_.pose.pose.orientation.x;
    tf_stamped_.transform.rotation.y = odom_.pose.pose.orientation.y;
    tf_stamped_.transform.rotation.z = odom_.pose.pose.orientation.z;
    tf_stamped_.transform.rotation.w = odom_.pose.pose.orientation.w;
    broadcaster_->sendTransform(tf_stamped_);
  }

  void js_cb(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
  {
    const auto new_left = msg->position[0] + msg->velocity[0] / rate_;
    const auto new_right = msg->position[1] + msg->velocity[1] / rate_;
    // Calculate updated odom
    config = diff_drive.fk(new_left, new_right);

    // Update odom message
    odom_.pose.pose.position.x = config.x;
    odom_.pose.pose.position.y = config.y;

    q.setRPY(0, 0, config.th);
    odom_.pose.pose.orientation.x = q.x();
    odom_.pose.pose.orientation.y = q.y();
    odom_.pose.pose.orientation.z = q.z();
    odom_.pose.pose.orientation.w = q.w();

    turtlelib::Twist2D tw = diff_drive.get_body_twist(new_left, new_right);

    odom_.twist.twist.linear.x = tw.x;
    odom_.twist.twist.linear.y = tw.y;
    odom_.twist.twist.angular.z = tw.omega;
  }

  void inital_pose_cb_(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request>,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    // Reset pose
    odom_.pose.pose.position.x = 0;
    odom_.pose.pose.position.y = 0;
    odom_.pose.pose.orientation.x = 0;
    odom_.pose.pose.orientation.y = 0;
    odom_.pose.pose.orientation.z = 0;
    odom_.pose.pose.orientation.w = 0;

    // Reset diff_drive object
    diff_drive = {track_width_, wheel_radius_};
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
