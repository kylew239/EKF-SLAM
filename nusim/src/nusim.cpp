#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include "nusim/srv/teleport.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

using namespace std::chrono_literals;

class NusimNode : public rclcpp::Node
{
public:
  NusimNode()
  : Node("nusim")
  {
    // Parameters
    declare_parameter("rate", 200.0);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("th0", 0.0);
    declare_parameter("arena_x_length", 2.0);
    declare_parameter("arena_y_length", 2.0);
    declare_parameter("obstacles_x", std::vector<double>());
    declare_parameter("obstacles_y", std::vector<double>());
    declare_parameter("obstacles_r", 0.15);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.16);
    declare_parameter("encoder_ticks_per_rad", 651.9);
    declare_parameter("input_nosie", 0.0);
    declare_parameter("slip_fraction", 0.0);

    // Vars
    rate_ = get_parameter("rate").as_double();
    x0_ = get_parameter("x0").as_double();
    y0_ = get_parameter("y0").as_double();
    th0_ = get_parameter("th0").as_double();
    arena_x_length = get_parameter("arena_x_length").as_double();
    arena_y_length = get_parameter("arena_y_length").as_double();
    arena_height = 0.25;
    arena_thickness = 0.05;
    obstacles_x_ = get_parameter("obstacles_x").as_double_array();
    obstacles_y_ = get_parameter("obstacles_y").as_double_array();
    obstacles_r_ = get_parameter("obstacles_r").as_double();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    diff_drive = {track_width_, wheel_radius_};
    wheel_vel = {0.0, 0.0};
    enc_tick_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    input_noise_ = get_parameter("input_noise").as_double();
    slip_frac_ = get_parameter("slip_fraction").as_double();
    wheel_vel_noise_ = std::normal_distribution<double> {0.0, input_noise_};
    slip_dist = std::uniform_real_distribution<double> {-slip_frac_, slip_frac_};

    // qos profile transient local
    rclcpp::QoS qos(20);
    qos.transient_local();

    // Publishers
    time_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    wall_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos);
    obstacle_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", qos);
    sensor_data_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);

    // Subscribers
    wheel_cmd_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10,
      std::bind(
        &NusimNode::wheel_cmd_cb, this,
        std::placeholders::_1)
    );

    // Services
    reset_server_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(
        &NusimNode::reset_callback, this,
        std::placeholders::_1,
        std::placeholders::_2));

    tp_server_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(
        &NusimNode::tp_callback, this,
        std::placeholders::_1,
        std::placeholders::_2));


    // TF Broadcaster
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_stamped_.header.frame_id = "nusim/world";
    tf_stamped_.child_frame_id = "red/base_footprint";
    q.setRPY(0, 0, th0_);
    tf_stamped_.transform.translation.x = x0_;
    tf_stamped_.transform.translation.y = y0_;
    tf_stamped_.transform.rotation.x = q.x();
    tf_stamped_.transform.rotation.y = q.y();
    tf_stamped_.transform.rotation.z = q.z();
    tf_stamped_.transform.rotation.w = q.w();


    // Arena Walls All
    wall_array_.markers.resize(4);
    for (int i = 0; i < 4; i++) {
      wall_array_.markers[i].header.frame_id = "nusim/world";
      wall_array_.markers[i].header.stamp = this->now();
      wall_array_.markers[i].ns = "walls";
      wall_array_.markers[i].id = i;
      wall_array_.markers[i].type = visualization_msgs::msg::Marker::CUBE;
      wall_array_.markers[i].action = visualization_msgs::msg::Marker::ADD;
      wall_array_.markers[i].pose.position.z = arena_height / 2;
      wall_array_.markers[i].scale.z = arena_height;
      wall_array_.markers[i].pose.orientation.w = 1.0;
      wall_array_.markers[i].color.a = 1.0;
      wall_array_.markers[i].color.r = 1.0;
    }

    // Individual Walls
    wall_array_.markers[0].pose.position.x = (arena_x_length + 0.05) / 2;
    wall_array_.markers[0].pose.position.y = 0.0;
    wall_array_.markers[0].scale.x = 0.05;
    wall_array_.markers[0].scale.y = arena_y_length + 0.1; // Fill in corners of the arena walls

    wall_array_.markers[1].pose.position.x = (-arena_x_length - 0.05) / 2;
    wall_array_.markers[1].pose.position.y = 0.0;
    wall_array_.markers[1].scale.x = 0.05;
    wall_array_.markers[1].scale.y = arena_y_length + 0.1;

    wall_array_.markers[2].pose.position.x = 0.0;
    wall_array_.markers[2].pose.position.y = (-arena_y_length - 0.05) / 2;
    wall_array_.markers[2].scale.x = arena_x_length;
    wall_array_.markers[2].scale.y = 0.05;

    wall_array_.markers[3].pose.position.x = 0.0;
    wall_array_.markers[3].pose.position.y = (arena_y_length + 0.05) / 2;
    wall_array_.markers[3].scale.x = arena_x_length;
    wall_array_.markers[3].scale.y = 0.05;

    wall_publisher_->publish(wall_array_);


    // Obstacles
    // Check length of arrays
    if (obstacles_x_.size() != obstacles_y_.size()) {
      RCLCPP_ERROR(this->get_logger(), "Obstacle array lengths are not equal");
      throw std::logic_error("obstacle array lengths are not equal");
    }

    // Build and publish
    obstacle_array_.markers.resize(obstacles_x_.size());
    const auto size = obstacles_x_.size();

    for (auto i = 0; i < size; i++) {
      obstacle_array_.markers.at(i).header.frame_id = "nusim/world";
      obstacle_array_.markers.at(i).header.stamp = this->now();
      obstacle_array_.markers.at(i).ns = "obstacles";
      obstacle_array_.markers.at(i).id = i;
      obstacle_array_.markers.at(i).type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle_array_.markers.at(i).action = visualization_msgs::msg::Marker::ADD;
      obstacle_array_.markers.at(i).pose.position.x = obstacles_x_.at(i);
      obstacle_array_.markers.at(i).pose.position.y = obstacles_y_.at(i);
      obstacle_array_.markers.at(i).pose.position.z = arena_height / 2;
      obstacle_array_.markers.at(i).scale.x = obstacles_r_ * 2;
      obstacle_array_.markers.at(i).scale.y = obstacles_r_ * 2;
      obstacle_array_.markers.at(i).scale.z = arena_height;
      obstacle_array_.markers.at(i).pose.orientation.w = 1.0;
      obstacle_array_.markers.at(i).color.a = 1.0;
      obstacle_array_.markers.at(i).color.r = 1.0;
    }
    obstacle_publisher_->publish(obstacle_array_);

    // Timer
    timer_ = create_wall_timer(1s / rate_, std::bind(&NusimNode::timer_callback, this));
  }

private:
  // Vars
  double rate_;
  uint64_t timestep_;
  geometry_msgs::msg::Transform tf_;
  tf2::Quaternion q;
  double x0_, y0_, th0_;
  double arena_x_length, arena_y_length, arena_height, arena_thickness;
  visualization_msgs::msg::MarkerArray wall_array_;
  visualization_msgs::msg::MarkerArray obstacle_array_;
  std::vector<double> obstacles_x_, obstacles_y_;
  double obstacles_r_, motor_cmd_per_rad_sec_;
  double track_width_, wheel_radius_;
  turtlelib::DiffDrive diff_drive = {track_width_, wheel_radius_};
  turtlelib::wheels wheel_vel = {0.0, 0.0};
  nuturtlebot_msgs::msg::SensorData sensor_;
  double enc_tick_per_rad_;
  double input_noise_, slip_frac_;
  std::normal_distribution<double> wheel_vel_noise_;
  std::uniform_real_distribution<double> slip_dist;

  // Generate random number and seed it
  std::random_device rd{};
  std::mt19937 gen_{rd()};

  // Publishers
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr tp_server_;

  // Subscribers
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub_;

  // TF Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::msg::TransformStamped tf_stamped_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void timer_callback()
  {
    // Publishing Timer
    std_msgs::msg::UInt64 msg;
    msg.data = timestep_;
    time_publisher_->publish(msg);
    timestep_++;

    // Update wheel positions
    double l_new = wheel_vel.l / rate_ + diff_drive.get_wheels().l;
    double r_new = wheel_vel.r / rate_ + diff_drive.get_wheels().r;

    // Calculate slip for wheels
    // If slip frac is 0, then the normal distribution will be all zeros,
    // so we don't need to check if slip_frac is 0
    double l_new_slip = l_new * (1 + slip_dist(gen_));
    double r_new_slip = r_new * (1 + slip_dist(gen_));

    // Update robot position based on slip, then update wheels to be the actual location
    diff_drive.fk(l_new_slip, r_new_slip);
    diff_drive.set_wheels({l_new, r_new});

    // Update transform
    tf_stamped_.transform.translation.x = diff_drive.get_config().x;
    tf_stamped_.transform.translation.y = diff_drive.get_config().y;

    q.setRPY(0.0, 0.0, diff_drive.get_config().th);
    tf_stamped_.transform.rotation.x = q.x();
    tf_stamped_.transform.rotation.y = q.y();
    tf_stamped_.transform.rotation.z = q.z();
    tf_stamped_.transform.rotation.w = q.w();

    // Broadcast Transform
    tf_stamped_.header.stamp = this->now();
    broadcaster_->sendTransform(tf_stamped_);

    // Update and publish sensor data
    // These use the non-slip wheel positions, since slip only affects robot pos
    sensor_.left_encoder = static_cast<int>(l_new * enc_tick_per_rad_);
    sensor_.right_encoder = static_cast<int>(r_new * enc_tick_per_rad_);
    sensor_.stamp = this->now();
    sensor_data_pub_->publish(sensor_);
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // Reset time
    timestep_ = 0;

    // Reset pose
    q.setRPY(0, 0, th0_);
    tf_stamped_.transform.translation.x = x0_;
    tf_stamped_.transform.translation.y = y0_;
    tf_stamped_.transform.rotation.x = q.x();
    tf_stamped_.transform.rotation.y = q.y();
    tf_stamped_.transform.rotation.z = q.z();
    tf_stamped_.transform.rotation.w = q.w();
  }

  void tp_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    // update the transform
    // No need to publish because timer will take care of that;
    q.setRPY(0, 0, request->theta);
    tf_stamped_.transform.translation.x = request->x;
    tf_stamped_.transform.translation.y = request->y;
    tf_stamped_.transform.rotation.x = q.x();
    tf_stamped_.transform.rotation.y = q.y();
    tf_stamped_.transform.rotation.z = q.z();
    tf_stamped_.transform.rotation.w = q.w();
  }

  void wheel_cmd_cb(const std::shared_ptr<nuturtlebot_msgs::msg::WheelCommands> msg)
  {
    // Update wheel velocities
    wheel_vel =
    {msg->left_velocity * motor_cmd_per_rad_sec_, msg->right_velocity * motor_cmd_per_rad_sec_};

    // Add noise to wheel commands
    // We don't want to do this in the main timer since it will continuously add noise
    // when no new wheel commands are received
    if (wheel_vel.l != 0) {
      wheel_vel.l += wheel_vel_noise_(gen_);
    }
    if (wheel_vel.r != 0) {
      wheel_vel.r += wheel_vel_noise_(gen_);
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NusimNode>());
  rclcpp::shutdown();
  return 0;
}
