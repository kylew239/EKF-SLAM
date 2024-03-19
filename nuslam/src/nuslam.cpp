/// \file
/// \brief Feature-based EKF SLAM for the NUTurtlebot

/// PARAMETERS:
///     path_size_max (int): Max size of the path
///     max_landmarks (int): Max number of expected landmarks
///     kalman_process_noise.theta (double): Kalman Process Noise for theta
///     kalman_process_noise.x (double): Kalman Process Noise for x
///     kalman_process_noise.y (double): Kalman Process Noise for y
///     kalman_R (double): R value for the Kalman filter
///     obstacles_r (double): Radius of the obstacles
/// PUBLISHERS:
///     ~/odom (nav_msgs/msg/Odometry): Corrected odometry of the robot
///     landmarks (visualization_msgs/msg/MarkerArray): marker array containing the landmarks detected
///     ~/path (nav_msgs/msg/Path): Path the robot has followed
/// SUBSCRIBERS:
///     uncorrected_odom (nav_msgs/msg/Odometry): Calculated odometry of the robot
///     fake_sensor (visualization_msgs/msg/MarkerArray): fake sensor data for the detected obstacles

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <random>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace std::chrono_literals;

class NuSlam : public rclcpp::Node
{
public:
  NuSlam()
  : Node("nuslam")
  {
    // Parameters
    declare_parameter("path_size_max", 200);
    declare_parameter("max_landmarks", 5);
    declare_parameter("kalman_process_noise.theta", 0.01);
    declare_parameter("kalman_process_noise.x", 0.01);
    declare_parameter("kalman_process_noise.y", 0.01);
    declare_parameter("kalman_R", 0.05);
    declare_parameter("obstacles_r", 0.038);

    // Vars
    mat_size = get_parameter("max_landmarks").as_int() * 2 + 3;
    Identity = arma::eye<arma::mat>(mat_size, mat_size);
    prev_state = {mat_size, arma::fill::zeros};
    detected = std::vector<bool>(mat_size, false);
    auto kalman_process_noise_theta = get_parameter("kalman_process_noise.theta").as_double();
    auto kalman_process_noise_x = get_parameter("kalman_process_noise.x").as_double();
    auto kalman_process_noise_y = get_parameter("kalman_process_noise.y").as_double();
    auto R = get_parameter("kalman_R").as_double();
    R_mat = {2 * mat_size, 2 * mat_size, arma::fill::zeros};
    path_size_max_ = get_parameter("path_size_max").get_parameter_value().get<size_t>();
    map_odom_tf_for_pub.header.frame_id = "map";
    map_odom_tf_for_pub.child_frame_id = "odom";
    path_.header.frame_id = "nusim/world";
    arena_height = 0.25;
    obstacle_r_ = get_parameter("obstacles_r").as_double();
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "green/base_footprint";

    // Initial previous covariance, but with robot = 0
    prev_covariance = {mat_size, mat_size, arma::fill::zeros};
    for (arma::uword i = 3; i < mat_size; i++) {
      // Use extremely large number
      prev_covariance(i, i) = 1e9;
    }

    // Q_bar
    Q_bar = {mat_size, mat_size, arma::fill::zeros};
    Q_bar(0, 0) = kalman_process_noise_theta;
    Q_bar(1, 1) = kalman_process_noise_x;
    Q_bar(2, 2) = kalman_process_noise_y;

    // R
    for (arma::uword i = 0; i < 2 * mat_size; i++) {
      R_mat(i, i) = R;
    }


    // qos profile transient local
    rclcpp::QoS qos(20);
    qos.transient_local();

    // Publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    landmark_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("landmarks", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);

    // Subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "uncorrected_odom", 10,
      std::bind(
        &NuSlam::odom_cb, this,
        std::placeholders::_1));

    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "fake_sensor", 10,
      std::bind(
        &NuSlam::fake_sensor_cb, this,
        std::placeholders::_1));


    // TF Broadcaster
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_stamped_.header.frame_id = "map";
    tf_stamped_.child_frame_id = "body_id_";

  }

private:
  // Vars
  rclcpp::Time curr_time; // Use the same time for all publishing
  turtlelib::Transform2D T_map_odom, T_odom_robot;
  arma::uword mat_size;
  arma::vec prev_state;
  std::vector<bool> detected;
  arma::mat prev_covariance, Q_bar, R_mat;
  nav_msgs::msg::Path path_;
  size_t path_size_max_;
  geometry_msgs::msg::TransformStamped map_odom_tf_for_pub;
  double arena_height, obstacle_r_;
  nav_msgs::msg::Odometry odom_;
  arma::mat Identity;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;

  // TF Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::msg::TransformStamped tf_stamped_;

  void odom_cb(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    // Get the odom quaternion and convert into roll pitch yaw
    tf2::Quaternion q;
    q.setW(msg->pose.pose.orientation.w);
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    // Update odom message's velocities
    odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    odom_.twist.twist.angular.z = msg->twist.twist.angular.z;

    // Save the x, y, and theta values
    T_odom_robot = {{msg->pose.pose.position.x, msg->pose.pose.position.y}, y};
  }

  void fake_sensor_cb(const std::shared_ptr<visualization_msgs::msg::MarkerArray> msg)
  {
    // Robot's position in map frame
    auto T_map_robot = T_map_odom * T_odom_robot;

    // Update state predictions to current state
    auto state_pred = prev_state;
    auto & state_pred_th = state_pred(0);
    auto & state_pred_x = state_pred(1);
    auto & state_pred_y = state_pred(2);
    state_pred_th = turtlelib::normalize_angle(T_map_robot.rotation());
    state_pred_x = T_map_robot.translation().x;
    state_pred_y = T_map_robot.translation().y;

    // Create A Matrix
    arma::mat A_mat = Identity;
    A_mat(1, 0) -= state_pred_y - prev_state(2);
    A_mat(2, 0) += state_pred_x - prev_state(1);

    // Covariance
    arma::mat covariance = A_mat * prev_covariance * A_mat.t() + Q_bar;

    // Correct prediction using fake_sensor data
    for (size_t i = 0; i < msg->markers.size(); i++) {
      const auto & obs = msg->markers.at(i);

      // Check if marker is set as out of range
      if (obs.action == visualization_msgs::msg::Marker::DELETE) {
        // go to next iteration of for loop
        continue;
      }

      // Bearing and range
      auto marker_range =
        std::sqrt(std::pow(obs.pose.position.x, 2) + std::pow(obs.pose.position.y, 2));
      auto marker_bearing =
        turtlelib::normalize_angle(std::atan2(obs.pose.position.y, obs.pose.position.x));

      // Position of marker
      auto & marker_pred_x = state_pred(3 + 2 * i);
      auto & marker_pred_y = state_pred(3 + 2 * i + 1);

      // If landmark has not been detected yet
      if (!detected.at(i)) {
        // mark it as detected
        detected.at(i) = true;

        // Update positions (previously uninitialized)
        marker_pred_x = state_pred_x + marker_range * std::cos(marker_bearing + state_pred_th);
        marker_pred_y = state_pred_y + marker_range * std::sin(marker_bearing + state_pred_th);
      }

      // Relative positions
      const auto dx = marker_pred_x - state_pred_x;
      const auto dy = marker_pred_y - state_pred_y;

      // distances
      const auto dist = std::pow(dx, 2) + std::pow(dy, 2);
      arma::vec est_dist = {std::sqrt(dist),
        turtlelib::normalize_angle(std::atan2(dy, dx) - state_pred_th)};
      arma::vec act_dist = {marker_range, marker_bearing};
      arma::mat d_dist = act_dist - est_dist;
      d_dist(1) = turtlelib::normalize_angle(d_dist(1));

      // Create H matrix
      arma::mat H {2, mat_size, arma::fill::zeros};
      H(1, 0) = -1.0;
      H(0, 1) = -dx / std::sqrt(dist);
      H(1, 1) = dy / dist;
      H(0, 2) = -dy / std::sqrt(dist);
      H(1, 2) = -dx / dist;
      H(0, 3 + 2 * i) = dx / std::sqrt(dist);
      H(1, 3 + 2 * i) = -dy / dist;
      H(0, 3 + 2 * i + 1) = dy / std::sqrt(dist);
      H(1, 3 + 2 * i + 1) = dx / dist;
      arma::mat H_tranposed = H.t();

      // Kalman Gain
      arma::mat kalman_gain = covariance * H_tranposed *
        (H * covariance * H_tranposed + R_mat.submat(2 * i, 2 * i, 2 * i + 1, 2 * i + 1)).i();

      // Update covariance
      covariance = (Identity - kalman_gain * H) * covariance;

      // Update state prediction using kalman gain
      state_pred += kalman_gain * d_dist;
      state_pred_th = turtlelib::normalize_angle(state_pred_th);
    }

    // Update time stamp for all publishing
    curr_time = this->now();

    // Path update and publish
    // Check if turtlebot has moved
    if (!(
        turtlelib::almost_equal(state_pred(0), prev_state(0)) &&
        turtlelib::almost_equal(state_pred(1), prev_state(1)) &&
        turtlelib::almost_equal(state_pred(2), prev_state(2))
    ))
    {
      // Create pose stamped
      geometry_msgs::msg::PoseStamped pose_s;
      pose_s.header.stamp = curr_time;
      pose_s.pose.position.x = state_pred_x;
      pose_s.pose.position.y = state_pred_y;
      tf2::Quaternion q_path;
      q_path.setRPY(0.0, 0.0, state_pred_th);
      pose_s.pose.orientation.x = q_path.x();
      pose_s.pose.orientation.y = q_path.y();
      pose_s.pose.orientation.z = q_path.z();
      pose_s.pose.orientation.w = q_path.w();

      // Add to path
      path_.poses.push_back(pose_s);

      // If path size has been reached, remove the first element
      if (path_.poses.size() > path_size_max_) {
        path_.poses.erase(path_.poses.begin());
      }
    }

    // Publish path
    path_pub_->publish(path_);

    // Update Transforms
    T_map_robot = {{state_pred_x, state_pred_y}, state_pred_th};
    T_map_odom = T_map_robot * T_odom_robot.inv();

    // Build and send map odom transform message
    map_odom_tf_for_pub.transform.translation.x = state_pred_x;
    map_odom_tf_for_pub.transform.translation.y = state_pred_y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state_pred_th);
    map_odom_tf_for_pub.transform.rotation.x = q.x();
    map_odom_tf_for_pub.transform.rotation.y = q.y();
    map_odom_tf_for_pub.transform.rotation.z = q.z();
    map_odom_tf_for_pub.transform.rotation.w = q.w();
    broadcaster_->sendTransform(map_odom_tf_for_pub);

    // Update and publish corrected odom
    odom_.pose.pose.position.x = T_map_odom.translation().x;
    odom_.pose.pose.position.y = T_map_odom.translation().y;
    odom_.pose.pose.orientation.x = q.x();
    odom_.pose.pose.orientation.y = q.y();
    odom_.pose.pose.orientation.z = q.z();
    odom_.pose.pose.orientation.w = q.w();
    odom_.header.stamp = curr_time;
    odom_pub_->publish(odom_);

    // Build and publish estimated landmarks
    visualization_msgs::msg::MarkerArray est_landmarks;
    for (long int i = 0; i < get_parameter("max_landmarks").as_int(); i++) {

      //Skip marker if it hasn't been seen
      if (!detected.at(i)) {
        continue;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // TODO: use parameters for radius and height
      marker.pose.position.z = arena_height / 2.0;
      marker.scale.x = obstacle_r_ * 2.0;
      marker.scale.y = obstacle_r_ * 2.0;
      marker.scale.z = arena_height;
      marker.color.g = 1.0;
      marker.color.a = 1.0;
      marker.id = i;
      marker.pose.position.x = state_pred(3 + 2 * i);
      marker.pose.position.y = state_pred(3 + 2 * i + 1);

      est_landmarks.markers.push_back(marker);
    }
    landmark_pub_->publish(est_landmarks);

    // Update states
    prev_state = state_pred;
    prev_covariance = covariance;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSlam>());
  rclcpp::shutdown();
  return 0;
}
