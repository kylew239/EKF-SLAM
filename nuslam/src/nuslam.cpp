// TODO:
// 1. change frame hard code to world > map (static) > odom > green (static)
// 2.

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
    declare_parameter("body_id", "odom");
    declare_parameter("odom_id", "map");
    declare_parameter("path_size_max", 200);
    declare_parameter("max_landmarks", 3);
    declare_parameter("kalman_process_noise.theta", 0.001);
    declare_parameter("kalman_process_noise.x", 0.001);
    declare_parameter("kalman_process_noise.y", 0.001);
    declare_parameter("kalman_R", 0.05);


    // Vars
    mat_size = get_parameter("max_landmark").as_int() * 2 + 3;
    prev_state = {mat_size, arma::fill::zeros};
    detected = std::vector<bool>(mat_size, false);
    auto kalman_process_noise_theta = get_parameter("kalman_process_noise.theta").as_double();
    auto kalman_process_noise_x = get_parameter("kalman_process_noise.x").as_double();
    auto kalman_process_noise_y = get_parameter("kalman_process_noise.y").as_double();
    auto R = get_parameter("kalman_R").as_double();
    R_mat = {2 * mat_size, 2 * mat_size, arma::fill::zeros};


    // Initial previous covariance, but with robot = 0
    prev_covariance = {mat_size, mat_size, arma::fill::zeros};
    for (int i = 3; i < mat_size; i++) {
      // Use extremely large number
      prev_covariance(i, i) = 1e9;
    }

    // Q_bar
    Q_bar = {mat_size, mat_size, arma::fill::zeros};
    Q_bar(0, 0) = kalman_process_noise_theta;
    Q_bar(1, 1) = kalman_process_noise_x;
    Q_bar(2, 2) = kalman_process_noise_y;

    // R
    for (int i = 0; i < 2 * mat_size; i++) {
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
  int mat_size;
  arma::vec prev_state;
  std::vector<bool> detected;
  arma::mat prev_covariance, Q_bar, R_mat;


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

  }

  void fake_sensor_cb(const std::shared_ptr<visualization_msgs::msg::MarkerArray> msg)
  {
    // Robot's position in map frame
    auto T_map_robot = T_map_odom * T_odom_robot;

    // Identity Matrix
    arma::mat Identity = arma::eye<arma::mat>(mat_size, mat_size);

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

    // Covariance prediction
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

      // Position
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

      // distance
      const auto dist = std::pow(dx, 2) + std::pow(dy, 2);

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

      // distance to landmark
      arma::vec est_dist = {std::sqrt(dist),
        turtlelib::normalize_angle(std::atan2(dy, dx) - state_pred_th)};
      arma::vec act_dist = {marker_range, marker_bearing};

      // Kalman Gain
      arma::mat kalman_gain = covariance * H_tranposed *
        (H * covariance * H_tranposed + R_mat.submat(2 * i, 2 * i, 2 * i + 1, 2 * i + 1)).i();
    }

  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSlam>());
  rclcpp::shutdown();
  return 0;
}
