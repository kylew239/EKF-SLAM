#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "turtlelib/diff_drive.hpp"
// #include "turtlelib/geometry2d.hpp"
// #include "turtlelib/se2d.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"  
#include "nuturtle_control/srv/control.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    // Parameters
    declare_parameter("frequency", 200.0);

    // Get values for params
    frequency_ = get_parameter("frequency").as_double();

    // Vars
    tw.linear.x = 0.0;
    tw.linear.y = 0.0;
    tw.linear.z = 0.0;
    tw.angular.x = 0.0;
    tw.angular.y = 0.0;
    tw.angular.z = 0.0;

    // Pubishers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Services
    stop_ = create_service<std_srvs::srv::Empty>("stop",
        std::bind(
        &Circle::reset_cb, this,
        std::placeholders::_1,
        std::placeholders::_2));
    
    reverse_ = create_service<std_srvs::srv::Empty>("stop",
        std::bind(
        &Circle::reverse_cb, this,
        std::placeholders::_1,
        std::placeholders::_2));

     control_ = create_service<nuturtle_control::srv::Control>("stop",
        std::bind(
        &Circle::control_cb, this,
        std::placeholders::_1,
        std::placeholders::_2));

    // Timer
    timer_ = create_wall_timer(1s / frequency_, std::bind(&Circle::timer_callback, this));

  }

private:
  // Parameters
  double frequency_;

  // Vars
  geometry_msgs::msg::Twist tw;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void timer_callback()
  {
    cmd_vel_pub_->publish(tw);
  }

  void reset_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    tw.linear.x = 0.0;
    tw.angular.z = 0.0;
  }

  void reverse_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    tw.linear.x = -tw.linear.x;
    tw.angular.z = -tw.angular.z;
  }

  void control_cb(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> req,
    std::shared_ptr<nuturtle_control::srv::Control::Response>){
        tw.angular.z = req->velocity;
        tw.linear.x = req->velocity * req->radius;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
