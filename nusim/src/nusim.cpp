#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nusim/srv/teleport.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"


using namespace std::chrono_literals;

class NusimNode : public rclcpp::Node{
    public:
        NusimNode()
        : Node("nusim"){
            // Parameters
            declare_parameter("rate", 200.0);
            declare_parameter("x0", 0.0);
            declare_parameter("y0", 0.0);
            declare_parameter("th0", 0.0);


            // Vars
            rate_ = get_parameter("rate").as_double();
            x0_ = get_parameter("x0").as_double();
            y0_ = get_parameter("y0").as_double();
            th0_ = get_parameter("th0").as_double();

            // Publishers
            time_publisher_ =  create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

            // Services
            reset_server_ = create_service<std_srvs::srv::Empty>("~/reset",
                                                                 std::bind(&NusimNode::reset_callback, this,
                                                                 std::placeholders::_1,
                                                                 std::placeholders::_2));
            tp_server_ = create_service<nusim::srv::Teleport>("~/teleport",
                                                              std::bind(&NusimNode::tp_callback, this,
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

        // Publishers
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_publisher_;

        // Services
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
        rclcpp::Service<nusim::srv::Teleport>::SharedPtr tp_server_;

        // TF Broadcaster
        std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
        geometry_msgs::msg::TransformStamped tf_stamped_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;
        
        // Callbacks
        void timer_callback(){
            // Publishing Timer
            std_msgs::msg::UInt64 msg;
            msg.data = timestep_;
            time_publisher_->publish(msg);
            timestep_++;
            
            // Broadcast Transform
            tf_stamped_.header.stamp = this->now();
            // tf_stamped_.transform.x = 
            // tf_stamped_.transform.y = 
            // tf_stamped_.transform.rotation.x = 
            // tf_stamped_.transform.rotation.y = 
            // tf_stamped_.transform.rotation.z = 
            // tf_stamped_.transform.rotation.w = 
            broadcaster_->sendTransform(tf_stamped_);
        }

        void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                            std::shared_ptr<std_srvs::srv::Empty::Response>){
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

        void tp_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
                            std::shared_ptr<nusim::srv::Teleport::Response>){
            // updadte transform
            // No need to publish because timer will take care of that;
            q.setRPY(0, 0, request->theta);
            tf_stamped_.transform.translation.x = request->x;
            tf_stamped_.transform.translation.y = request->y;
            tf_stamped_.transform.rotation.x = q.x();
            tf_stamped_.transform.rotation.y = q.y();
            tf_stamped_.transform.rotation.z = q.z();
            tf_stamped_.transform.rotation.w = q.w();
        }



};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NusimNode>());
    rclcpp::shutdown();
    return 0;
}
