#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class NusimNode : public rclcpp::Node{
    public:
        NusimNode()
        : Node("nusim"){
            // Parameters
            declare_parameter("rate", 200.0);

            // Vars
            rate_ = get_parameter("rate").as_double();

            // Publishers
            time_publisher_ =  create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

            // Services
            reset_server_ = create_service<std_srvs::srv::Empty>("~/reset",
                                                                 std::bind(&NusimNode::reset_callback, this,
                                                                 std::placeholders::_1,
                                                                 std::placeholders::_2));

            // Timer
            timer_ = create_wall_timer(1s / rate_, std::bind(&NusimNode::timer_callback, this));
        }
    private:
        // Vars
        double rate_;
        uint64_t timestep_;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_publisher_;

        // Services
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;
        
        // Callbacks
        void timer_callback(){
            // Publishing Timer
            std_msgs::msg::UInt64 msg;
            msg.data = timestep_;
            time_publisher_->publish(msg);
            timestep_++;
        }

        void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                            std::shared_ptr<std_srvs::srv::Empty::Response>){
            timestep_ = 0;
        }

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NusimNode>());
    rclcpp::shutdown();
    return 0;
}
