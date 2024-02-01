#include "turtlelib/diff_drive.hpp"

namespace turtlelib{
    DiffDrive::DiffDrive(double track_width, double wheel_radius){
        track_w = track_width;
        wheel_r = wheel_radius;
    }

    DiffDrive::DiffDrive(double track_width, double wheel_radius, state location){
        track_w = track_width;
        wheel_r = wheel_radius;
        config = location;
    }

    DiffDrive::DiffDrive(double track_width, double wheel_radius, state location, double left, double right){
        track_w = track_width;
        wheel_r = wheel_radius;
        config = location;
        wheelPos = {left, right};
    }

    DiffDrive::DiffDrive(double track_width, double wheel_radius, double left, double right){
        track_w = track_width;
        wheel_r = wheel_radius;
        wheelPos = {left, right};
    }

    state DiffDrive::fk(const double new_left, const double new_right){
        // Velocity from new positions
        // Refer to the second page of doc/Kinematics.pdf, step 1
        wheels vel = {new_left - wheelPos.l, new_right - wheelPos.r};

        // Update wheel angles
        wheelPos = {new_left, new_right};

        // Twist for calculations
        // Refer to the second page of doc/Kinematics.pdf, step 2
        Twist2D tw = {
            wheel_r/(track_w*2.0) * (vel.r - vel.l),
            wheel_r/2.0 * (vel.r + vel.l),
            0.0
        };

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
            config.x + t_wb_new.translation().x,
            config.y + t_wb_new.translation().y,
            config.th + t_wb_new.rotation()
        };
        return config;
    }

    wheels DiffDrive::ik(const Twist2D & tw){
        // diff drive robots cannot move in the y direction
        if(!almost_equal(tw.y, 0.0)){
            throw std::logic_error("Twist is invaled for a differential drive robot");
        }
        return wheels{ // Refer to the first page of doc/Kinematics.pdf
            (tw.x - tw.omega * track_w) / wheel_r,
            (tw.x + tw.omega * track_w) / wheel_r,
        };
    }

    state DiffDrive::get_config(){
        return config;
    }

}