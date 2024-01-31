#ifndef TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE
/// \file
/// \brief kinematics of the turtlebot

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"


namespace turtlelib{
    /// \brief Robot configuration
    struct state{
        /// \brief The x position of the robot
        double x;

        /// \brief The y position of the robot
        double y;

        /// \brief The orientation of the robot
        double th;
    };

    /// \brief Struct for keeping track of wheels
    struct wheels{
        /// \brief The left wheel
        double l;
        
        /// \brief The right wheel
        double r;
    };

    /// \brief Models the kinematics of a differential drive robot
    class DiffDrive{
    private:
        /// \brief The state of the robot
        state config;
        
        /// \brief The position of the robot's wheels
        wheels wheelPos;

        /// \brief Radius of the wheels
        double wheel_r = 0.0;

        /// \brief Distance between the wheels
        double track_w = 0.0;

    public:
        /// \brief Create a differential drive robot
        /// \param track_width Distance between the wheels of the robot, must be > 0
        /// \param wheel_radius Radius of the wheels, must be > 0
        DiffDrive(double track_width, double wheel_radius);

        /// \brief Create a differential drive robot
        /// \param track_width Distance between the wheels of the robot, must be > 0
        /// \param wheel_radius Radius of the wheels, must be > 0
        /// \param location Start location of the robot
        DiffDrive(double track_width, double wheel_radius, state location);

        /// \brief Create a differential drive robot
        /// \param track_width Distance between the wheels of the robot, must be > 0
        /// \param wheel_radius Radius of the wheels, must be > 0
        /// \param location Start location of the robot
        /// \param left Start position of the left wheel
        /// \param right Start position of the right wheel
        DiffDrive(double track_width, double wheel_radius, state location, double left, double right);

        /// \brief Create a differential drive robot
        /// \param track_width Distance between the wheels of the robot, must be > 0
        /// \param wheel_radius Radius of the wheels, must be > 0
        /// \param left Start position of the left wheel
        /// \param right Start position of the right wheel
        DiffDrive(double track_width, double wheel_radius, double left, double right);

        /// \brief Calculates new configuration of the robot given two new wheel angles
        /// \param new_left New angle of the left wheel
        /// \param new_right New angle of the right wheel
        /// \return The state of the robot
        state fk(const double new_left, const double new_right);

        /// \brief Calculates wheel velocities given a body twist
        /// \param tw Body twist to move the robot along
        /// \return The wheel velocities
        wheels ik(const Twist2D & tw);

        /// @brief Get the current configuration of the robot's position
        /// @return The robots position
        state get_config();
    };

}

#endif