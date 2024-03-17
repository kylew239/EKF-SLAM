# NUSLAM
A package for implementing Extended Kalman Filter SLAM. This package uses detected landmarks to correct odometry data.

# Quickstart
`ros2 launch nuslam slam.launch.py` to launch the simulation environment with EKF SLAM
* Red represents the ground-truth data
* Blue represents the odometry data
* Green represents data calculated from the EKF

# Demo Video in Simulation
https://github.com/ME495-Navigation/EKF-SLAM/assets/59859207/28ec64a6-9530-42b5-8020-3da1f0d5af02

# Launch
The `slam.launch.py` launchfile calls the `start_robot.launch.py` from `nuturtle_control` in simulation mode. It remaps the odometry data to `uncorrected_odom`.