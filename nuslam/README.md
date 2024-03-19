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

# Performance In Simulation

Ground Truth:
```
x: 0.0 m
y: 0.024 m
θ: -168.070 deg
```

Odometry:
```
x: -0.278 m
y: -0.057 m
θ: -168.070 deg
```

EKF-SLAM:
```
x: -0.001
y: 0.024
θ: -168.05 deg
```

Odomotery data had a positional error of 0.289m and no rotational error. Given that collisions cause the robot to slide along its original path, this data makes sense. The odometry data, with no sensor noise and rotational collisions, is off linearly but is perfect in terms of the angular position

EKF-SLAM data had a positional error of 0.001 m and a rotational error of 0.02 deg. This data is calculated from both LiDAR and odometry data, so it should be relatively accurate. As expected, EKF-SLAM greatly outperforms odometry data, especially when collisions are involved.