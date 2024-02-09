# NUTurtle Control
A package for controlling the NU Turtlebot3

# Quickstart
Procedure for cross-compiling onto the Turtlebot, assuming `raphael` is used
```
./aarch64 colcon_aarch64
rsync -av --delete aarch64_install/ msr@raphael:/home/msr/install
```

Procedure for running on the Turtlebot
```
// Connect to turtlebot
ssh -oSendEnv=ROS_DOMAIN_ID msr@raphael

// Start the robot
source install/setup.bash
ros2 run numsr_turtlebot numsr_turtlebot
ros2 launch nuturtle_control start_robot.launch.py cmd_src:=none robot:=localhost use_rviz:=false

// On the computer that is connected
ros2 launch nuturtle_control start_robot.launch.py cmd_src:=circle robot:=none use_rviz:=true
```

# Nodes
* turtle_control: Controls the robot by converting sensor data into joint states, and twists into wheel commands
* odometry: Calculates odometry from published joint states
* circle: Controls the robot to move in a circle

# Launch File
The robot can be started by calling `ros2 launch  nuturtle_control start_robot.launch.py`

The following arguments are available:
* cmd_src: Control source for the cmd_vel messages
    * none: Launch no additional nodes (default)
    * teleop: Launch the turtlebot3_teleop node in a new terminal window
    * circle: Launch the circle node
* robot: Controls whether the simulator or the actual robot is used
    * nusim: Launches the nusim simulator (default)
    * localhost: Launches the nodes required for use on the Turtlebot3
    * none: Launches no additional nodes
* use_rviz: Determines whether or not rviz is used
    * false: Don't launch rviz (default)
    * true: Launch rviz with different configuration files, depending on `robot`

# Demo Video
https://github.com/ME495-Navigation/EKF-SLAM/assets/59859207/de45c67a-7f25-41fc-be46-5fdcdcdeb7a0

# Odometry Performance
At the start, the odometry data is:
```
x: 0.323
y: -0.04
z: 0.0
```

At the end, the odometry data is:
```
x: 0.422
y: 0.045
z: 0.0
```

The error in odometry data is:
```
x: 0.099
y: 0.085
z: 0.0
```

The total error in odometry data is approximately 13cm
