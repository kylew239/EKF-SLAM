from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, AndSubstitution, NotEqualsSubstitution


def generate_launch_description():
    return LaunchDescription([
        # Args
        DeclareLaunchArgument('cmd_src',
                              default_value="none",
                              description="Determines the `cmd_vel` publisher (teleop | circle | none)"),
        DeclareLaunchArgument('robot',
                              default_value="nusim",
                              description="Determines whether the simulator or the actual robot is used (nusim | localhost | none)"),
        DeclareLaunchArgument('use_rviz',
                              default_value="false",
                              description='Determines whether or not rviz is used (true | false)'),

        # Publish Identity Transform
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_odom",
            arguments=['--frame-id', 'nusim/world',
                       '--child-frame-id', 'odom'],
            condition=IfCondition(NotEqualsSubstitution(
                LaunchConfiguration('robot'), "none"))
        ),

        # use_rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_nusim',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('nuturtle_control'),
                                                   'config',
                                                   'red_blue.rviz'])],
            condition=IfCondition(AndSubstitution(
                EqualsSubstitution(LaunchConfiguration('robot'), "nusim"),
                EqualsSubstitution(LaunchConfiguration('use_rviz'), "true")))
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_none',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('nuturtle_control'),
                                                   'config',
                                                   'blue.rviz'])],
            condition=IfCondition(AndSubstitution(
                EqualsSubstitution(LaunchConfiguration('robot'), "none"),
                EqualsSubstitution(LaunchConfiguration('use_rviz'), "true")))
        ),




        # CMD_SRC
        Node(
            package="turtlebot3_teleop",
            executable="teleop_keyboard",
            output="screen",
            prefix=["xterm -e"],  # Launch in a separate terminal window
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('cmd_src'), "teleop"))
        ),

        Node(
            package="nuturtle_control",
            executable="circle_node",
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('cmd_src'), "circle"))
        ),


        # # ROBOT

        # nusim
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("nusim"),
                "launch",
                "nusim.launch.py"
            ]),
            launch_arguments={
                'use_rviz': 'false',
                'use_jsp': 'false'
            }.items(),
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('robot'), "nusim"))
        ),

        # Blue robot
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("nuturtle_description"),
                "launch",
                "load_one.launch.py"
            ]),
            launch_arguments={
                'color': 'blue',
                'use_rviz': 'false',
                'use_jsp': 'false'
            }.items(),
            condition=IfCondition(NotEqualsSubstitution(
                LaunchConfiguration('robot'), "localhost"))
        ),

        Node(
            package='nuturtle_control',
            executable='turtle_control_node',
            parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                              "config",
                                              "diff_params.yaml"])],
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('robot'), "nusim")),
            remappings=[('wheel_cmd', 'red/wheel_cmd'),
                        ('joint_states', 'red/joint_states'),
                        ('sensor_data', 'red/sensor_data'),
                        ('wheel_left_link', 'red/wheel_left_link'),
                        ('wheel_right_link', 'red/wheel_right_link')]
        ),

        Node(
            package="nuturtle_control",
            executable="odom_node",
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('robot'), "nusim")),
            parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                              "config",
                                              "diff_params.yaml"]),
                        {'body_id': 'blue/base_footprint'},
                        {'odom_id': 'odom'},
                        {'wheel_left': 'wheel_left_joint'},
                        {'wheel_right': 'wheel_right_joint'}],
            remappings=[('joint_states', 'red/joint_states')]
        ),

        

        



        # localhost
        Node(
            package='nuturtle_control',
            executable='turtle_control_node',
            parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                              "config",
                                              "diff_params.yaml"])],
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('robot'), "localhost")),
            remappings=[('joint_states', 'blue/joint_states')]
        ),

        Node(
            package="nuturtle_control",
            executable="odom_node",
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('robot'), "localhost")),
            parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                              "config",
                                              "diff_params.yaml"]),
                        {'body_id': 'blue/base_footprint'},
                        {'odom_id': 'odom'},
                        {'wheel_left': 'wheel_left_joint'},
                        {'wheel_right': 'wheel_right_joint'}],
            remappings=[('joint_states', 'blue/joint_states')]
        ),






        # None
        
    ])
