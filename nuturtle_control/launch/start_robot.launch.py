from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, AndSubstitution, OrSubstitution


def generate_launch_description():
    return LaunchDescription([
        # Args
        DeclareLaunchArgument('cmd_src',
                              default_value="none",
                              description="Determines the `cmd_vel` publisher (screen | circle | none)"),
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
            arguments=['--frame-id', 'nusim/world',
                       '--child-id', 'odom']
        ),


        # CMD_SRC
        Node(
            package="turtlebot3_teleop",
            executable="teleop_keyboard.py",
            output="screen",
            prefix="gnome-terminal",  # Launch in a separate terminal window
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('cmd_src'), "teleop"))),

        Node(
            package="nuturtle_control",
            executable="circle_node",
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('cmd_src'), "circle"))),


        # ROBOT

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
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('robot'), "nusim"))),
        
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
        ),

        Node(
            package='nuturtle_control',
            executable='turtle_control_node',
            parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                                   "config",
                                                   "diff_params.yaml"])],
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('robot'), "nusim"))),
        
        Node(
            package="nuturtle_control",
            executable="odom_node",
            condition=IfCondition(OrSubstitution(
                                    EqualsSubstitution(LaunchConfiguration('robot'), "nusim"), 
                                    EqualsSubstitution(LaunchConfiguration('robot'), "localhost"))),
            parameters=[PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                                   "config",
                                                   "diff_params.yaml"]),
                        {'body-id': 'blue/base_footprint'},
                        {'odom-id': 'blue/odom'},
                        {'wheel-left': 'wheel_left_joint'},
                        {'wheel-right': 'wheel_right_join'}],
            remappings=[('odom', 'blue/odom')]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # TODO: Add config file for blue and red
            # arguments=['-d', PathJoinSubstitution([FindPackageShare('nusim'),
            #                                        'config',
            #                                        'nusim.rviz'])],
            condition=IfCondition(AndSubstitution(
                                    EqualsSubstitution(LaunchConfiguration('robot'), "nusim"), 
                                    EqualsSubstitution(LaunchConfiguration('use_rviz'), "true")))),



        # localhost






        # None
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # TODO: Add config file for blue
            # arguments=['-d', PathJoinSubstitution([FindPackageShare('nusim'),
            #                                        'config',
            #                                        'nusim.rviz'])],
            condition=IfCondition(AndSubstitution(
                                    EqualsSubstitution(LaunchConfiguration('robot'), "none"), 
                                    EqualsSubstitution(LaunchConfiguration('use_rviz'), "true")))),
    ])
