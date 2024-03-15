# TODO: 
# The map, consisting of the estimated obstacle locations (in green)
# A nav_msgs/Path corresponding to the path the robot takes according to slam (in green).


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, AndSubstitution, NotEqualsSubstitution


def generate_launch_description():
    return LaunchDescription([
        # Args
        DeclareLaunchArgument('robot',
                              default_value="nusim",
                              description="Determines whether the simulator or the actual robot" +
                              "is used (nusim | localhost | none)"),
        DeclareLaunchArgument('use_rviz',
                              default_value="false",
                              description='Determines whether or not rviz is used (true | false)'),


        # Publish Identity Transforms
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_map_tf",
            on_exit=Shutdown(),
            arguments=['--frame-id', 'nusim/world',
                       '--child-frame-id', 'map'],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_green_tf",
            on_exit=Shutdown(),
            arguments=['--frame-id', 'odom',
                       '--child-frame-id', 'green/base_footprint'],
        ),

        # robot = nusim
        GroupAction(
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('robot'), "nusim")),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2_slam',
                    output='screen',
                    on_exit=Shutdown(),
                    arguments=['-d', PathJoinSubstitution([FindPackageShare('nuslam'),
                                                           'config',
                                                           'nuslam.rviz'])],
                ),

                Node(
                    package='nuslam',
                    executable='nuslam',
                    name='nuslam',
                    on_exit=Shutdown(),
                ),

                # Include start_robot
                IncludeLaunchDescription(
                    PathJoinSubstitution([
                        FindPackageShare("nuturtle_control"),
                        "launch",
                        "start_robot.launch.py"
                    ]),
                    launch_arguments={
                        'use_rviz': 'false',
                        'use_jsp': 'false',
                        'odom_frame': 'uncorrected_odom',
                        'robot': 'nusim'
                    }.items(),
                ),

            ],
        ),

        # Include the green robot
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("nuturtle_description"),
                "launch",
                "load_one.launch.py"
            ]),
            launch_arguments={
                'color': 'green',
                'use_rviz': 'false',
                'use_jsp': 'true'
            }.items(),
        ),
    ])
