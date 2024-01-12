from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage


def generate_launch_description():
    return LaunchDescription([
        # Including load_one
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("nuturtle_description"),
                "launch",
                "load_one.launch.py"]),
            launch_arguments={
                'color': TextSubstitution(text='purple'),
                'use_rviz': TextSubstitution(text='false')
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("nuturtle_description"),
                "launch",
                "load_one.launch.py"]),
            launch_arguments={
                'color': TextSubstitution(text='red'),
                'use_rviz': TextSubstitution(text='false')
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("nuturtle_description"),
                "launch",
                "load_one.launch.py"]),
            launch_arguments={
                'color': TextSubstitution(text='green'),
                'use_rviz': TextSubstitution(text='false')
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("nuturtle_description"),
                "launch",
                "load_one.launch.py"]),
            launch_arguments={
                'color': TextSubstitution(text='blue'),
                'use_rviz': TextSubstitution(text='false')
            }.items()
        ),


        # Static Transform Publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='nusim_world_red',
            arguments=['--x', '0.3',
                       '--frame-id', TextSubstitution(text='nusim/world'),
                       '--child-frame-id', TextSubstitution(text='red/base_footprint')]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='nusim_world_green',
            arguments=['--y', '0.6',
                       '--frame-id', TextSubstitution(text='nusim/world'),
                       '--child-frame-id', TextSubstitution(text='green/base_footprint')]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='nusim_world_blue',
            arguments=['--x', '-0.71',
                       '--frame-id', TextSubstitution(text='nusim/world'),
                       '--child-frame-id', TextSubstitution(text='blue/base_footprint')]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='nusim_world_purple',
            arguments=['--y', '-0.9',
                       '--frame-id', TextSubstitution(text='nusim/world'),
                       '--child-frame-id', TextSubstitution(text='purple/base_footprint')]
        ),

        # RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('nuturtle_description'),
                                                   'config',
                                                   'basic_all.rviz'])],
            on_exit=Shutdown()
        )
    ])
