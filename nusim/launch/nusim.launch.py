from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Args
        DeclareLaunchArgument('param_file',
                              default_value=PathJoinSubstitution([FindPackageShare("nusim"),
                                                                  "config",
                                                                  "basic_world.yaml"]),
                              description="Paramter File file"),

        # Include launch
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("nuturtle_description"),
                "launch",
                "load_one.launch.py"
            ]),
            launch_arguments={
                'color': 'red',
                'use_rviz': 'false'
            }.items(),
        ),

        # Nodes
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('nusim'),
                                                   'config',
                                                   'nusim.rviz'])],
            on_exit=Shutdown()),

        Node(
            package='nusim',
            executable='nusim_node',
            on_exit=Shutdown(),
            parameters=[LaunchConfiguration('param_file')])

    ])
