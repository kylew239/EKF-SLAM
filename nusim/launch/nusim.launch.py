from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        # Args
        DeclareLaunchArgument('param_file',
                              default_value=PathJoinSubstitution([FindPackageShare("nusim"),
                                                                  "config",
                                                                  "basic_world.yaml"]),
                              description="Parameter File file"),
        DeclareLaunchArgument('use_jsp',
                              default_value="true",
                              description="Launches joint state publisher (true | false)"),
        DeclareLaunchArgument('use_rviz',
                              default_value="true",
                              description="Determines whether or not to use rviz"),

        # Nodes
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('nusim'),
                                                   'config',
                                                   'nusim.rviz'])],
            on_exit=Shutdown(),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_rviz'), "true"))),
        
        Node(
            package='nusim',
            executable='nusim_node',
            on_exit=Shutdown(),
            parameters=[LaunchConfiguration('param_file')]),

        # Include launch
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("nuturtle_description"),
                "launch",
                "load_one.launch.py"
            ]),
            launch_arguments={
                'color': 'red',
                'use_rviz': 'false',
                'use_jsp': LaunchConfiguration('use_jsp')
            }.items(),
        ),

    ])
