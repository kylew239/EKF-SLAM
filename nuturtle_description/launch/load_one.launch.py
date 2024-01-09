from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage


def generate_launch_description():
    return LaunchDescription([
        # Args/params
        DeclareLaunchArgument('xacro_file',
                              default_value=PathJoinSubstitution(
                                  [FindPackageShare("nuturtle_description"),
                                   "urdf/turtlebot3_burger.urdf.xacro"]),
                              description="The path to the xacro file. Defaults to urdf/turtle.urdf.xacro"),

        DeclareLaunchArgument('use_rviz',
                              default_value="true",
                              description="Controls whether or not rviz is launched (true | false)"),

        # Nodes
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{"robot_description": Command([ExecutableInPackage(
                                                            'xacro', 'xacro'),
                                                       TextSubstitution(
                                                           text=' '),
                                                       LaunchConfiguration(
                                                           'xacro_file')
                                                       ])}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', PathJoinSubstitution([FindPackageShare(
                                                        'nuturtle_description'),
                                                   'config',
                                                   'basic_purple.rviz'])],
            on_exit=Shutdown())
    ])