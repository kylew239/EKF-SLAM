from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage


def generate_launch_description():
    return LaunchDescription([
        # Args
        DeclareLaunchArgument('use_rviz',
                              default_value="true",
                              description="Launches rviz (true | false)"),
        DeclareLaunchArgument('use_jsp',
                              default_value="true",
                              description="Launches joint state publisher (true | false)"),
        DeclareLaunchArgument('color',
                              default_value="purple",
                              description="Color of the turtlebot",
                              choices=['purple', 'red', 'green', 'blue', '']),
        SetLaunchConfiguration(name='rviz_file',
                               value=[TextSubstitution(text='basic_'),
                                      LaunchConfiguration('color'),
                                      TextSubstitution(text='.rviz')]),

        # Nodes
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{"robot_description":
                         Command([ExecutableInPackage('xacro', 'xacro'),
                                  TextSubstitution(text=' '),
                                  PathJoinSubstitution([FindPackageShare("nuturtle_description"),
                                                        "urdf",
                                                        "turtlebot3_burger.urdf.xacro"]),
                                  TextSubstitution(text=' color:='),
                                  LaunchConfiguration('color')]),
                        "frame_prefix":
                         [LaunchConfiguration('color'),
                         TextSubstitution(text='/')]}],
            namespace=LaunchConfiguration('color')),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', PathJoinSubstitution([FindPackageShare('nuturtle_description'),
                                                   'config',
                                                   LaunchConfiguration('rviz_file')]),
                       '-f', [LaunchConfiguration('color'),
                              TextSubstitution(text='/base_footprint')]],
            on_exit=Shutdown(),
            namespace=LaunchConfiguration('color')),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_jsp')),
            namespace=LaunchConfiguration('color'))

    ])
