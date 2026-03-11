from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('emergency_landing_sim')
    gazebo_ros = FindPackageShare('gazebo_ros')

    world = PathJoinSubstitution([pkg_share, 'worlds', 'emergency_landing.world'])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'pixhawk450_6depth.urdf.xacro'])

    robot_description = Command(['xacro ', xacro_file])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([gazebo_ros, 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            'extra_gazebo_args': '--verbose -s libgazebo_ros_factory.so -s libgazebo_ros_init.so'
        }.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'pixhawk450_6depth',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', '6depth_gazebo.rviz'])],
        output='screen'
    )

    return LaunchDescription([gazebo, rsp, spawn, rviz])
