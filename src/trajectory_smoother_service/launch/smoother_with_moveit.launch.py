#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    smoother_params_file = LaunchConfiguration("smoother_params_file")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "moveit_joint_limits_file": moveit_joint_limits_file,
            "prefix": prefix,
            "use_sim_time": use_sim_time,
            "launch_rviz": launch_rviz,
            "launch_servo": launch_servo,
        }.items(),
    )

    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),  # or "ur_moveit_config" if you use the installed one
        "config",
        "kinematics.yaml",
    ])

    trajectory_smoother_node = Node(
        package="trajectory_smoother_service",
        executable="trajectory_smoother_node",
        name="trajectory_smoother",
        output="screen",
        parameters=[
            smoother_params_file,
            kinematics_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [moveit_launch, trajectory_smoother_node]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur15",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Package containing the robot description xacros.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="Primary robot description xacro file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt configuration package to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="SRDF xacro file for MoveIt.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("trajectory_smoother_service"),
                "config",
                "joint_limits.yaml",
            ]),
            description="MoveIt joint limits YAML file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Joint name prefix (useful for multi-robot setups).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time for MoveIt and the smoother node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with the MoveIt configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="false",
            description="Launch the MoveIt Servo node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "smoother_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("trajectory_smoother_service"),
                "config",
                "smoother_parameters.yaml",
            ]),
            description="YAML parameter file for the trajectory smoother service.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
