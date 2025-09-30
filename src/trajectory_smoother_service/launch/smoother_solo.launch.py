# trajectory_smoother_service/launch/smoother_solo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def launch_setup(context, *args, **kwargs):
    use_sim_time           = LaunchConfiguration("use_sim_time")
    planning_group         = LaunchConfiguration("planning_group")
    robot_name             = LaunchConfiguration("robot_name")
    ur_type                = LaunchConfiguration("ur_type")
    prefix                 = LaunchConfiguration("prefix")
    description_package    = LaunchConfiguration("description_package")
    description_file       = LaunchConfiguration("description_file")
    moveit_config_package  = LaunchConfiguration("moveit_config_package")
    moveit_srdf_file       = LaunchConfiguration("moveit_srdf_file")
    smoother_params_pkg    = LaunchConfiguration("smoother_params_pkg")
    smoother_params_file   = LaunchConfiguration("smoother_params_file")

    moveit_config_package  = LaunchConfiguration("moveit_config_package")
    moveit_srdf_file_arg   = LaunchConfiguration("moveit_srdf_file")  # 사용자가 직접 지정하면 우선

    # --- SRDF 파일 자동 탐색 ---
    pkg_share = FindPackageShare(moveit_config_package).perform(context)
    srdf_dir  = os.path.join(pkg_share, "srdf")

    # 우선순위: 사용자가 지정 → ur5e.srdf.xacro → ur.srdf.xacro → ur_macro.srdf.xacro → ur5e.srdf
    candidates = []
    user_spec = moveit_srdf_file_arg.perform(context)
    if user_spec:
        candidates.append(os.path.join(srdf_dir, user_spec))
    candidates += [
        os.path.join(srdf_dir, "ur5e.srdf.xacro"),
        os.path.join(srdf_dir, "ur.srdf.xacro"),
        os.path.join(srdf_dir, "ur_macro.srdf.xacro"),
        os.path.join(srdf_dir, "ur5e.srdf"),  # plain srdf일 수도
    ]

    srdf_path = next((p for p in candidates if os.path.exists(p)), None)
    if not srdf_path:
        raise RuntimeError(f"SRDF not found under: {srdf_dir}")

    # .xacro이면 xacro로, .srdf면 cat으로 읽기
    if srdf_path.endswith(".xacro"):
        srdf_cmd = [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            srdf_path,
            " ",
            "name:=",  robot_name, " ",
            "ur_type:=", ur_type, " ",
            "prefix:=",  prefix,
        ]
    else:
        # plain .srdf
        srdf_cmd = ["bash", "-lc", f"cat '{srdf_path}'"]

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(srdf_cmd),
            value_type=str,
        )
    }

    # 1) URDF → robot_description  (xacro 결과 XML을 문자열 파라미터로)
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
                "name:=",  robot_name, " ",
                "ur_type:=", ur_type, " ",
                "prefix:=",  prefix,
            ]),
            value_type=str,
        )
    }

    # # 2) SRDF → robot_description_semantic  (ur_moveit_config/srdf/ 에서 읽음)
    # robot_description_semantic = {
    #     "robot_description_semantic": ParameterValue(
    #         Command([
    #             PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
    #             PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_srdf_file]), " ",
    #             "name:=",  robot_name, " ",
    #             "ur_type:=", ur_type, " ",
    #             "prefix:=",  prefix,
    #         ]),
    #         value_type=str,
    #     )
    # }

    # (선택) 스무더 파라미터 YAML
    smoother_params_path = PathJoinSubstitution([
        FindPackageShare(smoother_params_pkg), "config", smoother_params_file
    ])

    node = Node(
        package="trajectory_smoother_service",
        executable="trajectory_smoother_node",
        name="trajectory_smoother",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"planning_group": planning_group},       # 예: ur5e_manipulator
            robot_description,
            robot_description_semantic,
            smoother_params_path,                     # YAML이 없으면 삭제해도 됨
        ],
    )
    return [node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("planning_group", default_value="ur5e_manipulator"),  # ← 당신 SRDF에 맞춤
        DeclareLaunchArgument("robot_name", default_value="ur5e"),
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("description_package", default_value="ur_description"),
        DeclareLaunchArgument("description_file", default_value="ur.urdf.xacro"),
        DeclareLaunchArgument("moveit_config_package", default_value="ur_moveit_config"),
        DeclareLaunchArgument("moveit_srdf_file", default_value="ur5e.srdf.xacro"), # ur.srdf.xacro면 이 값 바꿔줘
        DeclareLaunchArgument("smoother_params_pkg", default_value="trajectory_smoother_service"),
        DeclareLaunchArgument("smoother_params_file", default_value="smoother_parameters.yaml"),
        OpaqueFunction(function=launch_setup),
    ])
