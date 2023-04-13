import os

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


from ament_index_python.packages import get_package_share_directory


def launch_gz(context, *args, **kwargs):
    verbose = LaunchConfiguration("verbose").perform(context)
    gz_args_extra = LaunchConfiguration("gz_args_extra").perform(context)

    gz_args = []

    if verbose:
        gz_args.append("-v 3")

    if gz_args_extra:
        gz_args.append(gz_args_extra)

    print(" ".join(gz_args))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": " ".join(gz_args)}.items(),
    )

    return [gz_sim]


def generate_launch_description():
    pkg_travesim = get_package_share_directory("travesim")

    print(
        " ".join(
            [
                "-string",
                "$(",
                "tmp_file=$(mktemp);",
                f"xacro {os.path.join(pkg_travesim, 'urdf', 'generic_vss_robot.xacro')} > $tmp_file;",
                "gz sdf -p $tmp_file;",
                "rm $tmp_file",
                ")",
            ]
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "verbose",
                default_value="true",
                description="Enable verbose mode in Ignition Gazebo",
            ),
            DeclareLaunchArgument(
                "gz_args_extra",
                default_value="",
                description="Enable verbose mode in Ignition Gazebo",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-string",
                    "$(",
                    "tmp_file=$(mktemp);",
                    f"xacro {os.path.join(pkg_travesim, 'urdf', 'generic_vss_robot.xacro')} > $tmp_file;",
                    "gz sdf -p $tmp_file;",
                    "rm $tmp_file",
                    ")",
                ],
            ),
            # gazebo_resource,
            # gazebo_model,
            OpaqueFunction(function=launch_gz),
        ]
    )
