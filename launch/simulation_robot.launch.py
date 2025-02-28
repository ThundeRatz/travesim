from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration


def launch_gz(context, *args, **kwargs):
    verbose = LaunchConfiguration("verbose").perform(context)
    gz_args_extra = LaunchConfiguration("gz_args_extra").perform(context)
    world = LaunchConfiguration("world").perform(context)

    gz_args = []

    if verbose:
        gz_args.append("-v 3")

    if gz_args_extra:
        gz_args.append(gz_args_extra)

    gz_args.append("")

    args = f"-r {world} " + " ".join(gz_args)

    print(args)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": args}.items(),
    )

    return [gz_sim]


def generate_launch_description():

    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [
                    FindPackageShare("travesim"),
                    "urdf",
                    "generic_vss_robot.xacro",
                ]
            ),
        ]
    )

    robot_controllers = "/home/felipe/Documents/Github/vss_ws/src/travesim/config/diff_drive_controller.yaml"

    # PathJoinSubstitution(
    #     [
    #         FindPackageShare('travesim'),
    #         'config',
    #         'diff_drive_controller.yaml',
    #     ]
    # )

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
            DeclareLaunchArgument(
                "world",
                default_value="vss_field.world",
                description="Gazebo Sim GUI config file",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-topic", "robot_description", "-x", "0.4", "-z", "0.012"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "diff_drive_base_controller",
                    "--param-file",
                    robot_controllers,
                ],
            ),
            OpaqueFunction(function=launch_gz),
        ]
    )
