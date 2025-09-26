from re import S
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    controller_config_file = PathJoinSubstitution(
        [
            FindPackageShare("cartesian_planning_examples"),
            "config",
            "example_controllers.yaml",
        ]
    )

    cartesian_planning_config_file = PathJoinSubstitution(
        [
            FindPackageShare("cartesian_planning_examples"),
            "config",
            "cartesian_planning_example.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("cartesian_planning_examples"),
            "config",
            "cartesian_planning_example.rviz",
        ]
    )

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="The prefix appended to URDF",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Should mock (simulated) hardware be used?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config",
            default_value=controller_config_file,
            description="Path to the configuration file for ros2_control"
        )
    )
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    controller_config_file = LaunchConfiguration("controller_config")

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [
                FindPackageShare("cartesian_planning_examples"),
                "urdf",
                "robot6R.xacro",
            ]
        ),
        " use_mock_hardware:=", use_mock_hardware,
        " prefix:=", prefix,
    ])


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controller_config_file,
        ],
        output="both",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "--controller-manager", "controller_manager",
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description}
        ],
    )

    cartesian_planning_server = Node(
        package="cartesian_planning_server",
        executable="cartesian_planning_server",
        name="cartesian_planning_server",
        output="screen",
        parameters=[
            cartesian_planning_config_file,
            {'robot_description': robot_description}
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        control_node,
        controller_spawner,
        robot_state_publisher_node,
        cartesian_planning_server,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
