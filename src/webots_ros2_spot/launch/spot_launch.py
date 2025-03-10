#!/usr/bin/env python

import os
import launch
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


package_dir = get_package_share_directory("webots_spot")


def create_slam_and_octomap_nodes():
    slam_toolbox = Node(
        parameters=[
            {"use_sim_time": True},
            {"base_frame": "base_link"},
            {"max_laser_range": 10.0},
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
    )
    slam_rviz_config = os.path.join(
        get_package_share_directory("webots_spot"), "resource", "rl.rviz"
    )
    slam_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["--display-config=" + slam_rviz_config],
        parameters=[{"use_sim_time": True}],
    )
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        output='screen',
        parameters=[{
            "resolution": 0.1,
            "frame_id": "map",
            "base_frame_id": "base_link",
            "latch": False,
            "exploration": True,
            "multiple_pointclouds": False,
            "use_sim_time": True,
        }],
        remappings=[
            ("cloud_in", "/Spot/Velodyne_Puck/point_cloud")
        ],
    )
    return [slam_toolbox, slam_rviz, octomap_server]


# Define all the ROS 2 nodes that need to be restarted on simulation reset here
def get_ros2_nodes(*args):
    # SpotArm Driver node
    spotarm_ros2_control_params = os.path.join(
        package_dir, "resource", "spotarm_ros2_controllers.yaml"
    )
    spotarm_driver = WebotsController(
        robot_name="SpotArm",
        parameters=[
            {
                "robot_description": os.path.join(
                    package_dir, "resource", "spotarm_control.urdf"
                )
            },
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            spotarm_ros2_control_params,
        ],
    )

    # ROS2 control spawners for SpotArm
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["spotarm_joint_trajectory_controller", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["spotarm_joint_state_broadcaster", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    tiago_gripper_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=[
            "tiago_gripper_joint_trajectory_controller",
            "-c",
            "/controller_manager",
        ]
        + controller_manager_timeout,
    )

    ros2_control_spawners = [
        trajectory_controller_spawner,
        joint_state_broadcaster_spawner,
        tiago_gripper_joint_trajectory_controller_spawner,
    ]

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=spotarm_driver, nodes_to_start=ros2_control_spawners
    )

    initial_manipulator_positioning = Node(
        package="webots_spot",
        executable="retract_manipulator",
        output="screen",
    )

    slam_and_octomap_nodes = create_slam_and_octomap_nodes()

    return [spotarm_driver, waiting_nodes, initial_manipulator_positioning] + slam_and_octomap_nodes


def generate_launch_description():
    node_list = []
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", "spot.wbt"])
    )

    node_list.append(webots)

    ros2_supervisor = Ros2SupervisorLauncher()
    node_list.append(ros2_supervisor)

    spot_driver = WebotsController(
        robot_name="Spot",
        parameters=[
            {
                "robot_description": os.path.join(
                    package_dir, "resource", "spot_control.urdf"
                ),
                "use_sim_time": True,
                "set_robot_state_publisher": False,  # foot positions are wrong with webot's urdf
            }
        ],
        respawn=True,
    )
    node_list.append(spot_driver)

    with open(os.path.join(package_dir, "resource", "spot.urdf")) as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": True,
            }
        ],
    )

    node_list.append(robot_state_publisher)

    # spot_pointcloud2 = Node(
    #     package='webots_spot',
    #     executable='spot_pointcloud2',
    #     output='screen',
    # )

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Resetting SLAM and Octomap nodes')),
                *get_ros2_nodes()
            ],
        )
    )

    node_list.append(reset_handler)

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    node_list.append(webots_event_handler)

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[
            ("cloud_in", "/Spot/Velodyne_Puck/point_cloud"),
        ],
        parameters=[
            {
                "transform_tolerance": 0.01,
                "min_height": 0.0,
                "max_height": 1.0,
                "angle_min": -3.14,
                "angle_max": 3.14,
                "angle_increment": 0.00872,
                "scan_time": 0.1,
                "range_min": 0.9,
                "range_max": 100.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        name="pointcloud_to_laserscan",
    )
    node_list.append(pointcloud_to_laserscan_node)

    map_1m = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "1", "0", "0", "0", "map", "map_1m"],
    )
    node_list.append(map_1m)

    map_2m = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "2", "0", "0", "0", "map", "map_2m"],
    )
    node_list.append(map_2m)

    map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )
    node_list.append(map_odom)

    body_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "body"],
    )
    node_list.append(body_base_link)

    return LaunchDescription(
        node_list +
        get_ros2_nodes()
    )