import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    yaml_path = os.path.join(
        get_package_share_directory("vicon_localization"),
        "config",
        "vicon_localization.yaml",
    )

    with open(yaml_path, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    params = config["vicon_localization"]["ros__parameters"]
    vicon_world_frame = params["vicon_world_frame"]
    global_frame_id = params["global_frame_id"]
    world_map_xyz = params["world_map_xyz"]
    world_map_rpy = params["world_map_rpy"]
    base_frame_id = params["base_frame_id"]
    vicon_robot_frame = params["vicon_robot_frame"]
    base_vicon_xyz = params["base_vicon_xyz"]
    base_vicon_rpy = params["base_vicon_rpy"]

    world_map_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_map_publisher",
        arguments=[
            "--x", str(world_map_xyz[0]),
            "--y", str(world_map_xyz[1]),
            "--z", str(world_map_xyz[2]),
            "--roll", str(world_map_rpy[0]),
            "--pitch", str(world_map_rpy[1]),
            "--yaw", str(world_map_rpy[2]),
            "--frame-id", str(vicon_world_frame),
            "--child-frame-id", str(global_frame_id),
        ],
    )

    base_vicon_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="baselink_vicon_publisher",
        arguments=[
            "--x", str(base_vicon_xyz[0]),
            "--y", str(base_vicon_xyz[1]),
            "--z", str(base_vicon_xyz[2]),
            "--roll", str(base_vicon_rpy[0]),
            "--pitch", str(base_vicon_rpy[1]),
            "--yaw", str(base_vicon_rpy[2]),
            "--frame-id", str(base_frame_id),
            "--child-frame-id", str(vicon_robot_frame),
        ],
    )

    return LaunchDescription([world_map_publisher_node, base_vicon_publisher_node])