import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    return LaunchDescription([

        # Static TF publishers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('vicon_localization'),
                    'launch',
                    'static_calib_tf_publisher.launch.py'
                )
            )
        ),

        # AMCL Vicon lifecycle node
        Node(
            package='vicon_localization',
            executable='vicon_localization_node',
            name='vicon_localization',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('vicon_localization'),
                'config',
                'vicon_localization.yaml'),
                {'use_sim_time': True}
            ],
            remappings=[
                ('/vicon/pose','/ground_truth/pose')
            ]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_vicon',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['vicon_localization'],
                'use_sim_time': True
            }]
        ),

        # Simulator node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("unicycle_ode_simulator"),
                    "launch",
                    "burger_8traj_test.launch.py",
                )
            ),
            launch_arguments={
                "publish_localization": "false",
            }.items(),
        )
    ])
