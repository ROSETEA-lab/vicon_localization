# vicon_localization

A ROS 2 Jazzy **Lifecycle Node** that uses a Vicon rigid-body pose as an absolute localization source for Nav2. It computes the `map -> odom` TF transform from Vicon tracking and the robot's existing odometry, and periodically publishes a global pose and a global-frame `nav_msgs/msg/Odometry` message.

This package **replaces the global localization role of AMCL or robot_localization**, to test the Nav2 architecture on a real robot using a Vicon motion tracking system.

The `Odometry` message carries a 6-DoF twist derived from consecutive global poses using finite differences.

**Covariances are currently zero-filled**, not estimated from the Vicon measurement.


## Interfaces

| Direction | Name | Type / purpose |
| --- | --- | --- |
| Input | `/vicon/pose` (configurable) | `geometry_msgs/msg/PoseStamped` Vicon rigid-body pose in `vicon_world` |
| Input | TF `odom -> base_link` | Time-matched robot odometry |
| Input | TF `vicon_world -> map` | Static calibration |
| Input | TF `base_link -> vicon_robot` | Static rigid-body mounting calibration |
| Output | `/vicon_localization_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` pose of `base_link` in `map` |
| Output | `/vicon_localization_odom` | `nav_msgs/msg/Odometry` pose in `map`, twist in `base_link` |
| Output | TF `map -> odom` | Published when `tf_broadcast` is `true` |

Both output messages are published from a timer with rate `localization_publishing_rate` when a suitable sample and TF are available. The output message headers carry the **selected Vicon sample's timestamp**, not the time of publication. The TF is stamped at `t_vicon + transform_tolerance`.

## Coordinate frames

The TF tree expected by the supplied launch files is:

```text
vicon_world
  └── map                  (static: vicon_world -> map)
      └── odom             (dynamic: published by vicon_localization)
          └── base_link    (dynamic: provided by the robot odometry source)
              └── vicon_robot (static: base_link -> vicon_robot)
```

The **measured** `vicon_world -> vicon_robot` pose is sent as a **topic message**, not as a second edge in that TF tree.

All transforms are handled in **3D**; the implementation does not assume planar motion. The robot must publish a valid `odom -> base_link` TF, and the two static calibrations must be available through TF. The incoming Vicon message must use the configured `vicon_world_frame` as `header.frame_id`.

### Static calibrations

`static_calib_tf_publisher.launch.py` reads the calibration fields from `config/vicon_localization.yaml` and starts two `tf2_ros/static_transform_publisher` processes. The values are:

- `world_map_xyz` and `world_map_rpy`: origin and orientation of `map` **in** `vicon_world`.
- `base_vicon_xyz` and `base_vicon_rpy`: origin and orientation of `vicon_robot` **in** `base_link`.

## Installation

Install ROS 2 **Jazzy** and Nav2, then place this package under a colcon workspace:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src
# Copy or clone vicon_localization into ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select vicon_localization --symlink-install
source install/setup.bash
```


## Running

### With an existing Nav2 Lifecycle Manager

```bash
ros2 launch vicon_localization vicon_localization.launch.py
```

This starts the localization node and the static TF publishers, but **does not activate** the Lifecycle Node automatically. Include `vicon_localization` in the existing Lifecycle Manager's `node_names`, or transition the node manually:

```bash
ros2 lifecycle set /vicon_localization configure
ros2 lifecycle set /vicon_localization activate
```

### With its own Lifecycle Manager

```bash
ros2 launch vicon_localization vicon_localization_with_manager.launch.py
```

This starts the node, the static TF publishers, and `lifecycle_manager_vicon` with `autostart: true`.


### Simulation example

```bash
ros2 launch vicon_localization vicon_localization_simtest.launch.py
```

The simulation launch:

- starts the static TF publishers;
- starts `vicon_localization` with `use_sim_time: true`;
- remaps its input from `/vicon/pose` to `/ground_truth/pose`;
- starts a Lifecycle Manager with `use_sim_time: true`;
- includes `burger_8traj_test.launch.py` from the **separate** `unicycle_ode_simulator` package, passing `publish_localization:=false`.

This launch requires that simulator package to be installed and available in the ROS environment. Ensure the simulator publishes `/clock` and that all participating nodes use a compatible time source.

## Parameters

The default configuration is in [`config/vicon_localization.yaml`](config/vicon_localization.yaml):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `vicon_pose_topic` | `/vicon/pose` | Input `PoseStamped` topic |
| `global_frame_id` | `map` | Global localization frame |
| `odom_frame_id` | `odom` | Local odometry frame |
| `base_frame_id` | `base_link` | Robot base frame |
| `vicon_world_frame` | `vicon_world` | Frame declared by the Vicon input message |
| `vicon_robot_frame` | `vicon_robot` | Tracked rigid-body frame |
| `tf_broadcast` | `true` | Publish the `map -> odom` TF |
| `localization_publishing_rate` | `20.0` | Periodic publish callback frequency [Hz] |
| `lookup_timeout_sec` | `0.05` | Timeout when looking up static calibrations [s] |
| `max_pose_age_sec` | `0.5` | Maximum accepted Vicon sample age [s] |
| `transform_tolerance` | `0.20` | Offset added to the selected Vicon timestamp for TF [s] |
| `world_map_xyz` | `[0, 0, 0]` | Static `vicon_world -> map` translation [m] |
| `world_map_rpy` | `[0, 0, 0]` | Static `vicon_world -> map` rotation [rad] |
| `base_vicon_xyz` | `[0, 0, 0]` | Static `base_link -> vicon_robot` translation [m] |
| `base_vicon_rpy` | `[0, 0, 0]` | Static `base_link -> vicon_robot` rotation [rad] |

The `world_map_*` and `base_vicon_*` values are consumed by the **static TF launch file**, not declared as parameters of the C++ localization node.
