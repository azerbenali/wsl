# Coordinate frames

All frames follow [REP-103](https://www.ros.org/reps/rep-0103.html) (x forward,
y left, z up; optical frames z forward, x right, y down) and
[REP-105](https://www.ros.org/reps/rep-0105.html) (`map` → `odom` → base).

```
map                        rtabmap        (only when slam:=true)
└── odom                   DiffDrive via ros_gz_bridge (/tf)
    └── base_footprint
        └── base_link                  robot_state_publisher (/tf_static)
            ├── lidar_link             robot_state_publisher (/tf_static)
            ├── camera_link            robot_state_publisher (/tf_static)
            │   └── camera_optical_frame
            ├── imu_link               robot_state_publisher (/tf_static)
            ├── left_wheel_link        robot_state_publisher (/tf, from /joint_states)
            └── right_wheel_link       robot_state_publisher (/tf, from /joint_states)

world                      Gazebo; ground truth only, never on /tf
```

## Frame table

| Frame | Parent | Publisher | Pose in parent | Purpose |
|---|---|---|---|---|
| `map` | – | – | – | Globally consistent SLAM frame. Origin = robot pose when RTAB-Map started. |
| `odom` | `map` | `rtabmap` (`publish_tf`) | SLAM correction | Continuous, drifting odometry frame. Origin = spawn pose. |
| `base_footprint` | `odom` | Gazebo DiffDrive → bridge `/wsl/tf` → `/tf` | wheel odometry | Robot pose projected on the ground plane. Rotation centre (midpoint of the drive axle). |
| `base_link` | `base_footprint` | robot_state_publisher | z = 0.08 m | Body frame at axle height (wheel radius). |
| `lidar_link` | `base_link` | robot_state_publisher | (0.15, 0, 0.42) m → 0.50 m above ground | 3D LiDAR origin. `/lidar/points` is expressed here. |
| `camera_link` | `base_link` | robot_state_publisher | (0.315, 0, 0.11) m | Camera body, REP-103 orientation (x along the optical axis). Gazebo renders along its +x. |
| `camera_optical_frame` | `camera_link` | robot_state_publisher | rpy (−π/2, 0, −π/2) | Image and `CameraInfo` frame: z forward, x right, y down. |
| `imu_link` | `base_link` | robot_state_publisher | (0, 0, 0.06) m | IMU at the chassis centre, axes aligned with `base_link`. |
| `left_wheel_link` / `right_wheel_link` | `base_link` | robot_state_publisher | (0, ±0.22, 0) m | Continuous joints about +y; positive rotation rolls the robot forward. |
| `world` | – | Gazebo | – | Simulator frame. Appears only as `header.frame_id` of `/ground_truth/odom`. |

## Single authority per transform

| Transform | Only authority | Why |
|---|---|---|
| `map → odom` | `rtabmap` | SLAM is the only component with a global estimate. |
| `odom → base_footprint` | DiffDrive (bridged `/wsl/tf`) | The only odometry source. Nothing else bridges `/tf`. |
| `base_* → sensors` | robot_state_publisher | Generated from the same URDF Gazebo uses, so simulated sensor poses and TF cannot disagree. |

Ground truth is deliberately **not** a TF frame: `OdometryPublisher` writes its TF to
`/wsl/ground_truth/tf`, which is never bridged. Only `/ground_truth/odom` reaches ROS, and
only when `ground_truth:=true`.

## Verification

Automated:

- `wsl_description/test/test_urdf.py` checks the tree, the root, the optical rotation
  (optical z = base x, optical x = base −y, optical y = base −z) and the LiDAR height.
- `wsl_sim/test/test_system_smoke.launch.py` checks at runtime that `odom → camera_optical_frame`
  resolves, that `base_footprint → lidar_link` is (0.15, 0, 0.50) m, that sensor messages carry
  `lidar_link`, `camera_optical_frame` and `imu_link`, that the lowest LiDAR ring hits the floor
  0.5 m below `lidar_link`, and that without SLAM nothing publishes a transform from `map`.

Manual:

```bash
ros2 run tf2_tools view_frames
```

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```
