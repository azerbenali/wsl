# Architecture

WSL is a simulated warehouse robot plus the software that perceives, maps and
measures it. Five packages, each owning one responsibility:

| Package | Owns | Contains code? |
|---|---|---|
| `wsl_description` | Robot hardware: geometry, inertia, joints, sensor frames, Gazebo sensor and drive definitions | Xacro + tests |
| `wsl_sim` | Simulation runtime: warehouse world and assets, ROS–Gazebo bridge, full-system launch, RViz | World/launch/config + tests |
| `wsl_perception` | 3D point-cloud processing: validation, filtering, ground removal, clustering | C++ library + node |
| `wsl_slam` | RTAB-Map configuration and launch | Config + launch only |
| `wsl_benchmark` | Evaluation: scenarios, route driving, recording, metrics | Python |

`wsl_slam` has no code on purpose: RTAB-Map is mature, and the engineering work
is choosing its inputs and parameters. Forking it would add maintenance without
adding capability.

## Data flow

```mermaid
flowchart LR
    subgraph Gazebo Harmonic
        DD[DiffDrive]
        L[gpu_lidar 16x900 @10 Hz]
        C[camera 640x480 @20 Hz]
        I[IMU @100 Hz]
        GT[OdometryPublisher<br/>ground truth]
    end
    B[ros_gz_bridge<br/>bridge.yaml]
    GTB[ros_gz_bridge<br/>bridge_ground_truth.yaml]
    RSP[robot_state_publisher]
    P[perception_node]
    S[rtabmap]
    R[RViz]
    REC[benchmark recorder]
    RF[route_follower]

    DD -- /odom, /tf odom→base_footprint --> B
    L -- /lidar/points --> B
    C -- /camera/image_raw, camera_info --> B
    I -- /imu/data --> B
    B -- /joint_states --> RSP
    B --> P
    B --> S
    B --> R
    P -- /perception/* --> R
    S -- /tf map→odom, /rtabmap/map, mapPath --> R
    GT -. /ground_truth/odom .-> GTB
    GTB -.-> REC
    GTB -.-> RF
    RF -- /cmd_vel --> B
    B -- /cmd_vel --> DD
    P --> REC
    S --> REC
```

Dotted edges exist only in benchmark runs (`ground_truth:=true`).

## Interfaces

| Topic | Type | Producer | Consumers |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | teleop / `route_follower` | DiffDrive (via bridge) |
| `/odom` | `nav_msgs/Odometry` | DiffDrive | recorder |
| `/tf` | `tf2_msgs/TFMessage` | DiffDrive (`odom→base_footprint`), robot_state_publisher (wheels), rtabmap (`map→odom`) | all |
| `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher | all |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo JointStatePublisher | robot_state_publisher |
| `/lidar/points` | `sensor_msgs/PointCloud2` (`lidar_link`, x y z intensity ring, organised 16×900) | gpu_lidar | perception, rtabmap, RViz, recorder |
| `/camera/image_raw` | `sensor_msgs/Image` (`rgb8`, `camera_optical_frame`) | camera | RViz, recorder |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera | recorder |
| `/imu/data` | `sensor_msgs/Imu` (`imu_link`) | IMU | recorder |
| `/perception/filtered_points` | `PointCloud2` (`base_footprint`) | perception | recorder |
| `/perception/obstacle_points` | `PointCloud2` (`base_footprint`) | perception | RViz |
| `/perception/clusters` | `vision_msgs/Detection3DArray` | perception | recorder |
| `/perception/markers` | `visualization_msgs/MarkerArray` | perception | RViz |
| `/rtabmap/map` | `nav_msgs/OccupancyGrid` | rtabmap | RViz |
| `/rtabmap/mapPath` | `nav_msgs/Path` (optimised graph poses) | rtabmap | RViz, recorder |
| `/rtabmap/info` | `rtabmap_msgs/Info` | rtabmap | recorder |
| `/ground_truth/odom` | `nav_msgs/Odometry` (`world`) | Gazebo OdometryPublisher | recorder, route_follower |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | perception | operators |

Gazebo-side names live under `/wsl/`; `wsl_sim/config/bridge.yaml` is the only
place where they are mapped to the ROS names above. A test checks that every
bridged Gazebo topic is actually declared by the robot model.

## Decisions

**Single source of truth for geometry.** The URDF/Xacro in `wsl_description`
is used by both Gazebo (spawned from `/robot_description`) and
robot_state_publisher, so simulated sensor poses and TF cannot disagree.
Inertia is computed from the primitive shapes by Xacro macros rather than typed
in, and a test checks every tensor is positive definite and satisfies the
triangle inequality.

**Primitives instead of the reference meshes.** The MOGI reference meshes are a
0.40 × 0.20 m teaching robot; the WSL AMR is 0.60 × 0.38 m with a sensor mast.
Rescaling the meshes would distort the wheels, and primitives render cheaply in
every camera and GPU-LiDAR frame. No third-party assets are redistributed.

**Detailed visuals, envelope collisions.** Each warehouse asset has one
collision box around its outer envelope, while its visuals keep every upright,
deck and load for the LiDAR and camera, which render visuals and never touch
collision geometry. With per-part collision boxes (~800 static shapes) the
500 Hz DART step was the simulation bottleneck. A controlled experiment
(below) measured RTF 0.51 with per-part collisions and 0.99 with envelopes at
the same 2 ms step, with no change in what any sensor observes. A test keeps
it that way.

**Genuine wheel odometry with a calibration-error model.** DiffDrive integrates
wheel joint positions, so `/odom` drifts with slip. Simulated wheels barely
slip, so the scenarios set `odom_wheel_separation_scale` / `odom_wheel_radius_scale`:
DiffDrive then believes a wrong geometry for both control and odometry, which
reproduces the systematic errors of an uncalibrated real robot.

**SLAM reads odometry from TF, not the topic.** RTAB-Map looks up
`odom→base_footprint` at each scan's own stamp. This avoids approximate topic
synchronisation, and the scan is placed at the odometry pose of its capture time.

**Ground truth is isolated by construction.** It is published only when
`ground_truth:=true`, through a separate bridge, under `/ground_truth/`, with no
TF. The estimators (perception and rtabmap) neither subscribe to it nor receive
it through TF. `route_follower` steers on it on purpose: it is part of the
experiment harness and reproduces the same physical path in every run so SLAM
configurations are compared on identical input. It publishes only `/cmd_vel`.

**Perception: algorithms separate from transport.** `wsl_perception_core`
depends on PCL and the `sensor_msgs` type, not on rclcpp, so every stage is
unit-tested with synthetic scans. The node adds TF, parameters, diagnostics and
publishing.

**Unidirectional bridging.** Every bridge entry is `GZ_TO_ROS` or `ROS_TO_GZ`.
Bidirectional `/tf` bridging, as in the reference, risks loops and duplicate
authorities.

## QoS policy

- The bridge publishes reliable (ros_gz default).
- Consumers that only want the newest sample subscribe best-effort: perception
  (`SensorDataQoS`, depth 1) and RViz's raw LiDAR display. A best-effort
  subscriber matches a reliable publisher.
- **Large images are consumed reliable.** Measured in this setup (Fast DDS,
  same host): a best-effort subscriber to 921 KB `rgb8` frames received
  **2.4 of 20 Hz**, a reliable subscriber **20.00 Hz**. The LiDAR cloud
  (460 KB) arrived complete either way (10.00 Hz). RViz's image display and the
  recorder therefore use reliable.
- Perception publishes reliable (depth 2), so tools with default QoS can subscribe.

## Concurrency

- `perception_node`: single-threaded executor. The TF listener has its own
  thread, so the bounded TF wait (`tf_timeout_s`) cannot starve TF updates.
  Processing (5.6 ms mean on a synthetic 14.4 k-point scan in isolation,
  16.4 ms mean / 43 ms max in the full stack on 4 shared cores) stays far below
  the 100 ms LiDAR period. A depth-1 subscription drops stale clouds instead of
  queueing them if this ever changes.
- rtabmap processes at its 1 Hz detection rate with `topic_queue_size: 1`.
- Recorder and route follower are single-threaded Python nodes.

## Performance

Profiled before optimizing, on a 4-core x86_64 VM with Mesa llvmpipe
software rendering. Steady-state RTF from Gazebo's `/stats`, 20 samples after
20 s of warm-up:

| Configuration (Gazebo only) | RTF |
|---|---|
| Empty world + robot with all sensors | 0.965 |
| Warehouse, per-part collisions, no robot | 0.805 |
| Warehouse, per-part collisions + robot without sensors | 0.705 |
| Warehouse, per-part collisions + robot | 0.508 |
| … same, 4 ms physics step instead of 2 ms | 0.946 |
| **Warehouse, envelope collisions + robot (current)** | **0.988** |

Reducing sensor load barely helped (full ROS stack without SLAM: default 0.503,
camera at 5 Hz 0.499, LiDAR 8 × 450 0.497, both 0.536). Halving the
physics step rate nearly doubled RTF. The cost was the physics step over
hundreds of static collision shapes. Envelope collisions removed it without
the dynamics and timing cost of a coarser step: 4 ms is not a divisor of the
IMU's 10 ms period, so IMU samples would jitter.

Perception is far from being a bottleneck: 5.6 ms mean on a synthetic 16 × 900
aisle scan, 83% of it in Euclidean clustering (KD-tree radius search).
It is not optimized further while it uses < 10% of the sensor period.

## Failure handling

| Failure | Behaviour |
|---|---|
| Gazebo not up yet | `ros_gz_sim create` waits for `/world/<world>/create`; no sleeps in the launch. |
| Gazebo exits | `on_exit_shutdown` stops the whole launch. |
| Invalid perception parameters | Out-of-range value or inconsistent combination (e.g. `min_range_m ≥ max_range_m`): the node exits at startup naming the parameter. Runtime changes are validated as a whole and rejected with a reason. |
| Malformed `PointCloud2` | Empty cloud, missing frame, missing/non-float xyz, big-endian, or step/size inconsistency: rejected before any byte is read, counted, warning throttled. |
| NaN / Inf points | Dropped and counted per cloud. |
| TF unavailable | Bounded wait per cloud; the cloud is dropped with `Cannot transform lidar_link -> base_footprint at t=… s: …` (throttled). |
| Clock jumps back (sim reset) | Warned; statistics reset; tf2 clears its buffer on the jump. Duplicate stamps are dropped. |
| Sensor stalls | Perception diagnostics go ERROR after 2 s without clouds, WARN below 80% of the expected rate or when processing exceeds the sensor period. |
| SLAM absent | Route follower starts after `map_frame_timeout_s` with a warning; the recorder evaluates whatever trajectories exist. |
| Robot stuck / route too slow | Route follower aborts (`stuck`: < 0.5 m progress in 30 s; `timeout`), publishes the outcome, and the run is still written. |
| Unknown scenario / bad scenario file | Launch fails listing the available scenarios; unknown robot arguments in a scenario are rejected. |

## Configuration ownership

| Quantity | Owner | Overridable via |
|---|---|---|
| Robot geometry, mass, inertia | `wsl_description/urdf/wsl_amr.urdf.xacro` | – |
| Sensor simulation (rate, resolution, noise) | `wsl_description/urdf/wsl_amr.gazebo.xacro` | Xacro args ← launch args ← scenario `robot:` |
| ROS ↔ Gazebo topic mapping | `wsl_sim/config/bridge*.yaml` | – |
| World layout, physics rate | `wsl_sim/worlds/warehouse.sdf` | `world:=` |
| Perception parameters | `wsl_perception/config/perception.yaml` | `ros2 param set` at runtime (validated) |
| SLAM tuning | `wsl_slam/config/rtabmap.yaml` | – |
| Experiments | `wsl_benchmark/config/scenarios/*.yaml` | `scenario:=` |

## Testing strategy

| Level | What | Where |
|---|---|---|
| Unit (C++) | PointCloud2 parsing incl. hostile layouts, every pipeline stage, boundaries, config validation | `wsl_perception/test/*.cpp` |
| Property / fuzz | 300 random configs × clouds with NaN/Inf/extremes per invariant test; 2000 random byte layouts; libFuzzer target | `test_pipeline_properties.cpp`, `fuzz/` |
| Unit (Python) | Benchmark maths against analytic results, scenario validation, pure-pursuit closed loop | `wsl_benchmark/test/` |
| Model | URDF structure, inertia physics, frame conventions, check_urdf | `wsl_description/test/` |
| Asset consistency | Model URIs, bridge directions, GT isolation, bridge↔robot topic match | `wsl_sim/test/test_sim_assets.py` |
| Node | Real `perception_node` with real TF: detection, atomic reconfiguration, startup rejection | `test_perception_node.launch.py` |
| System | Headless full launch: sensors, frames, rates, TF authorities, driving (opt-in) | `test_system_smoke.launch.py` |
| Experiment | Scenario runs against ground truth | `ros2 launch wsl_benchmark benchmark.launch.py` |
