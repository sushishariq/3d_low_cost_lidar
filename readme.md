# 3D LiDAR Simulation — ROS 2 Humble

A ROS 2 simulation for a rotating 2D LiDAR scanner. It simulates a vertical 2D LiDAR scan, simulates the rotation encoder, and reconstructs the measurements into a 3D point cloud for visualization in RViz.

## Project Goal

The project is a software prototype for a 3D scanner intended for underground mine environments. The planned hardware uses an RPLIDAR A1 mounted vertically and rotated around a vertical axis. The simulation lets us test the reconstruction pipeline before integrating the physical LiDAR, motor, encoder, and ESP32.

**Current scope:** single-position 3D reconstruction. SLAM, multi-position mapping, and IMU integration are outside the current scope.

## Software Requirements

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- RViz 2

## Package and Nodes

Workspace: `~/3dlidar_ws`  
Package: `lidar_simulation`

| Node | Purpose | Output |
|---|---|---|
| `scan_simulator.py` | Simulates a vertical 2D LiDAR scan in an environment | `/scan` |
| `encoder_simulator.py` | Simulates rotating-platform angle and direction | `/encoder/state` |
| `reconstruction_node.py` | Synchronizes LiDAR rays with encoder angles and reconstructs 3D points | `/cloud` |

Expected ROS message types:

| Topic | Message type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR ranges, scan angles, and timing metadata |
| `/encoder/state` | `sensor_msgs/msg/JointState` | Simulated encoder angle/state |
| `/cloud` | `sensor_msgs/msg/PointCloud2` | Reconstructed 3D point cloud |

## Build the Workspace

```bash
cd ~/3dlidar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the build again after changing package code or configuration. If the workspace has already been built and there are no changes, just source the setup files.

## Run the Simulation

Open four terminals. Keep Terminals 1–3 running while viewing the result in RViz.

### Terminal 1 — Simulated LiDAR

```bash
source /opt/ros/humble/setup.bash
source ~/3dlidar_ws/install/setup.bash
ros2 run lidar_simulation scan_simulator
```

Publishes simulated `LaserScan` messages on `/scan`.

### Terminal 2 — Simulated Encoder

```bash
source /opt/ros/humble/setup.bash
source ~/3dlidar_ws/install/setup.bash
ros2 run lidar_simulation encoder_simulator
```

Publishes simulated encoder state on `/encoder/state`.

### Terminal 3 — 3D Reconstruction

```bash
source /opt/ros/humble/setup.bash
source ~/3dlidar_ws/install/setup.bash
ros2 run lidar_simulation reconstruction_node
```

Subscribes to `/scan` and `/encoder/state`, estimates the rotation angle for each LiDAR ray, and publishes the reconstructed point cloud on `/cloud`.

### Terminal 4 — RViz

```bash
source /opt/ros/humble/setup.bash
rviz2
```

In RViz:

1. Set **Fixed Frame** to the frame used by the published point cloud. If the cloud does not appear, inspect the `header.frame_id` of `/cloud`.
2. Select **Add → By topic**.
3. Select `/cloud` and add the `PointCloud2` display.
4. Adjust the view and point size if needed.

## Verify the Topics

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/3dlidar_ws/install/setup.bash
ros2 topic list
```

Expected topics:

```text
/scan
/encoder/state
/cloud
```

Check publication rates:

```bash
ros2 topic hz /scan
ros2 topic hz /encoder/state
ros2 topic hz /cloud
```

Inspect one point-cloud message:

```bash
ros2 topic echo /cloud --once
```

## How Reconstruction Works

The simulated LiDAR provides a range `r` and in-plane angle `theta`. The encoder provides the rotating plane's angle `phi`. The reconstruction node combines them to calculate 3D points.

For the current coordinate convention:

```text
x = r * cos(theta) * cos(phi)
y = r * cos(theta) * sin(phi)
z = z0 + r * sin(theta)
```

The node uses encoder history and interpolation to estimate the plane angle at each ray's acquisition time. This matters because the platform continues to rotate while a LiDAR scan is being acquired.

The exact angle convention and timestamp interpretation must be verified when switching from simulation to the real RPLIDAR driver and physical encoder.

## Troubleshooting

### `Package 'lidar_simulation' not found`

Build the workspace and source it in the same terminal:

```bash
cd ~/3dlidar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### `No executable found`

Check the installed executable names:

```bash
ros2 pkg executables lidar_simulation
```

The command names must match the package's installed console-script entry points. If a node cannot be found, inspect `setup.py` and the package configuration rather than guessing a different command.

### `/cloud` is missing

Confirm all three nodes are running, then check:

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /encoder/state
```

Check the reconstruction node's terminal for errors. It needs both input topics.

### The cloud is not visible in RViz

- Confirm that `/cloud` is publishing.
- Set RViz's Fixed Frame to the cloud's `header.frame_id`.
- Add the `/cloud` `PointCloud2` display.
- Inspect the cloud message:

```bash
ros2 topic echo /cloud --once
```

### ROS 2 commands cannot discover topics

Make sure all terminals use the same ROS 2 distribution and compatible environment. Source the Humble setup file and workspace setup file in each terminal.

## Current Limitations

- This is a simulation, not a validation of real hardware accuracy.
- Real LiDAR timestamp semantics and per-ray timing must be verified.
- ESP32-to-laptop clock alignment and serial latency must be handled during hardware integration.
- Mechanical alignment, bearing runout, encoder mounting, and servo motion have not yet been validated with the physical assembly.
- The 180° versus 360° sweep comparison remains a planned experiment; 360° is the default test mode.

## Next Steps

1. Finalize the mechanical CAD.
2. Build the MG996R, AS5600, and ESP32 assembly.
3. Implement timestamped encoder acquisition and servo control on the ESP32.
4. Add a ROS 2 serial bridge for `/encoder/state`.
5. Connect the physical RPLIDAR A1 and validate its timing.
6. Calibrate the scanner against known geometry.
7. Compare 180° and 360° sweeps under repeatable conditions.

## License

Add a license here if the project will be distributed publicly.
