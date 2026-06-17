# SCARA_ws

ROS 2 Humble workspace for a custom 3-DOF SCARA robot with camera-driven pick-and-place.

A downward-mounted Intel RealSense + YOLO detects bottle caps on the table; MoveIt 2 plans the trajectory; an analytical IK fallback talks directly to ODrive (revolute joints) and MKS (prismatic end-effector) servos over SocketCAN. The physical arm consistently lands within ~1–2 mm of the target after calibration.

## Hardware

- **Arm**: 3-DOF SCARA — 2 revolute (shoulder + elbow) + 1 prismatic Z screw-down end-effector
- **Drives**: 2× ODrive (revolute) + 1× MKS SERVO (prismatic) — all on a single SocketCAN bus
- **Sensing**: Intel RealSense D435i, downward-mounted ~690 mm above `base_link`
- **Compute**: any ROS 2 Humble-capable Linux host with SocketCAN

## Software stack

ROS 2 Humble · MoveIt 2 · Gazebo (GZ Sim) · `ros2_control` · Ultralytics YOLO · `pyrealsense2` · `python3-can`

## Repository layout

```
src/
├── scara_robot/           ament_cmake — URDF/Xacro, meshes, RViz/Gazebo launch,
│                          C++ MoveIt motion demos (move, pick_and_place,
│                          cam_pick_and_place)
├── scara_moveit_config/   ament_cmake — MoveIt 2 config (SRDF, kinematics,
│                          controllers, demo.launch.py)
└── SCARA_pkg/             ament_python — ODrive/MKS CAN bridge + RealSense+YOLO
                           detector + Python IK/FK + pick-and-place state machine
```

For a full narrative of every package, launch file, the calibration story, CAN protocol details, and tuned defaults, see [`SCARA_project_summary.md`](./SCARA_project_summary.md).

## Quick start

### Build

```bash
git clone https://github.com/piranut-op/SCARA_ws.git
cd SCARA_ws
colcon build --symlink-install
source install/setup.bash
```

### Visualise in RViz (no hardware required)

```bash
ros2 launch scara_robot rviz.launch.py
```

### MoveIt demo with fake hardware

```bash
ros2 launch scara_moveit_config demo.launch.py
```

### Camera-driven pick-and-place (real hardware)

Bring up CAN first:

```bash
sudo ip link set can0 up type can bitrate 250000
```

Then launch the headline application — MoveIt planner + analytical IK + CAN motors + RealSense:

```bash
ros2 launch scara_robot cam_pick_and_place.launch.py use_hardware:=true
```

Place a bottle cap on the workspace; after a 5 s detection lock the arm picks it up and places it at the configured target pose.

## YOLO weights

`detect_bottle_cap` defaults to loading weights from `~/scara_bot_ws/src/best_v2.pt`. Override per-run:

```bash
ros2 run SCARA_pkg detect_bottle_cap --ros-args -p model_path:=/abs/path/best_v2.pt
```

The `best_v2.pt` shipped in this repo is a small custom-trained YOLO model for bottle-cap detection.

## Calibration

Camera mount pose is passed as launch arguments (`cam_x`, `cam_y`, `cam_z`, `cam_roll`, `cam_pitch`, `cam_yaw`) and published as a static TF from `base_link` → `camera_color_optical_frame`. Defaults assume a straight-down-mounted camera. Full calibration walkthrough — including the CAD-vs-physical discrepancy on `ee_joint` and the 3-step verification checklist — is in [`SCARA_project_summary.md`](./SCARA_project_summary.md#8-calibration-procedure-and-findings).

## License

Apache-2.0 — see [`LICENSE`](./LICENSE).
