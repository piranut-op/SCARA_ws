# Camera-Driven Pick-and-Place

Two parallel implementations:

| Variant | IK solver | Trajectory engine | Hardware path | Launch |
|---|---|---|---|---|
| **A. MoveIt** *(recommended)* | MoveIt KDL (built-in) | `MoveGroupInterface` + `JointTrajectoryController` | MoveIt's controller stack (fake or real) | `ros2 launch scara_robot cam_pick_and_place.launch.py` |
| B. Analytical IK | `ikpos.py` closed-form | Python state machine + raw motor topics | Direct CAN via `newposition` + `mks_ee` | `ros2 launch SCARA_pkg cam_pick_and_place.launch.py` |

Both variants share the same vision front-end (`detect_bottle_cap` → `/bottle_cap/workspace_position`) and the same calibration concept (workspace-centre offset in the SCARA base frame). They differ in how the (x, y) target gets turned into joint motion.

---

## Variant A — MoveIt (default)

### What runs

```
detect_bottle_cap            RealSense + YOLO -> /bottle_cap/workspace_position
       v
cam_pick_and_place           stability gate + setPoseTarget on "arm" group
       v
MoveIt move_group + KDL      plans + executes via JointTrajectoryController
       v
ros2_control fake hardware   (default; replace with real controllers when ready)
```

### Hardware checklist

- [ ] Intel RealSense (D4xx) plugged in via USB 3
- [ ] `best_v2.pt` present at `~/scara_bot_ws/src/best_v2.pt`
- [ ] (optional) Real `ros2_control` setup wired through `scara_moveit_config` if not using fake hardware

### Calibration

Measure these in metres in the SCARA base frame (origin at joint-1 axis, +y forward):

| Variable | Meaning |
|---|---|
| `base_to_workspace_x_m` | Lateral offset from base to workspace centre |
| `base_to_workspace_y_m` | Forward offset from base to workspace centre |
| `place_x_m`, `place_y_m` | Where to drop the cap |
| `z_travel_m`, `z_pick_m` | Cartesian z of `Link_ee` in `base_link` frame for travel and engaged states |

`z_travel_m` / `z_pick_m` are **Cartesian** values, not joint values. To find them: launch MoveIt RViz, use the joint-state slider GUI to drag `ee_joint` between its limits, and read the resulting `Link_ee` z in the TF tree at "fully retracted" (use as `z_travel_m`) and "fully extended" (use as `z_pick_m`).

The pick xy must land inside whatever workspace MoveIt's KDL solver can satisfy at `z_pick_m`; if a plan fails, log will print `[<phase>] plan failed.` and the cycle aborts to cooldown.

### Bring up

```bash
cd ~/scara_bot_ws
colcon build --packages-select scara_robot SCARA_pkg
source install/setup.bash

ros2 launch scara_robot cam_pick_and_place.launch.py \
    base_to_workspace_x_m:=<your_x> \
    base_to_workspace_y_m:=<your_y> \
    z_travel_m:=<your_z_up> \
    z_pick_m:=<your_z_down> \
    place_x_m:=<drop_x> \
    place_y_m:=<drop_y>
```

This includes `scara_moveit_config/demo.launch.py` (move_group + RViz + fake controllers), the detector, and the C++ pick-and-place node. RViz shows the planned trajectory each cycle and `/scara_cam_pick_place_markers` cubes at engaged poses.

### Cycle

`approach_pick (xy_cap, z_travel)` → `engage_pick (xy_cap, z_pick)` → dwell → `retreat_pick` → `approach_place` → `engage_place` → dwell → `retreat_place` → `home_return` (joint zeros) → cooldown → idle.

Every step is a real MoveIt plan + execute, so timing is bounded by trajectory completion, not fixed sleeps.

### Tuning knobs (variant A launch arguments)

| Arg | Default | Notes |
|---|---|---|
| `class_filter` | `bottle_cap` | YOLO class to act on |
| `min_confidence` | `0.50` | Reject low-confidence detections |
| `stable_frames` | `8` | Frames within deadband before triggering |
| `stable_deadband_m` | `0.01` | Stability radius (m) |
| `z_travel_m` | `0.10` | Cartesian z for travel (above the cap) |
| `z_pick_m` | `0.04` | Cartesian z for engaging the cap |
| `place_x_m` / `place_y_m` | `0.10` / `0.25` | Drop location |
| `dwell_engage_s` | `1.0` | Hold time at engaged poses |
| `cooldown_s` | `2.0` | Ignore detections after a cycle |

### What variant A does NOT do

- **No software gripper.** The prismatic `ee_joint` carries the cap (vacuum / magnet / passive). If you have a discrete gripper trigger, you'll need to wire it in — this variant currently has no hook for that.
- **No retry / abort logic.** A failed plan logs and falls through to cooldown.
- **No `with_hardware` flag.** Defaults to MoveIt's fake-hardware demo; for real hardware, replace `scara_moveit_config/demo.launch.py` with your real `ros2_control` bring-up that exposes the same `arm_controller` / `JointTrajectoryController`.

---

## Variant B — Analytical IK + Python state machine

### What runs

```
detect_bottle_cap            RealSense + YOLO -> /bottle_cap/workspace_position
       v
pick_and_place_cam (Python)  stability gate -> /ik_target  +  /mks/ee_cmd
       v                                v
ikpos                                   mks_ee CAN driver
       v
newposition  ODrive CAN driver
```

### Hardware checklist

- [ ] Intel RealSense (D4xx) plugged in via USB 3
- [ ] ODrive powered, both motors enabled, CAN wired
- [ ] MKS servo powered, CAN wired
- [ ] CAN termination correct, single bus at 250 kbit/s
- [ ] SCARA workspace clear, kill switch in hand
- [ ] `best_v2.pt` present at `~/scara_bot_ws/src/best_v2.pt`

### Calibration

| Variable | Meaning |
|---|---|
| `base_to_workspace_x_m` | Lateral offset from base to workspace centre |
| `base_to_workspace_y_m` | Forward offset from base to workspace centre |
| `place_x_m`, `place_y_m` | Where to drop the cap |
| `home_x_m`, `home_y_m` | Park pose between cycles |

Workspace must satisfy `ikpos` limits: `x ∈ [-0.159, 0.159]`, `y ∈ [0.10, 0.39]`.
EE travel limits (from `mks_ee`): `[-0.045, +0.065]` m. Pick height is normally negative (down), travel/lift is positive (up).

### Bring up

```bash
sudo ip link set can0 up type can bitrate 250000

cd ~/scara_bot_ws
colcon build --packages-select SCARA_pkg
source install/setup.bash
```

#### Stage 1 — vision only (no motors)

```bash
ros2 launch SCARA_pkg cam_pick_and_place.launch.py with_hardware:=false
```

Verify with:

```bash
ros2 topic echo /bottle_cap/workspace_position
ros2 topic echo /ik_target
ros2 topic echo /mks/ee_cmd
```

A cap at the workspace centre should give `robot.x_cm ≈ 0`, `robot.y_cm ≈ 0`. The state machine should reach APPROACH after `stable_frames` (default 8) frames.

#### Stage 2 — full pipeline, single cap

Hand on the kill switch.

```bash
ros2 launch SCARA_pkg cam_pick_and_place.launch.py \
    base_to_workspace_x_m:=<your_x> \
    base_to_workspace_y_m:=<your_y> \
    place_x_m:=<drop_x> \
    place_y_m:=<drop_y>
```

### Tuning knobs (variant B launch arguments)

| Arg | Default | Notes |
|---|---|---|
| `class_filter` | `bottle_cap` | YOLO class to act on |
| `min_confidence` | `0.50` | Reject low-confidence detections |
| `stable_frames` | `8` | Frames within deadband before triggering |
| `stable_deadband_m` | `0.01` | Stability radius (m) |
| `ee_travel_m` | `0.060` | Safe travel height (joint-space, m) |
| `ee_pick_m` | `-0.040` | Touch-the-cap height (joint-space, m) |
| `ee_place_m` | `-0.040` | Drop height (joint-space, m) |
| `approach_s` | `2.0` | Wait for arm to reach pick xy |
| `descend_s` | `1.5` | Wait for EE to descend |
| `dwell_pick_s` | `0.8` | Hold while pick engages |
| `lift_s` | `1.5` | Wait for EE to retract |
| `to_place_s` | `2.5` | Wait for arm to traverse to drop xy |
| `dwell_place_s` | `0.8` | Hold while pick releases |
| `home_s` | `2.0` | Wait for arm to return home |
| `cooldown_s` | `2.0` | Ignore detections after a cycle |

The timings are **open-loop** — no motion feedback. Tune them against your real arm speed.

### Standalone state-machine test (no camera, no motors)

```bash
ros2 run SCARA_pkg ikpos
ros2 run SCARA_pkg pick_and_place_cam --ros-args -p base_to_workspace_y_m:=0.20

# in another shell — fake 10 stable frames at workspace centre
for i in $(seq 1 10); do
  ros2 topic pub --once /bottle_cap/workspace_position std_msgs/String \
    '{data: "[{\"class\":\"bottle_cap\",\"confidence\":0.9,\"center_px\":[320,240],\"depth_m\":0.5,\"cam_3d_m\":{\"X\":0,\"Y\":0,\"Z\":0.5},\"workspace\":{\"x_cm\":8.0,\"y_cm\":9.0},\"robot\":{\"x_cm\":0.0,\"y_cm\":0.0}}]"}'
  sleep 0.1
done
```

### What variant B does NOT do

- **No software gripper.** Same as variant A.
- **No retry / abort logic.** Same.
- **No camera in launch can be toggled off.** If the RealSense isn't connected, the detector dies and the launch tears down.

---

## Topics summary

| Topic | Type | Direction | Used by |
|---|---|---|---|
| `/bottle_cap/workspace_position` | `std_msgs/String` (JSON) | detector → both variants | `detect_bottle_cap` |
| `/bottle_cap/image` | `sensor_msgs/Image` | detector → user | `detect_bottle_cap` |
| `/scara_cam_pick_place_markers` | `visualization_msgs/MarkerArray` | variant A → RViz | `cam_pick_and_place` (C++) |
| `/ik_target` | `std_msgs/Float32MultiArray` `[x_m, y_m]` | variant B state machine → IK | `pick_and_place_cam` (Python) |
| `/odrive/angle_cmd` | `std_msgs/Float32MultiArray` `[1, θ1, 2, θ2]` | variant B IK → ODrive | `ikpos` |
| `/mks/ee_cmd` | `std_msgs/Float32` (m) | variant B state machine → MKS | `pick_and_place_cam` |
| MoveIt `JointTrajectoryController` action | `control_msgs/FollowJointTrajectory` | variant A move_group → controller | MoveIt internal |

## Files

### Variant A (MoveIt)
- `scara_robot/src/cam_pick_and_place.cpp` — C++ MoveIt state machine
- `scara_robot/launch/cam_pick_and_place.launch.py` — full stack including MoveIt demo

### Variant B (analytical)
- `SCARA_pkg/SCARA_pkg/pick_and_place_cam.py` — Python state machine
- `SCARA_pkg/SCARA_pkg/cap_to_target.py` — continuous-servo bridge (alternative to state machine)
- `SCARA_pkg/launch/cam_pick_and_place.launch.py` — full pick-and-place stack
- `SCARA_pkg/launch/perception_to_ik.launch.py` — continuous-servo stack (no state machine)

### Shared
- `SCARA_pkg/SCARA_pkg/detect_bottle_cap.py` — RealSense + YOLO detector + workspace localizer
- `src/best_v2.pt` — YOLO weights (loaded by the detector)
