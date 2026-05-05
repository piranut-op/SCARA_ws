# Camera-Driven Pick-and-Place

One-launch pipeline: RealSense + YOLO bottle-cap detector → SCARA IK → ODrive (joints 1–2) + MKS (prismatic EE), gated by a stability state machine.

## Hardware checklist

- [ ] Intel RealSense (D4xx) plugged in via USB 3
- [ ] ODrive powered, both motors enabled, CAN wired
- [ ] MKS servo powered, CAN wired
- [ ] CAN termination correct, single bus at 250 kbit/s
- [ ] SCARA workspace clear, kill switch in hand
- [ ] `best_v2.pt` present at `~/scara_bot_ws/src/best_v2.pt`

## Calibration (do once, redo if anything moves)

Measure these in metres in the SCARA base frame (origin at joint-1 axis, +y forward):

| Variable | Meaning |
|---|---|
| `base_to_workspace_x_m` | Lateral offset from base to workspace centre |
| `base_to_workspace_y_m` | Forward offset from base to workspace centre |
| `place_x_m`, `place_y_m` | Where to drop the cap |
| `home_x_m`, `home_y_m`   | Park pose between cycles |

Workspace must satisfy `ikpos` limits: `x ∈ [-0.159, 0.159]`, `y ∈ [0.10, 0.39]`.

EE travel limits (from `mks_ee`): `[-0.045, +0.065]` m. Pick height is normally negative (down), travel/lift is positive (up).

## Bring up

```bash
# every boot
sudo ip link set can0 up type can bitrate 250000

cd ~/scara_bot_ws
colcon build --packages-select SCARA_pkg
source install/setup.bash
```

## Test sequence

### Stage 1 — vision only (no motors)

```bash
ros2 launch SCARA_pkg cam_pick_and_place.launch.py with_hardware:=false
```

In another shell, place a cap on the workspace and verify:

```bash
ros2 topic echo /bottle_cap/workspace_position    # detector output
ros2 topic echo /ik_target                        # state-machine commands
ros2 topic echo /mks/ee_cmd                       # EE commands
```

A cap at the workspace centre should give `robot.x_cm ≈ 0`, `robot.y_cm ≈ 0`. The state machine should reach APPROACH after `stable_frames` (default 8) frames.

### Stage 2 — full pipeline, single cap

Hand on the kill switch.

```bash
ros2 launch SCARA_pkg cam_pick_and_place.launch.py \
    base_to_workspace_x_m:=<your_x> \
    base_to_workspace_y_m:=<your_y> \
    place_x_m:=<drop_x> \
    place_y_m:=<drop_y>
```

Place exactly one cap, watch one full cycle (APPROACH → DESCEND → DWELL → LIFT → TO_PLACE → DESCEND → DWELL → LIFT → HOME → COOLDOWN → IDLE), and confirm each phase looks right.

## Tuning knobs (all are launch arguments)

| Arg | Default | Notes |
|---|---|---|
| `class_filter` | `bottle_cap` | YOLO class to act on |
| `min_confidence` | `0.50` | Reject low-confidence detections |
| `stable_frames` | `8` | Frames within deadband before triggering |
| `stable_deadband_m` | `0.01` | Stability radius (m) |
| `ee_travel_m` | `0.060` | Safe travel height |
| `ee_pick_m` | `-0.040` | Touch-the-cap height |
| `ee_place_m` | `-0.040` | Drop height |
| `approach_s` | `2.0` | Wait for arm to reach pick xy |
| `descend_s` | `1.5` | Wait for EE to descend |
| `dwell_pick_s` | `0.8` | Hold while pick engages |
| `lift_s` | `1.5` | Wait for EE to retract |
| `to_place_s` | `2.5` | Wait for arm to traverse to drop xy |
| `dwell_place_s` | `0.8` | Hold while pick releases |
| `home_s` | `2.0` | Wait for arm to return home |
| `cooldown_s` | `2.0` | Ignore detections after a cycle |

The timings are open-loop — no motion feedback. Tune them against your real arm speed.

## What this does NOT do

- **No software gripper.** The MKS prismatic descend/lift is the pick mechanism. If you have a discrete gripper, set `gripper_topic:=/your_gripper` and a `Bool` will fire at pick/release moments.
- **No retry / abort logic.** A bad cycle just continues. Use the kill switch.
- **No camera in launch can be toggled off.** If the RealSense isn't connected, the detector dies and the launch tears down. Plug the camera in or run the state machine standalone with fake detections (see below).

## Standalone state-machine test (no camera, no motors)

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

The state machine will run a full cycle publishing to `/ik_target` and `/mks/ee_cmd` with nothing physical attached.

## Topics summary

| Topic | Type | Direction | Owner |
|---|---|---|---|
| `/bottle_cap/workspace_position` | `std_msgs/String` (JSON) | detector → state machine | `detect_bottle_cap` |
| `/bottle_cap/image` | `sensor_msgs/Image` | detector → user | `detect_bottle_cap` |
| `/ik_target` | `std_msgs/Float32MultiArray` `[x_m, y_m]` | state machine → IK | `pick_and_place_cam` |
| `/odrive/angle_cmd` | `std_msgs/Float32MultiArray` `[1, θ1, 2, θ2]` | IK → ODrive | `ikpos` |
| `/mks/ee_cmd` | `std_msgs/Float32` (m) | state machine → MKS | `pick_and_place_cam` |

## Files

- `SCARA_pkg/pick_and_place_cam.py` — state machine
- `SCARA_pkg/cap_to_target.py` — continuous-servo bridge (alternative to state machine)
- `launch/cam_pick_and_place.launch.py` — full pick-and-place stack
- `launch/perception_to_ik.launch.py` — continuous-servo stack (no state machine)
