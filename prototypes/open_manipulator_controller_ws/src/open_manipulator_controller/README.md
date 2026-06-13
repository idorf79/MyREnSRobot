# OpenManipulatorX ROS2 Controller (C++)

C++ ROS2 package for controlling the **ROBOTIS OpenManipulator-X** via
action servers.

---

## Package layout

```
open_manipulator_controller/
├── action/
│   ├── MoveToPosition.action
│   └── GripperControl.action
├── config/
│   └── controller_params.yaml
├── include/open_manipulator_controller/
│   ├── arm_controller.hpp        ← class declaration
│   └── predefined_positions.hpp  ← named joint poses (edit here!)
├── launch/
│   └── arm_controller.launch.py
├── src/
│   ├── arm_controller.cpp        ← action server implementation
│   ├── arm_controller_node.cpp   ← main() / executor
│   └── example_client.cpp        ← demo pick-and-place client
├── CMakeLists.txt
└── package.xml
```

---

## Prerequisites

| Requirement | Notes |
|---|---|
| ROS2 Humble or newer | Tested on Humble / Iron |
| `open_manipulator_msgs` | `sudo apt install ros-humble-open-manipulator-x` |
| OpenManipulator-X bringup | Must be running before the controller starts |

---

## Build

```bash
cd ~/ros2_ws/src
# Place or symlink this package here, then:
cd ~/ros2_ws
colcon build --packages-select open_manipulator_controller
source install/setup.bash
```

---

## Run

```bash
# Terminal 1 – robot bringup (real hardware)
ros2 launch open_manipulator_x open_manipulator_x.launch.py

# Terminal 2 – controller
ros2 launch open_manipulator_controller arm_controller.launch.py

# Terminal 3 – demo client (pick-and-place sequence)
ros2 run open_manipulator_controller example_client
```

---

## Action interfaces

### `/move_to_position` — `MoveToPosition`

**Goal**
| Field | Type | Description |
|---|---|---|
| `position_name` | `string` | Key from `predefined_positions.hpp` |
| `duration` | `float64` | Trajectory time in seconds (0 → use default 2.0 s) |

**Result**
| Field | Type | Description |
|---|---|---|
| `success` | `bool` | |
| `message` | `string` | |
| `final_joint_positions` | `float64[]` | Reached joint angles (rad) |

**Feedback** — published at `feedback_rate_hz`
| Field | Type | Description |
|---|---|---|
| `progress` | `float64` | 0.0 → 1.0 |
| `current_joint_positions` | `float64[]` | |
| `status` | `string` | Arm moving state string |

```bash
# CLI test
ros2 action send_goal /move_to_position \
  open_manipulator_controller/action/MoveToPosition \
  "{position_name: 'pick_front', duration: 2.5}"
```

---

### `/gripper_control` — `GripperControl`

**Goal**
| Field | Type | Description |
|---|---|---|
| `command` | `string` | `"open"` or `"close"` |
| `effort` | `float64` | 0.0–1.0 (reserved for future use) |

```bash
ros2 action send_goal /gripper_control \
  open_manipulator_controller/action/GripperControl \
  "{command: 'open', effort: 1.0}"
```

---

## Adding new positions

Open `include/open_manipulator_controller/predefined_positions.hpp` and add
an entry to the `positions` map. Joint values are in **radians**,
order: `[joint1, joint2, joint3, joint4]`.

```cpp
{"my_pose", {0.5, -0.45, 0.60, 0.10}},
```

Use `rqt_joint_trajectory_controller` or `ros2 topic echo /joint_states`
while positioning the arm with torque off to find your values.

---

## Parameters (`config/controller_params.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `feedback_rate_hz` | `10.0` | Feedback publish rate (Hz) |
| `goal_timeout_sec` | `30.0` | Extra timeout after trajectory duration |
