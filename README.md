# BLDC Motor Simulation for Gazebo Harmonic + ROS 2 Jazzy

This package contains a **BLDC motor simulation plugin** for Gazebo Harmonic, integrated with ROS 2 Jazzy.  
It simulates a BLDC motor’s dynamics and publishes its state directly to ROS 2 (`geometry_msgs/Vector3`) without requiring `ros_gz_bridge` for the state topic.  
Optionally, the motor can also receive its control commands directly from ROS 2.

---

## Quickstart

```bash
# 1 — Create workspace & clone
mkdir -p ~/bldc_ws/src
cd ~/bldc_ws/src
git clone <this_repo_url> bldc_gz_sim

# 2 — Build
cd ~/bldc_ws
colcon build --symlink-install
source install/setup.bash

# 3 — Launch simulation
ros2 launch bldc_gz_sim bldc_world.launch.py

# 4 — Send a voltage command
ros2 topic pub /bldc_motor/voltage std_msgs/msg/Float64 "{data: 5.0}"

# 5 — View state
ros2 topic echo /bldc_motor/state
```

---

## Features
- **BLDC dynamics model**:
  - Voltage → current → torque (linear approximation).
  - Configurable motor parameters (`R`, `Ke`, `Kt`).
- **Direct ROS 2 publishing** of motor state:
  - `geometry_msgs/Vector3` →  
    - `x` = omega (rad/s)  
    - `y` = current (A)  
    - `z` = torque (N·m)
- **Optional ROS 2 subscription** for voltage commands (`std_msgs/Float64`).
- **Rate-limited publishing and control loops**:
  - `<publish_rate_hz>` sets ROS 2 state publish frequency.
  - `<control_rate_hz>` sets control loop frequency (torque updates).
- **Gazebo Transport compatibility** for voltage commands if ROS control is disabled.

---

## Prerequisites
- **Ubuntu 24.04**
- **ROS 2 Jazzy**
- **Gazebo Harmonic**
- Build tools and dependencies:
  ```bash
  sudo apt update
  sudo apt install -y \
    build-essential cmake git python3-colcon-common-extensions \
    ros-jazzy-desktop ros-jazzy-rqt ros-jazzy-rqt-plot \
    libgz-sim8-dev libgz-math7-dev libgz-common5-dev
  ```

---

## Installation

**Clone into your ROS 2 workspace:**
```bash
mkdir -p ~/bldc_ws/src
cd ~/bldc_ws/src
git clone <this_repo_url> bldc_gz_sim
```

**Build:**
```bash
cd ~/bldc_ws
colcon build --symlink-install
source install/setup.bash
```

---

## SDF Plugin Parameters

**Example `<plugin>` tag inside your motor’s SDF:**
```xml
<plugin filename="bldc_motor_plugin" name="BLDCMotorPlugin">
  <joint_name>rotor_joint</joint_name>
  <resistance>0.5</resistance>
  <ke>0.02</ke>
  <kt>0.02</kt>

  <!-- Publish state to ROS 2 at 1 Hz -->
  <publish_rate_hz>1.0</publish_rate_hz>

  <!-- Control loop frequency (0.0 = every simulation step) -->
  <control_rate_hz>0.0</control_rate_hz>

  <!-- true = subscribe to ROS 2 voltage, false = use GZ transport -->
  <use_ros_voltage_sub>true</use_ros_voltage_sub>
</plugin>
```

**Parameters:**

| Name                  | Type   | Default      | Description |
|-----------------------|--------|--------------|-------------|
| `joint_name`          | string | rotor_joint  | The revolute joint name for rotor rotation. |
| `resistance`          | double | 0.5          | Motor phase resistance (Ω). |
| `ke`                  | double | 0.02         | Back-EMF constant (V·s/rad). |
| `kt`                  | double | 0.02         | Torque constant (N·m/A). |
| `publish_rate_hz`     | double | 50.0         | ROS 2 state publish frequency (Hz). |
| `control_rate_hz`     | double | 0.0          | Control loop frequency (Hz), 0.0 = every step. |
| `use_ros_voltage_sub` | bool   | false        | If true, subscribes to `/bldc_motor/voltage` in ROS 2. Otherwise uses Gazebo topic. |

---

## Topics

**When `<use_ros_voltage_sub>true</use_ros_voltage_sub>`:**

**ROS 2 subscription:**
```
/bldc_motor/voltage    (std_msgs/msg/Float64)
```

**ROS 2 publication:**
```
/bldc_motor/state      (geometry_msgs/msg/Vector3)
  x = omega (rad/s)
  y = current (A)
  z = torque (N·m)
```

---

**When `<use_ros_voltage_sub>false</use_ros_voltage_sub>`:**

**Gazebo Transport subscription:**
```
/bldc_motor/voltage    (gz.msgs.Double)
```

ROS 2 publication remains the same.

---

## Usage

**Launch simulation:**
```bash
ros2 launch bldc_gz_sim bldc_world.launch.py
```

**Send a voltage command (ROS 2 mode):**
```bash
ros2 topic pub /bldc_motor/voltage std_msgs/msg/Float64 "{data: 5.0}"
```

**Send a voltage command (Gazebo mode):**
```bash
gz topic pub /bldc_motor/voltage gz.msgs.Double "data: 5.0"
```

**View state in terminal (1 Hz):**
```bash
ros2 topic echo /bldc_motor/state --rate 1
```

**View state in `rqt_plot`:**
- `/bldc_motor/state/x` → omega (rad/s)  
- `/bldc_motor/state/y` → current (A)  
- `/bldc_motor/state/z` → torque (N·m)  

---

**Example Output (`ros2 topic echo /bldc_motor/state`):**
```
x: 314.159
y: 9.870
z: 0.197
```
