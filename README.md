# Kinova ROS Control

ROS1 Cartesian control for a **7-DOF Kinova Gen3 with a Robotiq gripper**.

## Installation

Baseline: Ubuntu 20.04 / ROS Noetic / Linux x86-64 / Kortex SDK 2.5.0.
Docker provides this environment on a newer Linux host.

```bash
git clone https://github.com/intelligent-control-lab/kinova_ros_control.git
cd kinova_ros_control
```

### Option 1: Docker

Install Docker Engine and the Compose plugin on Linux.

**1. Configure the connection**

Edit [`.env`](.env) to set the robot IP/login, ROS IP and camera serial numbers.
The included serials are D405 wrist `123622270997` and D415 fixed `211422062035`;
replace them with your cameras' serials. For clients on another machine, use
reachable host addresses for `ROS_IP` and `ROS_MASTER_URI`.

**2. Build the image**

```bash
docker compose build
```

The image includes the controller, librealsense 2.51.1 and realsense-ros 2.3.2.

**3. Start the container**

```bash
docker compose up -d
```

The container waits for commands; ROS nodes are started manually below.

### Option 2: Build from source

Requires Ubuntu 20.04 / ROS Noetic and a system-wide
[librealsense 2.51.1](https://github.com/realsenseai/librealsense/releases/tag/v2.51.1) installation.
Run commands from the repository root.

**1. Install build tools**

```bash
source /opt/ros/noetic/setup.bash
sudo apt-get install -y build-essential cmake curl unzip git python3-catkin-tools python3-rosdep
```

**2. Download Kortex SDK 2.5.0**

```bash
mkdir -p third_party/kortex_api
curl -fL https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip \
  -o /tmp/kortex-2.5.0.zip
unzip -q /tmp/kortex-2.5.0.zip -d third_party/kortex_api
```

**3. Download the RealSense ROS wrapper**

```bash
git clone --depth 1 --branch 2.3.2 https://github.com/realsenseai/realsense-ros.git src/realsense-ros
sed -i '/RS405_PID/s/0x0b0c/0x0b5b/' src/realsense-ros/realsense2_camera/include/constants.h  # Fix D405 USB product ID to avoid "Unsupported device".
```

**4. Install ROS dependencies**

If rosdep has not been initialized, run `sudo rosdep init` once.

```bash
rosdep update --rosdistro noetic --include-eol-distros
rosdep install --from-paths src --ignore-src -y --skip-keys librealsense2
```

**5. Build the workspace**

```bash
catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

## Launch Controller

**Docker:**

Enter the running container:

```bash
docker exec -it kinova_ros_control bash
```

Start the controller inside the container:

```bash
roslaunch kinova_ros_control kinova_bringup.launch
```

For cameras, enter the same container from another terminal and run:

```bash
roslaunch kinova_ros_control two_realsense.launch
```

**From source:**

Set your robot IP and camera serials in the launch arguments below.

```bash
source install/setup.bash
roslaunch kinova_ros_control kinova_bringup.launch ip_address:=192.168.1.10
```

In another terminal, start the cameras:

```bash
source install/setup.bash
roslaunch kinova_ros_control two_realsense.launch \
  wrist_serial:=123622270997 fixed_serial:=211422062035
```

[`two_realsense.launch`](src/kinova_ros_control/launch/two_realsense.launch) disables
auto exposure and sets D405 `stereo_module/exposure=30000` and D415
`rgb_camera/exposure=180` (camera setting units). Adjust these values for your lighting.

The controller connects to the robot and initializes its target from current FK.
Launch arguments: `ip_address`, `port`, `username`, `password`, `config_file`.
PID, velocity limits and admittance settings are in
[`kinova_cartesian_control.yaml`](src/kinova_ros_control/config/kinova_cartesian_control.yaml).
Stop with Ctrl-C; for Docker, finish with `docker compose down`.

### Control law

The controller uses PD feedback to generate Cartesian velocity commands, with
admittance compensation of the position target. Integral gains are zero in the
default configuration; the general PID form is shown below.

For current FK pose $(p,R)$ and compensated target $(p_d,R_d)$, the base-frame
error and per-axis PID output are

$$
e_k=\begin{bmatrix}p_d-p\\ \mathrm{Log}(R_dR^T)^\vee\end{bmatrix},\qquad
u_k=K_pe_k+K_i\Delta t\sum_{j=0}^{k}e_j+K_d\frac{e_k-e_{k-1}}{\Delta t}.
$$

The first derivative is zero; $\Delta t\leq0.1\,s$. Angular outputs are converted
from rad/s to deg/s for Kortex, then each twist component is clipped to its YAML
velocity limit.

The main loop has no fixed rate. Twist updates require more than 1 ms elapsed
(nominal cap: 1 kHz); actual timing depends on SDK and network latency.
The terminal logs the average main-loop frequency once per second.

Admittance adds an XYZ offset $x_a$ to the position target, integrated as

$$
x_{a,k+1}=\mathrm{clip}\left(x_{a,k}+\frac{f_{\mathrm{eff}}-Kx_{a,k}}{D}\Delta t,\,-x_{\max},\,x_{\max}\right),
\quad f_{\mathrm{eff}}=\mathrm{sgn}(f)\max(|f|-f_{\mathrm{dead}},0).
$$

### ROS Topics

Controller topics use the `/kinova_ros_control/` prefix. All coordinates are
relative to the robot base frame.
Camera topics below use their full paths (640×480 RGB, 30 Hz, depth disabled).

| Topic | Message type | Meaning / units |
| --- | --- | --- |
| `cartesian_target` | `geometry_msgs/PoseStamped` | Target position (m), quaternion |
| `gripper_target` | `sensor_msgs/JointState` | `position[0]`: 0 open, 1 closed |
| `feedback_cartesian_fk` | `geometry_msgs/PoseStamped` | FK end-effector pose used for control |
| `feedback_cartesian` | `geometry_msgs/PoseStamped` | Raw Kortex tool pose |
| `feedback_joint_states` | `sensor_msgs/JointState` | Seven joint positions, velocities, torques: rad, rad/s, Nm |
| `feedback_external_wrench` | `geometry_msgs/WrenchStamped` | Force (N), torque (Nm) |
| `feedback_gripper_state` | `sensor_msgs/JointState` | Position: 0 open, 1 closed; raw gripper velocity from Kortex SDK |
| `admittance_compensated_pose` | `geometry_msgs/PoseStamped` | Target after admittance |
| `admittance_offset` | `geometry_msgs/Point` | XYZ offset (m); published when admittance is enabled |
| `/wrist_cam/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | D405 wrist image |
| `/fixed_cam/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | D415 fixed image |

`feedback_cartesian_fk` may differ from `feedback_cartesian`. FK is based on the
nominal kinematic model defined by the URDF; use `feedback_cartesian_fk` as the reference.

### Add Wiggling

Local XY wiggling is intended to facilitate tight fitting assembly by searching
a small neighborhood around the nominal insertion target.

When enabled and $|F_z|>10\,N$, a circular XY offset is added to the admittance
position target:

$$
\delta p(t)=0.001
\begin{bmatrix}
\cos(6\pi t) \\
\sin(6\pi t) \\
0
\end{bmatrix}
\mathrm{m}.
$$

Wiggling defaults to enabled (1 mm radius, 3 Hz). Toggle it with the
`std_srvs/SetBool` service; use `false` to disable and reset its phase:

```bash
rosservice call /kinova_ros_control/set_wiggling "data: true"
```

The radius, frequency and force threshold are defined in
[`CartesianFollowingTwist.cpp`](src/robot_controller/src/Process/CartesianFollowingTwist.cpp).

## File Structure

```text
kinova_ros_control/
├── src/
│   ├── kinova_ros_control/   # ROS bridge, config and launch files
│   └── robot_controller/    # Gen3 API, FK, PID, admittance and wiggling
├── docker/                  # Dockerfile and entrypoint
├── .env                     # Robot, ROS network and camera settings
├── compose.yaml
└── README.md
```

## Troubleshoot

- The `realsense-ros` package thros an error when connecting to D405:

```text
[FATAL] [1765985243.605096505]: Unsupported device! Product ID: 0x0B5B
```

**Solution:** Modify `realsense-ros/realsense2_camera/include/constants.h`

```cpp
RS405_PID = 0x0B5B
```
