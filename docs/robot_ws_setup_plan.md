# Plan: `/home/unitree/Projects/ros2_ws` — sensor + detection workspace

Target machine: **real G1 robot** (`unitree@ubuntu`)  
Goal: one workspace that installs `livox_ros_driver2`, publishes LiDAR + RealSense topics (including depth), and runs the CenterPoint detection node — all recordable via `record_g1_bag.sh`.

---

## 0. Prerequisites (verify first)

```bash
# ROS2 distro on robot
echo $ROS_DISTRO          # expect: jazzy (or humble)
ros2 --version

# CUDA / torch availability (detection needs it)
nvidia-smi
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"

# RealSense driver installed?
ros2 pkg list | grep realsense
# if blank → install it (Step 3)

# livox_ros_driver2 available?
ros2 pkg list | grep livox
# if blank → build from source (Step 2)
# NOTE: g1pilot's livox_launcher.launch.py expects it at /ros2_ws/src/livox_ros_driver2
#       We will put it at /home/unitree/Projects/ros2_ws/src/livox_ros_driver2
```

---

## 1. Create workspace

```bash
mkdir -p ~/Projects/ros2_ws/src
cd ~/Projects/ros2_ws
```

---

## 2. Install `livox_ros_driver2`

```bash
cd ~/Projects/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git --depth 1

# Livox SDK2 (C++ library, required by driver)
git clone https://github.com/Livox-SDK/Livox-SDK2.git --depth 1
cd Livox-SDK2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
cd ~/Projects/ros2_ws
```

Fix the `MID360_config.json` **IP address** to match your robot's Mid-360:

```bash
# Default Mid-360 IP is 192.168.1.1xx — check with:
ip addr show   # find interface connected to LiDAR
# Edit:
nano ~/Projects/ros2_ws/src/livox_ros_driver2/config/MID360_config.json
# Set "host_net_info" → "cmd_data_ip" and "push_msg_ip" to the robot's LAN IP
# Set "lidar_configs" → "ip" to the Mid-360's IP
```

---

## 3. Install `realsense2_camera` (if not present)

```bash
# Option A — apt (easiest, jazzy)
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-realsense2-description

# Option B — if apt version is too old (check: apt show ros-jazzy-realsense2-camera)
cd ~/Projects/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git --branch ros2-master --depth 1
# also needs librealsense2 dev:
sudo apt install librealsense2-dev librealsense2-utils
```

---

## 4. Copy detection node from thesis repo

```bash
mkdir -p ~/Projects/ros2_ws/src/g1_detection
cp -r /path/to/thesis/G1_sim/detection/* ~/Projects/ros2_ws/src/g1_detection/
# Replace /path/to/thesis with actual path on robot

# Install Python deps for detection
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu124
# (adjust cu version to match nvidia-smi output)
pip3 install open3d numpy
```

---

## 5. Build workspace

```bash
cd ~/Projects/ros2_ws
source /opt/ros/jazzy/setup.bash        # or humble

# Build only the ROS packages (detection_node is run directly, not as colcon pkg)
colcon build --symlink-install \
    --packages-select livox_ros_driver2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# If realsense2_camera was cloned (Option B above):
colcon build --symlink-install \
    --packages-select realsense2_camera realsense2_description \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If build fails on livox: `sudo ldconfig` then retry (finds the Livox-SDK2 .so).

---

## 6. Write the unified sensor launch file

Create `~/Projects/ros2_ws/src/g1_sensors.launch.py`:

```python
"""
Launch: Mid-360 LiDAR + D435 RealSense (RGB + depth) on real G1.

source /opt/ros/jazzy/setup.bash
source ~/Projects/ros2_ws/install/setup.bash
ros2 launch ~/Projects/ros2_ws/src/g1_sensors.launch.py

Published topics:
  /livox/lidar              livox_ros_driver2/CustomMsg
  /utlidar/cloud            sensor_msgs/PointCloud2   (remapped from /livox/lidar_0)
  /g1/camera/color/image_raw         sensor_msgs/Image
  /g1/camera/color/camera_info       sensor_msgs/CameraInfo
  /g1/camera/depth/image_rect_raw    sensor_msgs/Image
  /g1/camera/depth/camera_info       sensor_msgs/CameraInfo
  /g1/camera/aligned_depth_to_color/image_raw  sensor_msgs/Image  (depth aligned to RGB)
"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

WS = Path.home() / "Projects/ros2_ws"
LIDAR_CONFIG = str(WS / "src/livox_ros_driver2/config/MID360_config.json")

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("lidar",  default_value="true"),
        DeclareLaunchArgument("camera", default_value="true"),

        # ── Mid-360 ──────────────────────────────────────────────────────────
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                "xfer_format":   1,          # 1 = PointCloud2
                "multi_topic":   0,
                "data_src":      0,
                "publish_freq":  10.0,
                "output_data_type": 0,
                "frame_id":      "livox_frame",
                "user_config_path": LIDAR_CONFIG,
            }],
            remappings=[
                ("/livox/lidar_0",  "/livox/lidar"),
                ("/livox/points_0", "/utlidar/cloud"),
            ],
            condition=IfCondition(LaunchConfiguration("lidar")),
        ),

        # ── D435 RealSense ────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("realsense2_camera"),
                             "launch", "rs_launch.py")
            ),
            launch_arguments={
                "camera_name":              "camera",
                "camera_namespace":         "g1",
                "enable_color":             "true",
                "color_width":              "640",
                "color_height":             "480",
                "color_fps":                "30",
                "enable_depth":             "true",
                "depth_width":              "640",
                "depth_height":             "480",
                "depth_fps":                "30",
                "align_depth.enable":       "true",   # depth_to_color aligned topic
                "pointcloud.enable":        "false",
                "enable_gyro":              "false",
                "enable_accel":             "false",
                "use_sim_time":             "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("camera")),
        ),
    ])
```

---

## 7. Run detection node (terminal 3)

```bash
source /opt/ros/jazzy/setup.bash
source ~/Projects/ros2_ws/install/setup.bash
cd ~/Projects/ros2_ws/src/g1_detection

python3 detection_node.py \
    --checkpoint pt/livox_model_1.pt \
    --input /utlidar/cloud
    # add --backend clustering if no GPU / no checkpoint
```

Detection publishes:
- `/g1/detections`        — `vision_msgs/Detection3DArray`
- `/g1/detection_markers` — `visualization_msgs/MarkerArray`

---

## 8. Record bag (terminal 4)

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag record \
    --output ~/bags/g1_$(date +%Y%m%d_%H%M%S) \
    --storage mcap \
    --compression-mode file \
    --compression-format zstd \
    /utlidar/cloud \
    /livox/lidar \
    /g1/camera/color/image_raw \
    /g1/camera/color/camera_info \
    /g1/camera/depth/image_rect_raw \
    /g1/camera/depth/camera_info \
    /g1/camera/aligned_depth_to_color/image_raw \
    /g1/detections \
    /g1/detection_markers \
    /tf \
    /tf_static
```

---

## 9. Verify topics are live

```bash
ros2 topic list | grep -E "livox|utlidar|g1"
ros2 topic hz /utlidar/cloud          # expect ~10 Hz
ros2 topic hz /g1/camera/color/image_raw   # expect ~30 Hz
ros2 topic hz /g1/camera/depth/image_rect_raw  # expect ~30 Hz
```

---

## Known gotchas

| Issue | Fix |
|---|---|
| `livox_ros_driver2_node` exits immediately | MID360_config.json IP wrong — set host IP to robot's LAN IP, lidar IP to 192.168.1.1xx |
| `/livox/lidar_0` not remapped | driver version uses different topic name — check with `ros2 topic list` then adjust remapping |
| RealSense "No device connected" | `rs-enumerate-devices` to confirm USB; `sudo udevadm control --reload-rules` if new install |
| depth topic missing | `align_depth.enable` requires D400-series firmware ≥ 5.12; run `rs-fw-update` if old |
| detection node: `ModuleNotFoundError: backend` | run from `g1_detection/` dir or add it to `PYTHONPATH` |
| `colcon build` fails: `livox_sdk2` not found | `sudo ldconfig` after installing Livox-SDK2 |
