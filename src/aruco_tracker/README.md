# ArUco Landing Pad Detector – ROS 2 Humble

<img src="board.png" alt="ArUco Board" width="300"/>


`landing_detector_node` detects an ArUco board that encodes your landing pad, publishes its pose in **ENU/map** coordinates, and provides a TF from `camera_link` to `landing_pad`.  It is designed for UAV precision‑landing stacks using **PX4 + MAVROS**.

---

## Features

| ✔ | Functionality |
|---|---------------|
| ✅ | ArUco marker & board detection with **OpenCV 4** |
| ✅ | Pose estimation with `cv::aruco::estimatePoseBoard` |
| ✅ | Publishes `geometry_msgs/PoseStamped` → `landing_pad/pose` |
| ✅ | Broadcasts TF: `camera_link → landing_pad` (+ intermediate board frame) |
| ✅ | Loads board geometry from **YAML / JSON** config file |
| ✅ | All topics/frames/configurable via ROS 2 parameters |

---

## Quick Start

```bash
# buil depends
sudo apt install nlohmann-json3-dev

# 1. Build the package (inside your ROS 2 Humble workspace)
cd ~/ros2_ws
colcon build --symlink-install --packages-select aruco_tracker
source install/setup.bash

# 2. Launch the node (camera topics must already be published)
ros2 launch aruco_tracker landing_detector.launch.py



#### Run the simulation environment
Launch PX4 sim
```
make px4_sitl gz_x500_mono_cam_down_aruco
```
Launch micro dds
```
MicroXRCEAgent udp4 -p 8888
```

Launch the ros_gz_bridge for getting the camera topic
```
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

Launch the ros_gz_bridge for getting the camera info topic (this is how we get camera intrinsics)
```
ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

Launch the ros2 nodes (aruco_tracker)
```
cd tracktor-beam/
source install/setup.bash 
ros2 run aruco_tracker aruco_tracker 
```

View the video (/image_proc is the annoted image)
```
ros2 run rqt_image_view rqt_image_view
```
