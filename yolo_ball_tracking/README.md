# Yolo Ball Tracking
Tracking a soccer ball using [yolov8](https://github.com/mgonzs13/yolov8_ros) with ROBOTIS_OP3 on webots.

## How to Use

### 1. Build
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:team-re-boot/op3_webots.git
git clone git@github.com:team-re-boot/op3_webots_demo.git
git clone git@github.com:mgonzs13/yolov8_ros.git
cd ../
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

### 2. Run
Launch webots simulation.
```bash
ros2 launch op3_webots webots_world.launch.py
```
Launch Ball Tracking Node.
```bash
ros2 launch yolo_ball_tracking yolo_ball_tracking.launch.xml
```
[![](https://img.youtube.com/vi/mLUrapHbc-E/0.jpg)](https://www.youtube.com/watch?v=mLUrapHbc-E)
