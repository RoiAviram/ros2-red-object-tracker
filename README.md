# ROS2 Red Object Tracker 🟥🐢

This ROS2 package uses image processing to detect a red object in real-time from a webcam feed, and control a Turtlesim robot accordingly.

## 📦 Package Name

`ros2_opencv`

## 🧠 What It Does

- 📷 Captures webcam video stream.
- 🧠 Detects red-colored objects using OpenCV (HSV masking + contour analysis).
- 🐢 Sends control commands to the Turtlesim robot:
  - **Forward** if red object is centered
  - **Turn left/right** if object is left/right in frame
  - **Stop** if object not detected

## 🗂️ Package Structure

ros2_opencv/
├── ros2_opencv/
│ ├── image_detection.py # Image processing & detection
│ ├── robot_control.py # Subscribes to detection & sends turtle commands
│ ├── cameraPublisher.py # (Optional) Test webcam feed publisher
│ ├── subscriberImage.py # (Optional) Test image subscriber node
│ └── init.py
├── package.xml
├── setup.py
├── setup.cfg
└── resource/
└── ros2_opencv


## 🔧 Dependencies

Make sure you have these installed in your ROS 2 environment:

- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `OpenCV`
- `cv_bridge`
- `turtlesim`

## 🚀 How to Run

**1. Build the package:**

```bash
cd ~/ws_ros2_opencv
colcon build
source install/setup.bash

**2. Launch the nodes manually:**

**In separate terminals:**

# Terminal 1 - Turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2 - Robot control node
ros2 run ros2_opencv robot_control

# Terminal 3 - Image detection node
ros2 run ros2_opencv image_detection


🖼️ Output Example

    🟥 Red object highlighted with bounding box

    🐢 Robot reacts based on object position

    Status messages logged (e.g., Turning left, Moving forward)

📌 Notes

    Detection is based on red HSV thresholds — tune them in image_detection.py if needed.

    Works best with plain background and proper lighting.

    Intended as demonstration project for ROS2 + computer vision.

🤖 Author

Roi Aviram | Electrical and Computer Engineering Student
🔗 github.com/RoiAviram

