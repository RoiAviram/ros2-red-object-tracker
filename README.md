**Vision-Based Autonomous Navigation | ROS2 & OpenCV 🟥🐢
**This project implements a modular, real-time autonomous navigation pipeline for a Turtlesim robot using ROS2 and Computer Vision. The system processes live webcam frames to detect specific visual cues (red objects) and translates them into movement commands while maintaining safety through collision avoidance logic.

**🏗️ System Architecture
**The system is designed with a decentralized approach, utilizing the Publisher/Subscriber pattern to ensure modularity and scalability.

Image Detection Node (image_detection.py):

Captures live video frames and converts them to the HSV color space for robust color segmentation.

Applies binary masking and contour analysis to isolate the target object.

Calculates the object's centroid and publishes directional commands (left, right, center, none) to a dedicated topic.

Robot Control Node (robot_control.py):

Subscribes to directional commands and the robot’s real-time pose (/turtle1/pose).

Translates high-level directions into precise Twist messages (linear and angular velocity).



Wall Avoidance Logic: Implements proximity monitoring to prevent collisions with the environment boundaries, ensuring system robustness.


🧠 Key Technical Features

Real-Time Processing: Optimized OpenCV pipeline for low-latency object tracking.



HSV Color Masking: Superior noise rejection compared to RGB-based detection.


Modular Node Design: Separation of concerns allows for easy debugging and potential integration with physical hardware.


Edge Case Handling: Integrated safety logic for boundary management.

📦 Installation & Setup
Prerequisites
ROS2 (Humble/Foxy)

Python 3.10+

OpenCV (opencv-python)

cv_bridge

Build
Bash

# Create and build the workspace
mkdir -p ~/ws_ros2/src
cd ~/ws_ros2/src
git clone https://github.com/RoiAviram/ros2-red-object-tracker.git
cd ..
colcon build --packages-select ros2_opencv
source install/setup.bash
🚀 Execution
Run the following commands in separate terminals:

Launch Turtlesim Environment: ros2 run turtlesim turtlesim_node

Start Image Processing Node: ros2 run ros2_opencv image_detection

Start Movement Control Node: ros2 run ros2_opencv robot_control

🎬 Demo
(Recommend: Add a GIF here of your optimized LinkedIn video)

Note: For a full video demonstration including the code and real-time logs, visit my LinkedIn post.

🛠️ Skills Demonstrated

Robotics: ROS2 Humble, Multi-node communication, Topic management.




Computer Vision: OpenCV, HSV Segmentation, Contour Analysis.



Software Engineering: Python, Linux (Ubuntu), Git, Modular System Design.


👨‍💻 Author

Roi Aviram Electrical and Computer Engineering Student @ Ben-Gurion University.


Former Algorithm Data Analyst & Automation Engineer (Units 9900, 8200).
