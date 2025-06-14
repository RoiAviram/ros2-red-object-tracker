#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.publisher = self.create_publisher(String,'diraction', 10)
        self.camera = cv2.VideoCapture(0)  
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.Color_detect_callback)

        # Define HSV range for red color (for differnt color detection change the first values, the two other parameters are for saturation and value)
        self.lower_red1 = np.array([0, 150, 150])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 150, 150])
        self.upper_red2 = np.array([180, 255, 255])

    def Color_detect_callback(self):
        success, frame = self.camera.read()
        if not success:
            self.get_logger().warning('Failed to capture frame from camera.')
            return

        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (640, 480))  

        #Converts the input image  from the BGR color space (used by OpenCV) to the HSV color space.
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create masks for red color (create binary black and white ('0','255') mask from the pixels of the red spectrum)
        mask1 = cv2.inRange(hsv_frame, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv_frame, self.lower_red2, self.upper_red2)
        mask = mask1 + mask2

        # Find the boundaries (contours) of white areas in the binary mask and igmore the small contures.
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)     
        MIN_CONTOUR_AREA = 250
        valid_contours = [contour for contour in contours if cv2.contourArea(contour) > MIN_CONTOUR_AREA]

        feature_position = "none"

        if valid_contours:
            largest_contour = max(valid_contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            box_center_x = x + w // 2    # Calculate the center of the box

            # Determine the red object position 
            frame_width = frame.shape[1]
            if box_center_x < frame_width // 3:
                feature_position = "left"
            elif box_center_x > 2 * frame_width // 3:
                feature_position = "right"
            else:
                feature_position = "center"
 
            # Draw boxes around the detected red objects
            for contour in valid_contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, 'Red Object', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publishing the position to robot control, and log it in the terminal (for debugging) 
        self.get_logger().info(f"Publishing direction: {feature_position}")
        self.publisher.publish(String(data=feature_position))

        # Display 
        cv2.putText(frame, f"Position: {feature_position}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.imshow('Detection', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
