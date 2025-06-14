#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String  
from turtlesim.msg import Pose

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.create_subscription(String,'diraction', self.detection_callback, 10)  #subscibes to image detection
        self.pose_subscriber = self.create_subscription(Pose,"/turtle1/pose",self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.current_pose = None
        self.current_command = "stop"

    def pose_callback(self, pose: Pose):
        self.current_pose = pose

    def detection_callback(self, msg):
        diraction = msg.data  # contains the detected direction ("left", "right", "center", or "none")
        self.get_logger().info(f"Received direction: {diraction}")
        twist = Twist()


        if self.current_pose is not None:      #check if the current node set
            if self.current_pose.x > 9.0 or self.current_pose.x < 2.0 or self.current_pose.y > 9.0 or self.current_pose.y < 2.0: #spin the turtle before the edge
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angular_speed  
            elif self.current_pose.x > 11.0 or self.current_pose.x < 0.1 or self.current_pose.y > 11.0 or self.current_pose.y < 0.1: #make sure he will not hit the edges
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed 
            else:    
                if diraction == "center":  # Move forward
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0
                    self.current_command = "move forward"
                elif diraction == "left":  # Turn left
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angular_speed
                    self.current_command = "turn left"
                elif diraction == "right":  # Turn right (minus agular)
                    twist.linear.x = self.linear_speed
                    twist.angular.z = -self.angular_speed   
                    self.current_command = "turn right"
                elif diraction == "none":  # Stop the robot
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.current_command = "stop"
        else:   #If no pose is available, stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.current_command = "stop"

        
        self.velocity_publisher.publish(twist)
        self.get_logger().info("Robot Command: " + self.current_command)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
