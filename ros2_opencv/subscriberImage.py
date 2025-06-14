#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

#cv_bridge is a package that cosists of a library for converting OpenCV images into a ROS image, and the opposite,
# so its serves as a bridge between ROS and OpenCV.
from cv_bridge import CvBridge

#import OpenCV
import cv2

class SubscriberNodeClass(Node):

    def __init__(self):
        super().__init__("subscriber_node")

        # CvBridge is used to convert OpenCV images to ROS2 messages that can be sent throught the topics
        self.bridgeObject = CvBridge()
 
        # name of the topic used to transfer the camera images
        # this topic should match the topic name in the publisher node
        self.topicNameFrames='topic_camera_image'

        # here the function "self.create_subscription" creates the subscriber that subscribes
        # to the messages of the type image, over the topic self.topicNameFrames
        self.subscription = self.create_subscription(Image,self.topicNameFrames,self.listener_callbackFunction,10)

        # counter tracking how many images are received
        self.i = 0

    #callback function called when a message is received
    def listener_callbackFunction(self, ROS2ImageMessage):
        # convert ROS2 message to OpenCV image
        frame = self.bridgeObject.imgmsg_to_cv2(ROS2ImageMessage)

        # display the image
        cv2.imshow('Image window', frame)
        cv2.waitKey(1)

        # logger displays the message on the screen
        self.get_logger().info('Receiving image number %d' % self.i)
        self.i += 1

#main function entry point of the code
def main(args=None):
    #initialize rclpy
    rclpy.init(args=args)

    #create subscriber object
    subscriberObject = SubscriberNodeClass()

    #spin - callback listener func is called recursively
    rclpy.spin(subscriberObject)

    subscriberObject.destroy_node()

    rclpy.shutdown()


if __name__== '__main__':
    main()
