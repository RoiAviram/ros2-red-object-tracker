#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

#cv_bridge is a package that cosists of a library for converting OpenCV images into a ROS image, and the opposite,
# so its serves as a bridge between ROS and OpenCV.
from cv_bridge import CvBridge

#import OpenCV
import cv2

class PublisherNodeClass(Node):

    def __init__(self):
        super().__init__("publisher_node")

        # creating an instance of the OpenCV VideoCapture object 
        # this is the camera device number
        self.cameraDeviceNumber=0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)

        # CvBridge is used to convert OpenCV images to ROS2 messages that can be sent throught the topics
        self.bridgeObject = CvBridge()
 
        # name of the topic used to transfer the camera images
        # this topic should match the topic name in the subscriber node
        self.topicNameFrames='topic_camera_image'

        # the queue size for messages
        self.queueSize=20

        # here the function "self.create_publisher" creates the publisher that publishes 
        # the messages of the type image, over the topic self.topicNameFrames and withself.queueSize
        self.publisher = self.create_publisher(Image,self.topicNameFrames,self.queueSize)

        # communication period in seconds
        self.periodCommunication = 0.02

        # create a timer that calls the function self.timer_callback every self.periodCommunication sec
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)

        # counter tracking hoe many images are published
        self.i = 0

    #callback function that calleed every self.periodCommunication se
    def timer_callbackFunction(self):

        #here we read the frame bu using the camera
        success, frame = self.camera.read()
        #resize the image
        frame = cv2.resize(frame, (820,640), interpolation=cv2.INTER_CUBIC)

        if success == True:
            #here we convert Opencv frame to 
            ROS2ImageMessage=self.bridgeObject.cv2_to_imgmsg(frame)
            #publish the image
            self.publisher.publish(ROS2ImageMessage)

        # logger display the massage on the screan
        self.get_logger().info('Publishing image number %d' % self.i)
        self.i += 1

#main function entrey point of the code
def main(args=None):
    #initialize rlcpy
    rclpy.init(args=args)

    #create puublisher object
    publisherObject = PublisherNodeClass()

    #spin - callback timer func is called recursvly
    rclpy.spin(publisherObject)

    publisherObject.destroy_node()

    rclpy.shutdown()


if __name__== '__main__':
    main()