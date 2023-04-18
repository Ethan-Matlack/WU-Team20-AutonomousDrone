#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraSubscriber:
    def __init__(self, topic_name="/roscam/cam/image_raw"):
        rospy.init_node('camera_subscriber')
        self.bridge = CvBridge()
        self.img = None
        rospy.Subscriber(topic_name, Image, self.image_callback)
        rospy.spin()

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img = img.copy()

    def get_image(self) -> cv2.cv2:
        return self.img
