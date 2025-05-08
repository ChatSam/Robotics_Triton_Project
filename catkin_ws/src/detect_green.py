#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class HSVInspector:
    def __init__(self):
        rospy.init_node('hsv_inspector')
        self.bridge = CvBridge()
        self.image = None
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
        cv2.namedWindow("HSV Inspector")
        cv2.setMouseCallback("HSV Inspector", self.mouse_callback)

    def image_cb(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CV bridge error: %s", str(e))

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.image is not None:
            hsv_img = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
            pixel = hsv_img[y, x]
            print(f"HSV at ({x}, {y}): {pixel}")  # e.g., [60 255 255]

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.image is not None:
                cv2.imshow("HSV Inspector", self.image)
                cv2.waitKey(1)
            rate.sleep()

if __name__ == '__main__':
    inspector = HSVInspector()
    inspector.run()
