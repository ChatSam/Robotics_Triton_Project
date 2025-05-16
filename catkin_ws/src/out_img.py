#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PinkMasker:
    def __init__(self):
        rospy.init_node('pink_masker_node', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        # Convert ROS image message to OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr("CV Bridge Error: {0}".format(e))
            return

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define pink color range (tweak as needed)
        lower_pink = np.array([140, 50, 50])
        upper_pink = np.array([170, 255, 255])

        pink_min = np.array([162, 160, 0])
        pink_max = np.array([172, 255, 255])
        green_min = np.array([66, 20, 124])
        green_max = np.array([83, 255, 255])
        # Create a mask
        mask = cv2.inRange(hsv, green_min, green_max)

        # Apply mask
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("L", result)
        cv2.waitKey(1)
        return
        imgray = cv2.GaussianBlur(cv2.cvtColor(result, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        # Show result
        ret, thresh = cv2.threshold(imgray, 127, 255, 0)
        edged = cv2.Canny(imgray, 30, 200)
        contours, hierarchy = cv2.findContours(edged,
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
       
        min_area = 20

        # Filter and draw contours
        cnt_mod = sorted([(cv2.contourArea(cnt), cnt) for cnt in contours if (cv2.contourArea(cnt)) > min_area], key=lambda a: a[0], reverse=True)
        cv2.imshow("Pink Mask", edged)
        print(cnt_mod[0][1].reshape(-1, 2).shape)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        pink_masker = PinkMasker()
        pink_masker.run()
    except rospy.ROSInterruptException:
        pass

