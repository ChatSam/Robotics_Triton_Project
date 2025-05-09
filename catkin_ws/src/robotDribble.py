#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist


pixels = 1000

class PuckDribbleFSM:
    def __init__(self):
        rospy.init_node('vision_dribble_omni_fsm')
        self.bridge = CvBridge()

        # Camera and image config
        self.cx = 320  # image center x (adjust if needed)
        self.pixels_threshold = 1000

        # HSV thresholds
        self.pink_min = np.array([162, 160, 0])
        self.pink_max = np.array([172, 255, 255])
        self.green_min = np.array([66, 20, 124])
        self.green_max = np.array([83, 255, 255])

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_cb)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.image = None
        self.distance = None
        self.state = 'SEARCH_PUCK'
        self.pink_x_shift = None
        self.pink_x_last = None
        self.goal_centers = [(None, 0), (None, 0)]
        self.rate = rospy.Rate(60)

    def image_cb(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Image conversion failed: %s", str(e))
        
        if self.image is None:
            return None, 0
        y_shape, x_shape, _ = self.image.shape
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        pink_mask = cv2.inRange(hsv, self.pink_min, self.pink_max)
        green_mask = cv2.inRange(hsv, self.green_min, self.green_max)
        if np.sum(pink_mask != 0) < pixels:
            self.pink_x_shift = None
            return
        _, pink_x = np.where(pink_mask != 0)
        x_mean = np.mean(pink_x, dtype=np.uint32)
        self.pink_x_shift = x_shape//2 - x_mean
        self.pink_x_last = self.pink_x_shift
        
        count = np.sum(pink_mask != 0)
        if count < self.pixels_threshold:
            return None, 0
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        goal_centers = []
        if contours:
            for contour in contours:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    goal_centers.append((cx,count))

        goal_centers.append((None, 0))
        if len(goal_centers) == 1:
            goal_centers.append((None, 0))
        self.goal_centers = goal_centers

    def laser_cb(self, msg):
        data = msg.ranges
        detected_range = data[0:14] + data[345:359]
        self.distance = np.min(detected_range)
        print(self.distance)

    def detect_blob(self, min_color, max_color):
        if self.image is None:
            return None, 0
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, min_color, max_color)
        count = np.sum(mask != 0)
        if count < self.pixels_threshold:
            return None, 0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = []
        if contours:
            for contour in contours:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    centers.append((cx,count))
        centers.append((None, 0))
        if len(centers) == 1:
            centers.append((None, 0))
        return centers

    def drive(self, x=0.0, y=0.0, z=0.0):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.cmd_pub.publish(twist)

    def stop(self):
        self.drive(0.0, 0.0, 0.0)

    def run(self):
        while not rospy.is_shutdown():
            puck_info = self.detect_blob(self.pink_min, self.pink_max)
            #puck_cx, puck_pixels = puck[0]
            goals_info = self.detect_blob(self.green_min, self.green_max)
            goal1_cx, goal_pixels = self.goal_centers[0]            
            goal2_cx, goal_pixels = self.goal_centers[1]
            
            '''
            error_x = None
            if puck_cx is not None:
                error_x = self.cx - puck_cx
                self.x_shift = error_x
                self.x_last = error_x
            else:
                self.x_shift = None
            '''
            
            ### FSM logic
            if self.state == 'SEARCH_PUCK':
                rospy.loginfo_throttle(2, "[FSM] SEARCH_PUCK")
                if self.pink_x_shift is not None and abs(self.pink_x_shift) <100:
                    self.state = 'APPROACH_PUCK'
                else:
                    self.drive(0.0, 0.0, 0.5 if (self.pink_x_last is None or self.pink_x_last > 0) else -0.5)

            elif self.state == 'APPROACH_PUCK':
                rospy.loginfo_throttle(2, "[FSM] APPROACH_PUCK")
                if self.pink_x_shift is not None and abs(self.pink_x_shift) <100:
                    # if puck_pixels < 67950:
                    if self.distance > 0.5:
                        z = 0.002 * self.pink_x_shift
                        x = 0.5 # if abs(error_x) < 100 else 0.0
                        self.drive(x, 0.0, z)
                    else:
                        self.stop()
                        self.state = 'ORBIT_PUCK'
                else:
                    self.state = 'SEARCH_PUCK'

            elif self.state == 'ORBIT_PUCK':
                rospy.loginfo_throttle(2, "[FSM] ORBIT_PUCK (omni)")
                # if not puck_cx:
                if self.pink_x_shift is None or abs(self.pink_x_shift) >100:
                    self.state = 'SEARCH_PUCK'
                elif goal1_cx and goal2_cx:
                    if goal1_cx > 300 and goal2_cx < 350:
                        z = 0.002 * self.pink_x_shift #if error_x is not None else 0.0
                        self.drive(0.0, 0.1, z)
                    else:
                        self.stop()
                        self.state = 'DRIBBLE_TO_GOAL'
                else:
                    z = 0.002 * self.pink_x_shift #if error_x is not None else 0.0
                    self.drive(0.0, 0.1, z)

            elif self.state == 'DRIBBLE_TO_GOAL':
                rospy.loginfo_throttle(2, "[FSM] DRIBBLE_TO_GOAL")
                if self.pink_x_shift and goal1_cx and goal2_cx:
                    z = 0.002 * self.pink_x_shift
                    self.drive(0.5, 0.0, z)
                else:
                    self.stop()
                    if self.pink_x_shift is None or abs(self.pink_x_shift) >100:
                        self.state = 'SEARCH_PUCK'
                    else:
                        self.state = 'ORBIT_PUCK'
            else: 
                self.state = 'SEARCH_PUCK'

            self.rate.sleep()

if __name__ == '__main__':
    print("HELLO")
    runner = PuckDribbleFSM()
    runner.run()
