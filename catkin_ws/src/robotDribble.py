#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist


class PuckDribbleFSM:
    def __init__(self):
        rospy.init_node('vision_dribble_omni_fsm')
        self.bridge = CvBridge()

        # Camera and image config
        self.cx = 320  # image center x (adjust if needed)
        self.pixels_threshold = 100

        # HSV thresholds
        self.pink_min = np.array([162, 160, 0])
        self.pink_max = np.array([172, 255, 255])
        self.green_min = np.array([66, 20, 124])
        self.green_max = np.array([83, 255, 255])

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_cb)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.image = None
        self.data = []
        self.distance = None
        self.state = 'SEARCH_PUCK'
        self.pink_center = None
        self.goal_center = None
        self.green_count = None
        #self.pink_x_last = None
        self.goal_center_1 = None
        self.goal_center_2 = None
        #self.goal_centers = [(None, 0), (None, 0)]
        self.rate = rospy.Rate(60)

    
    
    def image_cb(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Image conversion failed: %s", str(e))
        
    def detect(self):

        if self.image is None:
            return None, 0
        y_shape, x_shape, _ = self.image.shape
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        pink_mask = cv2.inRange(hsv, self.pink_min, self.pink_max)
        #print(f"pink mask", np.sum(pink_mask!=0))
        green_mask = cv2.inRange(hsv, self.green_min, self.green_max)
        #print(np.sum(green_mask!=0))
        
        if np.sum(pink_mask != 0) < self.pixels_threshold:
            self.pink_center = None
            print("no puck detected")
            #return
        else:
            _, pink_x = np.where(pink_mask != 0)
            pink_x_mean = np.mean(pink_x, dtype=np.uint32)
            pink_x_shift = x_shape//2 - pink_x_mean
            self.pink_center = pink_x_shift
        #print(self.pink_center)
        # self.pink_x_last = self.pink_x_shift
        
        # pink_contour = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # if pink_contour:
        #     c = max(pink_contour, key=cv2.contourArea)
        #     M = cv2.moments(c)
        #     if M["m00"] > 0:
        #         cx = int(M["m10"] / M["m00"])
        #         self.pink_center= cx
        #self.goal_centers = []
        if np.sum(green_mask != 0) < self.pixels_threshold:
            # self.goal_centers.append((None, 0))
            # if len(self.goal_centers) == 1:
            #     self.goal_centers.append((None, 0))
            self.goal_center = None
            self.goal_center_1 = None
            self.goal_center_2 = None
            print("no goal detected")
            # return
        else:           
            self.green_count = np.sum(green_mask != 0)
            _, green_x = np.where(green_mask != 0)
            green_x_mean = np.mean(green_x, dtype=np.uint32)
            green_x_shift = x_shape//2 - green_x_mean
            self.goal_center = green_x_shift
        #self.goal_centers = []
        
        # _, green_x = np.where(green_mask!=0)
        # green_left= []
        # green_right= []
        # for i in green_x:
        #     if i<=320:
        #         green_left.append(i)
        #     else:
        #         green_right.append(i)
        # # green_left = np.where(green_x<320)
        # # green_right = np.where(green_x>320)
        # print(len(green_left), len(green_right))
        # if len(green_left)> self.pixels_threshold-800:
        #     green_left_mean = np.mean(green_left, dtype = np.uint32)
        #     print("left detected")
        #     #self.goal_centers.append((x_shape//2 - green_left_mean, count))
        #     self.goal_center_1 = x_shape//2 - green_left_mean
        # else:
        #     self.goal_center_1 = None
             
        # if len(green_right) > self.pixels_threshold-800: 
        #     green_right_mean = np.mean(green_right, dtype = np.uint32)
        #     print("right detected")
        #     #self.goal_centers.append((x_shape//2 - green_right_mean, count))
        #     self.goal_center_2 = x_shape//2 - green_right_mean
        # else:
        #     self.goal_center_2 = None
        

        # if count < self.pixels_threshold:
        #     return None, 0
        # green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # self.goal_centers = []
        # if green_contours:
        #     for contour in green_contours:
        #         c = max(green_contours, key=cv2.contourArea)
        #         M = cv2.moments(c)
        #         if M["m00"] > 0:
        #             cx = int(M["m10"] / M["m00"])
        #             self.goal_centers.append((cx,count))

        # self.goal_centers.append((None, 0))
        # if len(self.goal_centers) == 1:
        #     self.goal_centers.append((None, 0))
        #self.goal_centers = goal_centers
        
        
    def laser_cb(self, msg):
        self.data = msg.ranges
        
    def get_distance(self):
        if len(self.data)==0:
            return None    
        #offset = 180
        detected_range = self.data[950:1025] #+ self.data[345-offset:359-offset]
        #print(self.data[160:190])
        return np.min(detected_range)
        #print(self.distance)

    # def detect_blob(self, min_color, max_color):
    #     if self.image is None:
    #         return None, 0
    #     hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    #     mask = cv2.inRange(hsv, min_color, max_color)
    #     count = np.sum(mask != 0)
    #     if count < self.pixels_threshold:
    #         return None, 0
    #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     centers = []
    #     if contours:
    #         for contour in contours:
    #             c = max(contours, key=cv2.contourArea)
    #             M = cv2.moments(c)
    #             if M["m00"] > 0:
    #                 cx = int(M["m10"] / M["m00"])
    #                 centers.append((cx,count))
    #     centers.append((None, 0))
    #     if len(centers) == 1:
    #         centers.append((None, 0))
    #     return centers

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
            # puck_info = self.detect_blob(self.pink_min, self.pink_max)
            # puck_cx, puck_pixels = puck[0]
            # goals_info = self.detect_blob(self.green_min, self.green_max)
            self.detect()
            self.distance = self.get_distance()
            # goal1_cx, goal_pixels = self.goal_centers[0]            
            # goal2_cx, goal_pixels = self.goal_centers[1]
            #self.state = 'ORBIT_PUCK'
            print(self.pink_center, self.distance, self.goal_center, self.green_count)
            
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
                #rospy.loginfo_throttle(2, "[FSM] SEARCH_PUCK")
                print("SEARCHING PUCK")
                if self.pink_center is not None and abs(self.pink_center) <100:
                    self.state = 'APPROACH_PUCK'
                    self.stop()
                    self.rate.sleep()
                else:
                    self.drive(0.0, 0.0, 0.3) if (self.pink_center is None or self.pink_center > 0) else self.drive(0.0, 0.0, -0.3)

            elif self.state == 'APPROACH_PUCK':
                #rospy.loginfo_throttle(2, "[FSM] APPROACH_PUCK")
                print("APPROACHING PUCK")
                if self.pink_center is not None and abs(self.pink_center) <100:
                    # if puck_pixels < 67950:
                    if self.distance > 0.25:
                        z = 0.002 #* self.pink_x_shift
                        x = 0.3 # if abs(error_x) < 100 else 0.0
                        self.drive(x, 0.0, z)
                    else:
                        self.stop()
                        self.state = 'ORBIT_PUCK'
                        self.rate.sleep()
                else:
                    self.state = 'SEARCH_PUCK'

            elif self.state == 'ORBIT_PUCK':
                #rospy.loginfo_throttle(2, "[FSM] ORBIT_PUCK (omni)")
                print("ORBITTING PUCK")
                # z = -0.4 #* self.pink_x_shift #if error_x is not None else 0.0
                # self.drive(0.0, 0.1, z)
                # if not puck_cx:
                if self.pink_center is None or abs(self.pink_center) >100:
                    self.state = 'SEARCH_PUCK'
                    self.stop()
                    self.rate.sleep()
                elif self.goal_center: #self.goal_center_1 and self.goal_center_2:
                    #if not (self.goal_center_1 > 0 and self.goal_center_2 < 0) or (self.goal_center_1 <0 and self.goal_center_2 > 0):
                    if abs(self.goal_center) > 200: 
                        z = 0.2#0.002 * self.pink_center #if error_x is not None else 0.0
                        self.drive(0.0, -0.05, z) if (self.goal_center is None or self.goal_center > 0) else self.drive(0.0, 0.05, -z)

                    else:
                        self.stop()
                        self.state = 'DRIBBLE_TO_GOAL'
                        self.rate.sleep()
                else:
                    z = 0.2 #* self.pink_x_shift #if error_x is not None else 0.0
                    self.drive(0.0, -0.05, z) if (self.goal_center is None or self.goal_center > 0) else self.drive(0.0, 0.05, -z)


            elif self.state == 'DRIBBLE_TO_GOAL':
                #rospy.loginfo_throttle(2, "[FSM] DRIBBLE_TO_GOAL")
                print("DRIBBLING TO GOAL")
                if self.pink_center and self.goal_center:#self.goal_center_1 and self.goal_center_2:
                    #z = 0.002 #* self.pink_x_shift
                    if self.green_count > 50000:
                        state = 'REACHED'
                        self.stop()
                        self.rate.sleep()
                    elif abs(self.pink_center) >100:
                        self.state = 'SEARCH_PUCK'
                        self.stop()
                        self.rate.sleep()
                    elif abs(self.goal_center > 200):
                        self.state = 'ORBIT_PUCK'
                        self.stop()
                        self.rate.sleep()
                    else:
                        self.drive(0.5, 0.0, 0.0)
                else :
                    if self.pink_center is None:
                        self.state = 'SEARCH_PUCK'
                        self.stop()
                        self.rate.sleep()
                    elif self.goal_center is None:
                        self.state = 'ORBIT_PUCK'
                        self.stop()
                        self.rate.sleep()
                    else:
                        continue
            elif state == 'REACHED':
                print("REACHED")
                continue 
            else: 
                self.state = 'SEARCH_PUCK'

            self.rate.sleep()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    print("HELLO")
    runner = PuckDribbleFSM()
    runner.run()
