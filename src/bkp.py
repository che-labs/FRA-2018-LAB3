#!/usr/bin/env python

import roslib
roslib.load_manifest('opencv_test')
import sys
import rospy
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from datetime import datetime, timedelta
import time
import math
import random


class ImageConverter:

    def __init__(self):
        self.bridge = CvBridge()
        self.scan = None
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.vel_msg = Twist()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.velocity_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        self.turn = 0
        self.is_hydrant = False
        self.is_dumpster = False
        self.encontre_hydrant = False
        self.encontre_dumpster = False
        self.avoid_target = 1.5
        self.find_target = .8
        self.sense = 1
        self.is_center = False

    def hay_obstaculo(self, ranges):
        for i, r in enumerate(ranges[270:440]):
            if not math.isnan(r) and r < self.avoid_target:
                # print('Index: {}'.format(i))
                if i < 85:
                    self.sense = -1
                else:
                    self.sense = 1
                return True
        return False

    def hay_objeto(self):
        if self.scan:
            print('Length: {}'.format(len(self.scan.ranges)))
            for r in self.scan.ranges[340:380]:
                if not math.isnan(r) and r < self.find_target:
                    print('Hay objeto a: {}'.format(r))
                    return True

        return False

    def scan_callback(self, scan_msg):
        """ scan will be of type LaserScan """
        self.scan = scan_msg

    def random_walk(self):
        # Wait until the first scan is available.
        while self.scan is None and not rospy.is_shutdown():
            rospy.sleep(.1)

        while not rospy.is_shutdown():

            hay_obstacle = self.hay_obstaculo(self.scan.ranges)

            if self.hay_objeto() and not ((self.is_hydrant or self.is_dumpster) and self.is_center):
                #print('HAY OBSTACULO , NO HAY OBJECTO')
                # Tengo que girar porque choco con algo que no es el objetivo
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = self.sense * 0.8  # rotation in radians/second
                self.velocity_pub.publish(self.vel_msg)
            elif self.is_hydrant or self.is_dumpster:
                print('NO HAY OBJECTO')
                self.go_to_object()
            else:
                #print('RANDOM WALK')
                self.vel_msg.linear.x = .5
                self.vel_msg.angular.z = 0
                self.velocity_pub.publish(self.vel_msg)

            self.rate.sleep()

    def go_to_object(self):
        self.vel_msg.linear.x = 0.6
        self.velocity_pub.publish(self.vel_msg)
        print('Center: {} --- Hay Objeto: {}'.format(self.is_center, self.hay_objeto()))
        if self.hay_objeto():
            if self.is_hydrant:
                rospy.loginfo('ENCONTRE HYDRANT! :-)')
                seconds = 2
                self.encontre_hydrant = True
            else:
                rospy.loginfo('ENCONTRE DUMPSTER! :-)')
                seconds = 3
                self.encontre_dumpster = True

            # go back 1.5 seconds
            now = datetime.now()
            wait = now + timedelta(seconds=1.5)
            while datetime.now().second < wait.second:
                self.vel_msg.linear.x = -0.8
                self.velocity_pub.publish(self.vel_msg)
            # turn n seconds
            now = datetime.now()
            sense = random.choice([-1, 1])
            wait = now + timedelta(seconds=seconds)
            while datetime.now().second < wait.second:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = sense * 1
                self.velocity_pub.publish(self.vel_msg)

            '''
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
            self.velocity_pub.publish(self.vel_msg)
            '''
            self.is_hydrant, self.is_dumpster = False, False

    def camera_callback(self, data):
        cv_image = {}
        hsv_img = {}

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)

        red_hydrant_lower = np.array([0, 50, 39], np.uint8)
        red_hydrant_upper = np.array([5, 255, 255], np.uint8)

        green_dumpster_lower = np.array([65, 0, 0], np.uint8)
        green_dumpster_upper = np.array([75, 255, 255], np.uint8)

        self.is_hydrant = False
        self.is_dumpster = False
        self.is_center = False

        if not self.encontre_hydrant:
            self.find_hydrant(cv_image, hsv_img, red_hydrant_lower, red_hydrant_upper)

        if not self.is_hydrant and not self.encontre_dumpster:
            self.find_dumpster(cv_image, hsv_img, green_dumpster_lower, green_dumpster_upper)

        '''
        if not self.hay_objeto():
            # find and draw hydrant contours
            self.find_hydrant(cv_image, hsv_img, red_hydrant_lower, red_hydrant_upper)
            # find and draw dumpster contours
            self.find_dumpster(cv_image, hsv_img, green_dumpster_lower, green_dumpster_upper)
        '''

        # draw contours in image window
        cv2.imshow("Show", cv_image)
        cv2.waitKey(3)

    def find_hydrant(self, cv_image, hsv_img, color_lower, color_upper):
        # draw hydrant contourns
        frame_threshed = cv2.inRange(hsv_img, color_lower, color_upper)
        ret, thresh = cv2.threshold(frame_threshed, 22, 255, 0)
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # draw in blue the contours that were founded
            # cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)
            cv2.drawContours(cv_image, contours, -1, 255, 3)

            # find the biggest area
            c = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(c)

            # draw the contour rectangle
            if w > 5 and h > 40 and not self.hay_objeto():
                if not self.is_dumpster:
                    self.is_hydrant = True
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, 'Hydrant', (x - 10, y - 10), font, 0.5, (11, 255, 255), 2, cv2.LINE_AA)

                if x or y:
                    self.center_husky(x, y, w, h)

    def find_dumpster(self, cv_image, hsv_img, color_lower, color_upper):
        frame_threshed = cv2.inRange(hsv_img, color_lower, color_upper)
        ret, thresh = cv2.threshold(frame_threshed, 22, 255, 0)
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # draw in blue the contours that were founded
            # cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)
            cv2.drawContours(cv_image, contours, -1, (217, 255, 179), 3)

            # find the biggest area
            c = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(c)

            # draw the book contour rentangle
            if w > 70 and h > 50 and not self.hay_objeto():
                if not self.is_hydrant:
                    self.is_dumpster = True
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (51, 153, 255), 2)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, 'Dumpster', (x - 10, y - 10), font, 0.5, (204, 0, 153), 2, cv2.LINE_AA)

                if x or y:
                    self.center_husky(x, y, w, h)

    def center_husky(self, x, y, w, h):
        #print('X: {} -- Y: {} -- W: {}'.format(x, y, w))
        if (self.is_hydrant or self.is_dumpster) and not self.hay_objeto():
            if (x+w/2) > 355:
                #rospy.loginfo("TURN RIGHT")
                #print('X: {} -- Y: {}'.format(x, y))
                self.turn += -0.1
                if self.turn < -0.5:
                    self.turn = -0.5
            elif (x+w/2) < 325:
                #rospy.loginfo("TURN LEFT")
                #print('X: {} -- Y: {}'.format(x, y))
                self.turn += 0.1
                if self.turn > 0.5:
                    self.turn = 0.5
            else:
                rospy.loginfo("CENTERED")
                self.turn = 0
                self.is_center = True

            self.vel_msg.angular.z = self.turn
            self.velocity_pub.publish(self.vel_msg)


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    husky = ImageConverter()
    try:
        husky.random_walk()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down")
        del husky
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
