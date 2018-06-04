#!/usr/bin/env python

import roslib
roslib.load_manifest('opencv_test')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class ImageConverter:

    def __init__(self):
        # self.image_pub = rospy.Publisher("/image_view_1527623193584228729/output", Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

        self.hydrant_cascade = cv2.CascadeClassifier('data/cascade.xml')

    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            # hsv_img2 = cv2.cvtColor(cv_image2, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)

        red_hydrant_lower = np.array([0, 50, 39], np.uint8)
        red_hydrant_upper = np.array([5, 255, 255], np.uint8)

        green_dumpster_lower = np.array([65, 0, 0], np.uint8)
        green_dumpster_upper = np.array([75, 255, 255], np.uint8)

        # draw hydrant contourns
        frame_threshed = cv2.inRange(hsv_img, red_hydrant_lower, red_hydrant_upper)
        ret, thresh = cv2.threshold(frame_threshed, 22, 255, 0)
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # draw in blue the contours that were founded
            # cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)
            cv2.drawContours(cv_image, contours, -1, 255, 3)

            # find the biggest area
            c = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(c)
            # draw the book contour (in green)

            if w > 20:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, 'Hydrant', (x - 10, y - 10), font, 0.5, (11, 255, 255), 2, cv2.LINE_AA)

        frame_threshed2 = cv2.inRange(hsv_img, green_dumpster_lower, green_dumpster_upper)
        ret2, thresh2 = cv2.threshold(frame_threshed2, 22, 255, 0)
        im2_, contours2, hierarchy2 = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours2) > 0:
            # draw in blue the contours that were founded
            # cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)
            cv2.drawContours(cv_image, contours2, -1, (217, 255, 179), 3)

            # find the biggest area
            c = max(contours2, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(c)
            # draw the book contour (in green)

            if w > 70 and h > 50:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (51, 153, 255), 2)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, 'Dumpster', (x - 10, y - 10), font, 0.5, (204, 0, 153), 2, cv2.LINE_AA)

        cv2.imshow("Show", cv_image)
        cv2.waitKey(3)

    '''
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
    '''


def main(args):
    ImageConverter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)