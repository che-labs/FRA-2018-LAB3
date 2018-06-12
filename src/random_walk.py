#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from random import randint
from sensor_msgs.msg import LaserScan
from functools import reduce
import time
import math

# globals
SCAN = None
target = 1.2


def hay_obstaculo(ranges):
    global target
    for r in ranges[270:440]:
        if not math.isnan(r) and r < target:
            return True

    return False


# This function will be called every time a new scan message is
# published.
def scan_callback(scan_msg):
    """ scan will be of type LaserScan """

    # Save a global reference to the most recent sensor state so that
    # it can be accessed in the main control loop.
    # (The global keyword prevents the creation of a local variable here.)
    global SCAN
    SCAN = scan_msg


def random_walk():
    global target
    rospy.init_node('random_walk', anonymous=True)

    velocity_pub = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rate = rospy.Rate(10)  # 10hz
    vel_msg = Twist()

    # Wait until the first scan is available.
    while SCAN is None and not rospy.is_shutdown():
        rospy.sleep(.1)

    while not rospy.is_shutdown():
        '''
        vel_msg.linear.x = randint(-5, 5)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = randint(-1, 1)

        msg = "Publicando la siguiente velocidad: {}".format(vel_msg)
        rospy.loginfo(msg)
        velocity_pub.publish(vel_msg)
        rate.sleep()
        '''
        # Back up if the scan is bad, or if we are too close.
        if hay_obstaculo(SCAN.ranges):
            vel_msg.linear.x = 0
            vel_msg.angular.z = .5   # rotation in radians/second
            print("Objeto en la distancia limite: {:.3f}".format(SCAN.ranges[360]))
        else:
            vel_msg.linear.x = .5
            vel_msg.angular.z = 0

        # print("Length: {}".format(len(SCAN.ranges)))
        velocity_pub.publish(vel_msg)
        # print("Range 90 grados: {:.3f}".format(SCAN.ranges[360]))
        print("Ranges length: {:.3f}".format(len(SCAN.ranges)))

        # msg = "Publicando la siguiente velocidad: {}".format(vel_msg)
        # rospy.loginfo(msg)
        # velocity_pub.publish(vel_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        random_walk()
    except rospy.ROSInterruptException:
        pass
