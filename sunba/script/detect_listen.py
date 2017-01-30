#!/usr/bin/env python

import rospy
import time
from rospy_tutorials.msg import Floats


def callback(data):
    rospy.loginfo("Listen to the detection node")
    angular = data[0]
    distance = data[1]
    print "Angular and Distance: ", data # angular, distance

def lis():
    rospy.init_node("lis", anonymous=True)
    rospy.Subscriber("/detection", Floats, callback)

    rospy.spin()

if __name__== '__main__':
    lis()


