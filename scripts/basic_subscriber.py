#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import numpy as np

import sys
import os

#------------------------------------------------

def callback(data):
    rospy.set_param("/my_new_param",str(data))

def listener():
    rospy.init_node('my_listener')
    rospy.Subscriber('/my_test_topic', String, callback,queue_size=1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupted')


if __name__ == '__main__':
    listener()
