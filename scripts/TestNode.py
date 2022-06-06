#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from bondpy import bondpy

import numpy as np

import sys
import os

#------------------------------------------------


'''
def callbackRaw(raw_img):
#    global aruco_success
    if bool_exit:
    else:

'''
    
#-----------------------------------------------------------------
'''
def Test():
#    rospy.Subscriber(myCam+myTop,myType,myCallk,queue_size = 1)
#    rospy.Service('cv_server', cv_server, callback_service)
    try:
        rospy.spin()
#    except KeyboardInterrupt:
'''
#---------------------------------------------------------------
'''
#this interrupt handler is really fast
import signal
def interrupt_handler(signal,frame):
    global interrupted
    interrupted=True
signal.signal(signal.SIGINT,interrupt_handler)
interrupted=False
    while True:
        if interrupted:
            break
'''
def bonding(name_of_bond,bond_formation_timeout):
    bond_topic=name_of_bond+"_topic"
    bond=bondpy.Bond(bond_topic,bond_name)#,on_broken=print("broken"))
    bond.start()
    if not bond.wait_until_formed(rospy.Duration(bond_formation_timeout)):
        raise Exception(name_of_bond+" could not be formed")
    print(name_of_bond+" correctly formed")
    #bond.wait_until_broken()
    #print(name_of_bond+" broke")

def chatter():
    pub=rospy.Publisher('test_topic', String, queue_size=10)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        machine_state=rospy.get_param('/machine_state')
        operation_mode=rospy.get_param('/operation_mode')
        if machine_state=='INIT':
            print('machine in INIT state')
            if operation_mode=='AUTO':
                print("operation mode is AUTO")
            elif operation_mode=='MANUAL':
                print("operation mode is MANUAL")

        elif machine_state=='RUN':
            print('machine in RUN state')
            if operation_mode=='AUTO':
                print("operation mode is AUTO")
            elif operation_mode=='MANUAL':
                print("operation mode is MANUAL")
            pub.publish("this is a message")
        #else:
        r.sleep()

'''
def init_operations():
    print('machine in INIT state')

#ISSUE: crash due to pub.publish if pub=rospy.Pub... inside chatter
def run_operations():
    print('machine in RUN state')
    pub.publish("this is a message")
'''
global machine_state
global operation_mode

if __name__ == '__main__':
    node_number=rospy.myargv(argv=sys.argv)[1]
    node_name='test_node'+str(node_number)
    bond_name=rospy.myargv(argv=sys.argv)[2]
    print('=== TEST NODE :'+node_name+ '===')
    rospy.init_node(node_name)
    print('bond name: '+bond_name)
    bond_timeout=5.0
    bonding(bond_name,bond_timeout)
    chatter()
