#!/usr/bin/env python

import rospy
import numpy as np

import sys
import os
import time

#----------------------------------------------------------------

global machine_state
global network_state
#global operation_mode

#-----------------------------------------------------------------
'''
def enum(**enums):
    return type('Enum', (), enums)

State_Type=enum(INIT=0,RUN=1,ERROR=100)
'''

def operationManager(operation_mode):
    if operation_mode=="MANUAL":
        text_wrapper=""
    elif operation_mode=="AUTO":
        text_wrapper=" !!! "
    print("___current operation mode: "+text_wrapper+str(operation_mode)+text_wrapper)
    op_mode_input=str(raw_input("enter CHANGE to switch to other mode(AUTO,MANUAL): "))

    if op_mode_input=="CHANGE":
        if operation_mode=="AUTO":
            rospy.set_param("operation_mode","MANUAL")
            time.sleep(1)

        elif operation_mode=="MANUAL":
            rospy.set_param("operation_mode","AUTO")
            time.sleep(1)
#-----------------------------------------------------------------

def stateChecker():
#    pub=rospy.Publisher('test_topic', String, queue_size=10)
    rospy.init_node('state_manager_node' ''', anonymous=True''')
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        machine_state=rospy.get_param('machine_state')
#        may add default value(if not set): get_param(PARAM,DEFAULT_VAL)
        network_state=rospy.get_param('network_ok')
        operation_mode=rospy.get_param('operation_mode')

        if machine_state=='INIT':
            if network_state:
                print('transitioning to RUNNING state')
                rospy.set_param('machine_state',"RUN")

        elif machine_state=='RUN':
            operationManager(operation_mode)

            if not network_state:
                print('transitioning to INITIALIZING state')
                rospy.set_param('machine_state',"INIT")


        loop_rate.sleep()

#---------------------------------------------------------------

if __name__ == '__main__':
    print('===STATE MANAGER===')
    print('setting up the system...')
    stateChecker()


