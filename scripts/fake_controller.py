#!/usr/bin/env python3

import rospy
import numpy as np
import os
from geometry_msgs.msg import Pose
import tf2_ros
from geometry_msgs.msg import TransformStamped
from arm_vision.msg import FoundArucos
from arm_control.srv import DummyMarker,DummyMarkerResponse
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



inquiries_service='aruco_inquiries'



def arucoInquiriesClient(id):
    rospy.wait_for_service(inquiries_service)

    try:
        inquirer=rospy.ServiceProxy(inquiries_service,DummyMarker)
        inquiry_result=inquirer(id)

        if inquiry_result.found:
            print(inquiry_result.pose)
        else: print('not found yet')

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def fakeController():
    robot= moveit_commander.RobotCommander()
    scene=moveit_commander.PlanningSceneInterface()
    group_name="manipulator"
    group=moveit_commander.MoveGroupCommander(group_name)
    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()
    # robot.get_current_state()
    print(robot.get_current_state())




    input_id=input('select number id: ')

    arucoInquiriesClient(int(input_id))

    
###########################################################

if __name__ == '__main__':
    
    node_name="controller"
    print('node name: {}\n simulates some actins of objectives'.
        format(node_name))

    rospy.init_node(node_name,anonymous=False)
    moveit_commander.roscpp_initialize([])
    fakeController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Esc key pressed; Closing node: {}'.format(node_name))


"""
MOVEIT

https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

https://ros-planning.github.io/moveit_tutorials/
"""