#!/usr/bin/env python

import rospy
from bondpy import bondpy
import numpy as np

import sys
import os

#------------------------------------------------
def broken_bond(bond_which_broke):
    print(bond_which_broke+" has died")

def bonder(bond_list):
    for current_bond in bond_names:
        current_topic=current_bond+"_topic"
        bond = bondpy.Bond(current_topic, current_bond)#, on_broken=broken_bond(current_bond))
        bond.start()

#def bondChecker(bond_list):




def networkChecker():
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        machine_state=rospy.get_param('machine_state')
#        bond operation

        if machine_state=='INIT':
            if bond_check_result:
                print('network ok')
                rospy.set_param('network_ok',True)

        elif machine_state=='RUN':
            if not bond_check_result:
                print('network ERROR!')
                rospy.set_param('network_ok',False)

        loop_rate.sleep()

#---------------------------------------------------------------

global machine_state
global network_state
global bond_check_result
global bond_names

if __name__ == '__main__':
    bond_names=rospy.myargv(argv=sys.argv)[1::]
    print('=== NETWORK MONITOR===')
    rospy.init_node('network_monitor_node' ''', anonymous=True''')
    print('checking network...')
    bond_check_result=True
    bonder(bond_names)
    networkChecker()
'''
===== BONDS =====

#Process A

from bondpy import bondpy

id = generate_unique_id()
# Sends id to B using an action or a service
bond = bondpy.Bond("example_bond_topic", id)
bond.start()
if not bond.wait_until_formed(rospy.Duration(1.0)):
    raise Exception('Bond could not be formed')
# ... do things with B ...
bond.wait_until_broken()
print "B has broken the bond"

#Process B:

from bondpy import bondpy

# Receives id from A using a service or an action
bond = bondpy.Bond("example_bond_topic", id)
bond.start()sing a service or an action
# ... do things ...
bond.break_bond()


_________________________________________________________________________________________---

BACKUP ON ROSPARAM

<arg name="foo"/>
<arg name="bar"/>
<arg name ="dir1" value="directory1"/>
<node pkg="rosbag" type="record" name="recorder" args="-o $(arg dir1)/($arg foo)"/>

roslaunch my_pkg my_launch.launch foo:=$(rosparam get foo)
'''

