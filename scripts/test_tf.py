#!/usr/bin/env python3

import imp
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__=="__main__":
    node_name="test_tf"
    rospy.init_node(node_name,anonymous=False)
    tf_buffer=tf2_ros.Buffer(cache_time=rospy.Duration(1))
    tf_listener=tf2_ros.TransformListener(tf_buffer,queue_size=None)
    BASE_FRAME="base_link"
    while not rospy.is_shutdown():
        reference_frame=input("select an object's reference frame: ")
        if reference_frame=="e" or reference_frame=="exit" or reference_frame=='q':
            break
        check_timeout=0
        while True:
            try:
                tf_result=tf_buffer.lookup_transform(BASE_FRAME,reference_frame,rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
                    tf2_ros.ExtrapolationException): continue
            finally:
                check_timeout+=1
                if check_timeout>50: break
        tf_result=TransformStamped()
        print(tf_result.transform)

    print('exiting...')

"""
https://medium.com/@shilpajbhalerao/ros-pylauncher-9ac50951e230
launch multiple nodes inside rospy script

"""