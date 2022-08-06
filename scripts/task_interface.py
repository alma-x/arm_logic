#!/usr/bin/env python3

from curses import KEY_UNDO
import rospy
import os

if __name__=="__main__":
    node_name="task_interface"
    rospy.init_node(node_name,anonymous=False)
    # launch=roslaunch.scriptapi.ROSLaunch()
    LAUNCH_COMMAND="roslaunch arm_logic objective"
    
    while not rospy.is_shutdown():
        try:
            selected_task=input("select an objective [1-10]: ")
            if isinstance(selected_task,int) \
            and selected_task>0 \
            and selected_task<11:
                launch_command=LAUNCH_COMMAND+str(selected_task)+".launch"
                if selected_task==2:
                    tags=str(input("select tags (e.g. 1 2 3 4): "))
                    launch_command+=" tags:="

                if selected_task==2:
                    tags=str(input("select tags (e.g. 1 2 3 4): "))
                    launch_command+=" tags:="

                if selected_task==2:
                    tags=str(input("select tags (e.g. 1 2 3 4): "))
                    launch_command+=" tags:="
                os.system(launch_command)
        except KeyboardInterrupt or rospy.ROSInterruptException:
            pass

"""
https://medium.com/@shilpajbhalerao/ros-pylauncher-9ac50951e230
launch multiople nodes inside rospy script



import subprocess

bashCommand = "cwm --rdf test.rdf --ntriples > test.nt"
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
output, error = process.communicate()

"""