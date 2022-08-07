#!/usr/bin/env python3

import rospy
import os
import subprocess

if __name__=="__main__":
    node_name="task_selector"
    rospy.init_node(node_name,anonymous=False)
    # launch=roslaunch.scriptapi.ROSLaunch()
    LAUNCH_COMMAND="roslaunch arm_logic objective"
    TASKS_DESCRIPTION="--- AVAILABLE TASKS, select [1-10]---\n"
    while not rospy.is_shutdown():
        print(TASKS_DESCRIPTION)
        try:
            selected_task=input("select an objective [1-10]: ")
            if selected_task=="e" or selected_task=="exit" or selected_task=='0':
                break

            selected_task=int(selected_task)
            if isinstance(selected_task,int) \
            and selected_task>0 \
            and selected_task<11:
                launch_command=LAUNCH_COMMAND+str(selected_task)+".launch"
                launch_command=launch_command.split()
                if selected_task==2:
                    tags=input('select tags in this format (e.g. "1 2 3 4"): ')
                    #TODO: add check over format
                    # launch_command+=' tags:='+tags
                    launch_arg='tags:='+tags
                    launch_command.append(launch_arg)


                if selected_task==4:
                    angle=input("select angle eg: 45.0): ")
                    # launch_command+=" angle:="+angle
                    launch_arg="angle:="+angle
                    launch_command.append(launch_arg)

                if selected_task==9:
                    tag=input('select tags (e.g. "1"): ')
                    # launch_command+=' tag:='+tag
                    launch_arg='tag:='+tag
                    launch_command.append(launch_arg)

                # print(launch_command)
                process=subprocess.Popen(launch_command)
                launch_output,launch_error=process.communicate()
                # os.system(launch_command)

        except KeyboardInterrupt or rospy.ROSInterruptException:
            pass

    print('exiting...')

"""
https://medium.com/@shilpajbhalerao/ros-pylauncher-9ac50951e230
launch multiple nodes inside rospy script

"""