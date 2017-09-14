#!/usr/bin/env python
import rospy
import os
import sys

"""
string_cmd = "gnome-terminal -x bash -c 'source ~/ROS_projects/icra2018/devel/setup.bash; rosrun distributed Algorithm_1.py 0'"
print '\n\n\n\n'
print 'Here is string_cmd: ', string_cmd1
print '\n\n\n\n'
"""

# Initial function
if __name__ == '__main__':



    #Read the robots index
    if len(sys.argv) < 2:
        print 'ERROR!!!\nThe the number of robots was not provided'
    else:
        R = int(sys.argv[1])

    print '\n\nHere is R', R, '\n\n'


    #"""
    #string_cmd = "gnome-terminal -x bash -c 'source ~/ROS_projects/coordination_milp/devel/setup.bash; rosrun sim_gazebo_ds control_ds_rob2.py'"
    #string_cmd = "gnome-terminal -x bash -c 'source ~/ROS_projects/icra2018/devel/setup.bash; rosrun distributed Algorithm_1.py 0'"
    string_cmd = "gnome-terminal -x bash -c 'rosrun distributed Algorithm_1.py 0'"
    os.system(string_cmd)
    string_cmd = "gnome-terminal -x bash -c 'rosrun distributed Algorithm_1.py 1'"
    os.system(string_cmd)
    #"""

    if R >= 3:
	string_cmd = "gnome-terminal -x bash -c 'rosrun distributed Algorithm_1.py 2'"
	os.system(string_cmd)

    if R >= 4:
	string_cmd = "gnome-terminal -x bash -c 'rosrun distributed Algorithm_1.py 3'"
	os.system(string_cmd)


    """
    string_cmd = "rosrun distributed Algorithm_1.py 0"
    os.system(string_cmd)
    string_cmd = "rosrun distributed Algorithm_1.py 1"
    os.system(string_cmd)
    """

