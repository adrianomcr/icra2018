#!/usr/bin/env python
import rospy
import os

"""
string_cmd = "gnome-terminal -x bash -c 'source ~/ROS_projects/icra2018/devel/setup.bash; rosrun distributed Algorithm_1.py 0'"
print '\n\n\n\n'
print 'Here is string_cmd: ', string_cmd1
print '\n\n\n\n'
"""

# Initial function
if __name__ == '__main__':



    #"""
    #string_cmd = "gnome-terminal -x bash -c 'source ~/ROS_projects/coordination_milp/devel/setup.bash; rosrun sim_gazebo_ds control_ds_rob2.py'"
    #string_cmd = "gnome-terminal -x bash -c 'source ~/ROS_projects/icra2018/devel/setup.bash; rosrun distributed Algorithm_1.py 0'"
    string_cmd = "gnome-terminal -x bash -c 'rosrun distributed Algorithm_1.py 0'"
    os.system(string_cmd)
    string_cmd = "gnome-terminal -x bash -c 'rosrun distributed Algorithm_1.py 1'"
    os.system(string_cmd)
    #"""

    """
    string_cmd = "rosrun distributed Algorithm_1.py 0"
    os.system(string_cmd)
    string_cmd = "rosrun distributed Algorithm_1.py 1"
    os.system(string_cmd)
    """

