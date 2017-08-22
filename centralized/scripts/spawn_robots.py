#!/usr/bin/env python
import rospy
import os

















# Initial function
if __name__ == '__main__':


    string_cmd = "gnome-terminal -x bash -c 'rosrun centralized follow_path_g.py 0'"
    os.system(string_cmd)
    string_cmd = "gnome-terminal -x bash -c 'rosrun centralized follow_path_g.py 1'"
    os.system(string_cmd)

    """
    string_cmd = "gnome-terminal -x bash -c 'xterm -e rosrun centralized follow_path_g.py 0'"
    os.system(string_cmd)
    string_cmd = "gnome-terminal -x bash -c 'xterm -e rosrun centralized follow_path_g.py 1'"
    os.system(string_cmd)
    """
