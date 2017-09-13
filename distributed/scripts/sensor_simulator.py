#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from distributed.msg import History, HistList, Broadcast, Intlist
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from random import randrange
from time import sleep
import tf
import scipy.io
import rospkg




global pose_0
pose_0 = [-10, 20, 0] #[x ,y, theta]
global pose_1
pose_1 = [20, -10, 0] #[x ,y, theta]
global pose_2
pose_2 = [-10, -10, 0] #[x ,y, theta]
global pose_3
pose_3 = [20, 20, 0] #[x ,y, theta]


global rho # Radius of communication
rho = 4
global DIST_INTO # Radius to enter in the communication graph
global DIST_LEAVE # Radius to leave the communication graph
DIST_INTO = 2.5
DIST_LEAVE = 5.5

global flag_0, flag_1, flag_3
flag_0 = 0
flag_1 = 0
flag_2 = 0
flag_3 = 0


# Callback routine to obtain the pose of robot 0
def callback_pose_0(data):

    global pose_0
    global flag_0

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    pose_0[0] = x
    pose_0[1] = y
    pose_0[2] = theta

    flag_0 = 1

    return
# ----------  ----------  ----------  ----------  ----------
# Callback routine to obtain the pose of robot 1
def callback_pose_1(data):

    global pose_1
    global flag_1

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    pose_1[0] = x
    pose_1[1] = y
    pose_1[2] = theta

    flag_1 = 1

    return
# ----------  ----------  ----------  ----------  ----------
# Callback routine to obtain the pose of robot 2
def callback_pose_2(data):

    global pose_2
    global flag_2

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    pose_2[0] = x
    pose_2[1] = y
    pose_2[2] = theta

    flag_2 = 1

    return
# ----------  ----------  ----------  ----------  ----------
# Callback routine to obtain the pose of robot 2
def callback_pose_3(data):

    global pose_3
    global flag_3

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    pose_3[0] = x
    pose_3[1] = y
    pose_3[2] = theta

    flag_3 = 1

    return
# ----------  ----------  ----------  ----------  ----------



# Callback routine to obtain the history of robot 0
def callback_hist_0(data):
    global H_0

    H_0['id'] = data.id
    H_0['specs'] = data.specs
    H_0['e_v'] = data.e_v
    H_0['e_uv'] = data.e_uv
    H_0['e_g'] = data.e_g
    H_0['T_a'] = data.T_a
    H_0['T_f'] = data.T_f
    H_0['currEdge'] = data.currEdge
    H_0['nextNode'] = data.nextNode
    H_0['pose'] = data.pose
    H_0['lastMeeting'] = list(data.lastMeeting)

    return
# ----------  ----------  ----------  ----------  ----------
# Callback routine to obtain the history of robot 1
def callback_hist_1(data):
    global H_1

    H_1['id'] = data.id
    H_1['specs'] = data.specs
    H_1['e_v'] = data.e_v
    H_1['e_uv'] = data.e_uv
    H_1['e_g'] = data.e_g
    H_1['T_a'] = data.T_a
    H_1['T_f'] = data.T_f
    H_1['currEdge'] = data.currEdge
    H_1['nextNode'] = data.nextNode
    H_1['pose'] = data.pose
    H_1['lastMeeting'] = list(data.lastMeeting)

    return
# ----------  ----------  ----------  ----------  ----------
# Callback routine to obtain the history of robot 2
def callback_hist_2(data):
    global H_2

    H_2['id'] = data.id
    H_2['specs'] = data.specs
    H_2['e_v'] = data.e_v
    H_2['e_uv'] = data.e_uv
    H_2['e_g'] = data.e_g
    H_2['T_a'] = data.T_a
    H_2['T_f'] = data.T_f
    H_2['currEdge'] = data.currEdge
    H_2['nextNode'] = data.nextNode
    H_2['pose'] = data.pose
    H_2['lastMeeting'] = list(data.lastMeeting)

    return
# ----------  ----------  ----------  ----------  ----------
# Callback routine to obtain the history of robot 3
def callback_hist_3(data):
    global H_3

    H_3['id'] = data.id
    H_3['specs'] = data.specs
    H_3['e_v'] = data.e_v
    H_3['e_uv'] = data.e_uv
    H_3['e_g'] = data.e_g
    H_3['T_a'] = data.T_a
    H_3['T_f'] = data.T_f
    H_3['currEdge'] = data.currEdge
    H_3['nextNode'] = data.nextNode
    H_3['pose'] = data.pose
    H_3['lastMeeting'] = list(data.lastMeeting)

    return
# ----------  ----------  ----------  ----------  ----------






def send_message_recompute_close(pub_comm_graph, ids):

    global H_0, H_1, H_2, H_3, HL
    global meeting_matrix

    if ids == [0,1]:
        H_A = H_0
        H_B = H_1
    elif ids == [0, 2]:
        H_A = H_0
        H_B = H_2
    elif ids == [0, 3]:
        H_A = H_0
        H_B = H_3
    elif ids == [1, 2]:
        H_A = H_1
        H_B = H_2
    elif ids == [1, 3]:
        H_A = H_1
        H_B = H_3
    elif ids == [2, 3]:
        H_A = H_2
        H_B = H_3




    #if (H_0['lastMeeting'] != H_1['lastMeeting'] or H_0['lastMeeting'] == []):  # Check if there is new information
    if (H_A['lastMeeting'] != H_B['lastMeeting'] or H_A['lastMeeting'] == []):  # Check if there is new information
        HL = HistList()

        HL.comGraphEvent = True
        meeting_matrix[ids[0]][ids[1]] = True

        # Adding information from robot 0
        H = History()
        H.id = H_A['id']
        H.specs = H_A['specs']
        H.e_v = H_A['e_v']
        H.e_uv = H_A['e_uv']
        H.e_g = H_A['e_g']
        H.T_a = H_A['T_a']
        H.T_f = H_A['T_f']
        H.currEdge = H_A['currEdge']
        H.nextNode = H_A['nextNode']
        H.pose = H_A['pose']
        H.lastMeeting = H_A['lastMeeting']
        HL.robList.append(H_A['id'])
        # HL.velocityList.append(0.4)
        HL.listOfH.append(H)

        # Adding information from robot 1
        H = History()
        H.id = H_B['id']
        H.specs = H_B['specs']
        H.e_v = H_B['e_v']
        H.e_uv = H_B['e_uv']
        H.e_g = H_B['e_g']
        H.T_a = H_B['T_a']
        H.T_f = H_B['T_f']
        H.currEdge = H_B['currEdge']
        H.nextNode = H_B['nextNode']
        H.pose = H_B['pose']
        H.lastMeeting = H_B['lastMeeting']
        HL.robList.append(H_B['id'])
        # HL.velocityList.append(0.55)
        HL.listOfH.append(H)

        pub_comm_graph.publish(HL)


    else:
        HL.comGraphEvent = True
        meeting_matrix[ids[0]][ids[1]] = True
        print 'Meeting happen but there is no new information'


    return


# ----------  ----------  ----------  ----------  ----------




# Primary routine
def sensor_simulator():

    global pose_0, pose_1, pose_2, pose_3
    global rho
    global H_0, H_1, H_2, H_3, HL
    global meeting_matrix

    H_0 = {'id': [],
         'specs': [],
         'e_v': [],
         'e_uv': [],
         'e_g': [],
         'T_a': [],
         'T_f': [],
         'currEdge': None,
         'lastMeeting': []
    }
    H_1 = {'id': [],
         'specs': [],
         'e_v': [],
         'e_uv': [],
         'e_g': [],
         'T_a': [],
         'T_f': [],
         'currEdge': None,
         'lastMeeting': []
    }
    H_2 = {'id': [],
         'specs': [],
         'e_v': [],
         'e_uv': [],
         'e_g': [],
         'T_a': [],
         'T_f': [],
         'currEdge': None,
         'lastMeeting': []
    }
    H_3 = {'id': [],
         'specs': [],
         'e_v': [],
         'e_uv': [],
         'e_g': [],
         'T_a': [],
         'T_f': [],
         'currEdge': None,
         'lastMeeting': []
    }



    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callback_pose_0)
    rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, callback_pose_1)
    rospy.Subscriber("/robot_2/base_pose_ground_truth", Odometry, callback_pose_2)
    rospy.Subscriber("/robot_3/base_pose_ground_truth", Odometry, callback_pose_3)
    rospy.Subscriber("/robot_0/history", History, callback_hist_0)
    rospy.Subscriber("/robot_1/history", History, callback_hist_1)
    rospy.Subscriber("/robot_2/history", History, callback_hist_2)
    rospy.Subscriber("/robot_3/history", History, callback_hist_3)
    rospy.init_node("sensor_similator")
    pub_comm_graph = rospy.Publisher("/comm_graph", HistList, queue_size=1)


    close_var = Bool()
    close_var.data = False
    HL = HistList()
    HL.comGraphEvent = False

    #Used to implement the "meeting trigger"
    #meeting_matrix = [[False,False,False],[False,False,False],[False,False,False]]
    meeting_matrix = [[False,False,False,False],[False,False,False,False],[False,False,False,False],[False,False,False,False]]


    count = 0
    freq = 4.0  # Hz --> smaller than the frequency of the robots
    rate = rospy.Rate(freq)



    # Wait until a first measurement is made
    #while flag_0 == 0 or flag_1 == 0 or flag_2 == 0:
    while flag_0 == 0 or flag_1 == 0:
        rate.sleep()

    """
    global mutex
    mutex = -1
    """
    global mutex0, mutex1, mutex2, mutex3
    mutex0, mutex1, mutex2, mutex3 = -1, -1, -1, -1

    #global conut0, conut1, conut3, conut3
    #conut0, conut1, conut2, conut3 = 0, 0, 0, 0
    global count01,count02,count03, count12,count13, count23
    count01,count02,count03, count12,count13, count23 = 0, 0, 0, 0, 0, 0

    sleep(0.1)

    while (not rospy.is_shutdown()):

        count = count + 1
        time = count / float(freq)

        """
        if mutex > 0 and time-mutex>15:
            mutex = -1
            print 'Mutex = -1', time
        """
        if mutex0 > 0 and time - mutex0 > 20:
            mutex0 = -1
            print 'Mutex0 = -1', time

        if mutex1 > 0 and time - mutex1 > 20:
            mutex1 = -1
            print 'Mutex1 = -1', time

        if mutex2 > 0 and time - mutex2 > 20:
            mutex2 = -1
            print 'Mutex2 = -1', time

        if mutex3 > 0 and time - mutex3 > 20:
            mutex3 = -1
            print 'Mutex3 = -1', time




        dist_0_1 = ((pose_0[0] - pose_1[0]) ** 2 + (pose_0[1] - pose_1[1]) ** 2) ** 0.5
        dist_0_2 = ((pose_0[0] - pose_2[0]) ** 2 + (pose_0[1] - pose_2[1]) ** 2) ** 0.5
        dist_0_3 = ((pose_0[0] - pose_3[0]) ** 2 + (pose_0[1] - pose_3[1]) ** 2) ** 0.5
        dist_1_2 = ((pose_1[0] - pose_2[0]) ** 2 + (pose_1[1] - pose_2[1]) ** 2) ** 0.5
        dist_1_3 = ((pose_1[0] - pose_3[0]) ** 2 + (pose_1[1] - pose_3[1]) ** 2) ** 0.5
        dist_2_3 = ((pose_2[0] - pose_3[0]) ** 2 + (pose_2[1] - pose_3[1]) ** 2) ** 0.5


        ids = [0, 1]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_0_1 < DIST_INTO and mutex0 < 0 and mutex1 < 0 and count01<2:
            send_message_recompute_close(pub_comm_graph, ids)
            #print 'Mutex = +1', time
            mutex0 = time
            mutex1 = time
            count01 = count01+1
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_0_1 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        ids = [0, 2]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_0_2 < DIST_INTO and mutex0 < 0 and mutex2 < 0 and count02<2:
            send_message_recompute_close(pub_comm_graph, ids)
            #print 'Mutex = +1', time
            mutex0 = time
            mutex2 = time
            count02 = count02 + 1
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_0_2 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        ids = [0, 3]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_0_3 < DIST_INTO and mutex0 < 0 and mutex3 < 0 and count03<2:
            send_message_recompute_close(pub_comm_graph, ids)
            #print 'Mutex = +1', time
            mutex0 = time
            mutex3 = time
            count03 = count03 + 1
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_0_3 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        ids = [1, 2]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_1_2 < DIST_INTO and mutex1 < 0 and mutex2 < 0 and count12<2:
            send_message_recompute_close(pub_comm_graph, ids)
            #print 'Mutex = +1', time
            mutex1 = time
            mutex2 = time
            count12 = count12 + 1
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_1_2 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        ids = [1, 3]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_1_3 < DIST_INTO and mutex1 < 0 and mutex3 < 0 and count13<2:
            send_message_recompute_close(pub_comm_graph, ids)
            #print 'Mutex = +1', time
            mutex1 = time
            mutex3 = time
            count13 = count13 + 1
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_1_3 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        ids = [2, 3]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_2_3 < DIST_INTO and mutex2 < 0 and mutex3 < 0 and count23<2:
            send_message_recompute_close(pub_comm_graph, ids)
            #print 'Mutex = +1', time
            mutex2 = time
            mutex3 = time
            count23 = count23 + 1
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_2_3 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        """
        ids = [0, 1]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_0_1 < DIST_INTO and mutex < 0:
            send_message_recompute_close(pub_comm_graph, ids)
            print 'Mutex = +1', time
            mutex = time
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_0_1 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        ids = [0, 2]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_0_2 < DIST_INTO and mutex < 0:
            send_message_recompute_close(pub_comm_graph, ids)
            print 'Mutex = +1', time
            mutex = time
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_0_2 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        ids = [0, 3]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_0_3 < DIST_INTO and mutex < 0:
            send_message_recompute_close(pub_comm_graph, ids)
            print 'Mutex = +1', time
            mutex = time
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_0_3 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        ids = [1, 2]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_1_2 < DIST_INTO and mutex < 0:
            send_message_recompute_close(pub_comm_graph, ids)
            print 'Mutex = +1', time
            mutex = time
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_1_2 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        ids = [1, 3]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_1_3 < DIST_INTO and mutex < 0:
            send_message_recompute_close(pub_comm_graph, ids)
            print 'Mutex = +1', time
            mutex = time
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_1_3 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False

        ids = [2, 3]
        if meeting_matrix[ids[0]][ids[1]] == False and dist_2_3 < DIST_INTO and mutex < 0:
            send_message_recompute_close(pub_comm_graph, ids)
            print 'Mutex = +1', time
            mutex = time
        elif meeting_matrix[ids[0]][ids[1]] == True and dist_2_3 > DIST_LEAVE:
            meeting_matrix[ids[0]][ids[1]] = False
        """




        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------









# Initial function
if __name__ == '__main__':




    try:
        sensor_simulator()
    except rospy.ROSInterruptException:
        pass

