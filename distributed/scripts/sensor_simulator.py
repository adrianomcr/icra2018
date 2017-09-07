#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from distributed.msg import History, HistList
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from random import randrange
from time import sleep
import tf
import scipy.io
import rospkg




global pose_0
pose_0 = [0, 0, 0] #[x ,y, theta]
global pose_1
pose_1 = [0, 0, 0] #[x ,y, theta]


global rho # Radius of communication
rho = 4
global DIST_INTO # Radius to enter in the communication graph
global DIST_LEAVE # Radius to leave the communication graph
DIST_INTO = 2.5
DIST_LEAVE = 5.5

global flag_0, flag_1
flag_0 = 0
flag_1 = 0


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




# Callback routine to obtain the history of robot 0
def callback_hist_0(data):
    global H_0

    H_0['e_v'] = data.e_v
    H_0['e_uv'] = data.e_uv
    H_0['e_g'] = data.e_g
    H_0['T_a'] = data.T_a
    H_0['T_f'] = data.T_f
    H_0['currEdge'] = data.currEdge
    H_0['nextNode'] = data.nextNode
    H_0['pose'] = data.pose
    H_0['lastMeeting'] = list(data.lastMeeting)



    #print 'H_0 received'

    return


# ----------  ----------  ----------  ----------  ----------
# Callback routine to obtain the history of robot 1
def callback_hist_1(data):
    global H_1

    H_1['e_v'] = data.e_v
    H_1['e_uv'] = data.e_uv
    H_1['e_g'] = data.e_g
    H_1['T_a'] = data.T_a
    H_1['T_f'] = data.T_f
    H_1['currEdge'] = data.currEdge
    H_1['nextNode'] = data.nextNode
    H_1['pose'] = data.pose
    H_1['lastMeeting'] = list(data.lastMeeting)

    #print 'H_1 received'

    return


# ----------  ----------  ----------  ----------  ----------






def send_message_recompute_close(pub_comm_graph):

    global H_0, H_1, HL

    #print 'lastMeeting robot 0:', H_0['lastMeeting']
    #print 'lastMeeting robot 1:', H_1['lastMeeting']

    #print 'H_0_last != H_1_last', H_0['lastMeeting'] != H_1['lastMeeting']


    if (H_0['lastMeeting'] != H_1['lastMeeting'] or H_0['lastMeeting'] == []):  # Check if there is new information
    #if (H_0['lastMeeting'] != H_1['lastMeeting'] or H_1['lastMeeting'] != []):  # Check if there is new information
        HL = HistList()

        HL.comGraphEvent = True

        #Adding information from robot 0
        H = History()
        H.e_v = H_0['e_v']
        H.e_uv = H_0['e_uv']
        H.e_g = H_0['e_g']
        H.T_a = H_0['T_a']
        H.T_f = H_0['T_f']
        H.currEdge = H_0['currEdge']
        H.nextNode = H_0['nextNode']
        H.pose = H_0['pose']
        H.lastMeeting = H_0['lastMeeting']
        HL.robList.append(0)
        HL.velocityList.append(0.4)
        HL.listOfH.append(H)


        # Adding information from robot 1
        H = History()
        H.e_v = H_1['e_v']
        H.e_uv = H_1['e_uv']
        H.e_g = H_1['e_g']
        H.T_a = H_1['T_a']
        H.T_f = H_1['T_f']
        H.currEdge = H_1['currEdge']
        H.nextNode = H_1['nextNode']
        H.pose = H_1['pose']
        H.lastMeeting = H_1['lastMeeting']
        HL.robList.append(1)
        HL.velocityList.append(0.55)
        HL.listOfH.append(H)

        pub_comm_graph.publish(HL)

        """
        print '\nHere is  H_0:'
        print H_0
        print '\nHere is  H_1:'
        print H_1
        """
    else:
        HL.comGraphEvent = True
        print 'Meeting happen but there is no new information'


    return


# ----------  ----------  ----------  ----------  ----------




# Primary routine
def sensor_simulator():

    global pose_0, pose_1
    global rho
    global H_0, H_1, HL

    H_0 = {'e_v': [],
         'e_uv': [],
         'e_g': [],
         'T_a': [],
         'T_f': [],
         'currEdge': None,
         'lastMeeting': []
    }
    H_1 = {'e_v': [],
         'e_uv': [],
         'e_g': [],
         'T_a': [],
         'T_f': [],
         'currEdge': None,
         'lastMeeting': []
    }



    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callback_pose_0)
    rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, callback_pose_1)
    rospy.Subscriber("/robot_0/history", History, callback_hist_0)
    rospy.Subscriber("/robot_1/history", History, callback_hist_1)
    rospy.init_node("sensor_similator")
    #pub_comm_graph = rospy.Publisher("/comm_graph", Bool, queue_size=1)
    pub_comm_graph = rospy.Publisher("/comm_graph", HistList, queue_size=1)


    close_var = Bool()
    close_var.data = False
    HL = HistList()
    HL.comGraphEvent = False


    count = 0
    freq = 4.0  # Hz --> smaller than the frequency of the robots
    rate = rospy.Rate(freq)



    # Wait until a first measurement is made
    while flag_0 == 0 or flag_1 == 0:
        rate.sleep()


    sleep(0.1)

    while (not rospy.is_shutdown()):

        count = count + 1

        time = count / float(freq)


        dist_0_1 = ((pose_0[0]-pose_1[0])**2 + (pose_0[1]-pose_1[1])**2)**0.5


        """
        print 'Here is H_0, (from the sensor simulator)'
        print H_0
        """


        """
        if dist_0_1 <= rho:
            #Send message to robots
            close_var.data = True
            pub_comm_graph.publish(close_var)
        """
        """
        if close_var.data == False and dist_0_1 < DIST_INTO:
            #close_var.data = True
            #pub_comm_graph.publish(close_var)
            send_message_recompute_close(pub_comm_graph)
        elif close_var.data == True and dist_0_1 > DIST_LEAVE:
            close_var.data = False
        """
        if HL.comGraphEvent == False and dist_0_1 < DIST_INTO:
            send_message_recompute_close(pub_comm_graph)
        elif HL.comGraphEvent == True and dist_0_1 > DIST_LEAVE:
            HL.comGraphEvent = False




        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------









# Initial function
if __name__ == '__main__':




    try:
        sensor_simulator()
    except rospy.ROSInterruptException:
        pass

