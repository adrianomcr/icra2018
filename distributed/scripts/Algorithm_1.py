#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from distributed.msg import History, Intlist, HistList
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from random import randrange
from time import sleep
import tf
import scipy.io
import rospkg
import sys

rp = rospkg.RosPack()
path = rp.get_path('distributed')
path = path + '/CPP'
sys.path.insert(0, path)
import cppsolver
import library2018 as myLib
import Algorithm_2 as Alg2



# This script make a robot to follow a path through a graph



global pose
pose = [0.1, 0.2, 0.001] # [x, y, theta]

global d  # for feedback linearization
d = 0.07

global Vd
Vd = 0.25

global time
time = 0

global Kp  # Proportional gain
Kp = 0.8

global laserVec
laserVec = [1 for i in range(270)]

global detected_pose
detected_pose = 0

#flag to indicate that the routes should be replanned
global replan_tasks
replan_tasks = False

#Lists that carries the history of the robots
global e_v, e_uv, e_g
global T_a, T_f
global H

# Callback routine to obtain the robot's pose
def callback_pose(data):
    #global x_n, y_n, theta_n
    global file_path, time
    global detected_pose

    x_n = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_n = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta_n = euler[2]  # orientaco 'theta' do robo no mundo

    if not file_path.closed:
        mystr = str(x_n) + "\t" + str(y_n) + "\t" + str(theta_n) + "\t" + str(time) + "\n"
        file_path.write(mystr)

    pose[0] = x_n
    pose[1] = y_n
    pose[2] = theta_n

    detected_pose = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the laser information
def callback_laser(data):
    global laserVec

    laserVec = data.ranges

    return
# ----------  ----------  ----------  ----------  ----------


# Callback routine to obtain data from another robot
def callback_comm_graph(data):

    #global id
    global list_of_H, list_of_robs, list_of_vels

    global replan_tasks

    # This function recieve a message from the centralized sensor simulator

    if data.comGraphEvent == True:
        replan_tasks = True
        list_of_H = data.listOfH
        list_of_robs = data.robList
        list_of_vels = data.velocityList

        """
        print "\nHere is robList:"
        print data.robList
        print "\nHere is velocityList:"
        print data.velocityList
        print "\nHere is listOfH[0]:"
        print data.listOfH[0]
        print "\nHere is listOfH[1]:"
        print data.listOfH[1]
        """

    return
# ----------  ----------  ----------  ----------  ----------







def receive_new_plan():
    return
# ----------  ----------  ----------  ----------  ----------







# Primary routine
def Algorithm_1():
    #global x_n, y_n, theta_n
    global pose
    global pub_rviz, pub_targ, pub_pose
    global freq
    global time, i
    global file_vel
    global N
    global n, nodes, C, PathM, w_s, PolC
    global Vd
    global laserVec
    global id
    global replan_tasks
    global H
    global list_of_H, list_of_robs, list_of_vels

    Hmsg = History()
    vel = Twist()

    count = 0

    #Define the node, publisher and subscribers
    my_str = '/robot_' + str(id) + '/cmd_vel'
    pub_stage = rospy.Publisher(my_str, Twist, queue_size=1)
    my_str = '/robot_' + str(id) + '/history'
    pub_hist = rospy.Publisher(my_str, History, queue_size=1)
    my_str = 'follow_graph_' + str(id)
    rospy.init_node(my_str)
    my_str = '/robot_' + str(id) + '/base_pose_ground_truth'
    rospy.Subscriber(my_str, Odometry, callback_pose)
    my_str = '/robot_' + str(id) + '/base_scan'
    rospy.Subscriber(my_str, LaserScan, callback_laser)
    rospy.Subscriber("/comm_graph", HistList, callback_comm_graph)

    freq = 10.0  # Hz
    rate = rospy.Rate(freq)

    # Running CPP in order to find a initial route
    while detected_pose == 0:
        rate.sleep()
    [current_node, dist] = myLib.get_current_node(original_graph, [pose[0], pose[1]])
    current_node = current_node + 1
    active_edges = range(1, len(PolC[0]) + 1)
    edges_listOfTuples = myLib.write_listOfTuples(original_graph, active_edges)
    Hole_path = cppsolver.CPP(edges_listOfTuples, current_node)

    #"""
    if(id == 0):
        Hole_path = [1, 2, 14, 13, 12, 10, 11, 9, 8, 7, 24,25,26,27,3]
    elif(id == 1):
        Hole_path = [30,28,24,23,22,21,32,17,3]
    #"""

    # ----------  ----------  ----------  ----------


    time_start = 0
    T = -2 #"initial flag value"
    pathNode, cx, cy, p, signal, new_task, new_path = [],[],[],[],[],[],[]
    edge = -1

    """
    LH = HistList()
    LH.robot_list.append(0)
    LH.robot_list.append(1)
    LH.robot_list.append(2)
    LH.listOfH.append(Hmsg)
    LH.listOfH.append(Hmsg)
    print '\nHere is LH:'
    print LH
    print '\nHere is LH.robot_list:'
    print LH.robot_list
    print '\nHere is LH.listOfH:'
    print LH.listOfH
    print '\nHere is len(LH.listOfH):'
    print len(LH.listOfH)
    print '\n'
    """



    while not rospy.is_shutdown():

        count = count + 1

        time = count / float(freq)

        #Keep moving through the edge
        if not replan_tasks:
            [H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge] = myLib.keep_moving(H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, original_graph, freq, pose, laserVec, d, Vd, Kp, id, edge)
        else:
            #CALL THE REPLANNING TASK HERE
            if (id == min(list_of_robs)):
                #Compute the replanning
                vel.linear.x, vel.angular.z = 0, 0
                pub_stage.publish(vel)
                Alg2.replanning(original_graph,virtual_graph,list_of_H, list_of_robs, list_of_vels)
                print 'Returnned from Alg2.replanning()'
                end_flag = True
            else:
                #Wait for the new result - HOW????
                list_of_H = []
                VX, WZ = 0, 0
                end_flag = True


        if end_flag:
            pub_stage.publish(vel)
            break




        #Publish speed
        vel.linear.x, vel.angular.z = VX, WZ
        pub_stage.publish(vel)


        #Write to file
        if not file_vel.closed:
            mystr = str(VX) + "\t" + str(WZ) + "\t" + "\t" + str(time) + "\n"
            file_vel.write(mystr)

        #Publish the history
        Hmsg.e_v, Hmsg.e_uv, Hmsg.e_g, Hmsg.T_a, Hmsg.T_f, Hmsg.currEdge = H['e_v'], H['e_uv'], H['e_g'], H['T_a'], H['T_f'], edge+1
        pub_hist.publish(Hmsg)

        #Wait
        rate.sleep()


    else: # else while
        rate.sleep()
        vel.linear.x = 0
        vel.angular.z = 0
        pub_stage.publish(vel)


# ---------- !! ---------- !! ---------- !! ---------- !! ----------









# Initial function
if __name__ == '__main__':

    global id
    global number_robots #Used only foor creating the topics
    global H
    number_robots = 2

    if len(sys.argv) < 2:
        print 'ERROR!!!\nThe robot id was not provided'
    else:
        id = int(sys.argv[1])
        #id = int(id)


    rp = rospkg.RosPack()
    path = rp.get_path('distributed')
    path = path + '/text/path_robot' + str(id) + '.txt'
    file_path = open(path, 'w')
    path = rp.get_path('distributed')
    path = path + '/text/vel_robot' + str(id) + '.txt'
    file_vel = open(path, 'w')

    original_graph = myLib.read_graph('Original_graph_36.mat')
    n = original_graph['n']
    nodes = original_graph['nodes']
    C = original_graph['C']
    PathM = original_graph['PathM']
    w_s = original_graph['w_s']
    PolC = original_graph['PolC']

    #Initialization of the history of each robot
    e_v = [] # visited
    e_uv = range(1,len(PolC[0])+1) # unvisited (must be visited)
    e_g = [] # assigned to other robots
    T_a = [] # list of list of robots assigned to an edge
    T_f = [] # list of list of robots forbidden to visit an edge
    H = {'e_v': e_v, 'e_uv': e_uv, 'e_g': e_g, 'T_a': T_a, 'T_f': T_f}


    virtual_graph = myLib.read_graph('Virtual_graph_36.mat')



    try:
        Algorithm_1()
    except rospy.ROSInterruptException:
        pass



    file_path.close()
    file_vel.close()
