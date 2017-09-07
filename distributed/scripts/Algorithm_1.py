#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from distributed.msg import History, HistList, Broadcast, Intlist
from std_msgs.msg import Int16
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from random import randrange
from time import sleep
import tf
import scipy.io
import pylab
import rospkg
import sys
import os

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
#global Vd
#Vd = 0.5
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

    global replan_tasks, finish_edge, id

    # This function recieve a message from the centralized sensor simulator

    if data.comGraphEvent == True:
        replan_tasks = True
        finish_edge = False
        list_of_H = data.listOfH
        list_of_robs = data.robList
        list_of_vels = data.velocityList

        """
        print 'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa'
        print 'Here is H_'+str(id)+' (received from sensor_simulator):'
        print list_of_H[id]
        print 'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa'
        """

        print '\n          ---------- Communication graph event received ----------\n'
        print 'Finishing Edge ...'

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

def callback_new_plan(data):


    global Hole_path
    global id, H
    global replan_tasks
    global waitting_new_plan
    global new_task
    global pose, original_graph

    if id in data.destinations:

        Hole_path = list(data.new_Hole_paths[id].data)
        # ISSO EH UM MIGUE  ----------  ----------  ----------
        [current_node, dist] = myLib.get_current_node(original_graph, [pose[0], pose[1]])
        current_node = current_node + 1
        if current_node != Hole_path[0]:
            aux = Hole_path
            Hole_path = [current_node]
            Hole_path = Hole_path + aux
        # ----------  ----------  ----------  ----------
        #print 'Here is Hole_path:'
        #print Hole_path
        Hmsg = data.listOfH[id]
        #print 'Here is Hmsg:'
        #print Hmsg
        H['e_v'] = list(Hmsg.e_v)
        H['e_uv'] = list(Hmsg.e_uv)
        H['e_g'] = list(Hmsg.e_g)
        H['T_a'] = list(Hmsg.T_a)
        H['T_f'] = list(Hmsg.T_f)
        H['lastMeeting'] = list(Hmsg.lastMeeting)
        # print 'Here is new H:'
        # print H

        print 'Here is list(Hmsg.lastMeeting):'
        print list(Hmsg.lastMeeting)

        print '\n     ----- New plan received -----'




        replan_tasks = False
        waitting_new_plan = False
        new_task = 1



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
    global replan_tasks, finish_edge
    global H
    global list_of_H, list_of_robs, list_of_vels
    global Hole_path
    global waitting_new_plan
    global new_task
    global original_graph

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
    #print 'my_str = ', my_str
    rospy.init_node(my_str)
    my_str = '/robot_' + str(id) + '/base_pose_ground_truth'
    rospy.Subscriber(my_str, Odometry, callback_pose)
    my_str = '/robot_' + str(id) + '/base_scan'
    rospy.Subscriber(my_str, LaserScan, callback_laser)
    rospy.Subscriber("/comm_graph", HistList, callback_comm_graph)

    #Read/Write in the topic of new plan
    pub_broadcast = rospy.Publisher('/new_path_topic', Broadcast, queue_size=1)
    rospy.Subscriber('/new_path_topic',Broadcast,callback_new_plan)

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

    """
    if(id == 0):
        Hole_path = [1, 2, 14, 13, 12, 10, 11, 9, 8, 7, 24, 25, 26, 27, 3]
        Hole_path = [1, 2, 3, 4, 5, 6, 1, 2, 14, 13, 12, 10, 11, 9, 8, 7, 24, 25, 26, 27, 3]
    elif(id == 1):
        Hole_path = [30, 28, 24, 23, 22, 21, 32, 17, 3]
        Hole_path = [30, 28, 24, 25, 26, 27, 30, 28, 24, 23, 22, 21, 32, 17, 3]
    """

    # ----------  ----------  ----------  ----------


    time_start = 0
    T = -2 #"initial flag value"
    pathNode, cx, cy, p, signal, new_task, new_path = [],[],[],[],[],[],[]
    edge = -1
    waitting_new_plan = False

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


        #print 'a'



        #print 'replan_tasks = ', replan_tasks


        count = count + 1

        time = count / float(freq)

        #print '\npathNode = ', pathNode, '\n'

        if not waitting_new_plan:
            if not replan_tasks:
                # Keep moving through the current edge
                [H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge] = myLib.keep_moving(H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, original_graph, freq, pose, laserVec, d, Vd, Kp, id, edge)
            else:
                # Keep moving just to complete the edge
                if(not finish_edge):
                    [H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge] = myLib.keep_moving(H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, original_graph, freq, pose, laserVec, d, Vd, Kp, id, edge)
                    if change_edge:
                        finish_edge = True
                        print '\nEdge finished\n'
                        vel.linear.x, vel.angular.z = 0, 0
                        pub_stage.publish(vel)
                else:
                    # CALL THE REPLANNING TASK HERE
                    if (id == min(list_of_robs)):
                        #Compute the replanning
                        vel.linear.x, vel.angular.z = 0, 0
                        pub_stage.publish(vel)
                        #print 'list_of_H:'
                        #print list_of_H
                        #print 'list_of_robs:'
                        #print list_of_robs
                        #print 'list_of_vels:'
                        #print list_of_vels
                        change_plan, Hole_path_list, new_Hists = Alg2.replanning(original_graph,virtual_graph,list_of_H, list_of_robs, list_of_vels)
                        #pylab.close("all")

                        #"""
                        #Broadcast to other robots
                        print '\nCreating Broadcast message ...\n'
                        B = Broadcast()
                        B.sender = id
                        B.destinations = [0,1]
                        IL = Intlist()
                        #print 'Here is Hole_path_list[0]'
                        #print Hole_path_list[0]
                        IL.data = list(Hole_path_list[0])
                        #print 'Here is IL'
                        #print IL
                        #print 'Here is type(IL)'
                        #print type(IL)
                        B.new_Hole_paths.append(IL)
                        IL = Intlist()
                        #print 'Here is Hole_path_list[0]'
                        #print Hole_path_list[0]
                        IL.data = list(Hole_path_list[1])
                        #print 'Here is IL'
                        #print IL
                        B.new_Hole_paths.append(IL)
                        #B.new_Hole_paths = Hole_path_list
                        B.listOfH = new_Hists

                        print '\nHere is B:'
                        print B
                        #print '\n'
                        #"""


                        waitting_new_plan = True
                        pub_broadcast.publish(B)
                        print 'New plan broadcasted'


                        #sleep(0.5)

                        #end_flag = True

                    else:
                        #Wait for the new result - HOW????


                        #list_of_H = []
                        VX, WZ = 0, 0
                        vel.linear.x, vel.angular.z = 0, 0
                        pub_stage.publish(vel)
                        #end_flag = True
                        waitting_new_plan = True


            """
            if end_flag or len(H['e_uv'])==0:
                vel.linear.x, vel.angular.z = 0, 0
                pub_stage.publish(vel)
                break
            """
            if end_flag:
                vel.linear.x, vel.angular.z = 0, 0
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
            Hmsg = History()
            #Hmsg.e_v, Hmsg.e_uv, Hmsg.e_g, Hmsg.T_a, Hmsg.T_f, Hmsg.currEdge, Hmsg.pose = H['e_v'], H['e_uv'], H['e_g'], H['T_a'], H['T_f'], edge, pose
            #Hmsg.e_v, Hmsg.e_uv, Hmsg.e_g, Hmsg.T_a, Hmsg.T_f, Hmsg.currEdge, Hmsg.pose = H['e_v'], H['e_uv'], H['e_g'], H['T_a'], H['T_f'], Hole_path[0], pose
            Hmsg.e_v, Hmsg.e_uv, Hmsg.e_g, Hmsg.T_a, Hmsg.T_f, Hmsg.currEdge, Hmsg.nextNode, Hmsg.pose, Hmsg.lastMeeting = H['e_v'], H['e_uv'], H['e_g'], H['T_a'], H['T_f'], edge, pathNode[0], pose, H['lastMeeting']
            pub_hist.publish(Hmsg)

            #Wait
            rate.sleep()

        else:
            #Wait
            rate.sleep()



    else: # else while
        rate.sleep()
        vel.linear.x = 0
        vel.angular.z = 0
        pub_stage.publish(vel)

        #break


# ---------- !! ---------- !! ---------- !! ---------- !! ----------









# Initial function
if __name__ == '__main__':



    print '__________ __________ __________ __________ __________ __________ __________'
    print '\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n'

    global id
    global number_robots #Used only foor creating the topics
    global H
    global Vd
    number_robots = 2

    if len(sys.argv) < 2:
        print 'ERROR!!!\nThe robot id was not provided'
    else:
        id = int(sys.argv[1])
        #id = int(id)


    if id == 0:
        Vd = 0.40
    elif id == 1:
        Vd = 0.55


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
    lastMeeting = [] #list of who was in the communicatio graph in the last meeting
    H = {'e_v': e_v, 'e_uv': e_uv, 'e_g': e_g, 'T_a': T_a, 'T_f': T_f, 'lastMeeting': lastMeeting}


    virtual_graph = myLib.read_graph('Virtual_graph_36.mat')



    try:
        Algorithm_1()
    except rospy.ROSInterruptException:
        pass


    """
    if rospy.is_shutdown:
        string_cmd = "gnome-terminal -x bash -c 'killall python'"
        os.system(string_cmd)
    """



    file_path.close()
    file_vel.close()
