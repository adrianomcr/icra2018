#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
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
import libraries2018 as myLib

#This script make a robot to follow a path through a graph




global x_n, y_n, theta_n
x_n = 0.1  # posicao x atual do robo
y_n = 0.2  # posicao y atual do robo
theta_n = 0.001  # orientacao atual do robo

global d  # for feedback linearization
d = 0.07

global Vd
Vd = 0.25

global time
time = 0

global Kp #Proportional gain
Kp = 0.8

global laserVec
laserVec = [1 for i in range(270)]


global detected_pose
detected_pose = 0



# Callback routine to obtain the robot's pose
def callback_pose(data):
    global x_n, y_n, theta_n
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

    mystr = str(x_n) + "\t" + str(y_n) + "\t" + str(theta_n) + "\t" + str(time) + "\n"

    file_path.write(mystr)

    detected_pose = 1

    return

# ----------  ----------  ----------  ----------  ----------


# Callback routine to obtain the laser information
def callback_laser(data):
    global laserVec
    global x_n, y_n, theta_n

    laserVec = data.ranges

    return

# ----------  ----------  ----------  ----------  ----------



# Feedback linearization
def feedback_linearization(Ux, Uy):
    global x_n, y_n, theta_n
    global d

    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy

    return (VX, WZ)


# ----------  ----------  ----------  ----------  ----------







def compute_velocity(cx, cy, p, T, dt, signal):
    global x_n, y_n, theta_n
    global Vd

    xref = 0
    yref = 0
    for k in range(6): #Compute refference position
        xref = xref + cx[6 - k - 1] * p ** k
        yref = yref + cy[6 - k - 1] * p ** k
    vx = 0
    vy = 0
    for k in range(1,6): #Compute refference velocity (diect evaluatioin of the polynimial)
        vx = vx + (k) * cx[6 - k - 1] * p ** (k - 1)
        vy = vy + (k) * cy[6 - k - 1] * p ** (k - 1)

    #Atualization of p such as the speed is constant
    v = sqrt((vx)**2+(vy)**2)
    p = p + ((dt) * (Vd / v))*signal

    #Compuete the speeds 'vx' and 'vy' for the feed forward such the resultant speed is constant
    vx_ff = (Vd * (vx / v)) * signal
    vy_ff = (Vd * (vy / v)) * signal

    #Compute the controller signals
    ux = Kp * (xref - x_n) + vx_ff
    uy = Kp * (yref - y_n) + vy_ff

    return (ux, uy, p)













# Primary routine
def follow_graph():
    global x_n, y_n, theta_n
    global pub_rviz, pub_targ, pub_pose
    global freq
    global time, i
    global file_vel
    global N
    global n, nodes, C, PathM, w_s, PolC
    global Vd
    global laserVec
    global id

    vel = Twist()

    count = 0

    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0

    my_str = '/robot_'+str(id)+'/cmd_vel'
    pub_stage = rospy.Publisher(my_str, Twist, queue_size=1)
    my_str = 'follow_graph_' + str(id)
    rospy.init_node(my_str)
    my_str = '/robot_' + str(id) + '/base_pose_ground_truth'
    rospy.Subscriber(my_str, Odometry, callback_pose)
    my_str = '/robot_' + str(id) + '/base_scan'
    rospy.Subscriber(my_str, LaserScan, callback_laser)

    freq = 10.0  # Hz
    rate = rospy.Rate(freq)


    #Running CPP in order to find a initial route
    while(detected_pose == 0):
        rate.sleep()
    [current_node, dist] = myLib.get_current_node(original_graph, [x_n,y_n])
    current_node = current_node+1
    active_edges = range(1,len(PolC[0])+1)
    edges_listOfTuples = myLib.write_listOfTuples(original_graph,active_edges)
    Hole_path = cppsolver.CPP(edges_listOfTuples,current_node)
    """
    print '\n\nAQUI ESTA Hole_path:'
    print Hole_path
    print 'current_node = ', current_node
    print '\n\n'
    """
    #a = a-w
    # ----------  ----------  ----------  ----------


    #----------
    """
    if (id == 0):
        Hole_path = [1, 2, 3, 4, 5, 6, 14, 13, 15, 33, 16, 17, 18, 19, 20, 32]
    elif (id == 1):
        Hole_path = [30, 28, 29, 21, 22, 31, 23, 24, 25, 26, 27, 7, 8, 36, 9, 35, 10, 11, 12, 34]
    else:
        Hole_path = [1]
    """

    time_start = 0

    i = Hole_path.pop(0)
    j = Hole_path[0]
    pathNode = myLib.getNodePath(i-1, j-1, PathM)

    i = pathNode.pop(0)
    j = pathNode[0]
    T = C[i][j] / Vd
    [edge, signal] = myLib.getEdge(i, j, PolC)
    [fr, to, cx, cy, cost] = myLib.getCoefs(edge, PolC)
    if (signal == 1):
        p = 0
    elif (signal == -1):
        p = 1
    T = C[i-1][j-1] / Vd

    new_task = 0
    new_path = 0
    # ----------

    sleep(0.1)

    while (not rospy.is_shutdown()):

        count = count + 1

        time = count / float(freq)

        print 'Robot ' + str(id)
        print 'Moving from i =', i, 'to', 'j =', j
        print 'time = ', time
        print 'task time: ', time - time_start, '/', T, '\n'


        if(time-time_start > T+1/freq):
            time_start = time
            if(len(pathNode) > 1):
                new_path = 1
            else:
                new_task = 1


        if(new_task == 1):
            if(len(Hole_path) > 1):
                i = Hole_path.pop(0)
                j = Hole_path[0]
                pathNode = myLib.getNodePath(i-1, j-1, PathM)

                new_task = 0
                new_path = 1
            else:
                print'\nNodes search completed\n'
                vel.linear.x = 0
                vel.angular.z = 0
                pub_stage.publish(vel)
                break
        if(new_path == 1):
            i = pathNode.pop(0)
            j = pathNode[0]
            T = C[i-1][j-1] / Vd
            [edge, signal] = myLib.getEdge(i,j,PolC)
            [fr, to, cx, cy, cost] = myLib.getCoefs(edge, PolC)
            if(signal == 1):
                p = 0
            elif(signal == -1):
                p = 1
            new_path = 0


        [ux, uy, p] = compute_velocity(cx, cy, p, T, 1/freq, signal)
        if(laserVec[135] < 0.15):
            vx_repulsive = -(0.05 / laserVec[135]) * cos(theta_n)
            vy_repulsive = -(0.05 / laserVec[135]) * sin(theta_n)
            ux = ux + vx_repulsive
            uy = ux + vy_repulsive
            print '                                       !!!!! Repulsive potential active !!!!!'
        [VX, WZ] = feedback_linearization(ux,uy)


        vel.linear.x = VX
        vel.angular.z = WZ

        pub_stage.publish(vel)


        mystr = str(VX) + "\t" + str(WZ) + "\t" + "\t" + str(time) + "\n"
        file_vel.write(mystr)

        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------









# Initial function
if __name__ == '__main__':

    global id

    #"""
    if len(sys.argv) < 2:
        print 'ERROR!!!'
        print 'The robot id was not provided'
        #print 'usage: follow_path.py ID px0 py0'
        id = 4
    else:
        id = sys.argv[1]
        id = int(id)
        #px0 = sys.argv[2]
        #px0 = int(px0)
        #py0 = sys.argv[3]
        #py0 = int(py0)
        #print '\nID = ', id, 'with type: ', type(id), '\n'
    #"""
    """
    id = rospy.get_param("id")
    #id = rospy.get_param("id")
    print id
    id = int(id)
    """


    rp = rospkg.RosPack()
    path = rp.get_path('distributed')
    path = path + '/text/path_robot'+str(id)+'.txt'
    file_path = open(path, 'w')
    path = rp.get_path('distributed')
    path = path + '/text/vel_robot'+str(id)+'.txt'
    file_vel = open(path, 'w')

    original_graph = myLib.read_graph('Original_graph_36.mat')
    n = original_graph['n']
    nodes = original_graph['nodes']
    C = original_graph['C']
    PathM = original_graph['PathM']
    w_s = original_graph['w_s']
    PolC = original_graph['PolC']

    virtual_graph = myLib.read_graph('Virtual_graph_36.mat')


    try:
        follow_graph()
    except rospy.ROSInterruptException:
        pass


    file_path.close()
    file_vel.close()
