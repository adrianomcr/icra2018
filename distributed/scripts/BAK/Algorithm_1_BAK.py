#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
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
import library2018 as myLib

# This script make a robot to follow a path through a graph




global x_n, y_n, theta_n
x_n = 0.1  # posicao x atual do robo
y_n = 0.2  # posicao y atual do robo
theta_n = 0.001  # orientacao atual do robo
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


global stop
stop = False


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

    global id
    global e_v, e_uv, e_g
    global T_a, T_f

    global stop

    # This function recieve a message from the centralized sensor simulator
    if data == True:
        stop == True



    return
# ----------  ----------  ----------  ----------  ----------



def keep_moving(time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path):

    end_flag = False

    if time - time_start > T + 1 / freq:
        time_start = time
        if len(pathNode) > 1:
            new_path = 1
        else:
            new_task = 1

    if new_task == 1:
        if len(Hole_path) > 1:
            i = Hole_path.pop(0)
            j = Hole_path[0]
            pathNode = myLib.getNodePath(i - 1, j - 1, PathM)

            new_task = 0
            new_path = 1

            print 'Robot ' + str(id)
            print 'Moving from i =', i, 'to', 'j =', j
            print 'time = ', time, '\n'

        else:
            print'\nNodes search completed\n'
            end_flag = True
            VX = 0
            WZ = 0
    if new_path == 1:
        i = pathNode.pop(0)
        j = pathNode[0]
        T = C[i - 1][j - 1] / Vd
        [edge, signal] = myLib.getEdge(i, j, PolC)
        [fr, to, cx, cy, cost] = myLib.getCoefs(edge, PolC)
        if signal == 1:
            p = 0
        elif signal == -1:
            p = 1
        new_path = 0

    [ux, uy, p] = myLib.compute_velocity(cx, cy, p, 1 / freq, signal, pose, Vd, Kp)
    if laserVec[135] < 0.15:
        vx_repulsive = -(0.05 / laserVec[135]) * cos(theta_n)
        vy_repulsive = -(0.05 / laserVec[135]) * sin(theta_n)
        ux = ux + vx_repulsive
        uy = ux + vy_repulsive
        print '                                       !!!!! Repulsive potential active !!!!!'
    [VX, WZ] = myLib.feedback_linearization(ux, uy, pose, d)

    return time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag
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

    vel = Twist()

    count = 0

    my_str = '/robot_' + str(id) + '/cmd_vel'
    pub_stage = rospy.Publisher(my_str, Twist, queue_size=1)
    my_str = 'follow_graph_' + str(id)
    rospy.init_node(my_str)
    my_str = '/robot_' + str(id) + '/base_pose_ground_truth'
    rospy.Subscriber(my_str, Odometry, callback_pose)
    my_str = '/robot_' + str(id) + '/base_scan'
    rospy.Subscriber(my_str, LaserScan, callback_laser)
    rospy.Subscriber("/comm_graph", Bool, callback_comm_graph)


    freq = 10.0  # Hz
    rate = rospy.Rate(freq)

    # Running CPP in order to find a initial route
    while detected_pose == 0:
        rate.sleep()

    x_n = pose[0]
    y_n = pose[1]
    theta_n = pose[2]

    print pose

    [current_node, dist] = myLib.get_current_node(original_graph, [x_n, y_n])
    current_node = current_node + 1
    active_edges = range(1, len(PolC[0]) + 1)
    edges_listOfTuples = myLib.write_listOfTuples(original_graph, active_edges)
    Hole_path = cppsolver.CPP(edges_listOfTuples, current_node)
    # ----------  ----------  ----------  ----------


    # ----------
    time_start = 0

    i = Hole_path.pop(0)
    j = Hole_path[0]
    pathNode = myLib.getNodePath(i - 1, j - 1, PathM)


    i = pathNode.pop(0)
    j = pathNode[0]

    [edge, signal] = myLib.getEdge(i, j, PolC)
    [fr, to, cx, cy, cost] = myLib.getCoefs(edge, PolC)
    if (signal == 1):
        p = 0
    elif signal == -1:
        p = 1
    T = C[i - 1][j - 1] / Vd

    new_task = 0
    new_path = 0
    print 'Robot ' + str(id)
    print 'Moving from i =', i, 'to', 'j =', j
    print 'time = ', time, '\n'
    # ----------

    sleep(0.1)

    while not rospy.is_shutdown():

        x_n = pose[0]
        y_n = pose[1]
        theta_n = pose[2]

        count = count + 1

        time = count / float(freq)


        """
        if time - time_start > T + 1 / freq:
            time_start = time
            if len(pathNode) > 1:
                new_path = 1
            else:
                new_task = 1

        if new_task == 1:
            if len(Hole_path) > 1:
                i = Hole_path.pop(0)
                j = Hole_path[0]
                pathNode = myLib.getNodePath(i - 1, j - 1, PathM)

                new_task = 0
                new_path = 1

                print 'Robot ' + str(id)
                print 'Moving from i =', i, 'to', 'j =', j
                print 'time = ', time, '\n'

            else:
                print'\nNodes search completed\n'
                vel.linear.x = 0
                vel.angular.z = 0
                pub_stage.publish(vel)
                break
        if new_path == 1:
            i = pathNode.pop(0)
            j = pathNode[0]
            T = C[i - 1][j - 1] / Vd
            [edge, signal] = myLib.getEdge(i, j, PolC)
            [fr, to, cx, cy, cost] = myLib.getCoefs(edge, PolC)
            if signal == 1:
                p = 0
            elif signal == -1:
                p = 1
            new_path = 0

        [ux, uy, p] = myLib.compute_velocity(cx, cy, p, 1 / freq, signal, pose, Vd, Kp)
        if laserVec[135] < 0.15:
            vx_repulsive = -(0.05 / laserVec[135]) * cos(theta_n)
            vy_repulsive = -(0.05 / laserVec[135]) * sin(theta_n)
            ux = ux + vx_repulsive
            uy = ux + vy_repulsive
            print '                                       !!!!! Repulsive potential active !!!!!'
        [VX, WZ] = myLib.feedback_linearization(ux, uy, pose, d)
        """

        #Keep moving through the edge
        [time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag] = keep_moving(time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path)
        if end_flag:
            pub_stage.publish(vel)
            break


        #Publish speed
        vel.linear.x = VX
        vel.angular.z = WZ
        pub_stage.publish(vel)

        #Write to file
        mystr = str(VX) + "\t" + str(WZ) + "\t" + "\t" + str(time) + "\n"
        file_vel.write(mystr)

        #Wait
        rate.sleep()
    else:
        rate.sleep()
        vel.linear.x = 0
        vel.angular.z = 0
        pub_stage.publish(vel)


# ---------- !! ---------- !! ---------- !! ---------- !! ----------









# Initial function
if __name__ == '__main__':

    global id
    global number_robots #Used only foor creating the topics
    number_robots = 2

    if len(sys.argv) < 2:
        print 'ERROR!!!'
        print 'The robot id was not provided'
        # print 'usage: follow_path.py ID px0 py0'
        id = 4
    else:
        id = sys.argv[1]
        id = int(id)


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

    virtual_graph = myLib.read_graph('Virtual_graph_36.mat')

    try:
        Algorithm_1()
    except rospy.ROSInterruptException:
        pass



    file_path.close()
    file_vel.close()
