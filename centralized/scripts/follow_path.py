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


# Function to read the data of the graph
def read_graph():

    global n, nodes, C, PathM, w_s, PolC
    rp = rospkg.RosPack()
    path = rp.get_path('centralized')
    path = path + '/graph/Graph_data_36_meters.mat'
    mat = scipy.io.loadmat(path)
    g = mat['graph']
    n = g['number_nodes']
    n = n.tolist()
    n = n[0][0]
    n = n.tolist()
    n = n[0][0]
    nodes = g['node_list']
    nodes = nodes.tolist()
    nodes = nodes[0][0]
    nodes = nodes.tolist()
    C = g['edge_matrix']
    C = C.tolist()
    C = C[0][0]
    C = C.tolist()
    PathM = g['path_matrix']
    PathM = PathM.tolist()
    PathM = PathM[0][0]
    PathM = PathM.tolist()
    w_s = g['w_s']
    w_s = w_s.tolist()
    w_s = w_s[0][0]
    w_s = w_s.tolist()
    w_s = w_s[0]
    PolC = g['Pol_coefs']
    PolC = PolC.tolist()
    PolC = PolC[0][0]
    PolC = PolC.tolist()

    return

# ----------  ----------  ----------  ----------  ----------

# Function to get the path in sequency of nodes
def getNodePath(i,j,PathM):

    #pathNode = PathM[i-1][j-1]
    pathNode = PathM[i][j]
    pathNode = pathNode[0]
    pathNode = pathNode.tolist()
    pathNode = pathNode[0]

    return (pathNode)
# ----------  ----------  ----------  ----------  ----------

# Function to get the polynomial coeficients
def getCoefs(edge,PolC):
    P = PolC[0][edge]

    #get "from" node
    fr = P[0]
    fr = fr.tolist()
    fr = fr[0]
    fr = fr[0]

    #get "to" node
    to = P[1]  # [0, 1, 2, 3] equiv [from, to, cx, cy ]
    to = to.tolist()
    to = to[0]
    to = to[0]

    #get coefficients in x direction
    cx = P[2]
    cx.tolist()
    cx = np.matrix(cx)
    cx = cx.T
    cx = cx.tolist()
    cx = cx[0]

    #get coefficients in y direction
    cy = P[3]
    cy.tolist()
    cy = np.matrix(cy)
    cy = cy.T
    cy = cy.tolist()
    cy = cy[0]


    return(fr, to, cx, cy)
# ----------  ----------  ----------  ----------  ----------

# Function to get the number of a edge given two adjacent nodes
def getEdge(i,j,PolC):

    print 'i = ', i
    print 'j = ', j

    for k in range(len(PolC[0])):
        [fr, to, cx, cy] = getCoefs(k,PolC)
        if (fr == i and to == j):
            print 'AA'
            return (k, 1)
        elif (fr == j and to == i):
            return (k, -1)


    print '\n!! --- Ther is no direct path between i and j --- !!\n'
    return (1)


# ----------  ----------  ----------  ----------  ----------



# Callback routine to obtain the robot's pose
def callback_pose(data):
    global x_n, y_n, theta_n
    global file_path, time

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

    vel = Twist()

    count = 0

    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0

    pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.init_node("follow_graph")
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose)
    rospy.Subscriber("/base_scan", LaserScan, callback_laser)


    freq = 10.0  # Hz
    rate = rospy.Rate(freq)



    #----------
    Hole_path = [1, 2, 14, 13, 12, 10, 11, 35, 9, 8, 36, 34, 15, 33, 16, 17, 18, 19, 20, 32, 21, 22, 31, 29, 28, 30, 24,
                 25, 26, 27, 23, 7, 3, 4, 5, 6]

    time_start = 0

    i = Hole_path.pop(0)
    j = Hole_path[0]
    pathNode = getNodePath(i-1, j-1, PathM)

    i = pathNode.pop(0)
    j = pathNode[0]
    T = C[i][j] / Vd
    [edge, signal] = getEdge(i, j, PolC)
    [fr, to, cx, cy] = getCoefs(edge, PolC)
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

        print 'time = ', time
        print 'task time: ', time - time_start, '/', T
        print '                       moving from i = ', i
        print '                                to j = ', j


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
                pathNode = getNodePath(i-1, j-1, PathM)

                new_task = 0
                new_path = 1
            else:
                print'\nNodes search completed\n'
                break
        if(new_path == 1):
            i = pathNode.pop(0)
            j = pathNode[0]
            T = C[i-1][j-1] / Vd
            [edge, signal] = getEdge(i,j,PolC)
            [fr, to, cx, cy] = getCoefs(edge, PolC)
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

    rp = rospkg.RosPack()
    path = rp.get_path('centralized')
    path = path + '/text/path_1.txt'
    file_path = open(path, 'w')
    path = rp.get_path('centralized')
    path = path + '/text/vel_1.txt'
    file_vel = open(path, 'w')


    read_graph()


    try:
        follow_graph()
    except rospy.ROSInterruptException:
        pass


    file_path.close()
    file_vel.close()
