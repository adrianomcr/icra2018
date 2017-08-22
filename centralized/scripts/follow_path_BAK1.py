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
d = 0.1

global Vd
Vd = 0.25

global time
time = 0

global Kp #Proportional gain
Kp = 0.5

global laserVec
laserVec = [1 for i in range(270)]


# Function to read the data of the graph
def read_graph():

    global n, nodes, C, PathM, w_s, PolC
    rp = rospkg.RosPack()
    path = rp.get_path('centralized')
    #path = path + '/graph/Graph_data_36_1.mat'
    path = path + '/graph/Graph_data_36_meters.mat'
    #mat = scipy.io.loadmat('./../files/Graph_data_36.mat')
    #mat = scipy.io.loadmat('./../graph/Graph_data_36_1.mat')
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
    """ #Use this to extract a path
    p = PathM[i-1][j-1]
    p = p[0]
    p = p.tolist()
    p = p[0]
    """
    w_s = g['w_s']
    w_s = w_s.tolist()
    w_s = w_s[0][0]
    w_s = w_s.tolist()
    w_s = w_s[0]
    PolC = g['Pol_coefs']
    PolC = PolC.tolist()
    PolC = PolC[0][0]
    PolC = PolC.tolist()
    """ #Use this to extract a path
    p = PolC[][e]
    
    p = p[0] #[0, 1, 2, 3] equiv [from, to, cx, cy ]
    p = p.tolist()
    p = p[0]
    
    p = p[2]
    p.tolist()
    p = np.matrix(p)
    p = p.T
    p = p.tolist()
    p = p[0]
    """

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
        #print 'fr = ', fr
        #print 'to = ', to
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

    #print "[x, y, theta] = ", x_n, y_n, theta_n

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

    xref0 = 0
    yref0 = 0
    for k in range(6): #Compute refference position
        xref0 = xref0 + cx[6 - k - 1] * p ** k
        yref0 = yref0 + cy[6 - k - 1] * p ** k
    xref = yref0
    yref = w_s[3]-xref0
    vx0 = 0
    vy0 = 0
    for k in range(1,6): #Compute refference velocity (diect evaluatioin of the polynimial)
        vx0 = vx0 + (k) * cx[6 - k - 1] * p ** (k - 1)
        vy0 = vy0 + (k) * cy[6 - k - 1] * p ** (k - 1)
    vx = vy0
    vy = -vx0

    #Atualization of p such as the speed is constant
    v = sqrt((vx)**2+(vy)**2)/T*T
    p = p + ((dt) * (Vd / v))*signal

    #Compuete the speeds 'vx' and 'vy' for the feed forward such the resultant speed is constant
    vx_ff = Vd * (vx / v)
    vy_ff = Vd * (vy / v)
    print 'p = ', [p]

    #print '[vx_ff, vy_ff] = ', [vx_ff, vy_ff]
    #print '[v_ff] = ', sqrt(vx_ff**2+ vy_ff**2)
    #print '[vx_ff, vy_ff, v_ff] = ', [vx_ff, vy_ff, sqrt(vx_ff**2+ vy_ff**2)]

    #print '[xref, yref] = ', [xref0, yref0]

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
    #rospy.Subscriber("/turtle1/cmd_vel", Twist, callback_teclado)


    freq = 10.0  # Hz
    rate = rospy.Rate(freq)

    """
    i = 9-1
    j = 10-1
    edge = 9-1
    """
    i = 1-1
    j = 2-1
    edge = 1 - 1
    #"""

    """
    pathNode = getNodePath(i, j, PathM)
    p = 0
    [fr, to, cx, cy] = getCoefs(edge, PolC)
    T = C[i][j] / Vd
    """

    """
    print 'pathNode = ', pathNode
    print 'cx = ', cx
    print 'cy = ', cy
    print 'C[i][j] = ', C[i][j]
    print 'T = ', T
    """


    #----------
    Hole_path = [1, 2, 14, 13, 12, 10, 11, 35, 9, 8, 36, 34, 15, 33, 16, 17, 18, 19, 20, 32, 21, 22, 31, 29, 28, 30, 24,
                 25, 26, 27, 23, 7, 3, 4, 5, 6]
    print 'Hole_path = ', Hole_path

    new_task = 0
    new_path = 0

    time_start = 0

    i = Hole_path.pop(0)
    j = Hole_path[0]
    pathNode = getNodePath(i-1, j-1, PathM)
    print 'CCCCCCCCCCCCCCCc'
    print 'i = ', i
    print 'j = ', j
    print 'pathNode = ', pathNode

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

    orientate = 0

    # ----------

    sleep(0.1)

    while (not rospy.is_shutdown()):

        count = count + 1
        time = count / float(freq)

        print '                          i = ', i
        print '                          j = ', j
        print '                          time = ', time
        print '                          time_start = ', time_start

        """
        if(time > T+1/freq):
            break
        print 'time = ', time
        """
        if(time-time_start > T+1/freq and orientate == 0):
            time_start = time
            if(len(pathNode) > 1):
                new_path = 1
            else:
                new_task = 1

        print 'time = ', time



        #"""
        #if (NEW_PATH):
        if(new_task == 1):
            i = Hole_path.pop(0)
            j = Hole_path[0]
            pathNode = getNodePath(i-1, j-1, PathM)

            new_task = 0
            new_path = 1
        #if (NEW_STRETCH_OF_PATH):
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
            orientate = 1;

        #"""



        if (orientate == 1):
            VX = 0
            """
            if (signal == 1):
                ang_ref = atan2((nodes[to-1][1]-nodes[fr-1][1]),(nodes[to-1][0]-nodes[fr-1][0]))
            else:
                ang_ref = atan2((nodes[fr - 1][1] - nodes[to - 1][1]), (nodes[fr - 1][0] - nodes[to - 1][0]))
            print '        fr, to = ', fr, to
            print '        ang_ref = ', ang_ref*180/pi
            WZ = 0.5*(ang_ref-theta_n)
            
            if(abs(ang_ref-theta_n)<0.1):
                orientate = 0
                time_start = time
            """
            orientate = 0
            time_start = time
        else:
            [ux, uy, p] = compute_velocity(cx, cy, p, T, 1/freq, signal)
            if(laserVec[135] < 0.15):
                vx_repulsive = -(0.05 / laserVec[135]) * cos(theta_n)
                vy_repulsive = -(0.05 / laserVec[135]) * sin(theta_n)
                ux = ux + vx_repulsive
                uy = ux + vy_repulsive
            [VX, WZ] = feedback_linearization(ux,uy)


        vel.linear.x = VX  # *0
        vel.angular.z = WZ  # *0 + 0.3

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
    #file_path = open('/home/adriano/ROS_projects/icra2018/src/centralized/text/path_1.txt', 'w')
    file_path = open(path, 'w')
    path = rp.get_path('centralized')
    path = path + '/text/vel_1.txt'
    #file_vel = open('/home/adriano/ROS_projects/icra2018/src/centralized/text/vel_1.txt', 'w')
    file_vel = open(path, 'w')


    #read_traj_ref()
    read_graph()

    #print 'This is the address:'
    #print x


    try:
        follow_graph()
    except rospy.ROSInterruptException:
        pass


    file_path.close()
    file_vel.close()