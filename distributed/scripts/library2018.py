#!/usr/bin/env python
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
#from time import sleep, time
import time
#from pylab import *
import pylab
from lpsolve55 import *
from lp_maker import *
import rospkg
import scipy.io


# Function to define the closest node from a given position
def get_current_node(graph,pose):
    nodes = graph['nodes']


    close_node = 0
    current_dist = (pose[0]-nodes[0][0])**2 + (pose[1]-nodes[0][1])**2
    for k in range(1,len(nodes)):
        dist = (pose[0] - nodes[k][0]) ** 2 + (pose[1] - nodes[k][1]) ** 2
        if(dist < current_dist):
            close_node = k
            current_dist = dist

    return close_node, dist**0.5
# ----------  ----------  ----------  ----------  ----------




# Function to write a list of tuples to describe a sub-graph
def write_listOfTuples(graph,list_active):
    """
    :param graph: a dictionary describing the full original graph
    :param list_active: list of idexes of the edge that must be visited (from 1 to the number of edges)
    :return: list_tuple: list of tuples containing the edges from, to and the cost
    """

    """
    Adriano, you should take care of what is going to happen when the graph is not connected
    After the use of MST
    """

    PolC = graph['PolC']

    list_tuple = []
    for k in range(len(PolC[0])):
        if (k + 1 in list_active):
            [fr, to, cx, cy, cost] = getCoefs(k, PolC)
            #print 'k+1, fr, to = ', k + 1,' ',fr,' ', to
            list_tuple.append((fr, to, cost))

    #print 'Here is the list of tuples:'
    #print list_tuple

    return list_tuple
# ----------  ----------  ----------  ----------  ----------













# Function to read the data of the graph
def read_graph(name):

    #global n, nodes, C, PathM, w_s, PolC
    rp = rospkg.RosPack()
    path = rp.get_path('distributed')
    path = path + '/graph/' + name
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
    Ccom = g['complete_edge_matrix']
    Ccom = Ccom.tolist()
    Ccom = Ccom[0][0]
    Ccom = Ccom.tolist()
    EdgeMap = g['map_edge_matrix']
    EdgeMap = EdgeMap.tolist()
    EdgeMap = EdgeMap[0][0]
    EdgeMap = EdgeMap.tolist()
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


    output = {'n': n,
              'nodes': nodes,
              'C': C,
              'Ccom': Ccom,
              'EdgeMap': EdgeMap,
              'PathM': PathM,
              'w_s': w_s,
              'PolC': PolC
              }

    return output
# ----------  ----------  ----------  ----------  ----------







# Function to get the path in sequency of nodes
def getNodePath(i,j,PathM):

    #pathNode = PathM[i-1][j-1]
    pathNode = PathM[i][j]
    pathNode = pathNode[0]
    pathNode = pathNode.tolist()
    pathNode = pathNode[0]

    return pathNode
# ----------  ----------  ----------  ----------  ----------







# Function to get the polynomial coefficients
def getCoefs(edge,PolC):
    P = PolC[0][edge]

    #get "from" node
    fr = P[0]
    fr = fr.tolist()
    fr = fr[0]
    fr = fr[0]

    #get "to" node
    to = P[1]  # [0, 1, 2, 3, 4] equiv [from, to, cx, cy, cost ]
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

    #get cost of the edge
    cost = P[4]
    cost = cost.tolist()
    cost = cost[0]
    cost = cost[0]


    return fr, to, cx, cy, cost
# ----------  ----------  ----------  ----------  ----------





# Function to get the number of a edge given two adjacent nodes
def getEdge(i,j,PolC):

    #print 'i = ', i
    #print 'j = ', j

    for k in range(len(PolC[0])):
        [fr, to, cx, cy, cost] = getCoefs(k,PolC)
        if (fr == i and to == j):
            return k, 1
        elif (fr == j and to == i):
            return k, -1


    print '\n!! --- Ther is no direct path between i and j --- !!\n'
    return 0


# ----------  ----------  ----------  ----------  ----------














# Functions used in Algorithm_1
# ----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
# ----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------

# Feedback linearization
def feedback_linearization(Ux, Uy, pose, d):
    #global x_n, y_n, theta_n
    #global pose
    #global d

    x_n = pose[0]
    y_n = pose[1]
    theta_n = pose[2]

    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy

    return (VX, WZ)
# ----------  ----------  ----------  ----------  ----------


def compute_velocity(cx, cy, p, dt, signal, pose, Vd, Kp):

    xref = 0
    yref = 0
    for k in range(6):  # Compute refference position
        xref = xref + cx[6 - k - 1] * p ** k
        yref = yref + cy[6 - k - 1] * p ** k
    vx = 0
    vy = 0
    for k in range(1, 6):  # Compute refference velocity (diect evaluatioin of the polynimial)
        vx = vx + (k) * cx[6 - k - 1] * p ** (k - 1)
        vy = vy + (k) * cy[6 - k - 1] * p ** (k - 1)

    # Atualization of p such as the speed is constant
    v = sqrt((vx) ** 2 + (vy) ** 2)
    p = p + ((dt) * (Vd / v)) * signal

    # Compuete the speeds 'vx' and 'vy' for the feed forward such the resultant speed is constant
    vx_ff = (Vd * (vx / v)) * signal
    vy_ff = (Vd * (vy / v)) * signal

    # Compute the controller signals
    ux = Kp * (xref - pose[0]) + vx_ff
    uy = Kp * (yref - pose[1]) + vy_ff

    return (ux, uy, p)
# ----------  ----------  ----------  ----------  ----------



#Function to apply a repulsive velocity and avoid collisions
def repulsive_potential(laserVec, pose, ux, uy):

    """
    print '\n\nHere is laserVec'
    print laserVec
    print '\n\n'
    print '\n\nHere is laserVec[135]'
    print laserVec[135]
    print '\n\n'
    print '\n\nHere is pose[2]'
    print pose[2]
    print '\n\n'
    """

    index = laserVec.index(min(laserVec))
    do = 0.35
    if laserVec[index] < do:
        phi = (-135 + index) * pi / 180.0  # angle of the object in the local frame
        print 'phi = ', phi
        theta = pose[2]
        beta = phi + theta  # angle of the object in the world frame
        grad_ob = [cos(beta), sin(beta)]
        gain = 0.1*(1/laserVec[index] - 1/do)*(1/laserVec[index]**1)
        vx_repulsive = -gain * grad_ob[0]
        vy_repulsive = -gain * grad_ob[1]
        ux = ux + vx_repulsive
        uy = uy + vy_repulsive
        print '                                       !!!!! Repulsive potential active !!!!!'

    """
    if laserVec[135] < 0.15:
        vx_repulsive = -(0.05 / laserVec[135]) * cos(pose[2])
        vy_repulsive = -(0.05 / laserVec[135]) * sin(pose[2])
        ux = ux + vx_repulsive
        uy = uy + vy_repulsive
        print '                                       !!!!! Repulsive potential active !!!!!'
    """

    return ux, uy
# ----------  ----------  ----------  ----------  ----------



def keep_moving(H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, original_graph, freq, pose, laserVec, d, Vd, Kp, id, edge):

    C = original_graph['C']
    PathM = original_graph['PathM']
    PolC = original_graph['PolC']
    EdgeMap = original_graph['EdgeMap']

    end_flag = False
    change_edge = False #Indicated a change on edge - useful to force robots to finish a edge before replanning

    if time - time_start > T + 1 / freq:
        change_edge = True
        time_start = time
        if len(pathNode) > 1:
            new_path = 1 #robot ended a direct path only
        else:
            new_task = 1 #robot ended a complete path as well

    if new_task == 1:
        if len(Hole_path) > 1:
            i = Hole_path.pop(0)
            j = Hole_path[0]
            pathNode = getNodePath(i - 1, j - 1, PathM)

            new_task = 0
            new_path = 1

        else:
            print'\nNodes search completed\n'
            end_flag = True
            VX = 0
            WZ = 0
    if new_path == 1:
        i = pathNode.pop(0)
        j = pathNode[0]
        T = C[i - 1][j - 1] / Vd
        [edge, signal] = getEdge(i, j, PolC)
        [fr, to, cx, cy, cost] = getCoefs(edge, PolC)
        if signal == 1:
            p = 0
        elif signal == -1:
            p = 1
        new_path = 0

        # Update the History

        edge = EdgeMap[i - 1][j - 1]
        # Remove the edge from the list of unvisited nodes
        if edge in H['e_uv']:
            H['e_uv'].pop(H['e_uv'].index(edge))
        # Add the edge to the list of visited edges
        if not edge in H['e_v']:
            H['e_v'].append(edge)
        # Ad the edge in the forbidden edges

        print '\nRobot ' + str(id)
        print 'Moving from i =', i, 'to', 'j =', j
        print 'Edge = ', EdgeMap[i - 1][j - 1]
        print 'e_v:\n', H['e_v']
        print 'e_uv:\n', H['e_uv']
        print 'Whole_path:\n', Hole_path



    [ux, uy, p] = compute_velocity(cx, cy, p, 1 / freq, signal, pose, Vd, Kp)

    [ux, uy] = repulsive_potential(laserVec, pose, ux, uy)
    """
    if laserVec[135] < 0.15:
        vx_repulsive = -(0.05 / laserVec[135]) * cos(pose[2])
        vy_repulsive = -(0.05 / laserVec[135]) * sin(pose[2])
        ux = ux + vx_repulsive
        uy = uy + vy_repulsive
        print '                                       !!!!! Repulsive potential active !!!!!'
    """
    [VX, WZ] = feedback_linearization(ux, uy, pose, d)

    return H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge
# ----------  ----------  ----------  ----------  ----------


# ----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
# ----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------



def merge_history():
    return



# ----------  ----------  ----------  ----------  ----------



