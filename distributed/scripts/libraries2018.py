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

    return(close_node, dist**0.5)
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
            list_tuple.append((fr, to, cost))

    #print 'Here is the list of tuples:'
    #print list_tuple

    return (list_tuple)
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
              'PathM': PathM,
              'w_s': w_s,
              'PolC': PolC
              }

    return (output)
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


    return(fr, to, cx, cy, cost)
# ----------  ----------  ----------  ----------  ----------





# Function to get the number of a edge given two adjacent nodes
def getEdge(i,j,PolC):

    #print 'i = ', i
    #print 'j = ', j

    for k in range(len(PolC[0])):
        [fr, to, cx, cy, cost] = getCoefs(k,PolC)
        if (fr == i and to == j):
            return (k, 1)
        elif (fr == j and to == i):
            return (k, -1)


    print '\n!! --- Ther is no direct path between i and j --- !!\n'
    return (1)


# ----------  ----------  ----------  ----------  ----------











