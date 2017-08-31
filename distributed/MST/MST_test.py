#!/usr/bin/env python
import rospy
import rospkg
import scipy.io
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import minimum_spanning_tree
import numpy as np
import pylab
import os

# Function to read the data of the graph
def read_graph():

    global n, nodes, C, Ccom, MapEdge, PathM, w_s, PolC
    rp = rospkg.RosPack()
    dir_path=os.getcwd()
    path = dir_path + '/..'
    path=path+'/graph/Original_graph_36.mat'
    mat = scipy.io.loadmat(path)
    g = mat['graph']
    # Number of nodes
    n = g['number_nodes']
    n = n.tolist()
    n = n[0][0]
    n = n.tolist()
    n = n[0][0]
    # Position of the nodes
    nodes = g['node_list']
    nodes = nodes.tolist()
    nodes = nodes[0][0]
    nodes = nodes.tolist()
    # Matrix with the costs of the direct edges
    C = g['edge_matrix']
    C = C.tolist()
    C = C[0][0]
    C = C.tolist()
    # Matrix with the costs of the complete graph
    Ccom = g['complete_edge_matrix']
    Ccom = Ccom.tolist()
    Ccom = Ccom[0][0]
    Ccom = Ccom.tolist()
    # Matrix that maps a node pair (i, j) to the index in the edge list
    MapEdge = g['complete_edge_matrix']
    MapEdge = MapEdge.tolist()
    MapEdge = MapEdge[0][0]
    MapEdge = MapEdge.tolist()
    # Matrix that contains the path between two nodes i and j through the complete graph
    PathM = g['path_matrix']
    PathM = PathM.tolist()
    PathM = PathM[0][0]
    PathM = PathM.tolist()
    # Size of the world
    w_s = g['w_s']
    w_s = w_s.tolist()
    w_s = w_s[0][0]
    w_s = w_s.tolist()
    w_s = w_s[0]
    # List of edges that contains the polynomial coefficients
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

# Function to get the polynomial coefficients
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

    #print 'i = ', i
    #print 'j = ', j

    for k in range(len(PolC[0])):
        [fr, to, cx, cy] = getCoefs(k,PolC)
        if (fr == i and to == j):
            return (k, 1)
        elif (fr == j and to == i):
            return (k, -1)


    print '\n!! --- Ther is no direct path between i and j --- !!\n'
    return (1)


# ----------  ----------  ----------  ----------  ----------





# Primary routine
def execute_MST():
    global n, nodes, C, Ccom, MapEdge, PathM, w_s, PolC

    """
    X = csr_matrix([[0, 8, 0, 3], [0, 0, 2, 5], [0, 0, 0, 6], [0, 0, 0, 0]])
    Tcsr = minimum_spanning_tree(X)
    print X
    print Tcsr.toarray().astype(int)
    Y = np.matrix(Tcsr.toarray().astype(int))
    Y = Y + Y.T
    print Y
    """

    X = csr_matrix(C)
    MST = minimum_spanning_tree(X)
    print '\n Here is the the graph:'
    print X
    #print MST.toarray().astype(int)
    #print MST.toarray()
    print '\n Here is the MST:'
    print MST

    print '\nHere I am'

    MSTarray = MST.toarray()
    pylab.axis('equal')
    pylab.axis(w_s)
    for i in range(n):
        for j in range(n):
            """
            if(MSTarray[i][j] != 0):

                #Get the position of the edge in the list
                [edge, signal] = getEdge(i+1, j+1, PolC)

                #Get the polynomial coefficients of the edge
                [fr, to, cx, cy] = getCoefs(edge, PolC)

                p0 = [0.01*k for k in range(101)]
                x = []
                y = []
                for p in p0:
                    x.append(0)
                    y.append(0)
                    for k in range(6):  # Compute refference position
                        x[-1] = x[-1] + cx[6 - k - 1] * p ** k
                        y[-1] = y[-1] + cy[6 - k - 1] * p ** k
                pylab.plot(x,y,'b-',linewidth=2.0)
            """
            if(C[i][j] != 0):
                #Get the position of the edge in the list
                [edge, signal] = getEdge(i+1, j+1, PolC)

                #Get the polynomial coefficients of the edge
                [fr, to, cx, cy] = getCoefs(edge, PolC)

                p0 = [0.01*k for k in range(101)]
                x = []
                y = []
                for p in p0:
                    x.append(0)
                    y.append(0)
                    for k in range(6):  # Compute refference position
                        x[-1] = x[-1] + cx[6 - k - 1] * p ** k
                        y[-1] = y[-1] + cy[6 - k - 1] * p ** k
                if(MSTarray[i][j] != 0):
                    pylab.plot(x, y, 'b-', linewidth=2.0)
                else:
                    pylab.plot(x, y, 'b--', linewidth=1.0)





    for k in range(n):
        pylab.plot(nodes[k][0],nodes[k][1],'r*',linewidth=2.0)
        pylab.plot(nodes[k][0],nodes[k][1],'ro',linewidth=2.0)


    pylab.show()











# ---------- !! ---------- !! ---------- !! ---------- !! ----------








# Initial function
if __name__ == '__main__':

    read_graph()

    try:
        execute_MST()
    except rospy.ROSInterruptException:
        pass




