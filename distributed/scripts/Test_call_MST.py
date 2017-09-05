#!/usr/bin/env python
import rospy
import rospkg
import scipy.io
from distributed.msg import History, Intlist, HistList
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import minimum_spanning_tree
import numpy as np
import pylab
import os


import rospkg
import sys
rp = rospkg.RosPack()
path = rp.get_path('distributed')
path = path + '/CPP'
sys.path.insert(0, path)
import cppsolver
import library2018 as myLib
import Algorithm_2 as Alg2
import Algorithm_2_BAK as Alg2_BAK






def MSTconnect(edgeset_all, cor):


    #Read original graph
    graph = myLib.read_graph('Original_graph_36.mat')
    n = graph['n']
    nodes = graph['nodes']
    C = graph['C']
    Ccom = graph['Ccom']
    PathM = graph['PathM']
    EdgeMap = graph['EdgeMap']
    w_s = graph['w_s']
    PolC = graph['PolC']



    #Create a set of nodes that must be visited by MST
    nodeset_all = []
    for edge in edgeset_all:
        [fr, to, cx, cy, cost] = myLib.getCoefs(edge - 1, PolC)
        nodeset_all.append(fr)
        nodeset_all.append(to)
    nodeset_all = list(set(nodeset_all))



    nodeset_all = sorted(nodeset_all)



    # Obtain a subgraph that consins only the selected nodes
    CA = np.matrix(C)
    CcomA = np.matrix(Ccom)
    exclude_list = []
    for k in range(n):
        if not k+1 in nodeset_all:
            exclude_list.append(k)
    CA = np.delete(CA,exclude_list,0)
    CA = np.delete(CA,exclude_list,1)
    CcomA = np.delete(CcomA,exclude_list,0)
    CcomA = np.delete(CcomA,exclude_list,1)
    CA = CA.tolist()
    CcomA = CcomA.tolist()

    # Apply MST on the complete subgraph
    XA = csr_matrix(CcomA)
    MSTA = minimum_spanning_tree(XA)

    # Make the resultant matrix be symmetric
    MSTarrayA = (np.matrix(MSTA.toarray())+(np.matrix(MSTA.toarray())).T).tolist()



    #Create a list of final edges
    final_list = []
    for iA in range(len(MSTarrayA)):
        for jA in range(len(MSTarrayA)):
            if (MSTarrayA[iA][jA] != 0 and iA>jA):
                i = nodeset_all[iA] - 1
                j = nodeset_all[jA] - 1
                path_i_j = myLib.getNodePath(i,j,PathM)

                # Loop for conections composed by more than one
                for count in range(len(path_i_j) - 1):
                    # Get the position of the edge in the list
                    [edge, signal] = myLib.getEdge(path_i_j[count], path_i_j[count + 1], PolC)

                    # Get the polynomial coefficients of the edge
                    [fr, to, cx, cy, cost] = myLib.getCoefs(edge, PolC)

                    final_list.append(edge+1)
    final_list = final_list+edgeset_all #add the initaial edges
    final_list = list(set(final_list))





    pylab.figure(3)
    pylab.axis('equal')
    pylab.axis(w_s)
    pylab.title('Final connected graph')
    for e in range(len(PolC[0])):

        [fr, to, cx, cy, cost] = myLib.getCoefs(e, PolC)

        p0 = [0.01 * k for k in range(101)]
        x = []
        y = []
        for p in p0:
            x.append(0)
            y.append(0)
            for k in range(6):  # Compute refference position
                x[-1] = x[-1] + cx[6 - k - 1] * p ** k
                y[-1] = y[-1] + cy[6 - k - 1] * p ** k

        if  e+1 in  final_list:
            pylab.plot(x, y, cor, linewidth=4.0,)
        else:
            pylab.plot(x, y, 'k--', linewidth=1.0)

    print 'final_list:'
    print final_list

    pylab.show()


    return final_list

# ----------  ----------  ----------  ----------  ----------  ----------  ----------







HL = HistList()

H = History()
H.e_v = [1, 14, 13, 12, 11, 10, 9, 8, 7, 34, 25, 26, 27, 28]
H.e_uv = [2, 3, 4, 5, 6, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 29, 30, 31, 32, 33, 35, 36, 37, 38, 39, 40]
H.e_g = []
H.T_a = []
H.T_f = []
H.currEdge = 28
#H = {'e_v': H.e_v, 'e_uv': H.e_uv, 'e_g': H.e_g, 'T_a': H.T_a, 'T_f': H.T_f, 'T_f': H.currEdge}
HL.listOfH.append(H)

H = History()
H.e_v = [33, 29, 25, 24, 23, 22, 36, 21, 34, 6]
H.e_uv = [1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 26, 27, 28, 30, 31, 32, 35, 37, 38, 39, 40]
H.e_g = []
H.T_a = []
H.T_f = []
H.currEdge = 6
#H = {'e_v': H.e_v, 'e_uv': H.e_uv, 'e_g': H.e_g, 'T_a': H.T_a, 'T_f': H.T_f, 'T_f': H.currEdge}
HL.listOfH.append(H)

original_graph = myLib.read_graph('Original_graph_36.mat')
virtual_graph = myLib.read_graph('Virtual_graph_36.mat')

[replan, division] = Alg2_BAK.replanning(original_graph,virtual_graph,[HL.listOfH[0], HL.listOfH[1]], [0, 1], [1, 1.1])
print 'division original indexes:'
print division


subgraph_rob0 = MSTconnect(division[0],'b')
edges_listOfTuples = myLib.write_listOfTuples(original_graph, subgraph_rob0)
print 'active_edges'
print edges_listOfTuples
Hole_path_r0 = cppsolver.CPP(sorted(edges_listOfTuples), 25)
print 'New route for robot 0:'
print Hole_path_r0
subgraph_rob1 = MSTconnect(division[1],'r')
edges_listOfTuples = myLib.write_listOfTuples(original_graph, subgraph_rob1)
Hole_path_r1 = cppsolver.CPP(edges_listOfTuples, 3)
print 'New route for robot 1:'
print Hole_path_r1

"""
[e_v: [1, 14, 13, 12, 11, 10, 9, 8, 7, 34, 25, 26, 27, 28]
e_uv: [2, 3, 4, 5, 6, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 29, 30, 31, 32, 33, 35, 36, 37, 38, 39, 40]
e_g: []
T_a: []
T_f: []
currEdge: 28, e_v: [33, 29, 25, 24, 23, 22, 36, 21, 34, 6]
e_uv: [1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 26, 27, 28, 30, 31, 32, 35, 37, 38, 39, 40]
e_g: []
T_a: []
T_f: []
currEdge: 6]
"""