#!/usr/bin/env python
import rospy
import rospkg
import scipy.io
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

#"""
nodeset_0 = [1, 2, 14, 13, 12, 10, 11, 9, 35, 15, 16, 17, 18]
nodeset_1 = [30, 28, 24, 29, 21, 22, 31, 23, 25]
nodeset_all = nodeset_0+nodeset_1
#"""
#"""
nodeset_0 = [1, 2, 14, 13, 12, 10, 11, 9, 35, 8, 7, 3]
nodeset_1 = [30, 28, 29, 21, 20, 17, 16]
nodeset_all = nodeset_0+nodeset_1
#"""
#"""
nodeset_0 = [1, 2, 14, 13, 12, 10, 11, 9, 35, 8, 7, 3]
nodeset_1 = [30, 28, 29, 21, 20, 17, 16]
nodeset_2 = [25, 26, 27]
nodeset_all = nodeset_0+nodeset_1+nodeset_2
#"""

print 'len(nodeset_0): ', len(nodeset_0)
print 'len(nodeset_1): ', len(nodeset_1)
print 'len(nodeset_2): ', len(nodeset_2)
print 'len(nodeset_all): ', len(nodeset_all)

nodeset_0 = sorted(nodeset_0)
nodeset_1 = sorted(nodeset_1)
nodeset_2 = sorted(nodeset_2)
nodeset_all = sorted(nodeset_all)

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

XA = csr_matrix(CcomA)
MSTA = minimum_spanning_tree(XA)



#MSTarrayA = MSTA.toarray()
MSTarrayA = (np.matrix(MSTA.toarray())+(np.matrix(MSTA.toarray())).T).tolist()
pylab.figure(1)
pylab.axis('equal')
pylab.axis(w_s)
pylab.title('Disconected graph')
pylab.figure(2)
pylab.axis('equal')
pylab.axis(w_s)
pylab.title('Result of MST')
for i in range(n):
    for j in range(n):
        if (C[i][j] != 0 and i>j):
            # Get the position of the edge in the list
            [edge, signal] = myLib.getEdge(i + 1, j + 1, PolC)

            # Get the polynomial coefficients of the edge
            [fr, to, cx, cy, cost] = myLib.getCoefs(edge, PolC)

            p0 = [0.01 * k for k in range(101)]
            x = []
            y = []
            for p in p0:
                x.append(0)
                y.append(0)
                for k in range(6):  # Compute refference position
                    x[-1] = x[-1] + cx[6 - k - 1] * p ** k
                    y[-1] = y[-1] + cy[6 - k - 1] * p ** k
            pylab.figure(1)
            pylab.plot(x, y, 'k--', linewidth=1.0)

            if i+1 in nodeset_0 and j+1 in nodeset_0:
                pylab.plot(x, y, 'b-', linewidth=2.0)
            elif i + 1 in nodeset_1 and j + 1 in nodeset_1:
                pylab.plot(x, y, 'b-', linewidth=2.0)
            elif i + 1 in nodeset_2 and j + 1 in nodeset_2:
                pylab.plot(x, y, 'b-', linewidth=2.0)

            pylab.figure(2)
            pylab.plot(x, y, 'k--', linewidth=1.0)


pylab.figure(2)
# Plotting the result of MST
for iA in range(len(MSTarrayA)):
    for jA in range(len(MSTarrayA)):
        if (MSTarrayA[iA][jA] != 0 and iA>jA):
            i = nodeset_all[iA]-1
            j = nodeset_all[jA]-1
            path_i_j = myLib.getNodePath(i,j,PathM)
            """
            print '\ni, j: ', i+1, j+1
            print path_i_j
            """

            #Loop for conections composed by more than one
            for count in range(len(path_i_j)-1):
                # Get the position of the edge in the list
                [edge, signal] = myLib.getEdge(path_i_j[count], path_i_j[count+1], PolC)

                # Get the polynomial coefficients of the edge
                [fr, to, cx, cy, cost] = myLib.getCoefs(edge, PolC)

                p0 = [0.01 * k for k in range(101)]
                x = []
                y = []
                for p in p0:
                    x.append(0)
                    y.append(0)
                    for k in range(6):  # Compute refference position
                        x[-1] = x[-1] + cx[6 - k - 1] * p ** k
                        y[-1] = y[-1] + cy[6 - k - 1] * p ** k

                if len(path_i_j) == 2:
                    pylab.plot(x, y, 'g-', linewidth=5.0)
                else:
                    pylab.plot(x, y, 'g--', linewidth=5.0)




# Plotting the nodes
for k in range(n):
    pylab.figure(1)
    pylab.text(nodes[k][0] + 0.05, nodes[k][1] + 0.05, k + 1, fontsize=15.0)
    pylab.figure(2)
    pylab.text(nodes[k][0] + 0.05, nodes[k][1] + 0.05, k + 1, fontsize=15.0)


pylab.show()