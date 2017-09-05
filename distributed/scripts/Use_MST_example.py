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
#rp = rospkg.RosPack()
#path = rp.get_path('distributed')
#path = path + '/CPP'
#sys.path.insert(0, path)
#import cppsolver
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



# Example of unconected sets of edges that must be visited
edgeset_0 = [1, 2, 14, 13, 12, 11, 10, 9, 39, 8, 7, 6]
edgeset_1 = [33, 30, 32, 17, 21, 22, 31]
edgeset_2 = [37]
edgeset_3 = [27, 28]
#edgeset_list = [edgeset_0]
#edgeset_list = [edgeset_0,edgeset_1]
edgeset_list = [edgeset_0,edgeset_1,edgeset_2]
#edgeset_list = [edgeset_0,edgeset_1,edgeset_2,edgeset_3]
edgeset_all = []
for k in range(len(edgeset_list)):
    edgeset_all = edgeset_all+edgeset_list[k]

print '\n\nedgeset_all: '
print edgeset_all
print '\n\n'



#Create a set of nodes that must be visited by MST
nodeset_list = []
nodeset_all = []
for k in range(len(edgeset_list)):
    nodeset_list.append([])
    for l in range(len(edgeset_list[k])):
        edge = edgeset_list[k][l]
        [fr, to, cx, cy, cost] = myLib.getCoefs(edge - 1, PolC)
        nodeset_list[k].append(fr)
        nodeset_list[k].append(to)
    nodeset_list[k] = list(set(nodeset_list[k]))
for k in range(len(nodeset_list)):
    nodeset_all = nodeset_all+nodeset_list[k]



"""
for k in range(len(nodeset_list)):
    print 'len(nodeset_',k,'): ', len(nodeset_list[k])
print 'len(nodeset_all): ', len(nodeset_all)
"""

# Sort the node lists
for k in range(len(nodeset_list)):
    nodeset_list[k] = sorted(nodeset_list[k])
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

            #"""
            for k in range(len(nodeset_list)):
                if i + 1 in nodeset_list[k] and j + 1 in nodeset_list[k]:
                    pylab.plot(x, y, 'b-', linewidth=2.0)
            #"""

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




#Create a list of final edges
""" # This methon does not include edges with more than one direct edge
final_list = []
for e in range(len(PolC[0])):
    [fr, to, cx, cy, cost] = myLib.getCoefs(e, PolC)
    if (fr in nodeset_all and to in nodeset_all):
        iA = nodeset_all.index(fr)
        jA = nodeset_all.index(to)
        if (MSTarrayA[iA][jA] != 0):
            #print 'edge = ', e+1
            final_list.append(e+1) #add the edges from MST
final_list = final_list+edgeset_all #add the initaial edges
final_list = list(set(final_list))
"""
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
        pylab.plot(x, y, 'y-', linewidth=4.0)
    else:
        pylab.plot(x, y, 'k--', linewidth=1.0)

print 'final_list:'
print final_list


#print 'MSTarrayA'
#print MSTarrayA
#print 'size(MSTarrayA) = ', len(MSTarrayA),' x ',len(MSTarrayA)
#print 'len(nodeset_all) = ', len(nodeset_all)


pylab.show()