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





#Read graph
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
print 'len(nodeset_0): ', len(nodeset_0)
print 'len(nodeset_1): ', len(nodeset_1)
print 'len(nodeset_all): ', len(nodeset_all)
#"""

nodeset_0 = sorted(nodeset_0)
nodeset_1 = sorted(nodeset_1)
nodeset_all = sorted(nodeset_all)

C0 = np.matrix(C)
Ccom0 = np.matrix(Ccom)
exclude_list = []
for k in range(n):
    if not k+1 in nodeset_0:
        exclude_list.append(k)
C0 = np.delete(C0,exclude_list,0)
C0 = np.delete(C0,exclude_list,1)
Ccom0 = np.delete(Ccom0,exclude_list,0)
Ccom0 = np.delete(Ccom0,exclude_list,1)
C0 = C0.tolist()
Ccom0 = Ccom0.tolist()

C1 = np.matrix(C)
Ccom1 = np.matrix(Ccom)
exclude_list = []
for k in range(n):
    if not k+1 in nodeset_1:
        exclude_list.append(k)
C1 = np.delete(C1,exclude_list,0)
C1 = np.delete(C1,exclude_list,1)
Ccom1 = np.delete(Ccom1,exclude_list,0)
Ccom1 = np.delete(Ccom1,exclude_list,1)
C1 = C1.tolist()
Ccom1 = Ccom1.tolist()


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


print '\nsize(Ccom0) = ', len(Ccom0), ' x ', len(Ccom0), '\n'
print '\nsize(Ccom1) = ', len(Ccom1), ' x ', len(Ccom1), '\n'
print '\nsize(CcomA) = ', len(CcomA), ' x ', len(CcomA), '\n'


X0 = csr_matrix(C0)
MST0 = minimum_spanning_tree(X0)

X1 = csr_matrix(C1)
MST1 = minimum_spanning_tree(X1)

XA = csr_matrix(CcomA)
MSTA = minimum_spanning_tree(XA)


#MSTarray0 = MST0.toarray()
#MSTarray1 = MST1.toarray()
#MSTarrayA = MSTA.toarray()
MSTarray0 = (np.matrix(MST0.toarray())+(np.matrix(MST0.toarray())).T).tolist()
MSTarray1 = (np.matrix(MST1.toarray())+(np.matrix(MST1.toarray())).T).tolist()
MSTarrayA = (np.matrix(MSTA.toarray())+(np.matrix(MSTA.toarray())).T).tolist()
pylab.figure(1)
pylab.axis('equal')
pylab.axis(w_s)
pylab.figure(2)
pylab.axis('equal')
pylab.axis(w_s)
for i in range(n):
    for j in range(n):
        if (C[i][j] != 0):
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
                """
                i0 = nodeset_0.index(i + 1)
                j0 = nodeset_0.index(j + 1)
                pylab.plot(x, y, 'b--', linewidth=2.0)
                if MSTarray0[i0][j0] != 0:
                    pylab.plot(x, y, 'b-', linewidth=2.0)
                """
                pylab.plot(x, y, 'b-', linewidth=2.0)
            elif i + 1 in nodeset_1 and j + 1 in nodeset_1:
                """
                i1 = nodeset_1.index(i + 1)
                j1 = nodeset_1.index(j + 1)
                pylab.plot(x, y, 'r--', linewidth=2.0)
                if MSTarray1[i1][j1] != 0:
                    pylab.plot(x, y, 'r-', linewidth=2.0)
                """
                pylab.plot(x, y, 'r-', linewidth=2.0)
            #else:
                #pylab.plot(x, y, 'k--', linewidth=1.0)


            pylab.figure(2)
            #"""
            pylab.plot(x, y, 'k--', linewidth=1.0)
            if i + 1 in nodeset_all and j + 1 in nodeset_all:
                iA = nodeset_all.index(i + 1)
                jA = nodeset_all.index(j + 1)

                print "iA, jA = ", iA, jA
                print 'size(MSTarrayA) = ', len(MSTarrayA), ' x ', len(MSTarrayA[0])
                if MSTarrayA[iA][jA] != 0 and iA>jA:
                    pylab.plot(x, y, 'g-', linewidth=7.0)
            #"""



"""
print '\nXA:'
print XA
print '\nMSTA:'
print MSTA
print '\nMSTarrayA:'
print MSTarrayA
#print MSTA[0]
#print type(MSTA[0])
"""


pylab.show()