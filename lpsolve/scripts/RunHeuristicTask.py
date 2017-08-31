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
#import libraries2018
import TA_function







def find_cluster(list_tree,id,level):
    matriz = list_tree[level]
    #print '\nlevel = ', level
    #print 'matriz[id] = ', matriz[id]
    if (matriz[id][1] == -1):
        result = matriz[id][0]
        #print 'AQUI ESTA O VALOR:', result
        return result
    else:
        x = find_cluster(list_tree,matriz[id][1],level+1)
        return x



n = 80 #number of nodes
m = n * (n - 1) #number of edges
R = 1 #number of robots
iterations = 4 #number of times the LP is going to run


depots = {0: [0],
          1: [0, 1],
          2: [4, 1, 6],
          #2: [38, 38, 38],
          3: [0, 1, 2, 3],
          }
speeds = {0: [1],
          1: [1, 1],
          2: [1, 1.1, 1.2],
          3: [1, 1, 1, 1],
          }
colors = {0 : 'r',
          1 : 'g',
          2: 'y',
          3: 'm',
          4: 'c',
          5: 'b',
          6: 'k',
          7: 'w',
          }

depots = depots[R-1]
speeds = speeds[R-1]



#Definition of the nodes
"""
p1 = [0.88, -0.06]
p2 = [-2.39, -0.81]
p3 = [-1.78, -0.46]
p4 = [0.61, 1.15]
p5 = [2.62, -1.25]
p6 = [-0.08, 1.08]
p7 = [0.65, 1.84]
p8 = [-2.45, -0.37]
p9 = [-0.79, 0.75]
p10 = [-2.47, 1.08]
p11 = [0.33, -1.06]
p12 = [-2.10, 1.14]
p13 = [-1.41, 1.19]
p14 = [1.71, -0.81]
p15 = [-1.57, 0.10]
p16 = [0.90, -1.57]
p17 = [-2.37, -1.56]
p18 = [-0.42, 0.79]
p19 = [1.51, -0.26]
p20 = [-0.78, -1.16]
p21 = [1.68, -0.44]
p22 = [-1.55, -0.38]
p23 = [-2.42, -1.47]
p24 = [2.65, 1.82]
p25 = [0.45, -1.76]
p26 = [-1.59, -0.59]
p27 = [1.93, -1.94]
p28 = [-2.74, -1.32]
p29 = [0.89, 0.93]
p30 = [0.89, -0.20]
p31 = [0.28, -0.81]
p32 = [1.47, -1.24]
p33 = [1.12, -1.27]
p34 = [-0.79, 0.50]
p35 = [1.68, -1.68]
p36 = [2.58, 1.10]
p37 = [-0.08, -0.26]
p38 = [-0.32, -0.77]
p39 = [0.05, 0.04]
p40 = [1.91, 1.18]
p41 = [0, 0]
"""
p1 = [1.888, 1.623]
p2 = [-2.238, 1.654]
p3 = [0.794, -1.610]
p4 = [-1.329, 0.188]
p5 = [2.745, 1.860]
p6 = [-2.054, 1.882]
p7 = [2.743, -0.058]
p8 = [1.802, -1.432]
p9 = [-0.469, 1.663]
p10 = [1.753, 1.838]
p11 = [0.934, -1.857]
p12 = [2.095, 1.736]
p13 = [1.072, 1.031]
p14 = [1.459, -0.431]
p15 = [0.933, -1.315]
p16 = [1.236, -1.873]
p17 = [-1.338, -1.815]
p18 = [-2.417, 1.294]
p19 = [1.169, -0.732]
p20 = [2.701, -1.862]
p21 = [-0.368, -0.474]
p22 = [1.593, 1.181]
p23 = [-1.879, -0.041]
p24 = [-0.326, 0.585]
p25 = [1.256, 1.019]
p26 = [-1.344, 0.719]
p27 = [0.931, -1.350]
p28 = [-2.286, -0.007]
p29 = [2.758, -0.638]
p30 = [0.512, -1.105]
p31 = [1.508, -0.980]
p32 = [0.036, 0.796]
p33 = [2.345, 1.837]
p34 = [0.283, -1.446]
p35 = [-2.104, -0.970]
p36 = [2.044, -0.983]
p37 = [1.886, -1.026]
p38 = [2.576, -0.600]
p39 = [-1.820, -0.996]
p40 = [0.696, -0.107]
p41 = [-0.890, 1.323]
p42 = [0.512, 0.199]
p43 = [2.503, -0.857]
p44 = [1.543, 1.015]
p45 = [-0.717, 0.271]
p46 = [-2.545, -1.784]
p47 = [0.185, 1.117]
p48 = [2.604, -1.480]
p49 = [0.413, -0.122]
p50 = [-2.929, -0.652]
p51 = [-2.027, 1.177]
p52 = [-1.133, 0.114]
p53 = [-2.006, 0.408]
p54 = [-1.422, 0.616]
p55 = [1.135, 0.993]
p56 = [-0.297, -1.665]
p57 = [-1.626, 1.653]
p58 = [-2.086, 1.303]
p59 = [0.230, 1.985]
p60 = [-2.531, -0.229]
p61 = [-2.360, 1.848]
p62 = [-2.972, 1.100]
p63 = [1.904, 1.475]
p64 = [-2.493, -0.401]
p65 = [-1.441, 1.200]
p66 = [-0.412, 1.643]
p67 = [-1.909, -0.945]
p68 = [-2.127, -1.456]
p69 = [2.216, 0.319]
p70 = [0.299, -1.420]
p71 = [2.118, 0.488]
p72 = [-0.894, 0.053]
p73 = [-0.589, -1.696]
p74 = [-1.561, -1.507]
p75 = [-1.897, -1.040]
p76 = [-0.496, -1.801]
p77 = [2.416, 1.779]
p78 = [-0.055, -0.043]
p79 = [-0.974, 1.600]
p80 = [-0.785, -1.555]

pts = []
for k in range(n):
    pts.append(eval('p%d' % (k + 1)))
pts = np.array(pts)
# ----------  ----------  ----------




list_of_clusters_points = []
list_of_solutioins = []
list_of_sizes = []

pts_0 = pts

list_tree = []

count_time = time.time()
for run in range(iterations):

    #time.sleep(0.5)

    n = len(pts)

    list_of_clusters_points.append(pts) # adding the first cluster (original graph)

    matriz = []
    for k in range(n):
        matriz.append([k,-1])

    tour = TA_function.execute_lp(n,R,pts,colors[run])

    print 'Here is tour:'
    print tour

    list_of_solutioins.append(tour)


    #Compute the new set of points
    tour = tour-1
    tour = tour.tolist()

    n_clusters = 0


    t = time.time()
    new_pts = []
    nodes_included = []
    print '\n'
    while (len(nodes_included) != n and time.time()-t < 1):
        #Find a node not included
        for k in range(n):
            flag = False
            if(not(k in nodes_included)):
                node_start = k
                flag = True
                break
        group = []
        if(flag): #If there still node to be processed
            node = node_start
            nodes_included.append(node)
            group.append([pts[node][0],pts[node][1]])
            matriz[node][1] = n_clusters
            while(True and time.time()-t < 1):
                id = tour[0].index(node)
                next_node = tour[1][id]
                if(next_node != node_start):
                    # there is more nodes in the cluster
                    node = next_node
                    nodes_included.append(node)
                    group.append([pts[node][0], pts[node][1]])
                    matriz[node][1] = n_clusters
                else:
                    # subtour closed
                    cx = 0 #x center of cluster
                    cy = 0 #y center of cluster
                    for k in range(len(group)):
                        cx = cx + group[k][0]
                        cy = cy + group[k][1]
                    cx = cx / len(group)
                    cy = cy / len(group)
                    new_pts.append([cx, cy])
                    n_clusters = n_clusters + 1

                    break

    list_tree.append(matriz)


    pts = new_pts

    #print 'Here is new pts:'
    #print pts


count_time = time.time()-count_time

n = len(pts)
matriz = []
for k in range(n):
    matriz.append([k,-1])
list_tree.append(matriz)


#print '\nlen(list_tree) = ', len(list_tree)
#print '\nlist_tree[0] = ', list_tree[0]
#print '\nlist_tree[1] = ', list_tree[1]
#print '\nlist_tree[2] = ', list_tree[2]
#print '\nlist_tree[3] = ', list_tree[3]
#print '\nlist_tree[4] = ', list_tree[4]


final_cluster = []
for k in range(len(pts_0)):
    cluster_of_k = find_cluster(list_tree, k, 0)
    final_cluster.append([k,cluster_of_k])

#"""
for k in range(len(pts_0)):
    pylab.plot(pts_0[k][0], pts_0[k][1], 'ok', markersize=3.0)
if(len(list_tree[iterations]) <= 8):
    for k in range(len(pts_0)):
        cor = colors[final_cluster[k][1]]
        pylab.plot(pts_0[k][0], pts_0[k][1], 'o', markersize=10.0, color=cor)
#"""


#x = find_cluster(list_tree,15,0)
#print 'Cluster of 15 is: ', x

print 'Elapsed time: ', count_time

pylab.show()

