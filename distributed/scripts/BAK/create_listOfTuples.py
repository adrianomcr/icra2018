#!/usr/bin/env python
import rospy
import os
import sys
import rospkg

rp = rospkg.RosPack()
path = rp.get_path('distributed')
path = path + '/CPP'
sys.path.insert(0, path)
import cppsolver
import libraries2018 as mylib



list_active = range(1,6+1) #INPUT - INDEXES OF EDGES
print 'List of active edges:', list_active

original_graph = mylib.read_graph('Original_graph_36.mat') #INPUT
n = original_graph['n']
nodes = original_graph['nodes']
C = original_graph['C']
PathM = original_graph['PathM']
w_s = original_graph['w_s']
PolC = original_graph['PolC']

list_tuple = []
for k in range(len(PolC[0])):
    if(k+1 in list_active):
        [fr, to, cx, cy, cost] = mylib.getCoefs(k, PolC)
        list_tuple.append((fr,to,cost))



print 'Here is the list of tuples:'
print list_tuple