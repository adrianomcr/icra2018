#!/usr/bin/env python
import rospy
from distributed.msg import History, Intlist
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
from time import sleep
import tf
import scipy.io
import rospkg
import sys


rp = rospkg.RosPack()
path = rp.get_path('distributed')
path = path + '/CPP'
sys.path.insert(0, path)
import cppsolver
import library2018 as myLib
import Task_Assignment as TA
import MST




def choose_depots():



    return
# ----------  ----------  ----------  ----------  ----------




def define_subsets(list_of_H,virtual_graph):

    pts_0 = virtual_graph['nodes']

    H0 = list_of_H[0] #History of robot 0
    H1 = list_of_H[1] #History of robot 1

    set_uv = []
    set_v = []
    pts = []
    for k in range(len(pts_0)):
        if (k+1 in H0.e_v or k+1 in H1.e_v) and k+1 != H0.currEdge and k+1 != H1.currEdge:
        #if (k+1 in H0.e_v or k+1 in H1.e_v):
            set_v.append(k + 1)
        else:
            set_uv.append(k+1)
            pts.append(pts_0[k])
        """
        if not k+1 in H0.e_v and not k+1 in H1.e_v:
            set_uv.append(k+1)
            pts.append(pts_0[k])
        else:
            set_v.append(k + 1)
        """

    #set_v = []
    #for k in range(len(pts_0)):
    #    if (not k + 1 in set_uv):
    #        set_v.append(k + 1)

    return set_uv,set_v, pts
# ----------  ----------  ----------  ----------  ----------



def replanning(original_graph, virtual_graph, list_of_H, list_of_robs, list_of_vels):


    # 'cahnde_plan' is True if the new plan is better than the old one
    change_plan = False

    H0 = list_of_H[0] #History of robot 0
    H1 = list_of_H[1] #History of robot 1


    set_uv, set_v, pts = define_subsets(list_of_H,virtual_graph)

    # Map the unvisited nodes with new labels
    C = virtual_graph['Ccom']
    Cuv = np.matrix(C) # Reduced cost matrix with only the unvisited edges (unvisited virtual nodes)
    exclude_list = (np.array(set_v)-1).tolist()
    Cuv = np.delete(Cuv, exclude_list, 0) # exclude lines
    Cuv = np.delete(Cuv, exclude_list, 1) # exclude columns
    Cuv = Cuv.tolist()



    # Call the task assignment function
    depots = [set_uv.index(H0.currEdge), set_uv.index(H1.currEdge)]
    speeds = [1, 1.1]
    colors = ['b', 'r']

    """
    print '\n'
    print 'depots = ', depots
    print 'speeds = ', speeds
    print 'size of Cuv = ', len(Cuv), 'x', len(Cuv[0])
    print '\n'
    """

    print '\nTask assignment function called'
    sol = TA.execute_lp(speeds, depots, colors, Cuv, pts)
    print 'Returned from Task assignment function\n'

    """
    print '\nHere is Cuv:'
    print Cuv

    print '\nHere is set_uv:'
    print set_uv
    print '\n'

    print '\nHere is set_v:'
    print set_v
    print '\n'
    
    print '\nlen(set_uv) = ', len(set_uv)
    print 'len(set_v) = ', len(set_v)
    print 'len(Cuv) = ', len(Cuv)
    print 'len(Cuv[0]) = ', len(Cuv[0])
    print '\n'
    """
    # ----------  ----------  ----------  ----------  ----------




    # Map the solution back to the original indexes
    n_uv = len(Cuv)
    m_uv = n_uv*(n_uv-1)
    u_var = []
    x1_var = []
    x2_var = []

    """
    print "len(sol)", len(sol)
    print "n_uv", n_uv
    print "m_uv", m_uv
    print "2*m_uv+1", n_uv+2*m_uv+1
    """
    F_var = sol.pop()
    #for k in range(n_uv):
    #    u_var.insert(0,sol.pop())
    for k in range(m_uv):
        x2_var.insert(0,sol.pop())
    for k in range(m_uv):
        x1_var.insert(0,sol.pop())

    division = [[],[]]
    k = -1
    for i in range(n_uv):
        for j in range(n_uv):
            if i!=j:
                k = k+1
                if (x1_var[k] == 1):
                    division[0].append(i)
                    division[0].append(j)
                if (x2_var[k] == 1):
                    division[1].append(i)
                    division[1].append(j)
    print 'division: (' + str(len(division[0])) + ',' + str(len(division[1])) + ')'
    print division
    division[0] = list(set(division[0]))
    division[1] = list(set(division[1]))
    #OBS: divisioin is a set of nodes of the virtual graph (edges of the original graph)

    """
    print 'division - indexes of subgraph:'
    print division
    """

    #Map the nodes bak to the original indexation

    print 'AAAAAAAAAAAAAAAAAAAA'
    print 'set_uv: (' + str(len(set_uv)) + ')'
    print set_uv
    print 'division: (' + str(len(division[0])) + ',' + str(len(division[1])) + ')'
    print division
    print 'AAAAAAAAAAAAAAAAAAAA'

    for k in range(len(division[0])):
        division[0][k] = set_uv[division[0][k]]
    for k in range(len(division[1])):
        division[1][k] = set_uv[division[1][k]]
    print '\nDeviding edges between the robots'
    for k in range(2):
        print 'Subset for robot ' + str(k) + ': ', division[k]
    # ----------  ----------  ----------  ----------  ----------






    # Call MST for every robot in the communication graph
    print '\nApplying MST to the disconnected subgraphs'
    subgraphs = []
    for k in range(2):
        subgraphs.append([])
        subgraphs[k] = MST.MSTconnect(division[k], colors[k])
    for k in range(2):
        print 'Subset of edges for robot ' + str(k) + ': ', subgraphs[k]
    # ----------  ----------  ----------  ----------  ----------





    # Cal CPP for every graph generated by the MST base algorithm
    print '\nApplying CPP to the connected subgraphs'
    Hole_path_list = []
    for k in range(2):
        Hole_path_list.append([])
        pose = (list_of_H[k]).pose

        #print 'pose = ', pose
        [current_node, dist] = myLib.get_current_node(original_graph, [pose[0], pose[1]])
        current_node = current_node+1
        #print 'current_node = ', current_node
        edges_listOfTuples = myLib.write_listOfTuples(original_graph, subgraphs[k])
        Hole_path_list[k] = cppsolver.CPP(sorted(edges_listOfTuples), current_node)
    for k in range(2):
        print 'Route for robot ' + str(k), ': ', Hole_path_list[k]
    # ----------  ----------  ----------  ----------  ----------







    # Check if the new plan is better than the previous one

    #CHECK STUFF HERE   !!!!!!!!!!  !!!!!!!!!!  !!!!!!!!!!

    # ----------  ----------  ----------  ----------  ----------









    #return change_plan
    return change_plan, division
# ----------  ----------  ----------  ----------  ----------





