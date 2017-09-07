#!/usr/bin/env python
import rospy
from distributed.msg import History, HistList, Broadcast
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
from time import sleep
import tf
import scipy.io
import rospkg
import sys
import pylab


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
            set_v.append(k + 1)
        else:
            set_uv.append(k+1)
            pts.append(pts_0[k])



    return set_uv,set_v, pts
# ----------  ----------  ----------  ----------  ----------



def replanning(original_graph, virtual_graph, list_of_H, list_of_robs, list_of_vels):


    # 'change_plan' is True if the new plan is better than the old one
    change_plan = False

    H0 = list_of_H[0] #History of robot 0
    H1 = list_of_H[1] #History of robot 1


    set_uv, set_v, pts = define_subsets(list_of_H,virtual_graph)

    print '\nHere is set_v (visited edges) in original inedexes:'
    print set_v
    print 'Here is set_uv (unvisited edges) in original inedexes:'
    print set_uv, '\n\n'



    # Map the unvisited nodes with new labels
    C = virtual_graph['Ccom']
    Cuv = np.matrix(C) # Reduced cost matrix with only the unvisited edges (unvisited virtual nodes)
    exclude_list = (np.array(set_v)-1).tolist()
    Cuv = np.delete(Cuv, exclude_list, 0) # exclude lines
    Cuv = np.delete(Cuv, exclude_list, 1) # exclude columns
    Cuv = Cuv.tolist()


    # Define the depot virtual nodes for the task assignment
    depots = [set_uv.index(H0.currEdge), set_uv.index(H1.currEdge)]
    speeds = [0.4, 0.55]
    colors = ['b', 'r']

    print '\nHere is depot virtual nodes (new indexes):'
    print depots
    print 'Here is depot virtual nodes (original indexes):'
    print [set_uv[depots[0]], set_uv[depots[1]]], '\n'





    print '\n ----- Task assignment function called -----'
    [sol, C_check0, C_check1] = TA.execute_lp(speeds, depots, colors, Cuv, pts)


    # ----------  ----------  ----------  ----------  ----------  ----------  ----------




    # Map the solution back to the original indexes
    n_uv = len(Cuv)
    m_uv = n_uv*(n_uv-1)
    x1_var = []
    x2_var = []


    F_var = sol.pop()
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
    division[0] = list(set(division[0]))
    division[1] = list(set(division[1]))



    #Map the nodes back to the original indexation
    for k in range(len(division[0])):
        division[0][k] = set_uv[division[0][k]]
    for k in range(len(division[1])):
        division[1][k] = set_uv[division[1][k]]


    # Excludie the used depot virtual nodes (they are edges that were already visited)
    for r in range(2):
        division[r].pop(division[r].index(list_of_H[r].currEdge))


    print '\nAssigned edges for the robots:'
    for k in range(2):
        print 'Subset for robot ' + str(k) + ': ', division[k]
    print '\n'
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------


    #Plot the disconnected graph
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------
    PolC = original_graph['PolC']
    pylab.figure(100)
    pylab.subplot(2, 2, 1)
    pylab.axis('equal')
    pylab.axis(virtual_graph['w_s'])
    pylab.title('Disconnected graph')
    pylab.subplot(2, 2, 2)
    pylab.axis('equal')
    pylab.axis(virtual_graph['w_s'])
    pylab.title('Disconnected graph')
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

        pylab.subplot(2, 2, 1)
        pylab.plot(x, y, 'k--', linewidth=1.0)
        pylab.subplot(2, 2, 2)
        pylab.plot(x, y, 'k--', linewidth=1.0)

        if e+1 in division[0]:
            pylab.subplot(2, 2, 1)
            pylab.plot(x, y, 'b', linewidth=4.0)
        if e+1 in division[1]:
            pylab.subplot(2, 2, 2)
            pylab.plot(x, y, 'r', linewidth=4.0)
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------













    # Call MST for every robot in the communication graph in order to make the graph connected
    print '\n ----- Applying MST to the disconnected subgraphs ------'
    subgraphs = []
    for r in range(2):
        subgraphs.append([])
        pylab.subplot(2,2,r+3)
        subgraphs[r] = MST.MSTconnect(division[r], (list_of_H[r]).nextNode, colors[r])
    for r in range(2):
        print 'Subset of edges for robot ' + str(r) + ': (original indexes)\n', subgraphs[r]
        print 'Start node for robot ' + str(r) + ':', (list_of_H[r]).nextNode
    # ----------  ----------  ----------  ----------  ----------


    # Save the result of TA and MST in a image
    import time
    hour = time.strftime("%Hh%Mm%Ss")
    date = time.strftime("d%dm%my%Y")
    rp = rospkg.RosPack()
    fig_name = rp.get_path('distributed')
    fig_name = fig_name + '/imagesResults/Results_' + date + '_' + hour + '.png'
    pylab.savefig(fig_name)
    # ----------  ----------  ----------  ----------  ----------



    #Plotting the heuristic costs
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------
    for i in range(len(pts)):

        x = pts[i][0]
        y = pts[i][1]

        pylab.subplot(2, 2, 1)
        pylab.text(x, y, ("%.2f" % C_check0[i]), fontsize=8.0)
        pylab.subplot(2, 2, 2)
        pylab.text(x, y, ("%.2f" % C_check1[i]), fontsize=8.0)
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------


    # Choose if the result of the planning will be platted or not
    SHOW_NEW_PLAN = False
    #SHOW_NEW_PLAN = True
    if SHOW_NEW_PLAN:
        pylab.show()





    # Cal CPP for every graph generated by the MST based algorithm
    print '\nApplying CPP to the connected subgraphs'
    Hole_path_list = []
    for k in range(2):
        Hole_path_list.append([])
        current_node = (list_of_H[k]).nextNode
        edges_listOfTuples = myLib.write_listOfTuples(original_graph, subgraphs[k])
        Hole_path_list[k] = cppsolver.CPP(sorted(edges_listOfTuples), current_node)
    for k in range(2):
        print 'Route for robot ' + str(k), ': ', Hole_path_list[k]
    print '\n'
    # ----------  ----------  ----------  ----------  ----------



    # Create the new list of histories (STILL MUST ADAPT TO A GENERAL NUMBER OF ROBOS)
    new_Hists = []
    H = History()
    H.e_v = set_v
    H.e_uv = division[0]
    H.e_g = division[1]
    H.T_a = []
    H.T_f = []
    H.lastMeeting = [0, 1] #AAAAAAAAAAA
    new_Hists.append(H)

    H = History()
    H.e_v = set_v
    H.e_uv = division[1]
    H.e_g = division[0]
    H.T_a = []
    H.T_f = []
    H.lastMeeting = [0, 1] #AAAAAAAAAAA
    new_Hists.append(H)
    # ----------  ----------  ----------








    # Check if the new plan is better than the previous one

    #CHECK STUFF HERE   !!!!!!!!!!  !!!!!!!!!!  !!!!!!!!!!
    #if something:
    #    change_plan = True

    # ----------  ----------  ----------  ----------  ----------









    #return the new plans
    return change_plan, Hole_path_list, new_Hists
# ----------  ----------  ----------  ----------  ----------








"""

original_graph = myLib.read_graph('Original_graph_36.mat')
n = original_graph['n']
nodes = original_graph['nodes']
C = original_graph['C']
PathM = original_graph['PathM']
w_s = original_graph['w_s']
PolC = original_graph['PolC']

virtual_graph = myLib.read_graph('Virtual_graph_36.mat')


list_of_H = []

H = History()
H.e_v = [1, 14, 13, 19, 20, 18, 21, 36, 22]
H.e_uv = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15, 16, 17, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 37, 38, 39, 40]
H.e_g = []
H.T_a = []
H.T_f = []
H.currEdge = 22
H.pose = [6.733957290649414, 3.901658535003662, -2.4089980125427246]
list_of_H.append(H)

H = History()
H.e_v = [33, 29, 26, 27, 28, 25]
H.e_uv = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 30, 31, 32, 34, 35, 36, 37, 38, 39, 40]
H.e_g = []
H.T_a = []
H.T_f = []
H.currEdge = 25
H.pose =  [9.91314697265625, 5.39360237121582, 3.084688425064087]
list_of_H.append(H)

#list_of_H = [H0, H1]

list_of_robs = [0, 1]
list_of_vels = [1, 1.1]


print '\n\nlist_of_H:'
print list_of_H
print 'list_of_robs:'
print list_of_robs

print 'list_of_vels:'
print list_of_vels, '\n\n'

change_plan, Hole_path_list, new_Hists = replanning(original_graph,virtual_graph,list_of_H, list_of_robs, list_of_vels)



"""







