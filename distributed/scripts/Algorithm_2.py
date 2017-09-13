#!/usr/bin/env python
import rospy
from distributed.msg import History, HistList, Broadcast, Intlist
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

    R = len(list_of_H)
    for r in range(R):
        exec ('H%d = list_of_H[%d]' % (r, r))
    #H0 = list_of_H[0] #History of robot 0
    #H1 = list_of_H[1] #History of robot 1

    set_uv = []
    set_v = []
    set_g = []
    pts = []
    for k in range(len(pts_0)): #loop over all of the orignal edges
        if k+1 == H0.currEdge or k+1 == H1.currEdge: #Add the current edges in the unvisited set
            set_uv.append(k + 1)
            pts.append(pts_0[k])
        elif (k+1 in H0.e_v or k+1 in H1.e_v or k+1 in H0.T_f or k+1 in H1.T_f): #Forgot about th visited edges (we are done with them)
            set_v.append(k + 1)
        #elif((k+1 in H0.e_g and (not k+1 in H1.e_uv)) or (k+1 in H1.e_g and (not k+1 in H0.e_uv))): #If the edge is assigned to other robot -> forget it
        elif ((k + 1 in H0.e_g ) or (k + 1 in H1.e_g)):  # If the edge is assigned to other robot -> forget it
            if (not(H0.id in H0.T_a[k].data or H0.id in H1.T_a[k].data or H1.id in H0.T_a[k].data or H1.id in H1.T_a[k].data)):
                print '-> Add to e_g: ', k+1
                set_g.append(k + 1)
            else:
                set_uv.append(k + 1)
                pts.append(pts_0[k])
        else: #In other case, we should search on this edge
            set_uv.append(k+1)
            pts.append(pts_0[k])



    #print 'AAAAAAAAAAAAA: set_g - (inside function)'
    #print set_g

    return set_uv, set_v, set_g, pts
# ----------  ----------  ----------  ----------  ----------









def replanning(original_graph, virtual_graph, list_of_H):

    #print 'Here is list_of_H'
    #print list_of_H



    colors_0 = ['b', 'r', 'g', 'y']

    # 'change_plan' is True if the new plan is better than the old one
    change_plan = False

    R = len(list_of_H)

    speeds = []
    search_speeds = []
    colors = []
    for r in range(R):
        exec ('H%d = list_of_H[%d]' % (r, r))
        exec ('speeds.append(H%d.specs[0])' % r)
        exec ('search_speeds.append(H%d.specs[1])' % r)
        exec ('colors.append(colors_0[H%d.id])' % r)






    set_uv, set_v, set_g, pts = define_subsets(list_of_H,virtual_graph)

    print '\nHere is set_v (visited edges) in original inedexes:'
    print set_v
    print 'Here is set_uv (unvisited edges) in original inedexes:'
    print set_uv
    print 'Here is set_g (assigned edges) in original inedexes:'
    print set_g, '\n\n'





    # Map the unvisited nodes with new labels
    C = virtual_graph['Ccom'] #matrix with the length costs
    Cuv = np.matrix(C) # Reduced cost matrix with only the unvisited edges (unvisited virtual nodes)
    C = virtual_graph['C_sp'] #matrix with the number of search points
    Cuv_sp = np.matrix(C) # Reduced cost matrix with only the unvisited edges (unvisited virtual nodes)
    #exclude_list = (np.array(set_v) - 1).tolist()
    exclude_list = (np.array(set_v) - 1).tolist() + (np.array(set_g) - 1).tolist()
    Cuv = np.delete(Cuv, exclude_list, 0) # exclude lines
    Cuv = np.delete(Cuv, exclude_list, 1) # exclude columns
    Cuv = Cuv.tolist()
    Cuv_sp = np.delete(Cuv_sp, exclude_list, 0) # exclude lines
    Cuv_sp = np.delete(Cuv_sp, exclude_list, 1) # exclude columns
    Cuv_sp = Cuv_sp.tolist()



    # Define the depot virtual nodes for the task assignment
    depots = []
    for r in range(R):
        exec ('depots.append(set_uv.index(H%d.currEdge))' % r)



    print '\nHere is depot virtual nodes (new indexes):'
    print depots
    print 'Here is depot virtual nodes (original indexes):'
    depots_ori = []
    for r in range(R):
        depots_ori.append(set_uv[depots[r]])
    print depots_ori, '\n'





    print '\n ----- Task assignment function called -----'
    [sol, C_check0, C_check1] = TA.execute_lp(speeds, search_speeds, depots, colors, Cuv, Cuv_sp, pts)




    # ----------  ----------  ----------  ----------  ----------  ----------  ----------




    # Map the solution back to the original indexes
    n_uv = len(Cuv)
    m_uv = n_uv*(n_uv-1)
    for r in range(R):
        exec('x%d_var = []' % r)



    F_var = sol.pop()
    for r in range(R):
        for k in range(m_uv):
            exec('x%d_var.insert(0,sol.pop())' % (R-r-1))





    division = [[],[]]
    k = -1
    for i in range(n_uv):
        for j in range(n_uv):
            if i!=j:
                k = k+1
                for r in range(R):
                    exec('xAux_var = x%d_var' % r)
                    if (xAux_var[k] == 1):
                        division[r].append(i)
                        division[r].append(j)
    for r in range(R):
        division[r] = list(set(division[r]))




    #Map the nodes back to the original indexation
    #print 'Here is set_uv'
    #print set_uv
    #print 'Here is division'
    #print division
    for r in range(R):
        for k in range(len(division[r])):
            #print r, k
            division[r][k] = set_uv[division[r][k]]



    # Excludie the used depot virtual nodes (they are edges that were already visited)
    for r in range(R):
        division[r].pop(division[r].index(list_of_H[r].currEdge))

    print '\nAssigned edges for the robots:'
    for r in range(R):
        #print 'Subset for robot ' + str(r) + ': ', division[r]
        print 'Subset for robot ' + str(list_of_H[r].id) + ': ', division[r]
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
            pylab.plot(x, y, colors[0], linewidth=3.0)
        if e+1 in division[1]:
            pylab.subplot(2, 2, 2)
            pylab.plot(x, y, colors[1], linewidth=3.0)
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------













    # Call MST for every robot in the communication graph in order to make the graph connected
    print '\n ----- Applying MST to the disconnected subgraphs ------'
    subgraphs = []
    for r in range(R):
        subgraphs.append([])
        pylab.subplot(2,2,r+3)
        subgraphs[r] = MST.MSTconnect(division[r], (list_of_H[r]).nextNode, colors[r], True)
    for r in range(R):
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
    for k in range(R):
        Hole_path_list.append([])
        current_node = (list_of_H[k]).nextNode
        edges_listOfTuples = myLib.write_listOfTuples(original_graph, subgraphs[k])
        Hole_path_list[k] = cppsolver.CPP(sorted(edges_listOfTuples), current_node)
    for k in range(R):
        print 'Route for robot ' + str(k), ': ', Hole_path_list[k]
    print '\n'
    # ----------  ----------  ----------  ----------  ----------


    #Create list T_a
    pts_0 = virtual_graph['nodes']
    T_a0 = [Intlist()]
    for k in range(len(PolC[0])-1):
        IL = Intlist()
        T_a0.append(IL)
    #print 'Here is T_a0'
    #print T_a0
    for k in range(len(pts_0)):
        T_a0[k].data = list(list_of_H[0].T_a[0].data) + list(list_of_H[1].T_a[1].data)
        T_a0[k].data = list(T_a0[k].data)
        #print 'Edge: ', k + 1
        if k + 1 in set_uv:  # if in unvisite list
            if k + 1 in division[0]:  # if in the assigned to the other one
                T_a0[k].data.append(list_of_H[0].id)
                #print 'Added robot', list_of_H[0].id, 'to edge', k + 1
            elif k + 1 in division[1]:
                T_a0[k].data.append(list_of_H[1].id)
                #print 'Added robot', list_of_H[1].id, 'to edge', k + 1
        T_a0[k].data = list(set(T_a0[k].data))
    #print 'Here is T_a0'
    #print T_a0


    #Create list T_f
    """
    T_f0 = [Intlist()]
    for k in range(len(PolC[0])-1):
        IL = Intlist()
        T_f0.append(IL)
    for k in range(len(pts_0)):
        T_f0[k].data = list(list_of_H[0].T_f[0].data) + list(list_of_H[1].T_f[1].data)
        T_f0[k].data = list(T_f0[k].data)
        if k + 1 in set_v:  # if in unvisite list
                T_a0[k].data.append(list_of_H[0].id)
        T_a0[k].data = list(set(T_a0[k].data))
    """
    T_f0 = []
    for k in range(len(list_of_H[0].T_f)):
        T_f0.append(list_of_H[0].T_f[k])
    for k in range(len(list_of_H[1].T_f)):
        T_f0.append(list_of_H[1].T_f[k])
    for k in range(len(set_v)):
        T_f0.append(set_v[k])
    T_f0 = list(set(T_f0))




    # Create the new list of histories (STILL MUST ADAPT TO A GENERAL NUMBER OF ROBOS)
    new_Hists = []
    for r in range(R):
        H = History()
        H.id = list_of_H[r].id
        H.e_v = set_v
        H.e_uv = division[r]
        for r2 in range(R):
            if r2!=r:
                H.e_g = division[r2] + set_g
        """
        IL = Intlist()
        H.T_a = [IL for i in range(len(PolC[0]))]
        for k in range(len(pts_0)):
            aux = list_of_H[0].T_a[0].data + list_of_H[1].T_a[1].data #list of robots assigned to edge k+1
            if k+1 in set_uv: #if in unvisite list
                if k+1 in division[-r+1]: #if in the assigned to the other one
                    aux = list(aux) + [list_of_H[-r+1].id] #add the id of the other
                    print 'Added robot', list_of_H[-r+1].id, 'to edge', k+1
            aux = list(set(aux)) #exclude repetitions
            H.T_a[k].data = aux #atribute to the T_a list of the robot
        """
        H.T_a = T_a0
        #H.T_f = []
        H.T_f = T_f0
        H.lastMeeting = []
        for r2 in range(R):
            H.lastMeeting.append(list_of_H[r2].id)
        new_Hists.append(H)








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
H.id = 0
H.specs = [0.55, pi/2]
H.e_v = [1, 14, 13, 19, 20, 18, 21, 36, 22]
H.e_uv = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15, 16, 17, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 37, 38, 39, 40]
H.e_g = []
H.T_a = []
H.T_f = []
H.currEdge = 22
H.pose = [6.733957290649414, 3.901658535003662, -2.4089980125427246]
list_of_H.append(H)

H = History()
H.id = 1
H.specs = [0.55, pi/2]
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
#list_of_vels = [1, 1.1]


print '\n\nHere is list_of_H:'
print list_of_H
print 'Here is list_of_robs:'
print list_of_robs


change_plan, Hole_path_list, new_Hists = replanning(original_graph,virtual_graph,list_of_H)



"""







