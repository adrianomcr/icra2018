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

"""
LP_MAKER  Makes mixed integer linear programming problems.

SYNOPSIS: lp_handle = lp_maker(f,a,b,e,vlb,vub,xint,scalemode,setminim)
  make the MILP problem
    max v = f'*x
      a*x <> b
        vlb <= x <= vub
        x(int) are integer

ARGUMENTS: The first four arguments are required:
        f: n vector of coefficients for a linear objective function.
        a: m by n matrix representing linear constraints.
        b: m vector of right sides for the inequality constraints.
        e: m vector that determines the sense of the inequalities:
                  e(i) < 0  ==> Less Than
                  e(i) = 0  ==> Equals
                  e(i) > 0  ==> Greater Than
      vlb: n vector of non-negative lower bounds. If empty or omitted,
           then the lower bounds are set to zero.
      vub: n vector of upper bounds. May be omitted or empty.
     xint: vector of integer variables. May be omitted or empty.
scalemode: Autoscale flag. Off when 0 or omitted.
 setminim: Set maximum lp when this flag equals 0 or omitted.

OUTPUT: lp_handle is an integer handle to the lp created.
"""




def execute_lp(n,R,pts,cor):

    m = n * (n - 1) #number of edges

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
              3: 'm'
              }

    depots = depots[R-1]
    speeds = speeds[R-1]


    w_s = [-3, 3, -2, 2]
    #Definition of the edges
    E = [[0 for i in range(2)] for j in range(m)]
    k = 0
    for i in range(n):
        for j in range(n):
            if(i != j):
                E[k][0] = i
                E[k][1] = j
                k = k+1
    """
    print '\nE = '
    for k in range(m):
        print E[k][:]
    #"""
    # ----------  ----------  ----------



    #Definition of the cost matrix
    C = [[0 for i in range(n)] for j in range(n)]
    for i in range(n):
        for j in range(n):
            C[i][j] = sqrt((pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2)
    C = np.matrix(C)
    for r in range(R):
        exec('C_%d = C/float(speeds[r])' % (r))
        exec('C_%d = C_%d.tolist()' % (r,r))
    C = C.tolist()
    """
    for r in range(R):
        print ''
        exec("print 'C_%d = '" % r)
        for k in range(n):
            exec ("print C_%d[k][:]" % r)
    """
    #  ----------  ----------  ----------

    #Definition of the cost matrix WITH THE HEURISTIC COST
    """
    for r in range(R):
        C = [[0 for i in range(n)] for j in range(n)]
        for i in range(n):
            for j in range(n):
                C[i][j] = sqrt((pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2)
                medium = [(pts[i][0] + pts[j][0])/2, (pts[i][1] + pts[j][1])/2]
                C[i][j] = C[i][j] + sqrt((medium[0] - pts[depots[r]][0]) ** 2 + (medium[1] - pts[depots[r]][1]) ** 2)
        C = np.matrix(C)
        exec ('C_%d = C/float(speeds[r])' % (r))
        exec ('C_%d = C_%d.tolist()' % (r, r))
    C = C.tolist()
    """
    """
    for r in range(R):
        print ''
        exec("print 'C_%d = '" % r)
        for k in range(n):
            exec ("print C_%d[k][:]" % r)
    """
    #  ----------  ----------  ----------



    #Definition of the x variables bounds
    vlb = []
    vub = []
    #Bouds of "X" variables
    for i in range(R*m):
        vlb.append(0)
        vub.append(1)
    # Bouds of "F" variable
    vlb.append(0*n)
    vub.append(1000*n) #big positive number
    # ----------  ----------  ----------


    #Definition of the cost vector
    c = []
    #Cost of "x" variables
    for k in range(R*m):
        c.append(0)
    # Cost of "F" variable
    c.append(1)
    """
    #Cost of "x" variables
    for r in range(R):
        for i in range(n):
            for j in range(n):
                if (i != j):
                    #c.append(C[i][j] / speeds[r])
                    exec('c.append(C_%d[%d][%d])' % (r,i,j))
    #Cost of the "F" variable
    c.append(0)
    """
    # ----------  ----------  ----------




    # "Arival constraints" ----------  ----------  ----------
    Aeq1 = [[0 for i in range(R*m)] for j in range(n-len(set(depots)))]
    count_line = -1
    for j in range(n): # 0 until n-1
        if(not j in depots): # if j belongs to V^
            count_line = count_line + 1
            for r in range(R):
                for k in range(m):  # 0 until m-1
                    if(E[k][1] == j):
                        Aeq1[count_line][r*m+k] = 1
    beq1 = [1 for i in range(n-len(set(depots)))]
    """
    print '\nAeq1 (',len(Aeq1),'x',len(Aeq1[0]),') = '
    #print np.matrix(Aeq1)
    for k in range(len(Aeq1)):
        print Aeq1[k][:]
    print '\nbeq1 = ', beq1
    """
    #----------  ----------  ----------  ----------  ----------  ----------



    #"Flux conservation constraints" ----------  ----------  ----------
    Aeq2 = [[0 for i in range(R*m)] for j in range(R*n)]
    for r in range(R):
        for p in range(n): # 0 until n-1
            for k in range(0, m):  # 0 until m-1
                if (E[k][1] == p):
                    Aeq2[r*n+p][r*m+k] = 1
                if (E[k][0] == p):
                    Aeq2[r*n+p][r*m+k] = -1
    beq2 = [0 for i in range(R*n)]
    """
    print '\nAeq2 (',len(Aeq2),'x',len(Aeq2[0]),') = '
    for k in range(len(Aeq2)):
        print Aeq2[k][:]
    print '\nbeq2 = ', beq2
    """
    #----------  ----------  ----------  ----------  ----------  ----------



    #"Depot constraints" ----------  ----------  ----------
    Aeq3 = [[0 for i in range(R*m)] for j in range(R)]
    for r in range(R):
        for k in range(0, m):  # 0 until m-1
            if (E[k][1] == depots[r]):
                Aeq3[r][r*m+k] = 1
    beq3 = [1 for i in range(R)]
    """
    print '\nAeq3 (',len(Aeq3),'x',len(Aeq3[0]),') = '
    for k in range(len(Aeq3)):
        print Aeq3[k][:]
    print '\nbeq3 = ', beq3
    """
    #----------  ----------  ----------  ----------  ----------  ----------





    #Joining all equality constraints ----------  ----------  ----------
    Aeq = [];
    for line in Aeq1:
        Aeq.append(line)
    for line in Aeq2:
        Aeq.append(line)
    for line in Aeq3:
        Aeq.append(line)
    beq = []
    for number in beq1:
        beq.append(number)
    for number in beq2:
        beq.append(number)
    for number in beq3:
        beq.append(number)
    #Adding 'F' variable
    Aeq = np.matrix(Aeq)
    zerosMatrix = np.zeros((len(Aeq),1))
    zerosMatrix = np.matrix(zerosMatrix)
    Aeq = np.concatenate((Aeq.T,zerosMatrix.T))
    Aeq = Aeq.T
    Aeq = Aeq.tolist()
    """
    print '\nAeq (',len(Aeq),'x',len(Aeq[0]),') = '
    for k in range(len(Aeq)):
        print Aeq[k][:]
    print '\nsize of Aeq:', len(Aeq),'x',len(Aeq[0])
    print '\nbeq = ', beq
    """
    #----------  ----------  ----------  ----------  ----------  ----------




    #MinMax constraints ----------  ----------  ----------
    AF = []
    bF = []
    AF = [[0 for i in range(R*m)] for j in range(R)]
    for r in range(R):
        k = -1
        for i in range(n):
            for j in range(n):
                if (i != j):
                    k = k + 1
                    AF[r][k+r*m] = C[i][j]
                    exec('AF[r][k+r*m] = C_%d[i][j]' % r)
    bF = [0 for i in range(R)]
    #Adding 'F' variable
    AF = np.matrix(AF)
    onesMatrix = np.ones((len(AF),1))
    onesMatrix = np.matrix(onesMatrix)*(-1)
    AF = np.concatenate((AF.T,onesMatrix.T))
    AF = AF.T
    AF = AF.tolist()
    """
    print '\nAF (',len(AF),'x',len(AF[0]),') = '
    for k in range(len(AF)):
        print AF[k][:]
    print '\nsize of AF:', len(AF),'x',len(AF[0])
    print '\nbF = ', bF
    """
    #AF = []
    #bF = []
    #----------  ----------  ----------  ----------  ----------  ----------


    #Joining all inequality constraints ----------  ----------  ----------
    Aineq = []
    for line in AF:
        Aineq.append(line)
    bineq = []
    for number in bF:
        bineq.append(number)
    """
    print '\nAineq (',len(Aineq),'x',len(Aineq[0]),') = '
    for k in range(len(Aineq)):
        print Aineq[k][:]
    print '\nsize of Aineq:', len(Aineq),'x',len(Aineq[0])
    print '\nbineq = ', bineq
    """
    #----------  ----------  ----------  ----------  ----------  ----------


    #Joining all constraints ----------  ----------  ----------
    A = []
    b = []
    #Equality constraints:
    for line in Aeq:
        A.append(line)
    for number in beq:
        b.append(number)
    #Subtour constraints:
    for line in Aineq:
        A.append(line)
    for number in bineq:
        b.append(number)
    #----------  ----------  ----------  ----------  ----------  ----------
    """
    print '\nA (',len(A),'x',len(A[0]),') = '
    for k in range(len(A)):
        print A[k][:]
    print '\nsize of A:', len(A),'x',len(A[0])
    print '\nb = ', b
    """


    # Definition of the type of constranis (> < =)
    e = []
    #Equality constraints (=)
    for k in range(len(Aeq)):
        e.append(0)
    # Inequality constraints (<)
    for k in range(len(Aineq)):
        e.append(-1)
    # ----------  ----------  ----------

    # Definition of the integer variable
    int_var = range(1, R*m + 1, 1)
    #int_var = []
    # ----------  ----------  ----------

    # Auto scale flag
    scalemode = 0
    # ----------  ----------  ----------

    # Definition of cost function operand (min or max)
    setminim = 1
    # ----------  ----------  ----------


    lp = lp_maker(c, A, b, e, vlb, vub, int_var, scalemode, setminim)
    """
    print '\n\n\n\n\n\n\n\n\n\n\n\n\n'
    print '\nLP problem created'
    print 'Nodes:', n
    print 'Robots: ', R
    print 'Variables:', len(A[1]), '   (',len(A[1])-1,'integers and', 1,'real )'
    print 'Constraints:', len(A), '   (',len(Aeq),'equalities and', len(Aineq),'inequalities )'
    print ''
    

    print 'LP solution started ...'
    """
    t = time.time()
    solvestat = lpsolve('solve', lp)
    elapsed = time.time() - t
    obj = lpsolve('get_objective', lp)
    x = lpsolve('get_variables', lp)[0]

    lpsolve('delete_lp', lp)

    #elapsed = time.time() - t

    """
    print ''
    if(solvestat == 0):
        print '----- Optimal solution found -----'
        #print '\nX = ', x
        print '\nObjective = ', obj
        #Print chosen edges
        for r in range(R):
            chosen = []
            for k in range(m):
                if(abs(x[r*m+k]-1) < 10**(-6)):
                    chosen.append(E[k])
            chosen = np.matrix(chosen)
            chosen = chosen.T
            print ('\nChoosen edges for robot %d:' % (r+1))
            print chosen+1
        print ''
        for r in range(R):
            timeTask = 0
            k = -1
            for i in range(n):
                for j in range(n):
                    if(i != j):
                        k = k + 1
                        if(abs(x[r*m+k]-1) < 10**(-6)):
                            exec('timeTask = timeTask + C_%d[i][j]' % r)
            print ('Time task for robot %d: ' % (r+1)), timeTask


    else:
        print '!!!!! Optimal solution NOT found !!!!!'
        print 'Return code', solvestat


    if(elapsed < 1):
        print '\nElapsed time: ', 1000 * elapsed, 'ms\n'
    else:
        print '\nElapsed time: ', elapsed, 's\n'
    """


    #Plotting results  ----------  ----------  ----------
    #pylab.figure()
    pylab.axis('equal')
    pylab.axis(w_s)
    for k in range(n):
        #pylab.plot(pts[k][0], pts[k][1], 'bo', linewidth=2.0)
        #pylab.plot(pts[k][0], pts[k][1], 'b*', linewidth=2.0)
        #pylab.plot(pts[k][0], pts[k][1], 'bo', markersize=10.0)
        #pylab.plot(pts[k][0], pts[k][1], 'b*', markersize=10.0)
        #pylab.text(pts[k][0]+0.04,pts[k][1]+0.04,k+1,fontsize=5.0)
        pylab.text(pts[k][0]+0.04,pts[k][1]+0.04,k,fontsize=10.0)

    """
    for i in range(n):
        for j in range(n):
            if(i != j):
                pylab.plot([pts[i][0],pts[j][0]], [pts[i][1],pts[j][1]],'b--',linewidth=0.5)
    """

    for r in range(R):
        k = -1
        for i in range(n):
            for j in range(n):
                if (i != j):
                    k = k + 1
                    if (abs(x[r * m + k] - 1) < 10 ** (-6)):
                        #pylab.plot([pts[i][0], pts[j][0]], [pts[i][1], pts[j][1]], colors[r], linewidth=3.0)
                        pylab.plot([pts[i][0], pts[j][0]], [pts[i][1], pts[j][1]], cor+'--', linewidth=2.0)



    """
    #Write in file  ----------  ----------  ----------  ----------  ----------  ----------  ----------
    #  ----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
    hour = time.strftime("%Hh%Mm%Ss")
    date = time.strftime("d%dm%my%Y")
    #path = '/home/adriano/ROS_projects/icra2018/src/lpsolve/results/Results_'+date+'_'+hour+'.txt'
    rp = rospkg.RosPack()
    path = rp.get_path('lpsolve')
    path = path + '/resultsHeuristics/Results_'+date+'_'+hour+'.txt'

    file_results = open(path, 'w')

    mystr = 'Nodes: ' + str(n) + '\n'
    file_results.write(mystr)
    mystr = 'Robots: ' + str(R) + '\n'
    file_results.write(mystr)
    mystr = 'Variables: ' + str(len(A[1])) + '   (' + str(len(A[1]) - n - 1) + ' integers and ' + str(n + 1) + ' reals)' + '\n'
    file_results.write(mystr)
    mystr = 'Constraints: ' + str(len(A)) + '   (' + str(len(Aeq)) + ' equalities and ' + str(len(Aineq)) + ' inequalities)' + '\n'
    file_results.write(mystr)

    file_results.write('\n')

    if (solvestat == 0):
        mystr = '----- Optimal solution found -----' + '\n'
        file_results.write(mystr)
        mystr = 'Objective = ' + str(obj) + '\n'
        file_results.write(mystr)
        # Print chosen edges
        for r in range(R):
            chosen = []
            for k in range(m):
                if (abs(x[r * m + k] - 1) < 10 ** (-6)):
                    chosen.append(E[k])
            chosen = np.matrix(chosen)
            chosen = chosen.T
            mystr = '\nChoosen edges for robot ' + str(r + 1) + ':' + '\n'
            file_results.write(mystr)
            mystr = str(chosen + 1) + '\n'
            file_results.write(mystr)

        file_results.write('\n')
        for r in range(R):
            timeTask = 0
            k = -1
            for i in range(n):
                for j in range(n):
                    if (i != j):
                        k = k + 1
                        if (abs(x[r * m + k] - 1) < 10 ** (-6)):
                            exec ('timeTask = timeTask + C_%d[i][j]' % r)

            mystr = 'Time task for robot ' + str(r+1) +': ' + str(timeTask) + '\n'
            file_results.write(mystr)

        #mystr = '\nX variables:\n'
        #file_results.write(mystr)
        #xs = np.matrix(x[0:R*m])
        xs = x[0:R*m]
        mystr = str(xs) + '\n'

        mystr = '\nX variables != 0:\n[  '
        for k in range(len(xs)):
            if(abs(xs[k]) > 0.0001):
                mystr = mystr + str(xs[k]) + '  '
        mystr = mystr + ']\n'
        #mystr =  ("%s" * len(xs)) % tuple(xs)
        #mystr = mystr + '\n'
        file_results.write(mystr)




    else:
        mystr = '!!!!! Optimal solution NOT found !!!!!'
        file_results.write(mystr)
        mystr = 'Return code ' + str(solvestat)
        file_results.write(mystr)


    if(elapsed < 1):
        mystr = '\nElapsed computation time: ' + str(1000 * elapsed) + ' ms\n'
    else:
        mystr = '\nElapsed computation time: ' + str(elapsed) + ' s\n'
    file_results.write(mystr)


    file_results.write('\n\n\n\n\n\n')

    mystr = '\nUsed points:\n'
    file_results.write(mystr)
    for k in range(n):
        mystr = 'p' + str(k+1) + ' = ' + str(pts[k]) + '\n'
        file_results.write(mystr)


    file_results.close()


    rp = rospkg.RosPack()
    fig_name = rp.get_path('lpsolve')
    fig_name = fig_name + '/resultsHeuristics/Results_' + date + '_' + hour + '.png'
    pylab.savefig(fig_name)
    #  ----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
    #  ----------  ----------  ----------  ----------  ----------  ----------  ----------  ----------
    """


    #pylab.show()


    if(solvestat == 0):
        for r in range(R):
            chosen = []
            for k in range(m):
                if(abs(x[r*m+k]-1) < 10**(-6)):
                    chosen.append(E[k])
            chosen = np.matrix(chosen)
            chosen = chosen.T
    tour = chosen+1

    return (tour)












