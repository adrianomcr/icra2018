#!/usr/bin/env python
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
from time import sleep, time
#from pylab import *
import pylab
from lpsolve55 import *
from lp_maker import *

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

#def execute_lp():

n = 11 #number of nodes
m = n * (n - 1) #number of edges
R = 3 #number of robots
depots = {1: 0,
          2: [0, 1],
          3: [0, 1, 2],
          }
speeds = {1: [1],
          2: [1, 1],
          3: [1, 1, 1],
          }

depots = depots[R]
speeds = speeds[R]


t = time()

#Definition of the nodes
p1 = [0, 0]
p2 = [1, 1]
p3 = [2, 1]
p4 = [1, 3]
p5 = [0, 2]
p6 = [1.1, 2.1]
p7 = [2, 2.5]
p8 = [0.6, 1.3]
p9 = [1.5, 1.4]
p10 = [0.4, 2.6]
p11 = [0, -0.5]
p12 = [0.1, -0.5]
p13 = [2.1, 2.5]
p14 = [-2.1, 2.5]
p15 = [-0.5, 2.5]
p16 = [2.1, 0]
pts = []
for k in range(n):
    pts.append(eval('p%d' % (k + 1)))
pts = np.array(pts)
# ----------  ----------  ----------



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
"""
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




#Definition of the x variables bounds
vlb = []
vub = []
#Bouds of "X" variables
for i in range(R*m):
    vlb.append(0)
    vub.append(1)
#Bouds of "u" variables
for i in range(n):
    vlb.append(0*n)
    vub.append(100*n) #big positive number
# Bouds of "F" variable
vlb.append(0*n)
vub.append(1000*n) #big positive number
# ----------  ----------  ----------


#Definition of the cost vector
c = []
"""
#Cost of "X" variables
for r in range(R):
    for i in range(n):
        for j in range(n):
            if (i != j):
                c.append(C[i][j])
#Cost of "u" variables
for i in range(n):
    c.append(0)
#Cost of the "F" variable
c.append(100000)
"""
#"""
for k in range(R*m+n):
    c.append(0)
c.append(1)
#"""
"""
print 'Cost vector:'
print c
print 'len(c) = ', len(c)
"""
# ----------  ----------  ----------




# "Arival constraints" ----------  ----------  ----------
#Aeq1 = [[0 for i in range(m)] for j in range(n-1)]
Aeq1 = [[0 for i in range(R*m)] for j in range(n-len(set(depots)))]
count_line = -1
for j in range(n): # 0 until n-1
    if(not j in depots): # if j belongs to V^
        count_line = count_line + 1
        for r in range(R):
            for k in range(m):  # 0 until m-1
                if(E[k][1] == j):
                    Aeq1[count_line][r*m+k] = 1

#beq1 = [1 for i in range(n-1)]
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
#Adding 'u' variables and 'F' variable
Aeq = np.matrix(Aeq)
zerosMatrix = np.zeros((len(Aeq),n+1))
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


#Subtour elimination constraints ----------  ----------  ----------
Asub = []
bsub = []
#Asub = [[0 for i in range(m+n)] for j in range(m)]
#bsub = [(n-1) for i in range(m)]
Asub = []
bsub = []
k = -1
count_line = -1 #constraints counter
for r in range(R):
    for i in range(n):
        for j in range(n):
            if (i != j):
                k = k + 1 #edge index counter
                if (i != depots[r] and j != depots[r]): #OBS: node 0 is the depot one
                    Asub.append([0 for z in range(R*m+n)])
                    bsub.append(n-1)
                    #count_line = count_line + 1
                    #Asub[count_line][i + (r + 1) * m] = 1
                    #Asub[count_line][j + (r + 1) * m] = -1
                    #Asub[count_line][k] = n
                    Asub[-1][i + R*m] = 1
                    Asub[-1][j + R*m] = -1
                    Asub[-1][k] = n
#Adding 'F' variablel
Asub = np.matrix(Asub)
zerosMatrix = np.zeros((len(Asub),1))
zerosMatrix = np.matrix(zerosMatrix)
Asub = np.concatenate((Asub.T,zerosMatrix.T))
Asub = Asub.T
Asub = Asub.tolist()
"""
print '\nAsub (',len(Asub),'x',len(Asub[0]),') = '
for k in range(len(Asub)):
    print Asub[k][:]
print '\nsize of Asub:', len(Asub),'x',len(Asub[0])
print '\nbsub = ', bsub
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
#Adding 'u' vaiables and 'F' variablel
AF = np.matrix(AF)
zerosMatrix = np.zeros((len(AF),n))
zerosMatrix = np.matrix(zerosMatrix)
onesMatrix = np.ones((len(AF),1))
onesMatrix = np.matrix(onesMatrix)*(-1)
AF = np.concatenate((AF.T,zerosMatrix.T))
AF = np.concatenate((AF,onesMatrix.T))
AF = AF.T
AF = AF.tolist()
"""
print '\nAF (',len(AF),'x',len(AF[0]),') = '
for k in range(len(AF)):
    print AF[k][:]
print '\nsize of AF:', len(AF),'x',len(AF[0])
print '\nbF = ', bF
"""
#----------  ----------  ----------  ----------  ----------  ----------


#Joining all inequality constraints ----------  ----------  ----------
Aineq = [];
for line in Asub:
    Aineq.append(line)
for line in AF:
    Aineq.append(line)
bineq = []
for number in bsub:
    bineq.append(number)
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
#e = [0 for i in range(n)]
e = []
#Equality constraints (=)
for k in range(len(Aeq)):
    e.append(0)
# Inequality constraints (<)
for k in range(len(Aineq)):
    e.append(-1)
#e = [random()*1 for i in range(n)] #test
# ----------  ----------  ----------

# Definition of the integer variables
#int_var = range(1, m + 1, 1)
#int_var = range(1, m + n + 1, 1)
int_var = range(1, R*m + 1, 1)
#int_var = []
# ----------  ----------  ----------

# Auto scale flag
scalemode = 1
# ----------  ----------  ----------

# Definition of cost function operand (min or max)
setminim = 1
# ----------  ----------  ----------

"""
print '\nc = ', c
print '\nA = '
for k in range(n):
    print A[k][:]
print '\nb = ', b
print '\ne = ', e
print '\nint_var = ', int_var
print '\nscalemode = ', scalemode
print '\nsetminim = ', setminim
"""
lp = lp_maker(c, A, b, e, vlb, vub, int_var, scalemode, setminim)
print '\nLP problem created'
print 'Nodes:', n
print 'Variables:', len(A[1]), '   (',len(A[1])-n-1,'integers and', n+1,'reals )'
print 'Constraints:', len(A)
print ''

print 'LP solution started ...'
solvestat = lpsolve('solve', lp)
obj = lpsolve('get_objective', lp)
x = lpsolve('get_variables', lp)[0]

lpsolve('delete_lp', lp)

elapsed = time() - t

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


#elapsed = time() - t
print '\nElapsed time: ', 1000 * elapsed, 'ms\n'


#Plotting results  ----------  ----------  ----------
w_s = [-1, 3, -1 ,4]
pylab.axis('equal')
pylab.axis(w_s)
for k in range(n):
    pylab.plot(pts[k][0],pts[k][1],'bo',linewidth=2.0)
    pylab.text(pts[k][0]+0.06,pts[k][1]+0.06,k+1,fontsize=15.0)

for i in range(n):
    for j in range(n):
        if(i != j):
            pylab.plot([pts[i][0],pts[j][0]], [pts[i][1],pts[j][1]],'b--',linewidth=0.5)

colors = {0 : 'r',
          1 : 'g',
          2: 'k'
          }
for r in range(R):
    k = -1
    for i in range(n):
        for j in range(n):
            if (i != j):
                k = k + 1
                if (abs(x[r * m + k] - 1) < 10 ** (-6)):
                    pylab.plot([pts[i][0],pts[j][0]], [pts[i][1],pts[j][1]],colors[r],linewidth=2.0)




pylab.show()
#  ----------  ----------  ----------  ----------



#  ----------  ----------  ----------  ----------  ----------  ----------  ----------

















