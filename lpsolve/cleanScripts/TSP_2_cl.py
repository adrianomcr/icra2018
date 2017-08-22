#!/usr/bin/env python
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
from time import sleep, time
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

n = 16
m = n * (n - 1)

t = time()

p1 = [0, 0]
p2 = [1, 1]
p3 = [2, 1]
p4 = [1, 3]
p5 = [0, 2]
p6 = [1, 2]
p7 = [2, 2.5]
p8 = [0.5, 1.5]
p9 = [1.5, 1.5]
p10 = [0.5, 2.5]
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

#Definition of the cost matrix
C = [[0 for i in range(n)] for j in range(n)]
for i in range(n):
    for j in range(n):
        C[i][j] = sqrt((pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2)
#  ----------  ----------  ----------

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


#Definition of the x variables bounds
vlb = []
vub = []
#Bouds of "X" variables
for i in range(m):
    vlb.append(0)
    vub.append(1)
#Bouds of "u" variables
for i in range(n):
    vlb.append(0*n) #big negative number
    vub.append(100*n) #big positive number
    #vlb.append(None)  # big negative number
    #vub.append(None)  # big positive number
# ----------  ----------  ----------


#Definition of the cost vector
c = []
#Cost of "X" variables
for i in range(n):
    for j in range(n):
        if (i != j):
            c.append(C[i][j])
#Cost of "u" variables
for i in range(n):
    c.append(0)
# ----------  ----------  ----------




# "Arival constraints" ----------  ----------  ----------
Aeq1 = [[0 for i in range(m)] for j in range(n-1)]
#Aeq1 = np.matrix(Aeq1)
for j in range(1,n): # 1 until n-1
    #line = [0 for k in range(m)]
    for k in range(0, m):  # 0 until m-1
        if(E[k][1] == j):
            #line[k] = 1
            Aeq1[j-1][k] = 1
beq1 = [1 for i in range(n-1)]
"""
print '\nAeq1 = '
for k in range(n-1):
    print Aeq1[k][:]
print '\nbeq1 = ', beq1
"""
#----------  ----------  ----------  ----------  ----------  ----------

#"Flux conservation constraints" ----------  ----------  ----------
Aeq2 = [[0 for i in range(m)] for j in range(n)]
for p in range(n): # 0 until n-1
    for k in range(0, m):  # 0 until m-1
        if (E[k][1] == p):
            Aeq2[p][k] = 1
        if (E[k][0] == p):
            Aeq2[p][k] = -1
beq2 = [0 for i in range(n)]
"""
print '\nAeq2 = '
for k in range(n):
    print Aeq2[k][:]
print '\nbeq2 = ', beq2
"""
#----------  ----------  ----------  ----------  ----------  ----------

#"Depot constraints" ----------  ----------  ----------
Aeq3 = [[0 for i in range(m)] for j in range(1)]
for k in range(0, m):  # 0 until m-1
    if (E[k][1] == 0): #OBS: node 0 is the depot one
        Aeq3[0][k] = 1
beq3 = [1]
"""
print '\nAeq3 = '
for k in range(n):
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
"""
print '\nAeq = '
for k in range(n+(n-1)+1):
    print Aeq[k][:]
print '\nbeq = ', beq
"""
#----------  ----------  ----------  ----------  ----------  ----------


#Subtour elimination constraints ----------  ----------  ----------
Aineq = []
bineq = []
Aineq = [[0 for i in range(m+n)] for j in range(m)]
bineq = [(n-1) for i in range(m)]
k = -1
for i in range(n):
    for j in range(n):
        if (i != j):
            k = k + 1
            if (i != 0 and j != 0): #OBS: node 0 is the depot one
                Aineq[k][i+m] = 1
                Aineq[k][j+m] = -1
                Aineq[k][k] = n
"""
print '\nAineq = '
for k in range(m):
    print Aineq[k][:]
print '\nbineq = ', bineq
"""
#----------  ----------  ----------  ----------  ----------  ----------



#Joining all constraints ----------  ----------  ----------
A = []
b = []
#Equality constraints:
for line in Aeq:
    line2 = line
    for k in range(n):
        line2.append(0)
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
print '\nA = '
for k in range(m):
    print A[k][:]
print '\nb = ', b
"""


"""
# Definition of the b vector
b = [2 for i in range(n)]
#b = [random()*1 for i in range(n)] #test
# ----------  ----------  ----------
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
int_var = range(1, m + 1, 1)
#int_var = []
# ----------  ----------  ----------

# Auto scale flag
scalemode = 0
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
print 'Variables:', len(A[1]), '   (',len(A[1])-n-1,'integers and', n+1,'real )'
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
    print '\nobjective = ', obj
    print '\nx = ', x
    #Print chosen edges
    chosen = []
    for k in range(m):
        if(abs(x[k]-1) < 10**(-6)):
            chosen.append(E[k])
    chosen = np.matrix(chosen)
    chosen = chosen.T
    print '\nChoosen edges:'
    print chosen+1

else:
    print 'Optimal solution NOT found'
    print 'Return code', solvestat


#elapsed = time() - t
print '\nElapsed time: ', 1000 * elapsed, 'ms\n'

#  ----------  ----------  ----------  ----------  ----------  ----------  ----------












