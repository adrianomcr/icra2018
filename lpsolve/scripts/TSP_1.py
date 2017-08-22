#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from random import randrange, random
from time import sleep, time
import tf
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

def execute_lp():
    n = 6
    m = n * (n - 1) / 2

    t = time()

    p1 = [0, 0]
    p2 = [1, 1]
    p3 = [2, 1]
    p4 = [1, 3]
    p5 = [0, 2]
    p6 = [1, 2]
    pts = []
    for k in range(n):
        pts.append(eval('p%d' % (k + 1)))
    pts = np.array(pts)

    C = [[0 for i in range(n)] for j in range(n)]
    for i in range(n):
        for j in range(n):
            C[i][j] = sqrt((pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2)

    vlb = []
    vub = []
    for i in range(m):
        vlb.append(0)
        vub.append(1)



    c = []
    for i in range(n):
        for j in range(n):
            if (i < j):
                c.append(C[i][j])

    # A = [[1, 1, 1, 1, 0, 0, 0, 0, 0, 0],[1, 0, 0, 0, 1, 1, 1, 0, 0, 0],[ 0, 1, 0, 0, 1, 0, 0, 1, 1, 0],[0, 0, 1, 0, 0, 1, 0, 1, 0, 1],[0, 0, 0, 1, 0, 0, 1, 0, 1, 1]]
    # count = 0
    tam = n - 1
    A = [[0 for i in range(m)] for j in range(n)]
    A = np.matrix(A)
    i = 0
    j = 0
    while (i < n):
        A[i, j:j + tam] = [1 for aux in range(n - i - 1)]
        if (i < n - 1):
            A[i + 1:n, j:j + tam] = np.matrix(np.identity(tam))
        i = i + 1
        j = j + tam
        tam = tam - 1
    A = A.tolist()

    b = [2 for i in range(n)]
    #b = [random()*1 for i in range(n)] #test

    e = [0 for i in range(n)]
    #e = [random()*1 for i in range(n)] #test

    int_var = range(1, m + 1, 1)
    #int_var = []

    scalemode = 1

    setminim = 1

    #"""
    print '\nc = ', c
    print '\nA = '
    for k in range(n):
        print A[k][:]
    print '\nb = ', b
    print '\ne = ', e
    print '\nint_var = ', int_var
    print '\nscalemode = ', scalemode
    print '\nsetminim = ', setminim
    #"""
    #lp = lp_maker(c, A, b, e, None, None, None, scalemode, setminim)
    #lp = lp_maker(c, A, b, e, None, None, int_var, scalemode, setminim)
    #lp = lp_maker(c, A, b, e, vlb, vub, None, scalemode, setminim)
    lp = lp_maker(c, A, b, e, vlb, vub, int_var, scalemode, setminim)

    #lpsolve('set_int', lp, int_var)

    solvestat = lpsolve('solve', lp)
    obj = lpsolve('get_objective', lp)
    x = lpsolve('get_variables', lp)[0]

    lpsolve('delete_lp', lp)

    print ''
    if(solvestat == 0):
        print '----- Optimal solution found -----'
    else:
        print 'Optimal solution NOT found'
        print 'Return code', solvestat
    print '\nobjective = ', obj
    print '\nx = ', x

    elapsed = time() - t
    print '\nElapsed time: ', 1000 * elapsed, 'ms\n'

#  ----------  ----------  ----------  ----------  ----------  ----------  ----------




# Rotina primaria
def example():


    #pub_variable = rospy.Publisher("/TOPIC_NAME", MSG_TYPE, queue_size=1)
    rospy.init_node("example")
    #rospy.Subscriber("/TOPIC_NAME", MSG_TYPE, callback_function)

    freq = 10.0  # Hz
    rate = rospy.Rate(freq)

    i = 0

    sleep(1)

    #while not rospy.is_shutdown():
    while not rospy.is_shutdown() and i == 0:
    #time0 = time();
    #while not rospy.is_shutdown() and i < 100:

        i = i + 1
        tempo = i / float(freq)

        execute_lp()


        rate.sleep()

    #elapsed0 = time() - time0
    #print '\nElapsed time: ', 1000 * elapsed0, 'ms\n'


# ---------- !! ---------- !! ---------- !! ---------- !! ----------










# Funcao inicial
if __name__ == '__main__':

    try:
        example()
    except rospy.ROSInterruptException:
        pass



