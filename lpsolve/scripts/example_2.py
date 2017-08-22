#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from random import randrange
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

    #Example from http://lpsolve.sourceforge.net/5.5/
    #Look for the text:
    #If we denote the number of acres allotted to corn by z, then the objective function becomes:


    f = [143, 60, 195]
    A = [[120, 210, 150.75], [110, 30, 125], [1, 1, 1]]
    b = [15000, 4000, 75]
    lp = lp_maker(f, A, b, [-1, -1, -1], None, None, None, 1, 0)
    #lpsolve('set_int', lp, [1, 1, 1])
    t = time()
    solvestat = lpsolve('solve', lp)
    elapsed = time() - t
    obj = lpsolve('get_objective', lp)
    x = lpsolve('get_variables', lp)[0]

    lpsolve('delete_lp', lp)

    print 'Elapsed time: ', 1000 * elapsed, 'ms'
    print 'obj = ', obj
    print 'x = ', x




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

        i = i + 1
        tempo = i / float(freq)

        execute_lp()


        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------










# Funcao inicial
if __name__ == '__main__':

    try:
        example()
    except rospy.ROSInterruptException:
        pass



