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




def execute_lp():
    t = time()
    lp = lpsolve('make_lp', 0, 4)
    lpsolve('set_verbose', lp, IMPORTANT)
    lpsolve('set_obj_fn', lp, [1, 3, 6.24, 0.1])
    lpsolve('add_constraint', lp, [0, 78.26, 0, 2.9], GE, 92.3)
    lpsolve('add_constraint', lp, [0.24, 0, 11.31, 0], LE, 14.8)
    lpsolve('add_constraint', lp, [12.68, 0, 0.08, 0.9], GE, 4)
    lpsolve('set_lowbo', lp, 1, 28.6)
    lpsolve('set_lowbo', lp, 4, 18)
    lpsolve('set_upbo', lp, 4, 48.98)
    lpsolve('set_col_name', lp, 1, 'COLONE')
    lpsolve('set_col_name', lp, 2, 'COLTWO')
    lpsolve('set_col_name', lp, 3, 'COLTHREE')
    lpsolve('set_col_name', lp, 4, 'COLFOUR')
    lpsolve('set_row_name', lp, 1, 'THISROW')
    lpsolve('set_row_name', lp, 2, 'THATROW')
    lpsolve('set_row_name', lp, 3, 'LASTROW')
    lpsolve('write_lp', lp, 'a.lp')
    print lpsolve('get_mat', lp, 1, 2)
    lpsolve('solve', lp)
    elapsed = time() - t
    print lpsolve('get_objective', lp)
    print lpsolve('get_variables', lp)[0]
    print lpsolve('get_constraints', lp)[0]
    #lpsolve('delete_lp', lp)
    print '\nElapsed time: ', 1000*elapsed, 'ms'






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



