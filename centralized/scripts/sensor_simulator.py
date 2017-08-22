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
from time import sleep
import tf
import scipy.io
import rospkg



#This script make a robot to follow a path through a graph




global x_n, y_n, theta_n
x_n = 0.1  # posicao x atual do robo
y_n = 0.2  # posicao y atual do robo
theta_n = 0.001  # orientacao atual do robo

global d  # for feedback linearization
d = 0.07

global Vd
Vd = 0.25

global time
time = 0

global Kp #Proportional gain
Kp = 0.8

global laserVec
laserVec = [1 for i in range(270)]




# Callback routine to obtain the robot's pose
def callback_pose(data):
    global x_n, y_n, theta_n
    global file_path, time

    x_n = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_n = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta_n = euler[2]  # orientaco 'theta' do robo no mundo

    mystr = str(x_n) + "\t" + str(y_n) + "\t" + str(theta_n) + "\t" + str(time) + "\n"

    file_path.write(mystr)

    return

# ----------  ----------  ----------  ----------  ----------








# Primary routine
def sensor_simulator():


    count = 0


    pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.init_node("follow_graph")
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose)



    freq = 10.0  # Hz
    rate = rospy.Rate(freq)




    sleep(0.1)

    while (not rospy.is_shutdown()):

        count = count + 1

        time = count / float(freq)



        pub_stage.publish(vel)



        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------









# Initial function
if __name__ == '__main__':




    try:
        sensor_simulator()
    except rospy.ROSInterruptException:
        pass

