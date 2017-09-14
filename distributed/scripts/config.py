#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from random import randrange
from time import sleep
import tf

global x_n, y_n, theta_n
x_n = 0.1  # posicao x atual do robo
y_n = 0.2  # posicao y atual do robo
theta_n = 0.001  # orientacao atual do robo




# Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global x_n, y_n, theta_n

    x_n = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_n = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta_n = euler[2]  # orientaco 'theta' do robo no mundo

    br = tf.TransformBroadcaster()
    br.sendTransform((x_n, y_n, 0), (x_q, y_q, z_q, w_q), rospy.Time.now(), "/robot_0/base_pose_ground_truth", "world")

    return
# ----------  ----------  ----------  ----------  ----------







# Rotina callback para a obtencao dos dados do laser
def callback_laser(data):
    global laserVec
    global x_n, y_n, theta_n

    laserVec = data.ranges

    br = tf.TransformBroadcaster()
    br.sendTransform((x_n, y_n, 0), quaternion_from_euler(0, 0, theta_n), rospy.Time.now(), "/robot_0/base_laser_link","world")
    return
# ----------  ----------  ----------  ----------  ----------






# Rotina para piblicar informacoes no rviz
def send_marker_to_rviz():
    global pub_pose



    mark_pose = Marker()
    mark_pose.header.frame_id = "world"
    mark_pose.header.stamp = rospy.Time.now()
    mark_pose.id = 0
    mark_pose.type = mark_pose.CUBE
    mark_pose.action = mark_pose.ADD
    mark_pose.scale.x = 0.22
    mark_pose.scale.y = 0.22
    mark_pose.scale.z = 0.06
    mark_pose.color.a = 1.0
    mark_pose.color.r = 0.0
    mark_pose.color.g = 0.0
    mark_pose.color.b = 1.0
    mark_pose.pose.position.x = x_n
    mark_pose.pose.position.y = y_n
    mark_pose.pose.position.z = 0.0
    quaternio = quaternion_from_euler(0, 0, theta_n)
    mark_pose.pose.orientation.x = quaternio[0]
    mark_pose.pose.orientation.y = quaternio[1]
    mark_pose.pose.orientation.z = quaternio[2]
    mark_pose.pose.orientation.w = quaternio[3]

    pub_pose.publish(mark_pose)



    return

# ----------  ----------  ----------  ----------  ----------





# Routine to send the communication range to rviz
def send_comm_range_to_rviz(circ_x0,circ_y0):

    global x_n, y_n, theta_n


    points_marker = MarkerArray()
    marker = Marker()
    #for p0 in range(1, 628, 1):
    for p in range(len(circ_x0)):
        #print "p0 = ", p0
        #p = p0 / 100.0
        #x = cos(phi) * (a * cos(p)) - sin(phi) * (b * sin(p)) + cx * 1
        #y = sin(phi) * (a * cos(p)) + cos(phi) * (b * sin(p)) + cy * 1
        #x = circ_x0 + x_n1
        #y = circ_y0 + y_n1
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        marker.id = p - 1
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = circ_x0[p] + x_n
        marker.pose.position.y = circ_y0[p] + y_n
        marker.pose.position.z = 0.1
        #print "marker = ", marker
        points_marker.markers.append(marker)
    circle_blue = points_marker

    return circle_blue, 1

# ----------  ----------  ----------  ----------  ----------






# Rotina primaria
def config():
    global x_n, y_n, theta_n
    global pub_pose, pub_pose1
    global freq

    vel = Twist()

    i = 0

    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0


    pub_pose = rospy.Publisher("/marker_pose", Marker, queue_size=1)
    pub_circ0 = rospy.Publisher("/marker_circ0", MarkerArray, queue_size=1)
    rospy.init_node("config")
    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callback_pose)
    rospy.Subscriber("/robot_0/base_scan", LaserScan, callback_laser)
    """
    pub_pose = rospy.Publisher("/marker_pose", Marker, queue_size=1)
    pub_circ0 = rospy.Publisher("/marker_circ0", MarkerArray, queue_size=1)
    rospy.init_node("config")
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose)
    rospy.Subscriber("/base_scan", LaserScan, callback_laser)
    """




    freq = 10.0  # Hz
    rate = rospy.Rate(freq)

    p = [2*pi*i/50.0 for i in range(50)]
    R = 2.5
    #circ_x0 = np.matrix([R*cos(i) for i in p])
    #circ_y0 = np.matrix([R*sin(i) for i in p])
    circ_x0 = [R*cos(i) for i in p]
    circ_y0 = [R*sin(i) for i in p]


    sleep(1)

    while not rospy.is_shutdown():

        i = i + 1
        time = i / float(freq)


        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "/map", "world")

        send_marker_to_rviz()

        [circle_blue, aux] = send_comm_range_to_rviz(circ_x0,circ_y0)

        pub_circ0.publish(circle_blue)

        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------










# Funcao inicial
if __name__ == '__main__':

    try:
        config()
    except rospy.ROSInterruptException:
        pass



