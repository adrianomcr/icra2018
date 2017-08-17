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

def callback_pose1(data):
    global x_n1, y_n1, theta_n1

    x_n1 = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_n1 = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q1 = data.pose.pose.orientation.x
    y_q1 = data.pose.pose.orientation.y
    z_q1 = data.pose.pose.orientation.z
    w_q1 = data.pose.pose.orientation.w
    euler1 = euler_from_quaternion([x_q1, y_q1, z_q1, w_q1])
    theta_n1 = euler1[2]  # orientaco 'theta' do robo no mundo

    br = tf.TransformBroadcaster()
    br.sendTransform((x_n1, y_n1, 0), (x_q1, y_q1, z_q1, w_q1), rospy.Time.now(), "/robot_1/base_pose_ground_truth", "world")

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

# Rotina callback para a obtencao dos dados do laser
def callback_laser1(data):
    global laserVec1
    global x_n1, y_n1, theta_n1

    laserVec1 = data.ranges

    br = tf.TransformBroadcaster()
    br.sendTransform((x_n1, y_n1, 0), quaternion_from_euler(0, 0, theta_n1), rospy.Time.now(), "/robot_1/base_laser_link","world")
    return
# ----------  ----------  ----------  ----------  ----------









# Rotina para piblicar informacoes no rviz
def send_marker_to_rviz():
    global pub_pose
    global pub_pose1



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

    # ----------  ----------  ----------

    mark_pose1 = Marker()
    mark_pose1.header.frame_id = "world"
    mark_pose1.header.stamp = rospy.Time.now()
    mark_pose1.id = 0
    mark_pose1.type = mark_pose1.CUBE
    mark_pose1.action = mark_pose1.ADD
    mark_pose1.scale.x = 0.22
    mark_pose1.scale.y = 0.22
    mark_pose1.scale.z = 0.06
    mark_pose1.color.a = 1.0
    mark_pose1.color.r = 0.0
    mark_pose1.color.g = 0.0
    mark_pose1.color.b = 1.0
    mark_pose1.pose.position.x = x_n1
    mark_pose1.pose.position.y = y_n1
    mark_pose1.pose.position.z = 0.0
    quaternio = quaternion_from_euler(0, 0, theta_n1)
    mark_pose1.pose.orientation.x = quaternio[0]
    mark_pose1.pose.orientation.y = quaternio[1]
    mark_pose1.pose.orientation.z = quaternio[2]
    mark_pose1.pose.orientation.w = quaternio[3]

    pub_pose1.publish(mark_pose1)

    return

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
    pub_pose1 = rospy.Publisher("/marker_pose1", Marker, queue_size=1)
    pub_particles = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1)
    rospy.init_node("config")
    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callback_pose)
    rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, callback_pose1)
    rospy.Subscriber("/robot_0/base_scan", LaserScan, callback_laser)
    rospy.Subscriber("/robot_1/base_scan", LaserScan, callback_laser1)



    freq = 10.0  # Hz
    rate = rospy.Rate(freq)


    sleep(1)

    while not rospy.is_shutdown():

        i = i + 1
        time = i / float(freq)


        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "/map", "world")

        send_marker_to_rviz()

        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------










# Funcao inicial
if __name__ == '__main__':

    try:
        config()
    except rospy.ROSInterruptException:
        pass



