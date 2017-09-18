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
import sys

global x_n, y_n, theta_n
x_n = 0.1  # posicao x atual do robo
y_n = 0.2  # posicao y atual do robo
theta_n = 0.001  # orientacao atual do robo



global Targ_pos #Position of the 20 targets
Targ_pos = [[6.391, 5.697], [1.529, 8.435], [6.752, 9.117], [10.371, 8.556], [2.958, 0.381], [9.102, 1.823],
            [9.036, 8.809], [6.631, 4.508], [9.263, 5.791], [2.784, 2.411], [2.223, 1.970], [2.116, 6.592],
            [5.055, 5.644], [9.824, 2.064], [3.973, 4.468], [11.120, 8.983], [11.347, 5.751], [9.824, 6.819],
            [9.383, 3.667], [0.340, 6.525]]



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
    global pub_targ



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

    mark_targ = Marker()
    mark_targ.header.frame_id = "world"
    mark_targ.header.stamp = rospy.Time.now()
    mark_targ.id = 0
    mark_targ.type = mark_targ.SPHERE
    mark_targ.action = mark_targ.ADD
    mark_targ.scale.x = 0.20
    mark_targ.scale.y = 0.20
    mark_targ.scale.z = 0.20
    mark_targ.color.a = 1.0
    mark_targ.color.r = 1.0
    mark_targ.color.g = 1.0
    mark_targ.color.b = 1.0
    mark_targ.pose.position.x = Targ_pos[0]
    mark_targ.pose.position.y = Targ_pos[1]
    mark_targ.pose.position.z = 0.0
    quaternio = quaternion_from_euler(0, 0, 0)
    mark_targ.pose.orientation.x = quaternio[0]
    mark_targ.pose.orientation.y = quaternio[1]
    mark_targ.pose.orientation.z = quaternio[2]
    mark_targ.pose.orientation.w = quaternio[3]

    pub_targ.publish(mark_targ)


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
    global pub_pose, pub_pose1, pub_targ
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
    pub_targ = rospy.Publisher("/marker_targ", Marker, queue_size=1)
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

    #Read the target index
    if len(sys.argv) < 3:
        print 'ERROR!!!\nThe target id position was not informed'
    else:
        tar_id = int(sys.argv[1])

    Targ_pos = Targ_pos[tar_id]

    try:
        config()
    except rospy.ROSInterruptException:
        pass



