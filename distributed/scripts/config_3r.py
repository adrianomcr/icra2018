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
global x_n1, y_n1, theta_n1
x_n1 = 0.1  # posicao x atual do robo
y_n1 = 0.2  # posicao y atual do robo
theta_n1 = 0.001  # orientacao atual do robo
global x_n2, y_n2, theta_n2
x_n2 = 0.1  # posicao x atual do robo
y_n2 = 0.2  # posicao y atual do robo
theta_n2 = 0.001  # orientacao atual do robo




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

def callback_pose2(data):
    global x_n2, y_n2, theta_n2

    x_n2 = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_n2 = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q2 = data.pose.pose.orientation.x
    y_q2 = data.pose.pose.orientation.y
    z_q2 = data.pose.pose.orientation.z
    w_q2 = data.pose.pose.orientation.w
    euler2 = euler_from_quaternion([x_q2, y_q2, z_q2, w_q2])
    theta_n2 = euler2[2]  # orientaco 'theta' do robo no mundo

    br = tf.TransformBroadcaster()
    br.sendTransform((x_n2, y_n2, 0), (x_q2, y_q2, z_q2, w_q2), rospy.Time.now(), "/robot_2/base_pose_ground_truth", "world")

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

# Rotina callback para a obtencao dos dados do laser
def callback_laser2(data):
    global laserVec2
    global x_n2, y_n2, theta_n2

    laserVec2 = data.ranges

    br = tf.TransformBroadcaster()
    br.sendTransform((x_n2, y_n2, 0), quaternion_from_euler(0, 0, theta_n2), rospy.Time.now(), "/robot_2/base_laser_link","world")
    return
# ----------  ----------  ----------  ----------  ----------









# Rotina para piblicar informacoes no rviz
def send_marker_to_rviz():
    global pub_pose
    global pub_pose1
    global pub_pose2



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
    mark_pose1.color.r = 1.0
    mark_pose1.color.g = 0.0
    mark_pose1.color.b = 0.0
    mark_pose1.pose.position.x = x_n1
    mark_pose1.pose.position.y = y_n1
    mark_pose1.pose.position.z = 0.0
    quaternio = quaternion_from_euler(0, 0, theta_n1)
    mark_pose1.pose.orientation.x = quaternio[0]
    mark_pose1.pose.orientation.y = quaternio[1]
    mark_pose1.pose.orientation.z = quaternio[2]
    mark_pose1.pose.orientation.w = quaternio[3]

    pub_pose1.publish(mark_pose1)

    # ----------  ----------  ----------

    mark_pose2 = Marker()
    mark_pose2.header.frame_id = "world"
    mark_pose2.header.stamp = rospy.Time.now()
    mark_pose2.id = 0
    mark_pose2.type = mark_pose2.CUBE
    mark_pose2.action = mark_pose2.ADD
    mark_pose2.scale.x = 0.22
    mark_pose2.scale.y = 0.22
    mark_pose2.scale.z = 0.06
    mark_pose2.color.a = 1.0
    mark_pose2.color.r = 0.0
    mark_pose2.color.g = 1.0
    mark_pose2.color.b = 0.0
    mark_pose2.pose.position.x = x_n2
    mark_pose2.pose.position.y = y_n2
    mark_pose2.pose.position.z = 0.0
    quaternio = quaternion_from_euler(0, 0, theta_n2)
    mark_pose2.pose.orientation.x = quaternio[0]
    mark_pose2.pose.orientation.y = quaternio[1]
    mark_pose2.pose.orientation.z = quaternio[2]
    mark_pose2.pose.orientation.w = quaternio[3]

    pub_pose2.publish(mark_pose2)

    return

# ----------  ----------  ----------  ----------  ----------





# Routine to send the communication range to rviz
def send_comm_range_to_rviz(circ_x0,circ_y0):

    global x_n, y_n, theta_n
    global x_n1, y_n1, theta_n1


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
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = circ_x0[p] + x_n1
        marker.pose.position.y = circ_y0[p] + y_n1
        marker.pose.position.z = 0.1
        #print "marker = ", marker
        points_marker.markers.append(marker)
    circle_red = points_marker




    points_marker = MarkerArray()
    marker = Marker()
    #for p0 in range(1, 628, 1):
    for p in range(len(circ_x0)):
        #print "p0 = ", p0
        #p = p0 / 100.0
        #x = cos(phi) * (a * cos(p)) - sin(phi) * (b * sin(p)) + cx * 1
        #y = sin(phi) * (a * cos(p)) + cos(phi) * (b * sin(p)) + cy * 1
        #x = circ_x0 + x_n2
        #y = circ_y0 + y_n2
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
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = circ_x0[p] + x_n2
        marker.pose.position.y = circ_y0[p] + y_n2
        marker.pose.position.z = 0.1
        #print "marker = ", marker
        points_marker.markers.append(marker)
    circle_green = points_marker




    return circle_blue, circle_red, circle_green

# ----------  ----------  ----------  ----------  ----------






# Rotina primaria
def config():
    global x_n, y_n, theta_n
    global pub_pose, pub_pose1, pub_pose2
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
    pub_pose2 = rospy.Publisher("/marker_pose2", Marker, queue_size=1)
    pub_circ0 = rospy.Publisher("/marker_circ0", MarkerArray, queue_size=1)
    pub_circ1 = rospy.Publisher("/marker_circ1", MarkerArray, queue_size=1)
    pub_circ2 = rospy.Publisher("/marker_circ2", MarkerArray, queue_size=1)
    rospy.init_node("config")
    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callback_pose)
    rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, callback_pose1)
    rospy.Subscriber("/robot_2/base_pose_ground_truth", Odometry, callback_pose2)
    rospy.Subscriber("/robot_0/base_scan", LaserScan, callback_laser)
    rospy.Subscriber("/robot_1/base_scan", LaserScan, callback_laser1)
    rospy.Subscriber("/robot_2/base_scan", LaserScan, callback_laser2)



    freq = 10.0  # Hz
    rate = rospy.Rate(freq)

    p = [2*pi*i/50.0 for i in range(50)]
    #R = 0.5
    #R = 1.5
    R = 2.5
    #R = 3.5
    #R = 4.5

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

        [circle_blue, circle_red, circle_green] = send_comm_range_to_rviz(circ_x0,circ_y0)

        pub_circ0.publish(circle_blue)
        pub_circ1.publish(circle_red)
        pub_circ2.publish(circle_green)

        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------










# Funcao inicial
if __name__ == '__main__':

    try:
        config()
    except rospy.ROSInterruptException:
        pass



