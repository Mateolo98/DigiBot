#! /usr/bin/env python
from ast import Global
from turtle import position
import rospy
import tf
import traceback
import time
import sys
import math
from operator import add
import string
import threading
import geometry_msgs.msg
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from math import sin, cos, pi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

lengthWheelBase=0.215
Vx=0.0
Vy=0.0
dt=0.0
Vth=0.0
x = 0.0
y = 0.0
th = 0.0 #Angle

#Callback function receive the encoders value
def callback(data):
	global Vx, Vth, dt
	encoders=data.vector
	leftspeed=encoders.x #speed(m/s)
	rightspeed=encoders.y #speed(m/s)
	rospy.loginfo("Left speed is: %5.7f", leftspeed)
	rospy.loginfo("Right speed is: %5.7f", rightspeed)
	dt=encoders.z
	Vx=(leftspeed+rightspeed)/2
	Vth=((rightspeed - leftspeed)/lengthWheelBase)
	
def calculateposition(current_time, last_time, r):
	global Vx, Vth, th, Vy, x, y,dt
	while not rospy.is_shutdown():
		try:
			current_time = rospy.Time.now()

			# compute odometry in a typical way given the velocities of the robot
			#dt = (current_time - last_time).to_sec()
			delta_x = (Vx * cos(th) - Vy * sin(th)) * dt
			delta_y = (Vx * sin(th) + Vy * cos(th)) * dt
			delta_th = Vth * dt

			x += delta_x
			y += delta_y
			th += delta_th

			# since all odometry is 6DOF we'll need a quaternion created from yaw
			odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

			# first, we'll publish the transform over tf
			odom_broadcaster.sendTransform(
				(x, y, 0.),
				odom_quat,
				current_time,
				"base_link",
				"odom"
			)

			# next, we'll publish the odometry message over ROS
			odom = Odometry()
			odom.header.stamp = current_time
			odom.header.frame_id = "odom"

			# set the position
			odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

			# set the velocity
			odom.child_frame_id = "base_link"
			odom.twist.twist = Twist(Vector3(Vx, Vy, 0), Vector3(0, 0, Vth))
			#---------------------------------------------------------------------------------------------------------
			#---------------------------------------------------------------------------------------------------------
			# publish the message
			odom_pub.publish(odom)
			#last_time = current_time
			r.sleep()
		except rospy.ROSInterruptException:
			rospy.on_shutdown(quit())


if __name__=='__main__':
	rospy.init_node('Odompublish', anonymous=True)
	rospy.Subscriber('encoder', Vector3Stamped, callback)
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
	odom_broadcaster = tf.TransformBroadcaster()
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()
	r = rospy.Rate(50.0)
	calculateposition(current_time, last_time, r)

	

