#!/usr/bin/env python

###################################################
 # based on Skate_bot.py developed by Vitor Izumino Sgrignoli
 # Updated in: April 24th, 2021 by josecn
 ###################################################

import serial
import rospy
import tf
from math import sin, cos, pi, sqrt
from std_msgs.msg import String
from struct import unpack, pack
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

DEVICE = rospy.get_param("~device",'/dev/ttyUSB0')
BAUD = rospy.get_param("~baud",57600)
usbPort = serial.Serial (DEVICE, BAUD , timeout=5)
#usbPort = serial.Serial ('/dev/ttyUSB0', 57600 , timeout=5)
r=(4.8*0.254)/2
b=0.10

def cmdVel_callback(cmdVel):
    dx = cmdVel.linear.x
    dy = cmdVel.linear.y
    dalpha = cmdVel.angular.z
    
    dthR = 100*(dx+b*dalpha)/r
    dthL = 100*(dx-b*dalpha)/r

    msgVel = pack ("hh",dthR,dthL)
    msgVelStr = '$'+msgVel+'#'
    usbPort.write(msgVelStr)
    

def LinxMotion():
    odom_pub = rospy.Publisher('/lynxMotion/odom', Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()
    rospy.init_node('odometry_publisher', anonymous=True)
    rospy.Subscriber('/lynxMotion/cmd_vel', Twist, cmdVel_callback)
    rate = rospy.Rate(100) #10Hz
    

    last_time = rospy.Time.now()
    x=0.0
    y=0.0
    alpha=0.0
    dx=0.0
    dy=0.0
    dalpha=0.0
    dthR=0.0
    dthL=0.0

    while not rospy.is_shutdown():
	current_time = rospy.Time.now()       
	inChar = usbPort.readline(1)
	if inChar == "$" :
	    inMessage = usbPort.read(5)
	    usbPort.flushInput()

            if  inMessage[4] == "#":
                dth = unpack ("hh", inMessage[0:4])
                dthR = dth[0]*0.01
		dthL = dth[1]*0.01
		dx = (r/2)*(cos(alpha)*dthR+cos(alpha)*dthL)
		dy = (r/2)*(sin(alpha)*dthR+sin(alpha)*dthL)
		dalpha = (r*dthR-r*dthL)/(2*b)

		dt = (current_time - last_time).to_sec()
		delta_alpha = dalpha * dt
		delta_x = dx * dt
		delta_y = dy * dt

		alpha += delta_alpha
		x += delta_x
		y += delta_y
	    else:
		usbPort.flushInput()
	else:
	    usbPort.flushInput()

	# converting into quaternion
	odom_quat = tf.transformations.quaternion_from_euler(0,0,alpha)

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
   	odom.twist.twist = Twist(Vector3(dx, dy, 0), Vector3(0, 0, dalpha))

  	# publish the message
   	odom_pub.publish(odom)

   	last_time = current_time
        rate.sleep()

if __name__ == '__main__':
    try:
        LinxMotion()
    except rospy.ROSInterruptException:
        pass
