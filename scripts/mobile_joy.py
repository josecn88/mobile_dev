#!/usr/bin/env python
    
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
        twist = Twist()
        twist.linear.x = 0.75*data.axes[1]
        twist.angular.z = 1*data.axes[0]
        pub.publish(twist)

# Intializes everything
def start():
        # publishing to "/mobile/joy" to control robot
        global pub
        pub = rospy.Publisher('/mobile/joy', Twist,queue_size=100)
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("/joy", Joy, callback)
        # starts the node
        rospy.init_node('joy2Mobile')
        rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Starting mobile_joy...")
    start()