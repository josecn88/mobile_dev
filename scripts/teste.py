#!/usr/bin/env python
import math
import numpy as np
import rospy 
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Mobile:
    pub = rospy.Publisher('/sim_p3at/cmd_vel', Twist, queue_size=100)
    #messages parameters
    odomMsg = Odometry()
    #joyMsg = Twist()
    cmd_vel = Twist()

    def __init__(self):
        
        #rospy.Subscriber('/sim_p3at/odom', Odometry, callback=self.odom_callback , callback_args='/sim_p3at/odom')
        pub = rospy.Publisher('/sim_p3at/cmd_vel', Twist, queue_size=100)

    def run(self):
        self.cmd_vel.linear.x = 1
        self.cmd_vel.angular.z = 0.5
        rospy.loginfo("cmd_vel msg: %s", self.cmd_vel)
        self.pub.publish(self.cmd_vel)





if __name__ == '__main__':
    
    
    rospy.init_node('mobile')

    rate = rospy.Rate(100)
    try:
        robot = Mobile()
        while not rospy.is_shutdown():
            robot.run()
            rate.sleep()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass