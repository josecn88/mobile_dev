#!/usr/bin/env python
import math
import numpy as np
import rospy 
import tf
from nav_msgs.msg import Odometry

class Target:
    # ROS variables
    pub = rospy.Publisher('/target/pose', Odometry, queue_size=100)
    target = Odometry()

    #leminiscate trajectory params
    a = 0
    c = 0
    step = 0.1
    t = 0

    def __init__(self):
        self.c = 10
        self.a = self.c*math.sqrt(2)
        rospy.init_node('targetNode')

    def leminiscate(self,time):
        x = self.a * math.cos(time)/(1+math.sin(time)**2)
        y = self.a * math.sin(time)*math.cos(time)/(1+math.sin(time)**2)
        return [x,y]

    def runTarget(self):
        point = self.leminiscate(self.t)
        self.target.pose.pose.position.x = point[0]
        self.target.pose.pose.position.y = point[1]
        self.target.twist.twist.linear.x = 0.5
        self.pub.publish(self.target)
        self.t = self.t + self.step


    def run(self):
        rate=rospy.Rate(0.5)
        while not rospy.is_shutdown():
            self.runTarget()
            rate.sleep()
        rospy.spin()



if __name__ ==  '__main__':
    tgt = Target()
    tgt.run()