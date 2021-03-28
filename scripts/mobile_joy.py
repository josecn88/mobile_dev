 #!/usr/bin/env python
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy

    def callback(data):
        twist = Twist()
        twist.linear.x = data.axes[1]
        twist.angular.z = data.axes[0]
        pub.publish(twist)

    # Intializes everything
    def start():
        # publishing to "/mobile/joy" to control robot
        global pub
        pub = rospy.Publisher('/mobile/joy', Twist)
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, callback)
        # starts the node
        rospy.init_node('Joy2Mobile')
        rospy.spin()

    if __name__ == '__main__':
        start()