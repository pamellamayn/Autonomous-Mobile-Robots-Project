#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickController:
    def __init__(self):
        # Initialize the node
        rospy.init_node('joystick_controller')

        # Publishers and Subscribers
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Parameters (You may need to adjust based on your joystick and robot)
        self.linear_axis = 1
        self.angular_axis = 0
        self.scale_linear = 0.5
        self.scale_angular = 1.0

    def joy_callback(self, msg):
        # Create a Twist message
        twist = Twist()
        twist.linear.x = msg.axes[self.linear_axis] * self.scale_linear
        twist.angular.z = msg.axes[self.angular_axis] * self.scale_angular

        # Publish the message
        self.twist_pub.publish(twist)

    def run(self):
        # Spin the node to keep it active
        rospy.spin()

if __name__ == '__main__':
    controller = JoystickController()
    controller.run()
