#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def move_forward():
    rospy.init_node('move_forward', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    try:
        duration = int(raw_input("Enter the time duration in seconds: "))
    except ValueError:
        print("Invalid input. Please enter an integer.")
        return

    twist = Twist()
    twist.linear.x = 0.2  # Adjust this value to set the desired linear velocity
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.get_time()
    while rospy.get_time() - start_time < duration:
        cmd_vel_pub.publish(twist)
        rate.sleep()

    twist.linear.x = 0  # Stop the robot
    cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass