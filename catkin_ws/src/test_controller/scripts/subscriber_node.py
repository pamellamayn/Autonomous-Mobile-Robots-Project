#!/usr/bin/env python3
import math
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan

looped = False
msg_queue = []

def one_roatation_plot(msgs):
    
    x_cords = []
    y_cords = []
    distances = []
    for msg in msgs:
        ranges = msg.ranges
        distances += ranges
        angles = [msg.angle_min + msg.angle_increment*i for i in range(len(msg.ranges))]
        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]
            x_cords.append(math.sin(angle) * distance)
            y_cords.append(math.cos(angle) * distance)

    max_val = msg.range_max + 5
    # print(sorted(distances))
    plt.xlim(-max_val, max_val)
    plt.ylim(-max_val, max_val)
    
    plt.scatter(x_cords, y_cords, s=.1)
    plt.show()

def lidar_scan_reciver(msg):
    print("msg recived")
    global msg_queue
    global looped

    if len(msg_queue) <= 1000:
        msg_queue.append(msg)
    
    # if msg_queue[0].angle_min < msg_queue[-1].angle_max and looped:
    if len(msg_queue) >= 1000:
        # one_roatation_plot(msg_queue)
        msg_queue = []
        looped = False
    # elif msg_queue[-2].angle_max - msg_queue[-1].angle_min > 0:
    #     looped = True

if __name__ == '__main__':
    rospy.init_node("subscriber_node")
    rospy.loginfo("STARTED Subscriber")

    sub = rospy.Subscriber("scan", LaserScan, callback=lidar_scan_reciver)

    rospy.spin()
