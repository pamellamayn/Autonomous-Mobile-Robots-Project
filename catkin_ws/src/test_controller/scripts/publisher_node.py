#!/usr/bin/env python3
import math
import rospy
import serial
from sensor_msgs.msg import LaserScan

previous_scan_time = 0

def create_scan(data):
    # print(data)
    mesured_data = [int(byte_data[2:], 16) for byte_data in data[5:-6]]
    # print(mesured_data)
    length = int(len(mesured_data)/3)
    msg = LaserScan()

    msg.range_min = 0.05
    msg.range_max = 12

    msg.ranges = []
    msg.intensities = []
    for i in range(0, length):
        lsb = mesured_data[i*3]
        msb = mesured_data[i*3+1]
        distance = ((msb << 8) | lsb)*.001
        if distance > msg.range_max or distance < msg.range_min:
            distance = 0
        msg.ranges.append(distance) # .001 mm -> m
        msg.intensities.append(mesured_data[i*3+2]) # raw number

    msb = int(data[4][2:], 16)
    lsb = int(data[3][2:], 16)
    msg.angle_min = math.radians((((msb << 8) | lsb))*.01) # rad

    msb = int(data[-5][2:], 16)
    lsb = int(data[-6][2:], 16)
    msg.angle_max = math.radians((((msb << 8) | lsb))*.01) # rad
    msg.angle_increment = (msg.angle_max - msg.angle_min)/(length-1) # rad / step

    msb = int(data[2][2:], 16)
    lsb = int(data[1][2:], 16)
    speed = math.radians((msb << 8) | lsb) # rad / sec

    msb = int(data[-3][2:], 16)
    lsb = int(data[-4][2:], 16)
    time = ((msb << 8) | lsb) * .001 # miliseconds to seconds

    global previous_scan_time
    msg.scan_time = time-previous_scan_time # time between messages
    previous_scan_time=time

    msg.time_increment = msg.scan_time/length # time between individual data points


    print(msg.angle_min, msg.angle_max, speed, time)
    return msg

if __name__ == '__main__':
    rospy.init_node("publisher_node")
    rospy.loginfo("STARTED PUBLISHER")

    pub = rospy.Publisher("scan", LaserScan, queue_size=10)
    rate = rospy.Rate(100)

    lidar = serial.Serial(port='/dev/ttyUSB0', baudrate=230400, bytesize=serial.EIGHTBITS, stopbits=1, parity=serial.PARITY_NONE, timeout=None)
    lidar.reset_input_buffer()
    lidar.reset_output_buffer()
    print(lidar.name)
    first = True
    message = []
    while not rospy.is_shutdown():
        message.append(lidar.read())
        if message[-1] == b'\x54' and len(message) == 47:
            # print(message)
            hex_codes = []
            for i in range(len(message)):
                hex_code = str(message[i])
                if '\\' in hex_code and 'x' not in hex_code:
                    hex_code = hex_code.replace('\\', '',1)
                if len(hex_code) == 4:
                    hex_code = hex(ord(hex_code[2:-1]))
                else:
                    hex_code = ("0x" + hex_code[4:-1])
                hex_codes.append(hex_code)
            if not first:
                msg = create_scan(hex_codes)
                pub.publish(msg)
                print("-----------------------------")
                
            else:
                first = False
            message = []
        elif message[-1] == b'\x54':
            print("BAD MESSAGE: len message: ", len(message), " message: ", message)
            message = []

            
        # rate.sleep()
