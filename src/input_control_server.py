#!/usr/bin/env python3
import socket
import struct

import numpy
import rospy
import cv2
import netifaces as ni
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

inputControlConnexion: socket


def handle_inputs():
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    data = b""
    while True:
        data += inputControlConnexion.recv(1024)
        x = struct.unpack('f', data[:4])[0]
        z = struct.unpack('f', data[4:8])[0]
        data = data[8:]

        if 90 <= z <= 270:
            speed = -x / 5
        else:
            speed = x / 5

        if 0 <= z <= 180:
            angle = -z / 360
        else:
            angle = (360 - z) / 360

        print("Speed: " + str(speed) + " Angle: " + str(angle))

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angle
        vel_pub.publish(msg)



if __name__ == '__main__':
    rospy.init_node("mobile_controller", anonymous=True)
    localIpAddress = ip = ni.ifaddresses('enp0s3')[ni.AF_INET][0]['addr']
    print(localIpAddress)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as streamSocket:
        streamSocket.bind((localIpAddress, 3699))
        streamSocket.listen(True)
        inputControlConnexion, mobileAddress = streamSocket.accept()

        with inputControlConnexion:
            print('3699: Connected by', mobileAddress)
            inputControlConnexion.settimeout(None)
            handle_inputs()
            rospy.spin()
