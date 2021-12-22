#!/usr/bin/env python3
import select
import socket
import struct
import sys

import netifaces as ni
import rospy
from geometry_msgs.msg import Twist

inputControlConnexion: socket


# read inputs from client and translates to Twist messages
def handle_inputs():
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    # store data from client
    data = b""
    while True:
        try:
            # wait before being able to read in the socket
            # we only want to read
            ready_to_read, ready_to_write, in_error = \
                select.select([inputControlConnexion, ], [], [], 5)
            if ready_to_read:
                data += inputControlConnexion.recv(1024)
                # get speed
                x = struct.unpack('f', data[:4])[0]
                # get angle of rotation
                z = struct.unpack('f', data[4:8])[0]
                # remove used data
                data = data[8:]

                # move forward or backward based on rotation, and convert to Twist format
                if 90 <= z <= 270:
                    speed = -x / 5
                else:
                    speed = x / 5

                # move left or right based on rotation,  and convert to Twist format
                if 0 <= z <= 180:
                    angle = -z / 360
                else:
                    angle = (360 - z) / 360

                print("Speed: " + str(speed) + " Angle: " + str(angle))

                # send Twist message to move the robot accordingly
                msg = Twist()
                msg.linear.x = speed
                msg.angular.z = angle
                vel_pub.publish(msg)
        except (select.error, struct.error):
            # On connection closed, close the connection and terminate the node
            print('Connexion closed, terminating node')
            inputControlConnexion.close()
            sys.exit()


if __name__ == '__main__':
    rospy.init_node("input_control_server", anonymous=True)
    localIpAddress = ip = ni.ifaddresses('enp0s3')[ni.AF_INET][0]['addr']

    # open server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as streamSocket:
        streamSocket.bind((localIpAddress, 3699))
        streamSocket.listen(True)

        # wait for a client to connect
        inputControlConnexion, mobileAddress = streamSocket.accept()

        # when a client is connected, read for inputs
        with inputControlConnexion:
            print('3699: Connected by', mobileAddress)
            inputControlConnexion.settimeout(None)
            handle_inputs()
