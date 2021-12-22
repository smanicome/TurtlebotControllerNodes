#!/usr/bin/env python3
import select
import socket

import numpy
import rospy
import cv2
import netifaces as ni
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


# For each frame, convert it into a list of bytes to send to the client with the size of the image
def send_image(image):
    br = CvBridge()
    try:
        # wait before being able to write in the socket
        # we only want to write
        ready_to_read, ready_to_write, in_error = \
            select.select([], [streamConnexion, ], [], 5)
        if ready_to_write:
            try:
                # first convert the frame to an OpenCV image
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                result, img_encode = cv2.imencode('.jpg', br.imgmsg_to_cv2(image), encode_param)
                # retrieve a list of bytes
                img_data = numpy.array(img_encode)
                string_data = img_data.tostring()
                # send both the size of the image and the image itself
                streamConnexion.send(len(string_data).to_bytes(length=8, byteorder="big"))
                streamConnexion.send(string_data)
            except CvBridgeError as e:
                rospy.logerr(e)
    except select.error:
        # On connection closed, close the connection and terminate the node
        print('Connexion closed, terminating node')
        streamConnexion.close()
        subscriber.unregister()
        rospy.signal_shutdown(reason="connexion closed")


if __name__ == '__main__':
    rospy.init_node("image_stream_server", anonymous=True)
    localIpAddress = ip = ni.ifaddresses('enp0s3')[ni.AF_INET][0]['addr']

    # open server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as streamSocket:
        streamSocket.bind((localIpAddress, 3698))

        streamSocket.listen(True)
        # wait for a client to connect
        streamConnexion, mobileAddress = streamSocket.accept()

        # when a client is connected, register camera subscriber and spin
        with streamConnexion:
            print('3698: Connected by', mobileAddress)
            subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, send_image)
            rospy.spin()
