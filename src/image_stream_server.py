#!/usr/bin/env python3
import select
import socket

import numpy
import rospy
import cv2
import netifaces as ni
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def send_image(image):
    br = CvBridge()
    try:
        ready_to_read, ready_to_write, in_error = \
            select.select([], [streamConnexion, ], [], 5)
        if ready_to_write:
            try:
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                result, img_encode = cv2.imencode('.jpg', br.imgmsg_to_cv2(image), encode_param)
                img_data = numpy.array(img_encode)
                string_data = img_data.tostring()
                streamConnexion.send(len(string_data).to_bytes(length=2, byteorder="big"))
                streamConnexion.send(string_data)
            except CvBridgeError as e:
                rospy.logerr(e)
    except select.error:
        print('Connexion closed, terminating node')
        streamConnexion.close()
        subscriber.unregister()
        rospy.signal_shutdown(reason="connexion closed")


if __name__ == '__main__':
    rospy.init_node("mobile_controller", anonymous=True)
    localIpAddress = ip = ni.ifaddresses('enp0s3')[ni.AF_INET][0]['addr']
    print(localIpAddress)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as streamSocket:
        streamSocket.bind((localIpAddress, 3698))

        streamSocket.listen(True)
        streamConnexion, mobileAddress = streamSocket.accept()

        with streamConnexion:
            print('3698: Connected by', mobileAddress)
            subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, send_image)
            rospy.spin()
