#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from assignment6_task1 import SimpleColorDetector
from assignment6_task1 import getYellowBounds

class image_subscriber:

    def __init__(self):
        self._bridge = CvBridge()
        self._subscriber = rospy.Subscriber('tennis_ball_image', Image, self._image_callback)
        rospy.on_shutdown(self._shutdown_callback)
        rospy.spin()

    def _image_callback(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Display original frame
            cv2.imshow('Original BGR image', cv_image)

            # Detect the balls
            SimpleColorDetector(cv_image, getYellowBounds())

            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def _shutdown_callback(self):
        cv2.destroyAllWindows()


def main():

    node_name = 'tennis_ball_listener'
    rospy.init_node(node_name)

    try:
        _is = image_subscriber()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()