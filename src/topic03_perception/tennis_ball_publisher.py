#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class _video_publisher:

    def __init__(self, video_path):
        self._bridge = CvBridge()
        self._video_source = video_path
        self._publisher = rospy.Publisher('tennis_ball_image', Image, queue_size=1)

        # Start Capturing
        self._capture = cv2.VideoCapture(self._video_source)

        self.rate = rospy.Rate(30)
        # Start Publishing
        while not rospy.is_shutdown():
            ret, frame = self._capture.read()
            
            if ret == True:
                frame = cv2.resize(frame, (0,0), fx=0.5,fy=0.5)
                try:
                    self._publisher.publish(self._bridge.cv2_to_imgmsg(frame, 'bgr8'))
                except CvBridgeError, e:
                    print(e)
            
            self.rate.sleep()


def main():
    node_name = 'tennis_ball_publisher'
    rospy.init_node(node_name)
    
    video_path = '/home/nikolas/rasberry_ws/src/scratch_folder/ros_essentials_cpp/src/topic03_perception/video/tennis-ball-video.mp4'
    try:
        _vp = _video_publisher(video_path)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()