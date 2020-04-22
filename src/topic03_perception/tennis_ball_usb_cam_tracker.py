#!/usr/bin/env python

import rospy
from tennis_ball_publisher import _video_publisher

def main():
    node_name = 'tennis_ball_usb_cam_tracker'
    rospy.init_node(node_name)
    
    video_path = 0 # WebCam
    try:
        _vp = _video_publisher(video_path)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()