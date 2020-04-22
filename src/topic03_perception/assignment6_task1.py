#!/usr/bin/env python 

import cv2
import time
import numpy as np

class SimpleColorDetector:
    def __init__(self, bgr_frame, color_bounds):
        self.bgr_frame = bgr_frame #frame in BGR
        self.color_bounds = color_bounds
        self.mask = self.__filterColor()
        self.contours = self.__getContours()
        self.__drawContours()
        pass


    def __bgr2hsv(self):
        return cv2.cvtColor(self.bgr_frame, cv2.COLOR_BGR2HSV)

    def __filterColor(self):
        self.hsv_frame = self.__bgr2hsv()
        return cv2.inRange(self.hsv_frame, self.color_bounds[0], self.color_bounds[1])

    def __getContours(self):
        _, contours, hierarchy = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def __drawContours(self):
        black_image = np.zeros([self.mask.shape[0], self.mask.shape[1],3],'uint8')
    
        for c in self.contours:
            area = cv2.contourArea(c)
            perimeter= cv2.arcLength(c, True)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if (area>500):
                cv2.drawContours(self.bgr_frame, [c], -1, (150,250,150), 1)
                cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
                cx, cy = self.__getContourCenter(c)
                cv2.circle(self.bgr_frame, (cx,cy),(int)(radius),(0,0,255),1)
                cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
                cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
                print ("Area: {}, Perimeter: {}".format(area, perimeter))
        print ("number of contours: {}".format(len(self.contours)))
        cv2.imshow("RGB Image Contours",self.bgr_frame)
        cv2.imshow("Black Image Contours",black_image)

    def __getContourCenter(self, contour):
        M = cv2.moments(contour)
        cx=-1
        cy=-1
        if (M['m00']!=0):
            cx= int(M['m10']/M['m00'])
            cy= int(M['m01']/M['m00'])
        return cx, cy
    
def getYellowBounds():
    yellow_lower_bounds = (30, 150, 100)
    yellow_upper_bounds = (50, 255, 255)
    return yellow_lower_bounds, yellow_upper_bounds

def main():
    #video_capture = cv2.VideoCapture(0)
    video_capture = cv2.VideoCapture('/home/nikolas/rasberry_ws/src/scratch_folder/ros_essentials_cpp/src/topic03_perception/video/tennis-ball-video.mp4')

    while(True):
        time.sleep(0.03)

        ret, frame = video_capture.read()
        if ret == True:
            frame = cv2.resize(frame, (0,0), fx=0.5,fy=0.5)

            # Display the original frame
            cv2.imshow("Original BGR",frame)

            # Detect the balls
            SimpleColorDetector(frame, getYellowBounds())

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()