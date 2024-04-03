from picamera2 import PiCamera2
import cv2
import time
import numpy as np
import sys
import math


picam2 = PiCamera2
picam2.start()
time.sleep()

if __name__=='__main__':
    print('Started')

    while True:
        try:
            #fectching each frame
            array = picam2.capture_array("main")
            cur_frame = cv2.cvtColor(array, cv2.Color_RGB2BGR)
            #Convert the image to a grayscale
            frame = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)

            if frame is None:
                break
            #Code for other function
            #Apply Gaussian Blur
            Gaussian = cv2.GaussianBlur (frame, (5, 5), 0)
            #Apply Canny edge detector
            dst = cv2.Canny(Gaussian, 50, 200, None, 3)
            #Prints out the image of Canny edge detector
            cv2.imshow('Canny edge', dst)
            height = frame.shape[0] #If it doesnt work use 639
            width = frame.shape[1] #

            #Points of Trapezoid
            bottom_left = [0,height]
            bottom_right = [width,height]
            top_right = [0.55*width,0.2*height]
            top_left = [0.45*width,0.2*height]

            #Masking and 
            mask = np.zeros_like(dst)

            roi_corners = np.array([bottom_left, bottom_right, top_left, top_right])

            cv2.fillPoly(mask, [roi_corners], 255)

            poi = cv2.bitwise_and(dst,mask)

            #Image of the mask with blur
            cv2.imshow('ROI',poi)

            cdstP = np.copy(cur_frame)

            lines = cv2.HoughLinesP(cdstP, 1, np.pi / 180, 50, None, 50, 10)
            #Draw Lines
            if lines is not None:
                  for i in range(0, len(lines)):
                        l = lines[i][0]
                        m = float((l[3]-l[1])/(l[2]-l[0]))
                        if (m > -1) and (m < 1) :
                            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]),(0,0,255), 3, cv2.LINE_AA)
                        if (m > 1) and (m < -1) :
                            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]),(0,0,255), 3, cv2.LINE_AA)
                
                cv2.imshow("Lines", cdstP)
            #for line in houghLinesP:
	        #    for x1,y1,x2,y2 in line:		
		    #        a = float((y2-y1)/(x2-x1))   		
	   
		            #if not np.isnan(a) or np.isinf(a) or (a == 0):
			        #    if (a > -1.0) and (a < -1.0) :
				    #        linesFiltered.append(line) 	
			        #    if (a > 1.0) and (a < 1.0) :
				    #        linesFiltered.append(line)

            keyboard = cv2.waitKey(30)
            if keyboard ==27:
                  break
        except KeyboardInterrupt:
            break

#cleanup
cv2.destoryAllWindows()
picam2.close()
print("Stopped")