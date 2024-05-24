from picamera2 import Picamera2
import cv2
import time
import numpy as np
import sys
import math

picam2 = Picamera2()
picam2.start()
time.sleep(1)
left_line_x = []
left_line_y = []
right_line_x = []
right_line_y = []

if __name__=='__main__':
	print('Started')
	
	while True:
		try:
			#fetching each frame
			array = picam2.capture_array("main")
			cur_frame = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
			frame = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY) #Convert the image to a grayscale
		
			if frame is None:
				break
			#Code for other function
			#Apply Gaussian Blur
			Gaussian = cv2.GaussianBlur(frame, (5, 5), 0)
			#Prints out the image of the Gaussian
			#cv2.imshow('Gaussian Blurring', Gaussian)
			
			#Apply Canny edge detector to the output image
			dst = cv2.Canny(Gaussian, 50, 200, None, 3)
			#cv2.imshow('Canny edge', dst) #Prints out the image of the Canny edge
			#Region of interest
			height = frame.shape[0]
			width = frame.shape[1]
			
			#Points of the trapezoid
			bottom_left = [0, height]
			top_left = [0.2 * width, height * 0.4]
			top_right = [0.8 * width, height * 0.4]
			bottom_right = [width, height]
			
			mask = np.zeros_like(dst)
			
			roi_corners = np.array([bottom_left, top_left, top_right, bottom_right], np.int32)
			Picamera2
			cv2.fillPoly(mask, [roi_corners], (255, 255, 255))
			
			#cv2.imshow("",mask)
			
			poi = cv2.bitwise_and(dst, mask)
			
			cv2.imshow('ROI', poi)
			
			cdst = cv2.cvtColor(poi, cv2.COLOR_GRAY2BGR)
			cdstP = np.copy(cdst)
			#Draw the lines			
			lines = cv2.HoughLinesP(poi, 1, np.pi / 180, 50, None, 50, 10)
			#Draw the lines
			if lines is not None:
				print('lines detected: ',  len(lines))		
				for i in range(0, len(lines)):
					l = lines[i][0]
					m = float((l[3]-l[1])/(l[2]-l[0]))
					if not np.isnan(m) or np.isinf(m) or (m == 0):
						if abs(l[2]-l[0]) < abs(l[3]- l[1]):
							cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
					print(lines)


					cv2.line(cdstP, (line[0], line[1]), (line[2], line[3]), (0,0,255), 3, cv2.LINE_AA)

					for line in lines:
						for x1, y1, x2, y2 in line:
							slope = (y2 - y1) / (x2 - x1) # <-- Calculating the slope.
							if math.fabs(slope) < 0.5: # <-- Only consider extreme slope
								
								continue
							if slope <= 0: # <-- If the slope is negative, left group.
								left_line_x.extend([x1, x2])
								left_line_y.extend([y1, y2])
							else: # <-- Otherwise, right group.
								right_line_x.extend([x1, x2])
								right_line_y.extend([y1, y2])
				min_y = height * 0.4 # <-- Just below the horizon
				max_y = frame.shape[0] # <-- The bottom of the image
				poly_left = np.poly1d(np.polyfit(
					left_line_y,
					left_line_x,
					deg=1
				))
				left_x_start = int(poly_left(max_y))
				left_x_end = int(poly_left(min_y))
				poly_right = np.poly1d(np.polyfit(
					right_line_y,
					right_line_x,
					deg=1
				))
				right_x_start = int(poly_right(max_y))
				right_x_end = int(poly_right(min_y))
				cv2.line(cdstP, (left_x_start, max_y), (left_x_end, min_y), (0,0,255), 3, cv2.LINE_AA)
				cv2.line(cdstP, (right_x_start, max_y), (right_x_end, min_y), (0,0,255), 3, cv2.LINE_AA)


			cv2.imshow("Lines", cdstP)
			
			#End of the code Functions
			cv2.imshow('Frame', cur_frame)
		
			keyboard = cv2.waitKey(30)
			if keyboard == 27:
				break
		except KeyboardInterrupt:
			break

#cleanup
cv2.destroyAllWindows()
picam2.close()
print('Stopped')

