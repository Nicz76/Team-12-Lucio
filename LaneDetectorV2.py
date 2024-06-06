from picamera2 import Picamera2
import cv2
import time
import numpy as np
import sys
import math

picam2 = Picamera2()
picam2.start()
time.sleep(1)
rho = 2
theta = np.pi/180
threshold = 15	
min_line_length = 15
max_line_gap = 5

def average_slope_intercept(lines):
    left_lines = []  # (slope, intercept)
    right_lines = [] # (slope, intercept)
    left_weights = [] # (length,)
    right_weights = [] # (length,)
    
    if lines is None:
        return None, None
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2 - x1 == 0:  # avoid division by zero
                continue
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            length = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
            if slope < 0:  # y is reversed in image
                left_lines.append((slope, intercept))
                left_weights.append(length)
            else:
                right_lines.append((slope, intercept))
                right_weights.append(length)
    
    left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
    
    return left_lane, right_lane

def make_line_points(y1, y2, line):
    if line is None:
        return None
    slope, intercept = line
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    y1 = int(y1)
    y2 = int(y2)
    return ((x1, y1), (x2, y2))

def lane_lines(image, lines):
    left_lane, right_lane = average_slope_intercept(lines)
    y1 = image.shape[0]  # bottom of the image
    y2 = y1 * 0.6  # slightly lower than the middle
    
    left_line = make_line_points(y1, y2, left_lane)
    right_line = make_line_points(y1, y2, right_lane)
    
    return left_line, right_line

def draw_lane_lines(image, lines, color=[0, 255, 0], thickness=5):
    left_line, right_line = lines
    line_image = np.zeros_like(image)
    if left_line is not None:
        cv2.line(line_image, *left_line, color, thickness)
    if right_line is not None:
        cv2.line(line_image, *right_line, color, thickness)
    return line_image

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
			
			#cv2.imshow('ROI', poi)
			
			cdst = cv2.cvtColor(poi, cv2.COLOR_GRAY2BGR)
			cdstP = np.copy(cdst)
			#Draw the lines			
			lines = cv2.HoughLinesP(cdstP, rho, theta, threshold, np.array([]), minLineLength=min_line_length, maxLineGap=max_line_gap)

			# Get lane lines
			left_line, right_line = lane_lines(array, lines)
			
			# Draw lane lines on the original image
			lane_image = draw_lane_lines(array, (left_line, right_line))
			result = cv2.addWeighted(array, 0.8, lane_image, 1, 0)
            
			# Calculate the deviation from the center
            if left_line is not None and right_line is not None:
				mid = width / 2
				left_x2 = left_line[1][0]
				right_x2 = right_line[1][0]
				lane_center = (left_x2 + right_x2) / 2
				deviation = lane_center - mid
				
				# Display deviation
				cv2.putText(result, f"Deviation: {deviation:.2f} pixels", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
				
				# Provide simple steering suggestions
				if deviation < -50:
					direction = "Turn Left"
				elif deviation > 50:
					direction = "Turn Right"
				else:
					direction = "On Track"
				
				cv2.putText(result, f"Steering: {direction}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    	
			# Show the frame
            cv2.imshow("Frame", result)
       		key = cv2.waitKey(1) & 0xFF
			
					
			cv2.line(cdstP, (x1_left,y_max), (x2_left,), (0,0,255), 3, cv2.LINE_AA)			
			cv2.imshow("Lines", cdstP)
			
			#End of the code Functions
			cv2.imshow('Frame', cur_frame)
		
			keyboard = cv2.waitKey(30)
			if keyboard == 27:
				break
		except KeyboardInterrupt:
		break
	time.sleep(0.1)

#cleanup
cv2.destroyAllWindows()
picam2.close()
print('Stopped')

