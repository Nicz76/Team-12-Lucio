from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np

# Initialize the camera and set parameters
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
raw_capture = PiRGBArray(camera, size=(640, 480))

# Allow the camera to warm up
import time
time.sleep(0.1)

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked = cv2.bitwise_and(img, mask)
    return masked

def draw_lines(img, lines):
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 5)

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

# Capture frames from the camera
for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    image = frame.array
    
    # Convert the frame to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Perform Canny edge detection
    edges = cv2.Canny(blur, 50, 150)
    
    # Define a region of interest (ROI)
    height, width = edges.shape
    roi_vertices = [(0, height), (width / 2, height / 2), (width, height)]
    cropped_edges = region_of_interest(edges, np.array([roi_vertices], np.int32))
    
    # Detect lines using Hough transform
    lines = cv2.HoughLinesP(cropped_edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=150)
    
    # Get lane lines
    left_line, right_line = lane_lines(image, lines)
    
    # Draw lane lines on the original image
    lane_image = draw_lane_lines(image, (left_line, right_line))
    result = cv2.addWeighted(image, 0.8, lane_image, 1, 0)
    
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
    
    # Clear the stream in preparation for the next frame
    raw_capture.truncate(0)
    
    # Break the loop on 'q' key press
    if key == ord("q"):
        break

# Cleanup
cv2.destroyAllWindows()
