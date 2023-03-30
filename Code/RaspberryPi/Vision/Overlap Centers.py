import cv2
import numpy as np

# Define the ranges of each color in HSV
blue_lower = np.array([100,50,50])
blue_upper = np.array([130,255,255])

yellow_lower = np.array([20,100,100])
yellow_upper = np.array([40,255,255])

red_lower1 = np.array([0,100,100])
red_upper1 = np.array([10,255,255])
red_lower2 = np.array([160,100,100])
red_upper2 = np.array([179,255,255])

black_lower = np.array([0,0,0])
black_upper = np.array([180,255,30])

# Create a VideoCapture object
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the video stream
    ret, frame = cap.read()
    
    if not ret:
        print("Error reading frame from video stream")
        break
    
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create masks for each color
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    black_mask = cv2.inRange(hsv, black_lower, black_upper)

    # Set pixels of each color to white
    frame[blue_mask != 0] = [255, 255, 255]
    frame[yellow_mask != 0] = [255, 255, 255]
    frame[red_mask != 0] = [255, 255, 255]
    frame[black_mask != 0] = [255, 255, 255]

    # Find the center of the white section
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    # Display the resulting image
    cv2.imshow("Result", frame)
    
    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the VideoCapture object and close all windows
cap.release()
cv2.destroyAllWindows()
