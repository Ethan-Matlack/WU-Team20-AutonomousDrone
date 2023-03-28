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
black_upper = np.array([180,185,90])

#frame = cv2.imread('Aruco/15.png')
#frame = cv2.imread('Pictures/image2.png')
frame = cv2.imread('Base/Image 1.jpeg')

x=1
while x<=1:
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create masks for each color
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    black_mask = cv2.inRange(hsv, black_lower, black_upper)

    '''# Set pixels of each color to white
    frame[blue_mask != 0] = [255, 255, 255]
    frame[yellow_mask != 0] = [255, 255, 255]
    frame[red_mask != 0] = [255, 255, 255]
    frame[black_mask != 0] = [255, 255, 255]'''


    # Convert the image to grayscale
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("Gray", gray)

    # Detect the edges in the image using Canny edge detection
    edges = cv2.Canny(frame, 50, 150)

    blur = cv2.blur(edges, (10,10))

    ret, thresh = cv2.threshold(blur, 127, 255, 0)

    # Find the contours in the image
    contours, hierarchy = cv2.findContours(edges,  cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Loop through all the contours
    for i, contour in enumerate(contours):
        # Calculate the moments of the contour
        M = cv2.moments(contour)
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Calculate the center of the contour
        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
        else:
            center_x=0
            center_y=0
        
        if np.any(blue_mask[center_x-10:center_x+10, center_y-10:center_y+10] == 0):
            if red_mask[center_x, center_y] == 0:
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            #print(center_x, center_y)
    
    # Calculate the center of the lens
    lens_center_x = int(frame.shape[1] / 2)
    lens_center_y = int(frame.shape[0] / 2)

    # Draw a circle at the center of the lens
    cv2.circle(frame, (lens_center_x, lens_center_y), 5, (0, 255, 0), -1)

    # Draw a line connecting the center of the square and the center of the lens
    cv2.line(frame, (center_x, center_y), (lens_center_x, lens_center_y), (255, 0, 0), 2)
    x=x+1

# Display the resulting image
cv2.imshow("Result", frame)
cv2.imwrite("Results/base1.jpg", frame)
cv2.destroyAllWindows()
