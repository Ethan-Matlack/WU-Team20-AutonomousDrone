import cv2
import numpy as np

# Load the image
img = cv2.imread('Base/Image 1.jpeg')

# Convert to HSV color space
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Define the range of red color in HSV
red_lower1 = np.array([0,100,100])
red_upper1 = np.array([10,255,255])
red_lower2 = np.array([160,100,100])
red_upper2 = np.array([179,255,255])

# Define the range of blue color in HSV
blue_lower = np.array([100,50,50])
blue_upper = np.array([130,255,255])

# Create a mask for red and blue color
red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
red_mask = cv2.bitwise_or(red_mask1, red_mask2)
blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

# Combine the masks to get the intersection
mask = cv2.bitwise_and(red_mask, blue_mask)

# Apply the mask to the original image
result = cv2.bitwise_and(img, img, mask=mask)

# Display the result
cv2.imshow('Result', result)
cv2.waitKey(0)
cv2.destroyAllWindows()
