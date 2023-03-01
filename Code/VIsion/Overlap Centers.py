import cv2
import numpy as np
import math

# Initialize the camera
camera = cv2.VideoCapture(0)

while True:
    # Capture the frame from the camera
    ret, frame = camera.read()

    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the edges in the image using Canny edge detection
    edges = cv2.Canny(gray, 50, 150)


    blur = cv2.blur(edges, (10,10))
    cv2.imshow("Blur", blur)

    # Find the contours in the image
    contours, hierarchy = cv2.findContours(edges,  cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Loop through all the contours

    for i, contour in enumerate(contours):
        # Calculate the moments of the contour
        M = cv2.moments(contour)

        # Calculate the center of the contour
        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
        else:
            center_x=0
            center_y=0
    
        cv2.circle(edges, (center_x, center_y), 5, (0, 0, 255), -1)

        # Calculate the center of the lens
        lens_center_x = int(frame.shape[1] / 2)
        lens_center_y = int(frame.shape[0] / 2)

    # Draw a circle at the center of the lens
    cv2.circle(edges, (lens_center_x, lens_center_y), 5, (0, 255, 0), -1)

    # Draw a line connecting the center of the square and the center of the lens
    cv2.line(edges, (center_x, center_y), (lens_center_x, lens_center_y), (255, 0, 0), 2)

    #Calculate Distance
    distance_x = round(((center_y-lens_center_y)**2),2)
    distance_y = round(((lens_center_x-center_x)**2),2)
    print ("X Distance: ", distance_x)
    print ("Y DIstance: ", distance_y)

    # Display the image
    cv2.imshow("Frame", frame)
    cv2.imshow("Edges", edges)

    # Check if the user pressed the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and destroy the windows
camera.release()
cv2.destroyAllWindows()
