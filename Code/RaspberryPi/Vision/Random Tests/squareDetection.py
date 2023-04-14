import cv2
import numpy as np

# Initialize the camera
camera = cv2.VideoCapture(0)

while True:
    # Capture the frame from the camera
    ret, frame = camera.read()

    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the edges in the image using Canny edge detection
    edges = cv2.Canny(gray, 50, 150)

    # Find the contours in the image
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Loop through all the contours
    for contour in contours:
        # Check if the contour is a square
        if len(contour) == 4:
            # Calculate the moments of the contour
            moments = cv2.moments(contour)

            # Calculate the center of the contour
            if moments["m00"] != 0:
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
            #else:
            #    center_x = 0
            #    center_y = 0

            # Draw a circle at the center of the contour
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    # Display the image
    cv2.imshow("Frame", frame)
    #cv2.imwrite('/home/pi/output.jpg', frame)

    # Check if the user pressed the 'q' key
    c = cv2.waitKey(1)
    if c == 27:
        break

# Release the camera and destroy the windows
camera.release()
cv2.destroyAllWindows()