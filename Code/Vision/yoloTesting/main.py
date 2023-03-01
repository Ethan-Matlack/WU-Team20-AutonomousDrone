import cv2
import numpy as np

# Define the lower and upper bounds for the white color range in BGR format
lower_white = np.array([200, 200, 200])
upper_white = np.array([255, 255, 255])

# Open the camera and wait for it to warm up
cap = cv2.VideoCapture(0)
cv2.waitKey(1000)

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Convert the frame to grayscale and threshold it to get only the white pixels
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    # Find the contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If we found any contours
    if len(contours) > 0:
        # Find the contour with the largest area, which should be the circle
        circle_contour = max(contours, key=cv2.contourArea)

        # Find the center of the circle
        (x, y), radius = cv2.minEnclosingCircle(circle_contour)
        center = (int(x), int(y))

        # Draw a line between the center of the image and the center of the circle
        image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        cv2.line(frame, image_center, center, (0, 0, 255), 2)

        # Draw a circle around the detected circle
        cv2.circle(frame, center, int(radius), (0, 255, 0), 2)

    # Display the resulting image
    cv2.imshow('frame', frame)

    # Wait for a key press and check if it was the escape key
    key = cv2.waitKey(1)
    if key == 27:
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
