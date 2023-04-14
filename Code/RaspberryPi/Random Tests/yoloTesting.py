import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cv2.waitKey(1000)

while True:

    # Display the resulting image
    cv2.imshow('frame', cap)

    # Wait for a key press and check if it was the escape key
    key = cv2.waitKey(1)
    if key == 27:
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
