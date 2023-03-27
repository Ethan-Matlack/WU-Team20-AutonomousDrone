import cv2
import numpy as np
import os


def main():

    camera = cv2.VideoCapture(0)

    while True:
        color_r = (10, 255, 255)
        color_g = (50, 255, 255)
        color_b = (120, 255, 255)

        lower_bound_r = np.array([0, 50, 50])
        upper_bound_r = np.array([20, 255, 255])
        lower_bound_g = np.array([40, 50, 50])
        upper_bound_g = np.array([70, 255, 255])
        lower_bound_b = np.array([90, 50, 50])
        upper_bound_b = np.array([140, 255, 255])

        _, frame = camera.read()
        cv2.imshow('Raw Camera', frame)
        frame = cv2.medianBlur(frame, 5)

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_mask_r = cv2.inRange(frame_hsv, lower_bound_r, upper_bound_r)
        # frame_mask_g = cv2.inRange(frame_hsv, lower_bound_g, upper_bound_g)
        frame_mask_b = cv2.inRange(frame_hsv, lower_bound_b, upper_bound_b)

        cv2.imshow('Red Mask  ', frame_mask_r)
        # cv2.imshow('Green Mask', frame_mask_g)
        cv2.imshow('Blue Mask ', frame_mask_b)

        # Check if the user pressed the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()
