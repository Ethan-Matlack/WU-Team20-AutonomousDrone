import cv2 as cv
import numpy as np


def contour_compare(contour_list_a, contour_list_b, search_radius, mask_threshold):
    contour_list_common = []
    for contour_a in contour_list_a:
        contour_a = cv.approxPolyDP(contour_a, 30, False)
        for contour_b in contour_list_b:
            contour_b = cv.approxPolyDP(contour_b, 30, False)
            for point_a in contour_a:
                for point_b in contour_b:
                    if abs(point_a[0][0] - point_b[0][0]) < search_radius:
                        if abs(point_a[0][1] - point_b[0][1]) < search_radius:
                            # Rejects points that fall near the edge of the mask
                            # neighbors = np.array([[point_a[0][0], point_a[0][1] - search_radius],
                            #                       [point_a[0][0], point_a[0][1] + search_radius],
                            #                       [point_a[0][0] + search_radius, point_a[0][1]],
                            #                       [point_a[0][0] - search_radius, point_a[0][1]]])
                            # flag = 0
                            # for neighbor in neighbors:
                            #     if (neighbor[0] - 1) > mask_all.shape[0] or (neighbor[1] - 1) > mask_all.shape[1]:
                            #         break
                            #     if mask_all[neighbor[0] - 1, neighbor[1] - 1] > mask_threshold:
                            #         flag = flag + 1
                            # if flag == 4:
                                avg_x = round((point_a[0][0] + point_b[0][0]) / 2)
                                avg_y = round((point_a[0][1] + point_b[0][1]) / 2)
                                contour_list_common.append((avg_x, avg_y))
    return contour_list_common


def display_cardinal_markers():
    for coordinate in contours_northSouth:
        if coordinate[0] != 0:
            cv.circle(frame_contours_bgr, coordinate, radius=10, color=(0, 0, 0), thickness=3)

    for coordinate in contours_eastWest:
        if coordinate[0] != 0:
            cv.rectangle(frame_contours_bgr,
                         (coordinate[0]-10, coordinate[1]-10),
                         (coordinate[0]+10, coordinate[1]+10),
                         color=(0, 0, 0), thickness=3)


clustering_distance = 30
contour_validity_mask_threshold = 200

# Define the ranges of each color in HSV
cyan_lower = np.array([80, 100, 100])
cyan_upper = np.array([130, 255, 255])
yellow_lower = np.array([20, 100, 100])
yellow_upper = np.array([40, 255, 255])
magenta_lower1 = np.array([0, 100, 100])
magenta_upper1 = np.array([10, 255, 255])
magenta_lower2 = np.array([160, 100, 100])
magenta_upper2 = np.array([179, 255, 255])
black_lower = np.array([0, 0, 0])
black_upper = np.array([255, 255, 90])

# Initializing the webcam
capture = cv.VideoCapture(0)

while True:
    # Read in the camera frame by frame
    ret, frame_raw = capture.read()

    # Convert the frame out of BGR to HSV
    frame_hsv = cv.cvtColor(frame_raw, cv.COLOR_BGR2HSV)

    # Create masks for each color
    mask_cyan = cv.inRange(frame_hsv, cyan_lower, cyan_upper)
    mask_magenta1 = cv.inRange(frame_hsv, magenta_lower1, magenta_upper1)
    mask_magenta2 = cv.inRange(frame_hsv, magenta_lower2, magenta_upper2)
    mask_magenta = cv.bitwise_or(mask_magenta1, mask_magenta2)
    mask_yellow = cv.inRange(frame_hsv, yellow_lower, yellow_upper)
    mask_black = cv.inRange(frame_hsv, black_lower, black_upper)
    mask_all = cv.bitwise_or(cv.bitwise_or(mask_cyan, mask_magenta), cv.bitwise_or(mask_yellow, mask_black))

    # Blur each mask to aid contour detection
    gaussian_blur_stddev_x = 19
    gaussian_blur_stddev_y = 19
    mask_cyan = cv.GaussianBlur(mask_cyan, (gaussian_blur_stddev_x, gaussian_blur_stddev_y), 0)
    mask_magenta = cv.GaussianBlur(mask_magenta, (gaussian_blur_stddev_x, gaussian_blur_stddev_y), 0)
    mask_yellow = cv.GaussianBlur(mask_yellow, (gaussian_blur_stddev_x, gaussian_blur_stddev_y), 0)
    mask_black = cv.GaussianBlur(mask_black, (gaussian_blur_stddev_x, gaussian_blur_stddev_y), 0)

    cv.namedWindow("All Masks", cv.WINDOW_NORMAL)
    cv.resizeWindow("All Masks", int(frame_raw.shape[1]), int(frame_raw.shape[0]))
    cv.imshow("All Masks", np.vstack((np.hstack((mask_magenta, mask_yellow)), np.hstack((mask_cyan, mask_black)))))

    # Detect edges for each color mask
    edges_cyan = cv.Canny(image=mask_cyan, threshold1=200, threshold2=230)
    edges_magenta = cv.Canny(image=mask_magenta, threshold1=200, threshold2=230)
    edges_yellow = cv.Canny(image=mask_yellow, threshold1=200, threshold2=230)
    edges_black = cv.Canny(image=mask_black, threshold1=200, threshold2=230)

    cv.namedWindow("All Edges", cv.WINDOW_NORMAL)
    cv.resizeWindow("All Edges", int(frame_raw.shape[1]), int(frame_raw.shape[0]))
    cv.imshow("All Edges", np.vstack((np.hstack((edges_magenta, edges_yellow)), np.hstack((edges_cyan, edges_black)))))

    # TODO: This flag should probably be re-evaluated. External may not be correct.
    contours_cyan, hierarchy_cyan = cv.findContours(edges_cyan, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    contours_magenta, hierarchy_magenta = cv.findContours(edges_magenta, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    contours_yellow, hierarchy_yellow = cv.findContours(edges_yellow, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    contours_black, hierarchy_black = cv.findContours(edges_black, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Create a blank white canvas
    frame_contours_bgr = np.zeros_like(frame_hsv)
    frame_contours_bgr.fill(255)

    # Draw the contours in their respective colors on the frame
    # The colors of the contours below have to be in BGR format
    cv.drawContours(frame_contours_bgr, contours_cyan,    -1, (205, 149,   0), 2)
    cv.drawContours(frame_contours_bgr, contours_magenta, -1, (78,   31, 162), 2)
    cv.drawContours(frame_contours_bgr, contours_yellow,  -1, (24,  208, 255), 2)
    cv.drawContours(frame_contours_bgr, contours_black,   -1, (43,   38,  34), 2)

    # Create the new sets of contours that define the cardinal directions relative to the pad
    contours_northSouth = \
        contour_compare(contours_cyan, contours_black, clustering_distance, contour_validity_mask_threshold) + \
        contour_compare(contours_magenta, contours_yellow, clustering_distance, contour_validity_mask_threshold)
    contours_eastWest = \
        contour_compare(contours_cyan, contours_magenta, clustering_distance, contour_validity_mask_threshold) + \
        contour_compare(contours_black, contours_yellow, clustering_distance, contour_validity_mask_threshold)

    # Display the resulting image
    display_cardinal_markers()
    cv.imshow("All Contours", frame_contours_bgr)
    cv.imshow("Combined Mask", mask_all)
    print("Refreshed!")

    # Press "k" to quit
    if cv.waitKey(27) == ord('k'):
        cv.destroyAllWindows()
        break
