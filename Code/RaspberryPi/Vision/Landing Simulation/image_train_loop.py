import cv2 as cv
import numpy as np
import os


def contour_compare(contour_list_a, contour_list_b, search_radius, mask_threshold):
    contour_list_common = []
    for contour_a in contour_list_a:
        # TODO: Evaluated whether or not approximating to open (False) or closed (True) contours is better
        contour_a = cv.approxPolyDP(contour_a, 30, True)
        for contour_b in contour_list_b:
            contour_b = cv.approxPolyDP(contour_b, 30, True)
            for point_a in contour_a:
                for point_b in contour_b:
                    if abs(point_a[0][0] - point_b[0][0]) < search_radius:
                        if abs(point_a[0][1] - point_b[0][1]) < search_radius:
                            avg_x = round((point_a[0][0] + point_b[0][0]) / 2)
                            avg_y = round((point_a[0][1] + point_b[0][1]) / 2)
                            contour_list_common.append((avg_x, avg_y))
    return contour_list_common


def display_cardinal_markers():
    for coordinate in contours_northSouth:
        if coordinate[0] != 0:
            cv.circle(frame_contours_bgr, coordinate, radius=5, color=(0, 255, 0), thickness=2)

    for coordinate in contours_eastWest:
        if coordinate[0] != 0:
            cv.rectangle(frame_contours_bgr,
                         (coordinate[0]-5, coordinate[1]-5),
                         (coordinate[0]+5, coordinate[1]+5),
                         color=(0, 255, 0), thickness=2)


clustering_distance = 15
contour_validity_mask_threshold = 200
norm_vect_scale_factor = 200
bestfitline_points = []
gaussian_blur_stddev_x = 19
gaussian_blur_stddev_y = 19

# Define the ranges of each color in HSV. Color space is [0-179, 0-255, 0-255]
# Updated via the color picker tool at 3:45PM on 2023-04-03
cyan_lower = np.array([80, 100, 100])
cyan_upper = np.array([130, 255, 255])
magenta_lower1 = np.array([0, 100, 100])
magenta_upper1 = np.array([10, 255, 255])
magenta_lower2 = np.array([160, 100, 100])
magenta_upper2 = np.array([179, 255, 255])
yellow_lower = np.array([20, 150, 150])
yellow_upper = np.array([40, 255, 255])
black_lower = np.array([40, 40, 40])
black_upper = np.array([179, 150, 150])

# Initializing the webcam
# capture = cv.VideoCapture(0)
# capture = cv.VideoCapture('TrainingImages/Real/TestIndoors_960x540_15fps.mp4')
# output = cv.VideoWriter('Output/Output.avi', cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), 15, (960, 540))

steps_per = 10
path_root_images = r"C:\Users\Administrator\Documents\GitHub\WU-Team20-AutonomousDrone\Code\RaspberryPi\Vision\Landing Simulation\TrainingImages"
path_rel_real_cleaned = r"real\cleaned"
path_rel_real_outdoor_shade = r"real\outdoor\shade"
path_rel_real_outdoor_sun = r"real\outdoor\sun"
path_rel_real_indoor = r"real\indoor"
path_rel_real_mixed_close = r"real\mixed_close"
path_rel_real_mixed_veryclose = r"real\mixed_veryclose"
path_rel_real_outdoor_mixed_overhead = r"real\outdoor\mixed_overhead"


active_path = path_rel_real_mixed_veryclose

break_flag = False

while not break_flag:

    for image_path in os.listdir(os.path.join(path_root_images, active_path)):
        for i in range(steps_per):
            # Read in the camera frame by frame
            # ret, frame_raw = capture.read()
            frame_raw = cv.resize(cv.imread(os.path.join(os.path.join(path_root_images, active_path), image_path)), (640, 480))
            # frame_raw = cv.imread('TrainingImages/Simulated/CMYK_on_WhiteBG.png')

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
            mask_cyan = cv.GaussianBlur(mask_cyan, (gaussian_blur_stddev_x, gaussian_blur_stddev_y), 0)
            mask_magenta = cv.GaussianBlur(mask_magenta, (gaussian_blur_stddev_x, gaussian_blur_stddev_y), 0)
            mask_yellow = cv.GaussianBlur(mask_yellow, (gaussian_blur_stddev_x, gaussian_blur_stddev_y), 0)
            mask_black = cv.GaussianBlur(mask_black, (gaussian_blur_stddev_x, gaussian_blur_stddev_y), 0)
            mask_all = cv.GaussianBlur(mask_all, (gaussian_blur_stddev_x, gaussian_blur_stddev_y), 0)

            cv.namedWindow("All Masks", cv.WINDOW_NORMAL)
            cv.resizeWindow("All Masks", int(frame_raw.shape[1]), int(frame_raw.shape[0]))
            cv.imshow("All Masks", np.vstack((
                np.hstack((mask_magenta, mask_yellow)),
                np.hstack((mask_cyan, mask_black)))))

            # Run Canny edge detection to find all the edges in the masks
            edges_cyan = cv.Canny(image=mask_cyan, threshold1=100, threshold2=200)
            edges_magenta = cv.Canny(image=mask_magenta, threshold1=100, threshold2=200)
            edges_yellow = cv.Canny(image=mask_yellow, threshold1=100, threshold2=200)
            edges_black = cv.Canny(image=mask_black, threshold1=100, threshold2=200)

            contours_cyan, hierarchy_cyan = cv.findContours(edges_cyan, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            contours_magenta, hierarchy_magenta = cv.findContours(edges_magenta, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            contours_yellow, hierarchy_yellow = cv.findContours(edges_yellow, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            contours_black, hierarchy_black = cv.findContours(edges_black, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            # contours_overall, hierarchy_overall = cv.findContours(mask_all, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            contours_cyan = sorted(contours_cyan, key=cv.contourArea, reverse=True)
            contours_magenta = sorted(contours_magenta, key=cv.contourArea, reverse=True)
            contours_yellow = sorted(contours_yellow, key=cv.contourArea, reverse=True)
            contours_black = sorted(contours_black, key=cv.contourArea, reverse=True)
            # contours_overall = sorted(contours_overall, key=cv.contourArea, reverse=True)


            # Create a blank white canvas of the same size as the HSV frame
            frame_contours_bgr = np.full_like(frame_hsv, 255)

            # Draw the contours in their respective colors on the frame
            # The colors of the contours below have to be in BGR format
            if contours_cyan:
                cv.drawContours(frame_contours_bgr, contours_cyan[0:5],    -1, (205, 149,   0), 2)
            if contours_magenta:
                cv.drawContours(frame_contours_bgr, contours_magenta[0:5], -1, (78,   31, 162), 2)
            if contours_yellow:
                cv.drawContours(frame_contours_bgr, contours_yellow[0:7],  -1, (24,  208, 255), 2)
            if contours_black:
                cv.drawContours(frame_contours_bgr, contours_black[0:10],   -1, (43,   38,  34), 2)
            # if contours_overall:
            #     cv.drawContours(frame_contours_bgr, contours_overall[0], -1, (0,   255,   0), 2)

            # Create the new sets of contours that define the cardinal directions relative to the pad
            contours_northSouth = \
                contour_compare(contours_cyan, contours_black,
                                clustering_distance, contour_validity_mask_threshold) + \
                contour_compare(contours_magenta, contours_yellow,
                                clustering_distance, contour_validity_mask_threshold)
            contours_eastWest = \
                contour_compare(contours_cyan, contours_magenta,
                                clustering_distance, contour_validity_mask_threshold) + \
                contour_compare(contours_black, contours_yellow,
                                clustering_distance, contour_validity_mask_threshold)

            # If the contours exist, fit a line of best fit to each of them using the least square method
            # ^^^ Temporarily switched to Huber method to hopefully expel the outliers more effectively
            if contours_northSouth and contours_eastWest:
                [ns_vx, ns_vy, ns_x1, ns_y1] = cv.fitLine(np.array(contours_northSouth), cv.DIST_HUBER, 0, 0.01, 0.01)
                [ew_vx, ew_vy, ew_x1, ew_y1] = cv.fitLine(np.array(contours_eastWest), cv.DIST_HUBER, 0, 0.01, 0.01)
                cv.circle(frame_contours_bgr, (int(ns_x1), int(ns_y1)), 10, (0, 255, 0), 2)
                cv.circle(frame_contours_bgr, (int(ew_x1), int(ew_y1)), 10, (0, 255, 0), 2)

                # TODO: This entire bit can be optimized. Intersect points can be calculated directly from the two fitLines.
                #  No need to used scaled norm vectors as intermediaries. Also, display funcs should be separated from those
                #  that are essential to the runtime calculation.
                # The following function is used to calculate the lines
                # (x, y) = (x0, y0) + t * (vx, vy), where t is the bounding values
                ns_x2 = (norm_vect_scale_factor * ns_vx) + ns_x1
                ns_y2 = (norm_vect_scale_factor * ns_vy) + ns_y1
                ew_x2 = (norm_vect_scale_factor * ew_vx) + ew_x1
                ew_y2 = (norm_vect_scale_factor * ew_vy) + ew_y1

                cv.line(frame_contours_bgr, (int(ns_x1), int(ns_y1)), (int(ns_x2), int(ns_y2)), (0, 255, 0), 3)
                cv.line(frame_contours_bgr, (int(ew_x1), int(ew_y1)), (int(ew_x2), int(ew_y2)), (0, 255, 0), 3)

                # TODO: Need to change this algo such that it calculates a line from points on both sides of the image. In
                #  cases where the camera is offset, it creates a distorted image, whereby referencing only one side (ex. only
                #  the Black/Yellow border when the camera is "low") the line becomes angled, which changes the intersection
                #  point and also creates a non-perpendicular relationship.
                # Calculate the intersection of the lines. Solves the denominator first. If zero, this is an edge case  where
                # the lines are parallel, and we shouldn't calculate the intersection point.
                denominator = ((ns_x1 - ns_x2) * (ew_y1 - ew_y2)) - ((ns_y1 - ns_y2) * (ew_x1 - ew_x2))
                if denominator != 0:
                    # Calculate some common elements that are shared between the following functions for speed
                    elem1 = (ns_x1 * ns_y2) - (ns_y1 * ns_x2)
                    elem2 = (ew_x1 * ew_y2) - (ew_y1 * ew_x2)
                    # Calculate the (x,y) intersection point between the two lines
                    intersect_x = ((elem1 * (ew_x1 - ew_x2)) - ((ns_x1 - ns_x2) * elem2)) / denominator
                    intersect_y = ((elem1 * (ew_y1 - ew_y2)) - ((ns_y1 - ns_y2) * elem2)) / denominator
                    # Display the intersection point as a circle on the image
                    cv.circle(frame_contours_bgr, (int(intersect_x), int(intersect_y)), 10, (0, 0, 255), -1)
                    # Display the intersection coordinates as text on the image
                    text_intersect = f"x: {int(intersect_x)}, y: {int(intersect_y)}"
                    text_contours = f"N-S: {len(contours_northSouth)}. E-W: {len(contours_eastWest)}"
                    cv.putText(frame_contours_bgr, text_intersect, (25, 25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv.putText(frame_contours_bgr, text_contours, (25, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Display the resulting image
            # display_cardinal_markers()
            cv.imshow("All Contours", frame_contours_bgr)
            cv.imshow("Raw Image", frame_raw)
            cv.imshow("Combined Mask", mask_all)
            # output.write(frame_contours_bgr)
            # cv.imshow("Merged Mask", mask_all)

            # Press "k" to quit
            if cv.waitKey(27) == ord('k'):
                break_flag = True
                break
        if break_flag:
            # capture.release()
            # output.release()
            cv.destroyAllWindows()
            break
