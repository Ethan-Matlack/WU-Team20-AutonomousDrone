# ----------------------------------------------------------------------------------------------------------------------
# PRIMARY RUNTIME FILE
# ----------------------------------------------------------------------------------------------------------------------

# CREATED BY:
# Senior Project Team 20
# Widener University - School of Engineering
# Ethan Matlack, Brian Chung, Dimple Gandevia, Chase Crane, Nick Olsen

# DEPLOYMENT NOTES:
# TODO: Complete a little write-up about how this code should be field-deployed.

# ----------------------------------------------------------------------------------------------------------------------
# NOTES
# ----------------------------------------------------------------------------------------------------------------------

# Camera is positioned such that CAMERA_UP is DRONE_RIGHT.
# In other words, CAMERA_LEFT is DRONE_FORWARD.
# Any camera inputs should be rotated 90 degrees CCW into DRONE frame.
# The landing station UP is defined as the Magenta/Yellow edge.

# ----------------------------------------------------------------------------------------------------------------------
# IMPORTS
# ----------------------------------------------------------------------------------------------------------------------

import cv2 as cv
import numpy as np
from pymavlink import mavutil
import time

# ----------------------------------------------------------------------------------------------------------------------
# CLASSES
# ----------------------------------------------------------------------------------------------------------------------


class Color:
    def __init__(self, bound_lower, bound_upper, mask=None, blur=None, edges=None, contours=None, hierarchy=None):

        self.lower_bound = bound_lower
        self.upper_bound = bound_upper
        self.mask = mask
        self.blur = blur
        self.edges = edges
        self.contours = contours
        self.hierarchy = hierarchy


class Display:
    def __init__(self, x=None, y=None, num_cnt_ns=None, num_cnt_ew=None, fps_count=None, angle=None):

        self.x_intersect = x
        self.y_intersect = y
        self.number_contours_ns = num_cnt_ns
        self.number_contours_ew = num_cnt_ew
        self.fps = fps_count
        self.angle_to_base = angle


# ----------------------------------------------------------------------------------------------------------------------
# FUNCTIONS
# ----------------------------------------------------------------------------------------------------------------------


# def send_land_message(x, y):
#     msg = drone.message_factory.landing_target_encode(
#         0,       # time_boot_ms (not used)
#         0,       # target num
#         0,       # frame
#         (x - resolution_processed_x/2)*horizontal_fov/resolution_processed_x,
#         (y - resolution_processed_y/2)*vertical_fov/resolution_processed_y,
#         0,       # altitude.  Not supported.
#         0, 0)     # size of target in radians
#     drone.send_mavlink(msg)
#     drone.flush()


def contour_compare(contour_list_a, contour_list_b, search_radius):
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


def angle_between_lines(uv1_x, uv1_y, uv2_x, uv2_y):
    line_dot_product = uv1_x*uv2_x + uv1_y*uv2_y
    # line1 = np.array([uv1_x, uv1_y])
    # line2 = np.array([uv2_x, uv2_y])
    # line_dot_product = np.dot(line1, line2)
    # line_dot_product = line_dot_product[0][0] + line_dot_product[1][0]
    # line1_mag = np.linalg.norm(line1)
    # line2_mag = np.linalg.norm(line2)
    # angle_between = np.arccos(line_dot_product / (line1_mag + line2_mag))
    angle_between = np.arccos(line_dot_product / 2)
    return angle_between


def line_intersection(p1, p2, p3, p4):

    # This assumes that an intersection point actually exists. Must validate prior to calling!!

    # Line 1 dy, dx and determinant
    a11 = (p1[1] - p2[1])
    a12 = (p2[0] - p1[0])
    b1 = (p1[0] * p2[1] - p2[0] * p1[1])

    # Line 2 dy, dx and determinant
    a21 = (p3[1] - p4[1])
    a22 = (p4[0] - p3[0])
    b2 = (p3[0] * p4[1] - p4[0] * p3[1])

    # Construction of the linear system coefficient matrix
    coefficient = np.array([[a11, a12], [a21, a22]])

    # Right hand side vector
    b = -np.array([b1, b2])

    # Solve, assuming that an intersection point exists
    try:
        intersection_point = np.linalg.solve(coefficient, b)
        # print('Intersection point detected at:', intersection_point)

        if 0 < intersection_point[0] < frame_contours_bgr.shape[0] \
                and 0 < intersection_point[1] < frame_contours_bgr.shape[1]:
            return intersection_point
        else:
            return None

    except np.linalg.LinAlgError:
        # print('No single intersection point detected')
        return None


def get_base_heading(uv1_x, uv1_y, uv2_x, uv2_y):
    angle_between = angle_between_lines(uv1_x, uv1_y, uv2_x, uv2_y)

    if np.deg2rad(60) < angle_between < np.deg2rad(120):
        # Sums the unit vectors to create a new "average" vector that is halfway (angle-wise) between the two
        uv_x = uv1_x + uv2_x
        uv_y = uv1_y + uv2_y

        # Checks the angle between the average unit vector and the x-axis
        angle_2_x_axis = angle_between_lines(uv_x, uv_y, 0, 1)

        # Since the calculated angle is the average of the two, the x-oriented vector is -45deg shifted.
        heading_offset = angle_2_x_axis - np.deg2rad(45)

        # TODO: Correction into DRONE frame. Probably wrong.
        # heading_offset = heading_offset - np.deg2rad(90)

    else:
        heading_offset = None

    return heading_offset


def generate_weight_array(length, flat_distance):
    flat_distance = int(flat_distance)
    weights = np.empty(length)
    weights[:flat_distance] = 1
    weights[flat_distance+1:] = np.linspace(1, 0, num=length-(flat_distance+1))
    return weights


def generate_contours(color, gauss_blur_x, gauss_blur_y, canny_threshold1, canny_threshold2):
    # Create masks for each color
    if type(color.upper_bound) is tuple:
        mask1 = cv.inRange(frame_hsv, color.lower_bound[0], color.upper_bound[0])
        mask2 = cv.inRange(frame_hsv, color.lower_bound[1], color.upper_bound[1])
        color.mask = cv.bitwise_or(mask1, mask2)
    else:
        color.mask = cv.inRange(frame_hsv, color.lower_bound, color.upper_bound)

    # Blur each mask to aid contour detection
    color.mask = cv.GaussianBlur(color.mask, (gauss_blur_x, gauss_blur_y), 0)

    # Run Canny edge detection to find all the edges in the masks
    color.edges = cv.Canny(image=color.mask, threshold1=canny_threshold1, threshold2=canny_threshold2)

    # Find the external contours only using the chain approximation method to limit the points
    color.contours, color.hierarchy = cv.findContours(color.edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Sort the contours based on their area. Largest contours are first in the array.
    color.contours = sorted(color.contours, key=cv.contourArea, reverse=True)


def display_output():

    latest_disp.x_intersect = int(latest_disp.x_intersect)
    latest_disp.y_intersect = int(latest_disp.y_intersect)

    cv.circle(frame_contours_bgr,
              (latest_disp.x_intersect, latest_disp.y_intersect),
              10, (0, 0, 255), -1)

    cv.line(frame_contours_bgr,
            (latest_disp.x_intersect, latest_disp.y_intersect),
            (int(resolution_processed_x/2), int(resolution_processed_y/2)),
            (0, 0, 255), 3)

    # Draws a line from the BASE center along its NS direction
    cv.line(frame_contours_bgr,
            (latest_disp.x_intersect, latest_disp.y_intersect),
            (latest_disp.x_intersect + int(100*np.sin(latest_disp.angle_to_base)),
             latest_disp.y_intersect + int(100*np.cos(latest_disp.angle_to_base))),
            (0, 255, 0), 3)

    cv.putText(frame_contours_bgr,
               f"x: {latest_disp.x_intersect}, y: {latest_disp.y_intersect}",
               (25, 25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv.putText(frame_contours_bgr,
               f"N-S: {int(latest_disp.number_contours_ns)}. E-W: {int(latest_disp.number_contours_ew)}",
               (25, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv.putText(frame_contours_bgr,
               f"FPS: {int(fps)}",
               (25, 75), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv.putText(frame_contours_bgr,
               f"Angle: {int(np.rad2deg(latest_disp.angle_to_base))} deg",
               (25, 100), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv.imshow("All Contours", frame_contours_bgr)
    cv.imshow("Raw Image", frame_raw)


# ----------------------------------------------------------------------------------------------------------------------
# INITIALIZATION
# ----------------------------------------------------------------------------------------------------------------------

# Define the ranges of each color in HSV. Color space is [0-179, 0-255, 0-255]
# These are used to create class for each color, which will be modified later
cyan = Color(np.array([80, 100, 100]), np.array([130, 255, 255]))
magenta = Color((np.array([0, 100, 100]), np.array([160, 100, 100])),
                (np.array([10, 255, 255]), np.array([179, 255, 255])))
yellow = Color(np.array([20, 100, 100]), np.array([40, 255, 255]))
black = Color(np.array([0, 0, 3]), np.array([130, 225, 105]))

# Camera Details (Raspberry Pi Cam V2)
resolution_camera_x = 1920
resolution_camera_y = 1080
horizontal_fov = np.deg2rad(62.2)
vertical_fov = np.deg2rad(48.8)

# Processed resolutions (not raw cam)
resolution_processed_x = 720
resolution_processed_y = 480

# Variables for point, line, and intersect calculations
clustering_distance = 20
lookback = 25
dynamic_lookback_counter = 0

# Generate empty array to populate with historical intersections
intersects_x = []
intersects_y = []
headings = []

# Generate the array of weights to apply to the historical intersections
weight_array = generate_weight_array(lookback, 5)

# Init the FPS counter at 0
fps = 0

# ----------------------------------------------------------------------------------------------------------------------
# SETUP
# ----------------------------------------------------------------------------------------------------------------------

# Initializing the camera
capture = cv.VideoCapture(1)
print("Warming up camera...")
# time.sleep(3)

# Create an object to write displayable info to
latest_disp = Display(x=0, y=0, num_cnt_ns=0, num_cnt_ew=0, fps_count=0, angle=0)

# Connect to the drone using MAVLink protocol
# TODO: Verify that this is the correct device to talk over (perhaps /dev/serial0)
# drone = mavutil.mavlink_connection('/dev/ttyAMA0')

# Wait for the first heartbeat to set the system and component ID of remote system
# drone.wait_heartbeat()
# print("Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))

# ----------------------------------------------------------------------------------------------------------------------
# MAIN BODY
# ----------------------------------------------------------------------------------------------------------------------

# Initialize params for a little FPS counter
start_time = time.time()
time_x = 1  # How often the frame rate is updated (in sec)
time_counter = 0

while capture.isOpened():

    # relative_altitude = drone.messages['ALTITUDE'].altitude_relative

    # Read in the camera frame by frame
    _, frame_raw = capture.read()

    # Convert the frame out of BGR to HSV
    frame_hsv = cv.resize(cv.cvtColor(frame_raw, cv.COLOR_BGR2HSV),
                          (resolution_processed_x, resolution_processed_y))

    # Create all the contours
    generate_contours(cyan, 19, 19, 100, 200)
    generate_contours(magenta, 19, 19, 100, 200)
    generate_contours(yellow, 19, 19, 100, 200)
    generate_contours(black, 19, 19, 100, 200)

    # Create a blank white canvas of the same size as the HSV frame
    frame_contours_bgr = np.full_like(frame_hsv, 255)

    if cyan.contours:
        cv.drawContours(frame_contours_bgr, cyan.contours[0:5],    -1, (205, 149,   0), 2)
    if magenta.contours:
        cv.drawContours(frame_contours_bgr, magenta.contours[0:5], -1, (78,   31, 162), 2)
    if yellow.contours:
        cv.drawContours(frame_contours_bgr, yellow.contours[0:7],  -1, (24,  208, 255), 2)
    if black.contours:
        cv.drawContours(frame_contours_bgr, black.contours[0:10],   -1, (43,   38,  34), 2)

    # Create the new sets of contours that define the cardinal directions relative to the pad
    contours_northSouth = \
        contour_compare(cyan.contours, black.contours, clustering_distance) + \
        contour_compare(magenta.contours, yellow.contours, clustering_distance)
    contours_eastWest = \
        contour_compare(cyan.contours, magenta.contours, clustering_distance) + \
        contour_compare(black.contours, yellow.contours, clustering_distance)

    latest_disp.number_contours_ns = len(contours_northSouth)
    latest_disp.number_contours_ew = len(contours_eastWest)

    # If the contours exist, fit a line of best fit to each of them using the huber method
    if contours_northSouth and contours_eastWest:
        [ns_vx, ns_vy, ns_x1, ns_y1] = cv.fitLine(np.array(contours_northSouth), cv.DIST_HUBER, 0, 0.01, 0.01)
        [ew_vx, ew_vy, ew_x1, ew_y1] = cv.fitLine(np.array(contours_eastWest), cv.DIST_HUBER, 0, 0.01, 0.01)

        # Calculate the angular heading correction between camera frame and the base frame
        heading_to_base = get_base_heading(ns_vx, ns_vy, ew_vx, ew_vy)
        if heading_to_base is not None:

            # Calculate the intersection point for the two lines
            ns1 = [ns_x1[0], ns_y1[0]]
            ns2 = [ns_x1[0] + 250 * ns_vx[0], ns_y1[0] + 250 * ns_vy[0]]
            ew1 = [ew_x1[0], ew_y1[0]]
            ew2 = [ew_x1[0] + 250 * ew_vx[0], ew_y1[0] + 250 * ew_vy[0]]

            # TODO: This is temporary. Should be saved to the display object.
            # cv.line(frame_contours_bgr, (ns1[0], ns1[1]), (ns2[0], ns2[1]), (0, 255, 0), 3)
            # cv.line(frame_contours_bgr, (ew1[0], ew1[1]), (ew2[0], ew2[1]), (0, 255, 0), 3)

            intersection = line_intersection(ns1, ns2, ew1, ew2)
            if intersection is not None:

                if dynamic_lookback_counter < lookback:
                    dynamic_lookback_counter += 1
                lookback_dynamic = dynamic_lookback_counter

                headings.insert(0, heading_to_base[0])
                intersects_x.insert(0, intersection[0])
                intersects_y.insert(0, intersection[1])

                weighted_heading_to_base =\
                    np.average(headings[:lookback_dynamic], weights=weight_array[:lookback_dynamic])
                weighted_intersect_x =\
                    np.average(intersects_x[:lookback_dynamic], weights=weight_array[:lookback_dynamic])
                weighted_intersect_y =\
                    np.average(intersects_y[:lookback_dynamic], weights=weight_array[:lookback_dynamic])

                # Write the latest intersects to the display object
                latest_disp.angle_to_base = weighted_heading_to_base
                latest_disp.x_intersect = int(weighted_intersect_x)
                latest_disp.y_intersect = int(weighted_intersect_y)

                # Issue the precision landing command to the flight controller
                # send_land_message(weighted_intersect_x, weighted_intersect_y)

    display_output()

    # Used to log the FPS. Calcs passed time each frame
    time_counter += 1
    if (time.time() - start_time) > time_x:
        fps = time_counter / (time.time() - start_time)
        latest_disp.fps = fps
        time_counter = 0
        start_time = time.time()

    # Press "k" to quit
    if cv.waitKey(27) == ord('k'):
        capture.release()
        # output.release()
        cv.destroyAllWindows()
        break

    # # Break the loop if the drone has landed
    # if drone.motors_disarmed:
    #     capture.release()
    #     break
