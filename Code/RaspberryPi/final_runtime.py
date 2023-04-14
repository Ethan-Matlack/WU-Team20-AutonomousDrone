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

# OpenCV
import cv2 as cv
import numpy as np

# Dronekit & Mavlink
from pymavlink import mavutil
from pymavlink.CSVReader import CSVReader
from pymavlink.DFReader import DFReader_binary, DFReader_text
from pymavlink.mavutil import mavtcp, mavtcpin, mavudp, mavmcast, mavchildexec, mavmmaplog, mavlogfile, mavserial, mavlink

# Helper Libraries
# import multiprocessing
# import threading

# Python
import time
# import argparse
# from typing import Optional

# ----------------------------------------------------------------------------------------------------------------------
# CLASSES
# ----------------------------------------------------------------------------------------------------------------------


class Copter:

    # Type hints for the serial object so that the IDE knows what to expect and I get working auto-complete :)
    serial: mavtcp | mavtcpin | mavudp | mavmcast | DFReader_binary | CSVReader | DFReader_text | mavchildexec | \
            mavmmaplog | mavlogfile | mavserial | mavlink

    def __init__(self):
        self.rangefinder_distance = None
        self.battery_voltage = None
        self.battery_remaining = None

    # Establishes the initial connection from the RPi to the Pixhawk. Takes in a specified comm device to talk over
    def connect(self, device: str = '/dev/ttyAMA0', baud: int = 57600) -> None:

        # Connect to the pixhawk using MAVLink protocol on the specified device
        # TODO: Verify that this is the correct device to talk over (perhaps /dev/serial0)
        self.serial = mavutil.mavlink_connection(device, baud)
        self.serial.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
              (self.serial.target_system, self.serial.target_component))

    def request_message_interval(self, message_id: int, frequency_hz: float):
        """
        Request MAVLink message in a desired frequency, documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self.serial.mav.command_long_send(
            self.serial.target_system,
            self.serial.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # Command
            0,  # Confirmation
            message_id,  # Message ID:
            1e6 / frequency_hz,  # Frequency in Hz
            0, 0, 0, 0, 0  # Un-used
        )

    def get_sensors(self):

        try:
            msg_sys_status = copter.serial.recv_match(type='SYS_STATUS', blocking=False)
            self.battery_voltage = msg_sys_status.voltage_battery / 1000
            self.battery_remaining = msg_sys_status.battery_remaining
        except:
            pass

        try:
            msg_distance_sensor = copter.serial.recv_match(type='DISTANCE_SENSOR', blocking=False)
            self.rangefinder_distance = msg_distance_sensor.current_distance / 100
        except:
            pass

        print(
            f"Battery voltage: {self.battery_voltage}, remaining: {self.battery_remaining}%. Distance: {self.rangefinder_distance}")

    def send_heartbeat(self):
        self.serial.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                       mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def send_land_local(self, x, y):
        self.serial.mav.command_long_send(
            self.serial.target_system,
            self.serial.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL,
            0,
            1, 0, 0, 0, 0, 0, 0)

    def send_landing_target(self, x_rad, y_rad, time_usec=0):
        msg = self.serial.message_factory.landing_target_encode(
            time_usec,  # Time target data was processed
            0,  # Target number
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # Frame
            x_rad,  # X-axis angular offset, in radians
            y_rad,  # Y-axis angular offset, in radians
            self.rangefinder_distance,  # distance, in meters
            0,  # Target x-axis size, in radians
            0,  # Target y-axis size, in radians
            0,  # x    float    X Position of the landing target on MAV_FRAME
            0,  # y    float    Y Position of the landing target on MAV_FRAME
            0,  # z    float    Z Position of the landing target on MAV_FRAME
            (1, 0, 0, 0),  # q    float[4]    Quaternion of landing target orientation
            # (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            2,  # Type of landing target: 3 = Other marker
            0,  # Position_valid boolean
        )
        self.serial.send_mavlink(msg)
        self.serial.flush()

    def nav_land(self):
        self.serial.mav.command_long_send(
            self.serial.target_system,
            self.serial.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0, 0
        )


class Color:
    def __init__(self, bound_lower, bound_upper, mask=None, blur=None, edges=None, contours=None, hierarchy=None):

        self.lower_bound = bound_lower
        self.upper_bound = bound_upper
        self.mask = mask
        self.blur = blur
        self.edges = edges
        self.contours = contours
        self.hierarchy = hierarchy


class Base:
    """
    The base class stores all the information that describes the positioning and orientation of the base in the camera
    frame. It takes in two sets of contours that describe the North-South and East-West boundaries between the edges of
    the landing base. Ultimately, its output is the position and heading of the base relative to the camera frame. This
    class contains four member functions that read in the line contours, retrieve the intersection of the base, retrieve
    the heading of the base, and generate the weighted values necessary for increasing accuracy over time.
    """
    def __init__(self):

        # Initialize as undefined, then the helper functions can assign attributed as the program moves along.
        # Some of these (num_cont) will be assigned directly in code and not via helper funcs.

        self.ew_line = None
        self.ns_line = None

        self.ns_vx = None
        self.ns_vy = None
        self.ns_px = None
        self.ns_py = None

        self.ew_vx = None
        self.ew_vy = None
        self.ew_px = None
        self.ew_py = None

        self.heading = None
        self.heading_accuracy = None

        self.intersection = None

        self.w_heading = None
        self.w_intersect_x = None
        self.w_intersect_y = None

        self.num_cont_ns = None
        self.num_cont_ew = None

        # Historical list variables
        self.heading_list = []
        self.intersect_x_list = []
        self.intersect_y_list = []

    # Read in the best fit lines for the two contours
    def read_lines(self, ns_line=None, ew_line=None):
        # Bring in the point,vector lines
        self.ns_line = ns_line
        self.ew_line = ew_line

        # We can only try to read these in once they get defined
        # if (self.ns_line is not None) and (self.ew_line is not None):
        # Brought in via ns_line
        self.ns_vx = ns_line[0]
        self.ns_vy = ns_line[1]
        self.ns_px = ns_line[2]
        self.ns_py = ns_line[3]

        # Brought in via ew_line
        self.ew_vx = ew_line[0]
        self.ew_vy = ew_line[1]
        self.ew_px = ew_line[2]
        self.ew_py = ew_line[3]

    # Calculate the (x,y) intersect of the lines representing the base center
    def get_intersection(self):
        print("get_intersection is called")

        try:
            # Calculate the intersection point for the two lines
            ns1 = [int(self.ns_px), int(self.ns_py)]
            ns2 = [int(self.ns_px + (200 * self.ns_vx)), int(self.ns_py + (200 * self.ns_vy))]
            ew1 = [int(self.ew_px), int(self.ew_py)]
            ew2 = [int(self.ew_px + (200 * self.ew_vx)), int(self.ew_py + (200 * self.ew_vy))]

        # Handles the case where the values are not yet assigned
        except AttributeError:
            return None

        self.intersection = line_intersection(ns1, ns2, ew1, ew2)
        print(f"intersection: {self.intersection}")

        return self.intersection

    # Calculate the angular heading correction between camera frame and the base frame
    def get_heading(self):
        print("get_heading is called")
        try:
            angle_between = angle_between_lines(self.ns_vx, self.ns_vy, self.ew_vx, self.ew_vy)
            self.heading_accuracy = 1 - abs(angle_between - np.pi/2)/(np.pi/2)

        # Handles the case where the values are not yet assigned
        except AttributeError:
            return None

        print(f"angle_between: {angle_between}, accuracy: {self.heading_accuracy}")
        if 0.65 < self.heading_accuracy:
            # Sums the unit vectors to create a new "average" vector that is halfway (angle-wise) between the two
            uv_x = self.ns_vx + self.ew_vx
            uv_y = self.ns_vy + self.ew_vy

            # Checks the angle between the average unit vector and the x-axis
            # This is done to follow unit circle convention
            angle_2_x_axis = angle_between_lines(uv_x, uv_y, 1, 0)

            # Since the calculated angle is the average of the two, the x-oriented vector is -45deg shifted.
            self.heading = angle_2_x_axis - np.deg2rad(45)

            # TODO: Correction into DRONE frame. Probably wrong.
            # heading = heading_offset - np.deg2rad(90)

        else:
            self.heading = None

        return self.heading

    # Generate the weighted values for heading and intersect based on the weighting function
    def gen_weighted_values(self, lookback_depth):

        # Write the current values to the un-weighted list
        self.heading_list.insert(0, self.heading[0])
        self.intersect_x_list.insert(0, self.intersection[0])
        self.intersect_y_list.insert(0, self.intersection[1])

        # Use the un-weighted list to do a weighted average against the weights array
        self.w_heading = \
            np.average(self.heading_list[:lookback_depth], weights=weight_array[:lookback_depth])
        self.w_intersect_x = \
            np.average(self.intersect_x_list[:lookback_depth], weights=weight_array[:lookback_depth])
        self.w_intersect_y = \
            np.average(self.intersect_y_list[:lookback_depth], weights=weight_array[:lookback_depth])

        self.w_intersect_x

# ----------------------------------------------------------------------------------------------------------------------
# FUNCTIONS
# ----------------------------------------------------------------------------------------------------------------------


def contour_compare(contour_list_a, contour_list_b, search_radius):
    contour_list_common = []
    for contour_a in contour_list_a:
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
    # Only accepts unit vectors for speed improvements.
    # Change the "divide by 2" portion of the angle_between calc to "MagV1*MagV2" to generalize.
    dot_product = uv1_x*uv2_x + uv1_y*uv2_y
    angle_between = np.arccos(dot_product / 2)
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

    if (base.w_intersect_x is not None) and (base.w_intersect_y is not None):

        cv.circle(frame_contours_bgr,
                  (int(base.w_intersect_x), int(base.w_intersect_y)),
                  10, (0, 0, 255), -1)

        cv.line(frame_contours_bgr,
                (int(base.w_intersect_x), int(base.w_intersect_y)),
                (int(resolution_processed_x/2), int(resolution_processed_y/2)),
                (0, 0, 255), 3)

        if (base.ns_line is not None) and (base.ew_line is not None):

            # Draws a line from the BASE center along its NS direction
            cv.line(frame_contours_bgr,
                    (int(base.w_intersect_x), int(base.w_intersect_y)),
                    (int(base.ns_px), int(base.ns_py)), (0, 255, 0), 3)

            # Draws a line from the BASE center along its EW direction
            cv.line(frame_contours_bgr,
                    (int(base.w_intersect_x), int(base.w_intersect_y)),
                    (int(base.ew_px), int(base.ew_py)), (0, 255, 0), 3)

        cv.putText(frame_contours_bgr,
                   f"x: {int(base.w_intersect_x)}, y: {int(base.w_intersect_y)}",
                   (25, 25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if base.w_heading is not None:
            cv.putText(frame_contours_bgr,
                       f"Angle: {int(np.rad2deg(base.heading))} deg",
                       (25, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if (base.num_cont_ns is not None) and (base.num_cont_ew is not None):

            cv.putText(frame_contours_bgr,
                       f"N-S: {int(base.num_cont_ns)}. E-W: {int(base.num_cont_ew)}",
                       (25, 75), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if fps is not None:
            cv.putText(frame_contours_bgr,
                       f"FPS: {int(fps)}",
                       (25, 100), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv.imshow("All Contours", frame_contours_bgr)
    cv.imshow("Raw Image", frame_raw)
    # print(f"Base Intercept x: {base.w_intersect_x}")
    # print(f"Base Intercept y: {base.w_intersect_y}")
    # print(f"NS_VX: {base.ns_vx}, NS_VY: {base.ns_vy}")
    print('...')


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
lookback_counter = 0

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
time.sleep(3)

# Create base and copter objects
base = Base()
copter = Copter()

# ----------------------------------------------------------------------------------------------------------------------
# MAIN BODY
# ----------------------------------------------------------------------------------------------------------------------

copter.connect('COM4')
copter.request_message_interval(mavutil.mavlink.BATTERY_STATUS, 5)
copter.request_message_interval(mavutil.mavlink.DISTANCE_SENSOR, 10)

# Initialize params for a little FPS counter
start_time = time.time()
time_x = 1  # How often the frame rate is updated (in sec)
time_counter = 0

while capture.isOpened():

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

    base.num_cont_ns = len(contours_northSouth)
    base.num_cont_ew = len(contours_eastWest)

    # If the contours exist, fit a line of best fit to each of them using the huber method
    if contours_northSouth and contours_eastWest:

        base.read_lines(cv.fitLine(np.array(contours_northSouth), cv.DIST_HUBER, 0, 0.01, 0.01),
                        cv.fitLine(np.array(contours_eastWest), cv.DIST_HUBER, 0, 0.01, 0.01))

        if (base.get_intersection() is not None) and (base.get_heading() is not None):

            # Until we've reached the desired lookback depth, continue incrementing.
            # This ensures that we're not indexing into empty portions of the arrays.
            if lookback_counter < lookback:
                lookback_counter += 1
            lookback_dynamic = lookback_counter

            # Create the weighted heading and intersect
            base.gen_weighted_values(lookback_dynamic)

            # Issue the precision landing command to the flight controller
            # TODO: Need to transform this into angle_x and angle_y
            copter.send_landing_target(x_rad, y_rad)
            print(f"Issuing land command:"
                  f"Angle_x: {base.w_intersect_x},"
                  f"Angle_y: {base.w_intersect_y},"
                  f"Distance: {copter.rangefinder_distance}")

    # display_output()

    copter.send_heartbeat()

    # Used to log the FPS. Calcs passed time each frame
    time_counter += 1
    if (time.time() - start_time) > time_x:
        fps = time_counter / (time.time() - start_time)
        time_counter = 0
        start_time = time.time()

    # Press "k" to quit
    if cv.waitKey(27) == ord('k'):
        capture.release()
        # output.release()
        cv.destroyAllWindows()
        break
