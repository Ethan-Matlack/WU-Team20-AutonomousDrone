#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import pymavlink
from pymavlink import mavutil
import time
import sys
import threading

# IMPORTANT: ENSURE THE RUN STATE FLAG IS SET CORRECTLY!!!
# Only 'drone', 'usb', and 'simulation' are valid options
run_state = 'drone'

# This flag disables the image output window(s). Set to false when code is deployed
show_displays = False

if run_state == 'simulation':
    from simulation_setup import *

# ----------------------------------------------------------------------------------------------------------------------
# CLASSES
# ----------------------------------------------------------------------------------------------------------------------


class Copter:

    def __init__(self):

        self.resolution_camera_x = None
        self.resolution_camera_y = None
        self.horizontal_fov = None
        self.vertical_fov = None
        self.resolution_processed_x = None
        self.resolution_processed_y = None

        self.master = None
        self.mode = None
        self.rangefinder_distance = None
        self.battery_voltage = None
        self.battery_remaining = None

    def connect(self, device: str = '/dev/serial0', baud: int = 921000) -> None:
        """
        Establishes the initial connection from the RPi to the Pixhawk.

        Args:
            device (str): device to talk over (default: '/dev/ttyAMA0')
            baud (int): baud rate (default: 57600)

        Returns:
            None
        """
        # Connect to the pixhawk using MAVLink protocol on the specified device
        # TODO: Verify that this is the correct device to talk over (perhaps /dev/serial0)
        self.master = mavutil.mavlink_connection(device, baud)
        self.master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
              (self.master.target_system, self.master.target_component))

    def setup_camera_details(self, camera: str):

        if camera == 'RaspberryPiV2':
            self.resolution_camera_x = 1920
            self.resolution_camera_y = 1080
            self.resolution_processed_x = self.resolution_camera_x / 3
            self.resolution_processed_y = self.resolution_camera_y / 3
            self.horizontal_fov = np.deg2rad(62.2)
            self.vertical_fov = np.deg2rad(48.8)
        elif camera == 'simulation':
            # These values might not be correct
            self.resolution_camera_x = 1920
            self.resolution_camera_y = 1080
            self.resolution_processed_x = 640
            self.resolution_processed_y = 480
            self.horizontal_fov = np.deg2rad(62.2)
            self.vertical_fov = np.deg2rad(48.8)
        else:
            print('You must use a predefined camera!')
            raise Exception

    def request_message_interval(self, message_id: int, frequency_hz: float):
        """
        Request MAVLink message in a desired frequency, documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # Command
            0,  # Confirmation
            message_id,  # Message ID:
            1e6 / frequency_hz,  # Frequency in Hz
            0, 0, 0, 0, 0  # Un-used
        )

    def get_sensors(self):
        """
        Get the sensor data.

        Returns:
            None
        """

        msg_heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=False)
        if msg_heartbeat:
            if msg_heartbeat.get_type() == "BAD_DATA" and mavutil.all_printable(msg_heartbeat.data):
                sys.stdout.write(msg_heartbeat.data)
                sys.stdout.flush()
            else:
                self.mode = mavutil.mode_string_v10(msg_heartbeat)
                print(self.mode)

        msg_sys_status = self.master.recv_match(type='SYS_STATUS', blocking=False)
        if msg_sys_status:
            if msg_sys_status.get_type() == "BAD_DATA" and mavutil.all_printable(msg_sys_status.data):
                sys.stdout.write(msg_sys_status.data)
                sys.stdout.flush()
            else:
                self.battery_voltage = msg_sys_status.voltage_battery / 1000
                self.battery_remaining = msg_sys_status.battery_remaining

        msg_distance_sensor = self.master.recv_match(type='DISTANCE_SENSOR', blocking=False)
        if msg_distance_sensor:
            if msg_distance_sensor.get_type() == "BAD_DATA" and mavutil.all_printable(msg_distance_sensor.data):
                sys.stdout.write(msg_distance_sensor.data)
                sys.stdout.flush()
            else:
                self.rangefinder_distance = msg_distance_sensor.current_distance / 100

        print(
            f"Battery voltage: {self.battery_voltage}, remaining: {self.battery_remaining}%. Distance: {self.rangefinder_distance}m")

    def send_heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)

    def send_landing_target(self, x_rad, y_rad, time_usec=0):
        self.master.mav.landing_target_send(
            time_usec,  # time_usec
            1,  # target_num
            mavutil.mavlink.MAV_FRAME_GLOBAL,  # frame; AP ignores
            x_rad,  # angle x (radians)
            y_rad,  # angle y (radians)
            self.rangefinder_distance,  # distance to target
            0, 0)

    def set_yaw(self, angle_degrees):
        """
        Set the copter's yaw position in degrees.

        Parameters:
            angle_degrees (float): The angle to set the copter's yaw to, in degrees.

        Returns:
            None
        """
        # Convert the angle from degrees to radians
        angle_radians = np.deg2rad(angle_degrees)

        # Create a MAV_CMD_CONDITION_YAW message
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            angle_radians,  # param 1: target angle
            0, 0, 0, 0, 0, 0)  # params 2-7 not used


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
        """
        Initializes an instance of the Base class with all attributes set to None except for the historical list variables.
        """
        # Initialize as undefined, then the helper functions can assign attributed as the program moves along.
        # Some of these (num_cont) will be assigned directly in code and not via helper funcs.

        self.offset_x = None
        self.offset_y = None
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
        self.w_rad_x = None
        self.w_rad_y = None

        self.num_cont_ns = None
        self.num_cont_ew = None

        # Historical list variables
        self.heading_list = []
        self.intersect_x_list = []
        self.intersect_y_list = []

    # Read in the best fit lines for the two contours
    def read_lines(self, ns_line=None, ew_line=None):
        """
        Reads in the best fit lines for the North-South and East-West contours and stores the point and vector components
        of each line as attributes of the Base instance.

        Args:
            ns_line (tuple): A tuple containing the vector and point components of the best fit line for the North-South contour.
            ew_line (tuple): A tuple containing the vector and point components of the best fit line for the East-West contour.

        Returns:
            None
        """
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
        """
        Calculates the intersection point for the two lines representing the base center.

        Returns:
            tuple: The (x,y) coordinates of the intersection point.
                   Returns None if the required values are not yet assigned.
        """
        try:
            # Calculate the intersection point for the two lines
            ns1 = (int(self.ns_px), int(self.ns_py))
            ns2 = (int(self.ns_px + (200 * self.ns_vx)), int(self.ns_py + (200 * self.ns_vy)))
            ew1 = (int(self.ew_px), int(self.ew_py))
            ew2 = (int(self.ew_px + (200 * self.ew_vx)), int(self.ew_py + (200 * self.ew_vy)))

        # Handles the case where the values are not yet assigned
        except AttributeError:
            return None

        self.intersection = line_intersection(ns1, ns2, ew1, ew2)

        return self.intersection

    # Calculate the angular heading correction between camera frame and the base frame
    def get_heading(self):
        """
        Calculates the angular heading correction between camera frame and the base frame.

        Returns:
            float: The angular heading correction in radians.
                   Returns None if the required values are not yet assigned.
        """
        try:
            angle_between = angle_between_lines(self.ns_vx, self.ns_vy, self.ew_vx, self.ew_vy)
            self.heading_accuracy = 1 - abs(angle_between - np.pi / 2) / (np.pi / 2)

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
        """
        Generates the weighted values for heading and intersection point based on the weighting function.

        Args:
            lookback_depth (int): The number of previous values to consider for the weighted average.

        Returns:
            None
        """
        # Write the current values to the un-weighted list
        self.heading_list.insert(0, self.heading[0])
        self.intersect_x_list.insert(0, self.intersection[0])
        self.intersect_y_list.insert(0, self.intersection[1])

        heading_list_length = len(self.heading_list)
        intersect_x_list_length = len(self.intersect_x_list)
        intersect_y_list_length = len(self.intersect_y_list)

        depth_heading = depth_x = depth_y = lookback_depth

        if heading_list_length < lookback_depth:
            depth_heading = heading_list_length
        if intersect_x_list_length < lookback_depth:
            depth_x = intersect_x_list_length
        if intersect_y_list_length < lookback_depth:
            depth_y = intersect_y_list_length

        # Use the un-weighted list to do a weighted average against the weights array
        self.w_heading = np.average(self.heading_list[:depth_heading], weights=weight_array[:depth_heading])
        self.w_intersect_x = np.average(self.intersect_x_list[:depth_x], weights=weight_array[:depth_x])
        self.w_intersect_y = np.average(self.intersect_y_list[:depth_y], weights=weight_array[:depth_y])

        self.offset_x = self.w_intersect_x - (copter.resolution_processed_x / 2)
        self.offset_y = self.w_intersect_y - (copter.resolution_processed_y / 2)
        print(
            f"Intersection: ({int(self.w_intersect_x)}, {int(self.w_intersect_y)})"
            f"... Offset: ({int(self.offset_x)}, {int(self.offset_y)})")
        self.w_rad_x = self.offset_x * copter.horizontal_fov / copter.resolution_processed_x
        self.w_rad_y = self.offset_y * copter.vertical_fov / copter.resolution_processed_y

    def clear_session(self):
        self.heading_list= []
        self.intersect_x_list = []
        self.intersect_y_list = []
        self.w_heading = None
        self.w_intersect_x = None
        self.w_intersect_y = None
        self.ns_line = None
        self.ew_line = None
        self.offset_x = None
        self.offset_y = None
        self.w_rad_x = None
        self.w_rad_y = None


class FreshestFrame(threading.Thread):
    def __init__(self, capture_frame, name='FreshestFrame'):
        self.capture = capture_frame
        assert self.capture.isOpened()

        # this lets the read() method block until there's a new frame
        self.cond = threading.Condition()

        # this allows us to stop the thread gracefully
        self.running = False

        # keeping the newest frame around
        self.frame = None

        # passing a sequence number allows read() to NOT block
        # if the currently available one is exactly the one you ask for
        self.latestnum = 0

        # this is just for demo purposes
        self.callback = None

        super().__init__(name=name)
        self.start()

    def start(self):
        self.running = True
        super().start()

    def release(self, timeout=None):
        self.running = False
        self.join(timeout=timeout)
        self.capture.release()

    def run(self):
        counter = 0
        while self.running:
            # block for fresh frame
            (rv, img) = self.capture.read()
            assert rv
            counter += 1

            # publish the frame
            with self.cond:  # lock the condition for this operation
                self.frame = img if rv else None
                self.latestnum = counter
                self.cond.notify_all()

            if self.callback:
                self.callback(img)

    def read(self, wait=True, seqnumber=None, timeout=None):
        # with no arguments (wait=True), it always blocks for a fresh frame
        # with wait=False it returns the current frame immediately (polling)
        # with a seqnumber, it blocks until that frame is available (or no wait at all)
        # with timeout argument, may return an earlier frame;
        #   may even be (0,None) if nothing received yet

        with self.cond:
            if wait:
                if seqnumber is None:
                    seqnumber = self.latestnum + 1
                if seqnumber < 1:
                    seqnumber = 1

                rv = self.cond.wait_for(lambda: self.latestnum >= seqnumber, timeout=timeout)
                if not rv:
                    return (self.latestnum, self.frame)

            return (self.latestnum, self.frame)


# ----------------------------------------------------------------------------------------------------------------------
# FUNCTIONS
# ----------------------------------------------------------------------------------------------------------------------


def contour_compare(contour_list_a, contour_list_b, search_radius):
    """
    Compare two lists of contours and return a list of common points found in both lists of contours, rounded to the
    nearest integer.

    Args:
        contour_list_a (list): A list of contours
        contour_list_b (list): A list of contours
        search_radius (int): The search radius in pixels for comparing the contours

    Returns:
        contour_list_common (list): A list of (x, y) points where the contours intersect within the search radius
    """
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
    """
    Calculates the angle in radians between two unit vectors.

    Args:
        uv1_x (float): x-component of the first unit vector.
        uv1_y (float): y-component of the first unit vector.
        uv2_x (float): x-component of the second unit vector.
        uv2_y (float): y-component of the second unit vector.

    Returns:
        angle_between (float): The angle in radians between the two unit vectors.
    """
    # Only accepts unit vectors for speed improvements.
    # Change the "divide by 2" portion of the angle_between calc to "MagV1*MagV2" to generalize.
    dot_product = uv1_x * uv2_x + uv1_y * uv2_y
    angle_between = np.arccos(dot_product / 2)
    return angle_between


def line_intersection(p1, p2, p3, p4):
    """
    Calculates the intersection point between two lines.

    Args:
        p1 (tuple): A tuple containing the (x, y) coordinates of the first point on line 1.
        p2 (tuple): A tuple containing the (x, y) coordinates of the second point on line 1.
        p3 (tuple): A tuple containing the (x, y) coordinates of the first point on line 2.
        p4 (tuple): A tuple containing the (x, y) coordinates of the second point on line 2.

    Returns:
        tuple or None: A tuple containing the (x, y) coordinates of the intersection point if it exists, None otherwise.
    """
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
    """
    Generates a numpy array with a length specified by the user. The first "flat_distance" elements of the array have a
    value of 1, and the remaining elements decrease linearly to a value of 0.

    Args:
        length: An integer specifying the length of the output numpy array.
        flat_distance: An integer specifying the number of elements at the beginning of the array with a value of 1.

    Returns:
        weights: A numpy array with a shape of (length,) and values that decrease linearly from 1 to 0 over its length.
    """
    flat_distance = int(flat_distance - 1)
    weights = np.empty(length)
    weights[:flat_distance] = 1
    weights[flat_distance:] = np.linspace(1, 0, length - flat_distance)
    print(weights)
    return weights


def generate_contours(color, gauss_blur_x, gauss_blur_y, canny_threshold1, canny_threshold2):
    """
    Takes a color object and generates a mask from the color's lower and upper bounds. Then, applies Gaussian blurring
    and Canny edge detection to the mask to detect the edges of the image. Finally, uses contour detection to find the
    contours of the edges in the image, sorting them by area with the largest contours first.

    Args:
        color: A color object with lower and upper bounds for HSV color space.
        gauss_blur_x: An integer specifying the x-axis Gaussian blur kernel size.
        gauss_blur_y: An integer specifying the y-axis Gaussian blur kernel size.
        canny_threshold1: An integer specifying the lower threshold for Canny edge detection.
        canny_threshold2: An integer specifying the upper threshold for Canny edge detection.

    Returns:
        None.
    """
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
    color.contours, color.hierarchy = cv.findContours(color.edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]

    # Sort the contours based on their area. Largest contours are first in the array.
    color.contours = sorted(color.contours, key=cv.contourArea, reverse=True)


def display_output():
    """
    Display output on the processed image frame.

    This function displays various lines and text on the processed image frame including the location of the
    base's center, the angle of the base, the number of contours in the north-south and east-west directions, and
    the current frames-per-second. It also draws circles and lines to represent the base and its direction.

    Returns:
        None
    """
    if (base.w_intersect_x is not None) and (base.w_intersect_y is not None):

        cv.circle(frame_contours_bgr,
                  (int(base.w_intersect_x), int(base.w_intersect_y)),
                  10, (0, 0, 255), -1)

        cv.line(frame_contours_bgr,
                (int(base.w_intersect_x), int(base.w_intersect_y)),
                (int(copter.resolution_processed_x / 2), int(copter.resolution_processed_y / 2)),
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
    # cv.imshow("Raw Image", frame_raw)
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
yellow = Color(np.array([20, 100, 100]), np.array([30, 255, 255]))
black = Color(np.array([0, 0, 3]), np.array([130, 225, 105]))

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

# Create base and copter objects
print("Creating base object")
base = Base()
print("Creating copter object")
copter = Copter()

if run_state == 'simulation':
    camera_type = 'simulation'
    camera_subscriber = CameraSubscriber()
elif run_state == 'usb' or run_state == 'drone':
    camera_type = 'RaspberryPiV2'
    capture = cv.VideoCapture(0, cv.CAP_DSHOW)
    capture.set(cv.CAP_PROP_FPS, 30)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, copter.resolution_processed_x)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, copter.resolution_processed_y)
    capture_fresh = FreshestFrame(capture)
    frame_count = 0

copter.setup_camera_details(camera_type)

# ----------------------------------------------------------------------------------------------------------------------
# MAIN BODY
# ----------------------------------------------------------------------------------------------------------------------

print("Attempting to connect...")
if run_state == 'simulated':
    copter.connect('udp:127.0.0.1:14550')
elif run_state == 'usb':
    copter.connect('COM4', 9600)
else:
    copter.connect()
print("===============================================================")

copter.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 0.1)
copter.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 15)

# Initialize params for a little FPS counter
start_time = time.time()
time_x = 1  # How often the frame rate is updated (in sec)
time_counter = 0

while True:

    copter.get_sensors()

    # Read in the camera frame by frame
    if run_state == 'simulated':
        frame_raw = camera_subscriber.get_image()  # For simulation
    elif run_state == 'usb' or run_state == 'drone':
        # _, frame_raw = capture.read()
        t0 = time.perf_counter()
        frame_count, frame_raw = capture_fresh.read(seqnumber=frame_count + 1)
        dt = time.perf_counter() - t0

    # Convert the frame out of BGR to HSV
    frame_hsv = cv.cvtColor(frame_raw, cv.COLOR_BGR2HSV)

    # Create all the contours
    generate_contours(cyan, 19, 19, 100, 200)
    generate_contours(magenta, 19, 19, 100, 200)
    generate_contours(yellow, 19, 19, 100, 200)
    generate_contours(black, 19, 19, 100, 200)

    # Create a blank white canvas of the same size as the HSV frame
    frame_contours_bgr = np.full_like(frame_hsv, 255)

    if cyan.contours:
        cv.drawContours(frame_contours_bgr, cyan.contours[0:5], -1, (205, 149, 0), 2)
    if magenta.contours:
        cv.drawContours(frame_contours_bgr, magenta.contours[0:5], -1, (78, 31, 162), 2)
    if yellow.contours:
        cv.drawContours(frame_contours_bgr, yellow.contours[0:7], -1, (24, 208, 255), 2)
    if black.contours:
        cv.drawContours(frame_contours_bgr, black.contours[0:10], -1, (43, 38, 34), 2)

    print(f"Copter mode: {copter.mode}")
    if copter.mode == 'LAND' or copter.mode == 'STABILIZE':

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

                # Turn the craft the face 0 degrees and keep it there
                copter.set_yaw(90)

                # Issue the precision landing command to the flight controller
                if copter.rangefinder_distance:
                    if copter.rangefinder_distance > 0.15:
                        copter.send_landing_target(base.w_rad_x, base.w_rad_y)
                        print(f"ISSUING LAND COMMAND: "
                              f"Angle_x (deg): {np.rad2deg(base.w_rad_x)}, "
                              f"Distance: {copter.rangefinder_distance}")

        if copter.mode != 'LAND':
            base.clear_session()

    if show_displays:
        display_output()
    copter.send_heartbeat()

    # Used to log the FPS. Calcs passed time each frame
    time_counter += 1
    if (time.time() - start_time) > time_x:
        fps = time_counter / (time.time() - start_time)
        time_counter = 0
        start_time = time.time()

    # Press "k" to quit
    if cv.waitKey(27) == ord('k'):
        cv.destroyAllWindows()
        if not run_state:
            # capture.release()
            capture_fresh.release()
        break
