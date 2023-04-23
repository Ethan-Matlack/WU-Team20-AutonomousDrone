#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from pymavlink import mavutil
import time
import sys
import threading
import asyncio

# -------------------------------------------------------------
# IMPORTANT: ENSURE THE RUN STATE FLAG IS SET CORRECTLY!!!
# Only 'drone', 'usb', 'simulation', and 'cam-only', are valid options
run_state = 'cam-only'
# -------------------------------------------------------------

# This flag disables the image output window(s). Set to false when code is deployed to boost performance
show_displays = True

if run_state == 'simulation':
    from simulation_setup import *

# ----------------------------------------------------------------------------------------------------------------------
# CLASSES
# ----------------------------------------------------------------------------------------------------------------------


class Copter:

    def __init__(self):

        self.master = None
        self.mode = None
        self.distance_sensor = None
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
        self.master = mavutil.mavlink_connection(device, baud)
        print('Connecting to primary flight computer...')
        time.sleep(1)
        self.master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
              (self.master.target_system, self.master.target_component))

    # SENSORS ---------------------------------------------------------------------------------------------------------

    def request_message_interval(self, message_id: int, frequency_hz: int):
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

    def request_datastream_all(self, frequency_hz: int):
        self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component,
                                                 mavutil.mavlink.MAV_DATA_STREAM_ALL, frequency_hz, 1)

    async def get_msg_heartbeat(self):
        while True:
            # Extract the MODE from the heartbeat
            msg_heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True)
            if msg_heartbeat:
                if msg_heartbeat.get_type() == "BAD_DATA" and mavutil.all_printable(msg_heartbeat.data):
                    sys.stdout.write(msg_heartbeat.data)
                    sys.stdout.flush()
                else:
                    self.mode = mavutil.mode_string_v10(msg_heartbeat)
                    # asyncio.create_task(msg_ready_heartbeat(self.mode))
                    print(self.mode)
            await asyncio.sleep(0.1)

    async def get_msg_sys_status(self):
        while True:
            # Extract the battery voltage and percentage remaining from system status
            msg_sys_status = self.master.recv_match(type='SYS_STATUS', blocking=True)
            if msg_sys_status:
                if msg_sys_status.get_type() == "BAD_DATA" and mavutil.all_printable(msg_sys_status.data):
                    sys.stdout.write(msg_sys_status.data)
                    sys.stdout.flush()
                else:
                    self.battery_voltage = msg_sys_status.voltage_battery / 1000
                    self.battery_remaining = msg_sys_status.battery_remaining
                    # asyncio.create_task(msg_ready_battery_voltage(self.battery_voltage))
                    # asyncio.create_task(msg_ready_battery_remaining(self.battery_remaining))
            await asyncio.sleep(0.1)

    async def get_msg_distance_sensor(self):
        while True:
            # Read the rangefinder distance directly
            msg_distance_sensor = self.master.recv_match(type='DISTANCE_SENSOR', blocking=True)
            if msg_distance_sensor:
                if msg_distance_sensor.get_type() == "BAD_DATA" and mavutil.all_printable(msg_distance_sensor.data):
                    sys.stdout.write(msg_distance_sensor.data)
                    sys.stdout.flush()
                else:
                    self.distance_sensor = msg_distance_sensor.current_distance / 100
                    # asyncio.create_task(msg_ready_disance_sensor(self.distance_sensor))
            await asyncio.sleep(0.1)

    # COMMUNICATION ---------------------------------------------------------------------------------------------------

    def send_heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)

    # CONTROL ---------------------------------------------------------------------------------------------------------

    def send_landing_target(self, x_rad, y_rad, time_usec=0):
        self.master.mav.landing_target_send(
            time_usec,  # time_usec
            1,  # target_num
            mavutil.mavlink.MAV_FRAME_GLOBAL,  # frame; AP ignores
            x_rad,  # angle x (radians)
            y_rad,  # angle y (radians)
            self.distance_sensor,  # distance to target
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


class Camera:

    def __init__(self, dimensions_pxl, scale_factor, fov_degrees):

        self.dimensions_px = (int(dimensions_pxl[0]*scale_factor), int(dimensions_pxl[1]*scale_factor))
        self.midpoint_px = (int(self.dimensions_px[0]/2), int(self.dimensions_px[1]/2))
        self.fov_degrees = (np.deg2rad(fov_degrees[0]), np.deg2rad(fov_degrees[1]))
        self.fps = 0
        self.start_time = None
        self.frame_count = None

    async def read(self):
        raise NotImplementedError()

    def start_fps_monitor(self):
        self.fps = 0
        self.start_time = time.time()

    def stop_fps_monitor(self):
        elapsed_time = time.time() - self.start_time
        self.fps = int(self.frame_count / elapsed_time)


class PiCamera(Camera):

    def __init__(self, dimensions_pxl, scale_factor, fov_degrees, device_number):
        super().__init__(dimensions_pxl, scale_factor, fov_degrees)
        self.capture = cv.VideoCapture(device_number)
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, dimensions_pxl[0])
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, dimensions_pxl[1])
        self.frame_count = 0
        self.loop = asyncio.get_running_loop()

    async def read(self):
        self.frame_count += 1
        _, frame = await self.loop.run_in_executor(None, self.capture.read)
        return frame


class SimCamera(Camera):

    def __init__(self, dimensions_pxl, scale_factor, fov_degrees, ros_stream):
        super().__init__(dimensions_pxl, scale_factor, fov_degrees)
        self.capture = ros_stream
        self.frame_count = 0
        self.loop = asyncio.get_running_loop()

    async def read(self):
        self.frame_count += 1
        # TODO: This might only be passing a frame...
        _, frame = await self.loop.run_in_executor(None, self.capture)
        return frame


class LocalCamera(Camera):

    def __init__(self, dimensions_pxl=(1920, 1080), scale_factor=1, fov_degrees=(60, 50), device_number=0):
        super().__init__(dimensions_pxl, scale_factor, fov_degrees)
        self.capture = cv.VideoCapture(device_number)
        # self.capture.set(cv.CAP_PROP_FRAME_WIDTH, dimensions_pxl[0])
        # self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, dimensions_pxl[1])
        self.frame_count = 0
        self.loop = asyncio.get_event_loop()

    async def read(self):
        self.frame_count += 1
        _, frame = await self.loop.run_in_executor(None, self.capture.read())
        return frame


class Color:
    def __init__(self, bound_lower, bound_upper):
        self.bound_lower = bound_lower
        self.bound_upper = bound_upper
        self.mask = None
        self.edges = None
        self.contours = None
        self.hierarchy = None


class Image:

    def __init__(self):

        self.mask = None
        self.blur = None
        self.edges = None
        self.contours = None
        self.hierarchy = None
        self.frame_raw = []
        self.frame_hsv = []
        self.frame_contours_bgr = []

    def convert_raw2hsv(self):

        # Convert the frame out of BGR to HSV
        self.frame_hsv = cv.cvtColor(self.frame_raw, cv.COLOR_BGR2HSV)

    async def gen_contours(self, mask_color, gauss_blur: tuple, thresh1, thresh2, partial, line_color, line_thick):
        """
        Takes a color object and generates a mask from the color's lower and upper bounds. Then, applies Gaussian blurring
        and Canny edge detection to the mask to detect the edges of the image. Finally, uses contour detection to find the
        contours of the edges in the image, sorting them by area with the largest contours first.

        Args:
            mask_color: A color object with lower and upper bounds for HSV color space.
            gauss_blur: An tuple (x, y) specifying the Gaussian blur kernel size.
            thresh1: An integer specifying the lower threshold for Canny edge detection.
            thresh2: An integer specifying the upper threshold for Canny edge detection.
            partial:
            line_color:
            line_thick:

        Returns:
            None.
        """

        # Create masks for each color
        if mask_color == 'magenta':
            mask1 = cv.inRange(self.frame_hsv, mask_color.lower_bound[0], mask_color.upper_bound[0])
            mask2 = cv.inRange(self.frame_hsv, mask_color.lower_bound[1], mask_color.upper_bound[1])
            mask_color.mask = cv.bitwise_or(mask1, mask2)
        else:
            mask_color.mask = cv.inRange(self.frame_hsv, mask_color.lower_bound, mask_color.upper_bound)

        # Blur each mask to aid contour detection
        mask_color.mask =\
            cv.GaussianBlur(mask_color.mask, gauss_blur, 0)

        # Run Canny edge detection to find all the edges in the masks
        mask_color.edges =\
            cv.Canny(image=mask_color.mask, threshold1=thresh1, threshold2=thresh2)

        # Find the external contours only using the chain approximation method to limit the points
        mask_color.contours, mask_color.hierarchy =\
            cv.findContours(mask_color.edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]

        # Sort the contours based on their area. Largest contours are first in the array.
        mask_color.contours =\
            sorted(mask_color.contours, key=cv.contourArea, reverse=True)

        if mask_color.contours and show_displays:
            # Create a blank white canvas of the same size as the HSV frame
            self.frame_contours_bgr = np.full_like(self.frame_hsv, 255)
            cv.drawContours(self.frame_contours_bgr, mask_color.contours[partial[0]:partial[1]],
                            -1, line_color, line_thick)


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
        Initializes an instance of the Base class with all attributes set to None except for the historical list
        variables.
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
        Reads in the best fit lines for the North-South and East-West contours and stores the point and vector
        components of each line as attributes of the Base instance.

        Args:
            ns_line (tuple): The vector and point components of the best fit line for the North-South contour.
            ew_line (tuple): The vector and point components of the best fit line for the East-West contour.

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

        self.offset_x = self.w_intersect_x - (camera.dimensions_px[0] / 2)
        self.offset_y = self.w_intersect_y - (camera.dimensions_px[1] / 2)
        print(
            f"Intersection: ({int(self.w_intersect_x)}, {int(self.w_intersect_y)})"
            f"... Offset: ({int(self.offset_x)}, {int(self.offset_y)})")
        self.w_rad_x = self.offset_x * camera.fov_degrees[0] / camera.dimensions_px[0]
        self.w_rad_y = self.offset_y * camera.fov_degrees[1] / camera.dimensions_px[1]

    def clear_session(self):
        self.heading_list = []
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


# ----------------------------------------------------------------------------------------------------------------------
# FUNCTIONS
# ----------------------------------------------------------------------------------------------------------------------


async def contour_compare(contour_list_a, contour_list_b, common_list, search_radius):
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

        if 0 < intersection_point[0] < camera.dimensions_px[0] \
                and 0 < intersection_point[1] < camera.dimensions_px[1]:
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

        cv.circle(image.frame_contours_bgr,
                  (int(base.w_intersect_x), int(base.w_intersect_y)),
                  10, (0, 0, 255), -1)

        cv.line(image.frame_contours_bgr,
                (int(base.w_intersect_x), int(base.w_intersect_y)),
                (camera.midpoint_px[0], camera.midpoint_px[1]),
                (0, 0, 255), 3)

        if (base.ns_line is not None) and (base.ew_line is not None):
            # Draws a line from the BASE center along its NS direction
            cv.line(image.frame_contours_bgr,
                    (int(base.w_intersect_x), int(base.w_intersect_y)),
                    (int(base.ns_px), int(base.ns_py)), (0, 255, 0), 3)

            # Draws a line from the BASE center along its EW direction
            cv.line(image.frame_contours_bgr,
                    (int(base.w_intersect_x), int(base.w_intersect_y)),
                    (int(base.ew_px), int(base.ew_py)), (0, 255, 0), 3)

        cv.putText(image.frame_contours_bgr,
                   f"x: {int(base.w_intersect_x)}, y: {int(base.w_intersect_y)}",
                   (25, 25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if base.w_heading is not None:
            cv.putText(image.frame_contours_bgr,
                       f"Angle: {int(np.rad2deg(base.heading))} deg",
                       (25, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if (base.num_cont_ns is not None) and (base.num_cont_ew is not None):
            cv.putText(image.frame_contours_bgr,
                       f"N-S: {int(base.num_cont_ns)}. E-W: {int(base.num_cont_ew)}",
                       (25, 75), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # if fps is not None:
        #     cv.putText(image.frame_contours_bgr,
        #                f"FPS: {int(fps)}",
        #                (25, 100), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv.imshow("All Contours", image.frame_contours_bgr)
    # cv.imshow("Raw Image", image.frame_raw)
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
global lookback_counter

# Generate the array of weights to apply to the historical intersections
weight_array = generate_weight_array(lookback, 5)

# ----------------------------------------------------------------------------------------------------------------------
# SETUP
# ----------------------------------------------------------------------------------------------------------------------

# Create base object for storing localization data
print("Creating BASE object")
base = Base()

# Create copter objects for communications/sensors
print("Creating COPTER object")
copter = Copter()

# Create image object for storing vision frames
print("Creating IMAGE object")
image = Image()

# Creating camera object conditionally
print("Creating CAMERA object")

if run_state == 'simulation':
    ros_camera = ROSCameraSubscriber()
    camera = SimCamera(dimensions_pxl=(640, 480), scale_factor=1, fov_degrees=(62.2, 48.8), ros_stream=ros_camera)
elif run_state == 'usb' or run_state == 'drone':
    camera = PiCamera(dimensions_pxl=(1920, 1080), scale_factor=(1/3), fov_degrees=(62.2, 48.8), device_number=0)
elif run_state == 'cam-only':
    camera = LocalCamera()
else:
    print('You must use a predefined camera!')
    raise Exception

# ----------------------------------------------------------------------------------------------------------------------
# MAIN BODY
# ----------------------------------------------------------------------------------------------------------------------

print("===============================================================")
print("Attempting to connect...")

if run_state == 'cam-only':
    print("Running without a FC!")
elif run_state == 'simulation':
    copter.connect('udp:127.0.0.1:14550')
elif run_state == 'usb':
    copter.connect('COM4', 115200)
else:
    copter.connect('/dev/AMA0')

if run_state != 'cam-only':
    # copter.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 0.1)
    # copter.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 15)
    copter.request_datastream_all(10)
print("===============================================================")


async def coroutine_image_preprocess():
    while True:
        # Read a new camera frame asynchronously
        image.frame_raw = await camera.read()
        image.convert_raw2hsv()

        # Create all the contours
        await asyncio.gather(
            image.gen_contours(cyan, (19, 19), 100, 200, partial=(0, 5), line_color=(205, 149, 0), line_thick=2),
            image.gen_contours(magenta, (19, 19), 100, 200, partial=(0, 5), line_color=(78, 31, 162), line_thick=2),
            image.gen_contours(yellow, (19, 19), 100, 200, partial=(0, 7), line_color=(24, 208, 255), line_thick=2),
            image.gen_contours(black, (19, 19), 100, 200, partial=(0, 10), line_color=(43, 38, 34), line_thick=2)
        )
        await asyncio.sleep(0.1)


async def coroutine_landing_algorithm():

    buffer1 = []
    buffer2 = []
    buffer3 = []
    buffer4 = []
    # Create the new sets of contours that define the cardinal directions relative to the pad
    await asyncio.gather(
        contour_compare(cyan.contours, black.contours, buffer1, clustering_distance),
        contour_compare(magenta.contours, yellow.contours, buffer2, clustering_distance),
        contour_compare(cyan.contours, magenta.contours, buffer3, clustering_distance),
        contour_compare(black.contours, yellow.contours, buffer4, clustering_distance)
    )
    contours_northSouth = buffer1 + buffer2
    contours_eastWest = buffer3 + buffer4

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
            copter.set_yaw(0)

            # Issue the precision landing command to the flight controller
            if copter.distance_sensor:
                if copter.distance_sensor > 0.15:
                    copter.send_landing_target(base.w_rad_x, base.w_rad_y)
                    print(f"ISSUING LAND COMMAND: "
                          f"Angle_x (deg): {np.rad2deg(base.w_rad_x)}, "
                          f"Distance: {copter.distance_sensor}")


async def main_loop():

    camera.start_fps_monitor()

    await coroutine_image_preprocess()

    if copter.mode == 'LAND':
        await copter.get_msg_distance_sensor()
        await coroutine_landing_algorithm()

    print(f"Battery voltage: {copter.battery_voltage}, remaining: {copter.battery_remaining}%."
          f"Distance: {copter.distance_sensor}m, Copter mode: {copter.mode}")

    if show_displays:
        display_output()
    copter.send_heartbeat()

    camera.stop_fps_monitor()
    print(f"FPS: {int(camera.fps)}")

asyncio.run(main_loop())
