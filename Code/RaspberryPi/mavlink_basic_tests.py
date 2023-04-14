from pymavlink import mavutil
from pymavlink.CSVReader import CSVReader
from pymavlink.DFReader import DFReader_binary, DFReader_text
from pymavlink.mavutil import mavtcp, mavtcpin, mavudp, mavmcast, mavchildexec, mavmmaplog, mavlogfile, mavserial


class Copter:

    # Type hints for the serial object so that the IDE knows what to expect and I get working auto-complete :)
    serial: mavtcp | mavtcpin | mavudp | mavmcast | DFReader_binary | CSVReader | DFReader_text | mavchildexec | \
            mavmmaplog | mavlogfile | mavserial

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

    def request_message_once(self, message_id: int):
        self.serial.mav.command_long_send(
            self.serial.target_system,
            self.serial.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command
            0,  # Confirmation
            message_id,  # Message ID
            0, 0, 0, 0, 0,  # Parameters
            1  # Response target
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
            self.rangefinder_distance = msg_distance_sensor.current_distance
        except:
            pass

        print(f"Battery voltage: {self.battery_voltage}, remaining: {self.battery_remaining}%. Distance: {self.rangefinder_distance}")

    def send_heartbeat(self):
        self.serial.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                       mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def send_landing_target(self, message_id, frequency_hz):
        self.serial.mav.command_long_send(
            self.serial.target_system,
            self.serial.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL,  # Command
            0,  # Confirmation
            0,  # Target number
            0,  # Offset
            0,  # Descend rate
            0,  # Yaw angle
            0,  # Y-axis position
            0,  # X-axis position
            0)  # Z-axis position


copter = Copter()
copter.connect('COM4')
copter.request_message_interval(132, 10)  # DISTANCE_SENSOR @ 10Hz
copter.request_message_interval(147, 0.1)  # BATTERY_STATUS @ 0.1Hz

while True:

    copter.get_sensors()
