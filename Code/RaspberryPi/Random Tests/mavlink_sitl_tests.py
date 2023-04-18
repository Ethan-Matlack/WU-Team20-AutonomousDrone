from pymavlink import mavutil

from pymavlink.CSVReader import CSVReader
from pymavlink.DFReader import DFReader_binary, DFReader_text
from pymavlink.mavutil import mavtcp, mavtcpin, mavudp, mavmcast, mavchildexec, mavmmaplog, mavlogfile, mavserial


class Copter:

    # Type hints for the serial object so that the IDE knows what to expect and I get working auto-complete :)
    serial: mavtcp | mavtcpin | mavudp | mavmcast | DFReader_binary | CSVReader | DFReader_text | mavchildexec | \
            mavmmaplog | mavlogfile | mavserial

    def __init__(self):
        self.battery_voltage0 = None
        self.battery_remaining = None
        self.rangefinder_dist_m = None

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

        # self.serial.recv_match(type="BATTERY_STATUS", blocking=True)
        # self.serial.recv_match(type="BATTERY_STATUS", blocking=True)
        # self.serial.recv_match(type="DISTANCE_SENSOR", blocking=True)

        # self.battery_voltage0 = self.serial.messages['BATTERY_STATUS'].voltages[0] / 1000
        # self.battery_remaining = self.serial.messages['BATTERY_STATUS'].battery_remaining
        # self.rangefinder_dist_m = self.serial.messages['DISTANCE_SENSOR'].current_distance

    def send_heartbeat(self):
        self.serial.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                       mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def send_landing_target(self):
        self.serial.mav.command_long_send(
            self.serial.target_system,
            self.serial.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # Command
            0,  # Confirmation
            message_id,  # Message ID:
            1e6 / frequency_hz,  # Frequency in Hz
            0, 0, 0, 0, 0  # Un-used
        )

copter = Copter()
copter.connect()

