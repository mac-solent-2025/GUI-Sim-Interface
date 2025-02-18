

from datetime import datetime
from threading import Thread
import serial
import pynmea2
from typing import List, Tuple
import time
from model import NMEA, RMC_Message, TTM_Message, Waypoint
from constants import RMC, ROT, HDT
from exception import OperationalException
import waypoints  # Module with get_distance and get_bearing

# Tunable Parameters for Vessel Navigation
DEFAULT_BAUD_RATE = 115200
DEFAULT_ACCEPTANCE_RADIUS = 3.0     # in meters
DEFAULT_LOOKAHEAD_DISTANCE = 10.0     # in meters
WAYPOINT_FILE_PATH = "waypoints.txt"

def convert_dm_to_decimal(dm_value, direction):
    """
    Convert a ddmm.mmmm string to decimal degrees and fix the sign.
    """
    value = pynmea2.dm_to_sd(dm_value)
    if direction.upper() in ('S', 'W'):
        return -abs(value)
    return abs(value)

class VesselNavigator:
    def __init__(self,
                 serial_connection,
                 baud_rate: int = DEFAULT_BAUD_RATE,
                 acceptance_radius: float = DEFAULT_ACCEPTANCE_RADIUS,
                 lookhead_distance: float = DEFAULT_LOOKAHEAD_DISTANCE):
        """
        Initialize the navigation system.
        """
        self.acceptance_radius = acceptance_radius
        self.lookhead_distance = lookhead_distance
        self.current_waypoint_index = 0
        self.waypoints: List[Waypoint] = []
        self.baud_rate = baud_rate
        self.running = False
        self.navigation_active = False  # Flag to track if navigation is active
        self.serial_connection = serial_connection

        # These attributes hold the latest parsed NMEA data.
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0
        self.current_speed = 0.0

    def load_waypoints_from_file(self, waypoint_file_path: str = WAYPOINT_FILE_PATH):
        with open(waypoint_file_path, 'r', encoding='utf-8') as f:
            while line := f.readline():
                waypoint_sentence = line.rstrip()
                waypoint = self.parse_waypoint_data(waypoint_sentence)
                self.add_waypoint(waypoint)
        self.current_waypoint_index = 0

    def setup(self):
        self.load_waypoints_from_file()

    def add_waypoint(self, waypoint: Waypoint):
        """Add a waypoint (latitude, longitude, and name)."""
        self.waypoints.append(waypoint)
        self.current_waypoint_index = 0

    def parse_gps_data(self, nmea_sentence: str) -> RMC_Message:
        """Parse a GPRMC sentence and return current position and heading (in decimal degrees)."""
        msg = pynmea2.parse(nmea_sentence)
        if not isinstance(msg, pynmea2.RMC):
            raise ValueError("Not a GPRMC sentence")
        rmc_message = RMC_Message()
        rmc_message.latitude = convert_dm_to_decimal(msg.lat, msg.lat_dir)
        rmc_message.longitude = convert_dm_to_decimal(msg.lon, msg.lon_dir)
        rmc_message.speed = msg.spd_over_grnd
        rmc_message.heading = msg.true_course if msg.true_course else 0
        return rmc_message

    def parse_waypoint_data(self, nmea_sentence: str) -> Waypoint:
        """Parse an NMEA-formatted waypoint sentence and immediately convert coordinates."""
        msg = pynmea2.parse(nmea_sentence)
        waypoint = Waypoint()
        waypoint.latitude = convert_dm_to_decimal(msg.lat, msg.lat_dir)
        waypoint.longitude = convert_dm_to_decimal(msg.lon, msg.lon_dir)
        waypoint.name = msg.waypoint_id
        return waypoint

    def parse_ttm_data(self, nmea_sentence: str) -> TTM_Message:
        msg = pynmea2.parse(nmea_sentence)
        message = TTM_Message()
        return message

    def start_parsing(self):
        """Start a background thread that continuously parses incoming NMEA data."""
        self.running = True
        thread = Thread(target=self._nmea_loop, daemon=True)
        thread.start()

    def _nmea_loop(self):
        while self.running:
            if self.serial_connection.in_waiting:
                line = self.serial_connection.readline().decode('ascii', errors='replace').strip()
                try:
                    msg = pynmea2.parse(line)
                    if msg.sentence_type == "RMC":
                        self.current_lat = convert_dm_to_decimal(msg.lat, msg.lat_dir)
                        self.current_lon = convert_dm_to_decimal(msg.lon, msg.lon_dir)
                        try:
                            self.current_speed = float(msg.spd_over_grnd)
                        except (ValueError, TypeError):
                            pass
                    elif msg.sentence_type == "HDT":
                        try:
                            self.current_heading = float(msg.heading)
                        except (ValueError, TypeError):
                            pass
                except pynmea2.nmea.ParseError:
                    pass
            time.sleep(0.05)

    def get_status(self):
        """Return the current navigation status as a dictionary."""
        return {
            "lat": self.current_lat,
            "lon": self.current_lon,
            "speed": self.current_speed,
            "heading": self.current_heading,
        }

    def send_nmea_command(self, command):
        checksum = 0
        for char in command[1:]:
            checksum ^= ord(char)
        command_str = f"{command}*{checksum:02X}\r\n"
        self.serial_connection.write(command_str.encode())
        print(f"Sent: {command_str.strip()}")

    def start_navigation(self):
        self.navigation_active = True
        self.send_nmea_command("$CCNVO,2,1.0,0,0.0")

    def stop_navigation(self):
        self.navigation_active = False
        self.running = False
        self.send_nmea_command("$CCNVO,0,1.0,0,0.0")
