from datetime import datetime
import math
from threading import Thread
import serial
import pynmea2
from typing import List, Tuple
import time
from model import NMEA, RMC_Message, Waypoint
from constants import RMC, ROT, HDT
from exception import OperationalException

class VesselNavigator:
    def __init__(self,
                 serial_port: str,
                 baud_rate: int = 115200,
                 acceptance_radius: float = 3.0,
                 lookhead_distance: float = 10.0):
        """
        Initialize the navigation system
        
        Args:
            acceptance_radius: Distance in meters to consider waypoint reached
            lookhead_distance: Distance to look ahead on path for steering
        """
        self.acceptance_radius = acceptance_radius
        self.lookhead_distance = lookhead_distance
        self.current_waypoint_index = 0
        self.waypoints: List[Waypoint] = []
        self.serial_port = serial_port

        self.baud_rate = baud_rate

        self.serial_connection = None
        self.running = False

    def load_waypoints_from_file(self, waypoint_file_path: str):
        with open(waypoint_file_path, 'r', encoding='utf-8') as f:
            while line := f.readline():
                waypoint_sentence = line.rstrip()

                waypoint = self.parse_waypoint_data(waypoint_sentence)

                self.add_waypoint(waypoint)

        self.current_waypoint_index = 0


    def add_waypoint(self, waypoint: Waypoint):
        """Add list of waypoints (latitude, longitude pairs)"""
        self.waypoints.append(waypoint)
        self.current_waypoint_index = 0

    def get_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two points using Haversine formula"""
        R = 6371000  # Earth radius in meters

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_phi/2) * math.sin(delta_phi/2) +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(delta_lambda/2) * math.sin(delta_lambda/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def get_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate bearing between two points"""
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        diff_long = math.radians(lon2 - lon1)
        
        x = math.sin(diff_long) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) -
             math.sin(lat1) * math.cos(lat2) * math.cos(diff_long))
        
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        
        return (initial_bearing + 360) % 360
    
    def parse_gps_data(self, nmea_sentence: str) -> RMC_Message:
        """Parse GPRMC sentence and return current position and heading"""
        msg = pynmea2.parse(nmea_sentence)
        if not isinstance(msg, pynmea2.RMC):
            raise ValueError("Not a GPRMC sentence")

        rmc_message = RMC_Message()
        rmc_message.latitude = msg.latitude
        rmc_message.longitude = msg.longitude
        rmc_message.speed = msg.spd_over_grnd
        rmc_message.heading = msg.true_course if msg.true_course else 0

        return rmc_message

    def parse_waypoint_data(self, nmea_sentence: str) -> Waypoint:
        msg = pynmea2.parse(nmea_sentence)

        waypoint = Waypoint()
        waypoint.latitude = msg.latitude
        waypoint.longitude = msg.longitude
        waypoint.name = msg.waypoint_name

        return waypoint
    
    def calculate_control_signals(self, rms_message: RMC_Message) -> Tuple[float, float]:
        """
        Calculate control signals for the vessel's propellers
        Returns: (left_thrust, right_thrust) as values between -1 and 1
        """
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return 0, 0

        # current_lat, current_lon = current_pos
        target_waypoint = self.waypoints[self.current_waypoint_index]

        current_lat = rms_message.latitude
        current_lon = rms_message.longitude
        current_heading = rms_message.heading

        target_lat = target_waypoint.latitude
        target_lon = target_waypoint.longitude


        # Check if we've reached the current waypoint
        distance_to_waypoint = self.get_distance(
            current_lat,
            current_lon,
            target_lat,
            target_lon
        )

        if distance_to_waypoint < self.acceptance_radius:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                return 0, 0  # Stop at final waypoint
            target_lat, target_lon = self.waypoints[self.current_waypoint_index]
        
        # Calculate desired heading
        desired_heading = self.get_bearing(current_lat, current_lon, target_lat, target_lon)
        
        # Calculate heading error (-180 to +180 degrees)
        heading_error = ((desired_heading - current_heading + 180) % 360) - 180
        
        # Calculate thrust commands based on heading error
        # Simple P controller for demonstration
        K_p = 0.5  # Proportional gain
        thrust_difference = (K_p * (heading_error / 180.0)) * 100  # Normalized to [-1, 1]
        
        # Base forward thrust
        base_thrust = 70  # 70% forward thrust
        
        # Calculate individual motor thrusts
        left_thrust = base_thrust - thrust_difference
        right_thrust = base_thrust + thrust_difference
        
        # Clamp values between -100 and 100
        left_thrust = max(-100, min(100, left_thrust))
        right_thrust = max(-100, min(100, right_thrust))
        
        return left_thrust, right_thrust

    def run_navigation_loop(self):
        """Main navigation loop"""
        with serial.Serial(self.serial_port, self.baud_rate, timeout=1) as ser:
            while True:
                try:
                    # Read GPS data
                    if ser.in_waiting:
                        line = ser.readline().decode('ascii', errors='replace')
                        if line.startswith('$GPRMC'):
                            # Parse GPS data
                            rms_message = self.parse_gps_data(line)

                            # Calculate control signals
                            port_thrust, starboard_thrust = self.calculate_control_signals(
                                rms_message
                            )

                            # Apply control signals to thrusters
                            self.send_engine_command(
                                port_thrust_power=port_thrust,
                                starboard_thrust_power=starboard_thrust
                            )

                            # Optional: Log navigation data
                            print(f"Position: ({rms_message.latitude}, {rms_message.longitude}), "
                                  f"Heading: {rms_message.heading}°, "
                                  f"Thrust L/R: {port_thrust:.2f}/{starboard_thrust:.2f}")

                except Exception as e:
                    print(f"Error in navigation loop: {e}")
                    continue

                time.sleep(0.1)  # Prevent CPU overload

    def open_serial_connection(self):
        try:
            self.serial_connection = serial.Serial(self.serial_port.get(), 115200, timeout=1)

        except serial.SerialException:
            return False

        return True

    def send_nmea_command(self, command):
        if not self.serial_connection:
            if not self.open_serial_connection():
                return
        checksum = 0
        for char in command[1:]:
            checksum ^= ord(char)
        command_str = f"{command}*{checksum:02X}\r\n"
        self.serial_connection.write(command_str.encode())
        print(f"Sent: {command_str.strip()}")

    def stop_navigation(self):
        self.running = False
        self.send_nmea_command("$CCNVO,0,1.0,0,0.0")

    def start_navigation(self):
        self.send_nmea_command("$CCNVO,2,1.0,0,0.0")
        self.running = True

        navigator_thread = Thread(target=self.run_navigation_loop, daemon=True)
        navigator_thread.start()

    def stop_all(self):
        self.send_nmea_command("$CCMCO,0.0,0.00,0.00")

    def send_engine_command(self, port_thrust_power: float, starboard_thrust_power: float):
        if not (-100 <= port_thrust_power <= 100 and -100 <= starboard_thrust_power <= 100):
            raise OperationalException("Engine values must be between -100 and +100")

        command = f"$CCMCO,0.0,{port_thrust_power:.2f},{starboard_thrust_power:.2f}"
        self.send_nmea_command(command)
        print(f"Command sent: {command}")
    