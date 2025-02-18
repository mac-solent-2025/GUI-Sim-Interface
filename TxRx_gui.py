import serial
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import pynmea2
import time
from datetime import datetime
from waypoint_navigation import VesselNavigator, convert_dm_to_decimal
import waypoints  # Module with get_distance and get_bearing

class NMEA_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("NMEA Navigation & Debug Display")
        
        # Main Navigation Data Display
        self.nav_data = tk.StringVar(value="Waiting for navigation data...")
        ttk.Label(root, textvariable=self.nav_data, wraplength=400, justify="left").grid(row=0, column=0, columnspan=2, pady=5)
        
        # Waypoint info display
        self.waypoint_number = tk.StringVar(value="Waypoint: N/A")
        ttk.Label(root, textvariable=self.waypoint_number).grid(row=1, column=0, columnspan=2, pady=2)
        self.distance_to_waypoint = tk.StringVar(value="Distance: N/A")
        ttk.Label(root, textvariable=self.distance_to_waypoint).grid(row=2, column=0, columnspan=2, pady=2)
        self.bearing_to_waypoint = tk.StringVar(value="Bearing: N/A")
        ttk.Label(root, textvariable=self.bearing_to_waypoint).grid(row=3, column=0, columnspan=2, pady=2)
        
        # Vessel status display
        self.vessel_speed = tk.StringVar(value="Speed: N/A")
        ttk.Label(root, textvariable=self.vessel_speed).grid(row=4, column=0, columnspan=2, pady=2)
        self.vessel_heading = tk.StringVar(value="Heading: N/A")
        ttk.Label(root, textvariable=self.vessel_heading).grid(row=5, column=0, columnspan=2, pady=2)
        
        # Debug output display
        self.debug_output = tk.StringVar(value="Debug Info:")
        ttk.Label(root, textvariable=self.debug_output, wraplength=400, justify="left", foreground="blue").grid(row=6, column=0, columnspan=2, pady=5)
        
        # Debug details: current position, target, desired heading, heading error.
        self.current_pos_str = tk.StringVar(value="Current: N/A")
        ttk.Label(root, textvariable=self.current_pos_str, wraplength=400, justify="left").grid(row=7, column=0, columnspan=2)
        self.target_pos_str = tk.StringVar(value="Target: N/A")
        ttk.Label(root, textvariable=self.target_pos_str, wraplength=400, justify="left").grid(row=8, column=0, columnspan=2)
        self.desired_heading_str = tk.StringVar(value="Desired Heading: N/A")
        ttk.Label(root, textvariable=self.desired_heading_str, wraplength=400, justify="left").grid(row=9, column=0, columnspan=2)
        self.heading_error_str = tk.StringVar(value="Heading Error: N/A")
        ttk.Label(root, textvariable=self.heading_error_str, wraplength=400, justify="left").grid(row=10, column=0, columnspan=2)
        
        # Navigation control buttons
        self.start_nav_button = ttk.Button(root, text="Start Navigation", command=self.start_navigation)
        self.start_nav_button.grid(row=11, column=0, pady=5)
        self.stop_nav_button = ttk.Button(root, text="Stop Navigation", command=self.stop_navigation)
        self.stop_nav_button.grid(row=11, column=1, pady=5)
        
        # Open serial port on startup and create persistent VesselNavigator.
        self.serial_connection = serial.Serial("COM4", 115200, timeout=1)
        self.vessel_navigator: VesselNavigator = VesselNavigator(serial_connection=self.serial_connection)
        self.vessel_navigator.setup()
        
        self.running = False

    def start_navigation(self):
        self.vessel_navigator.start_navigation()
        self.running = True
        threading.Thread(target=self.receive_navigation_data, daemon=True).start()

    def stop_navigation(self):
        self.running = False
        self.vessel_navigator.stop_navigation()

    def receive_navigation_data(self):
        received_data = {"GPRMC": None, "SPROT": None, "SPHDT": None, "TIME": None}
        while self.running:
            if self.serial_connection.in_waiting > 0:
                response = self.serial_connection.readline().decode(errors='ignore').strip()
                if response.startswith("$GPRMC") or response.startswith("$SPROT") or response.startswith("$SPHDT"):
                    print(f"Received: {response}")
                    self.parse_nmea_sentence(response, received_data)
            time.sleep(0.5)

    def parse_nmea_sentence(self, sentence, received_data):
        try:
            msg = pynmea2.parse(sentence)
            if msg.sentence_type == "RMC":
                received_data["TIME"] = datetime.utcnow().strftime("%H:%M:%S UTC")
                # For display purposes, use already converted decimal degrees.
                lat_dmm = self.format_dmm(msg.latitude, msg.lat_dir)
                lon_dmm = self.format_dmm(msg.longitude, msg.lon_dir)
                received_data["GPRMC"] = f"Lat: {lat_dmm}, Long: {lon_dmm}, Speed: {msg.spd_over_grnd} kn, Course: {msg.true_course}°"
                self.vessel_speed.set(f"Speed: {msg.spd_over_grnd} kn")
                self.vessel_heading.set(f"Heading: {msg.true_course}°")
                
                # Use our helper to convert raw ddmm.mmmm to decimal degrees.
                current_lat = convert_dm_to_decimal(msg.lat, msg.lat_dir)
                current_lon = convert_dm_to_decimal(msg.lon, msg.lon_dir)
                self.current_pos_str.set(f"Current: {current_lat:.6f}, {current_lon:.6f}")
                
                if self.vessel_navigator and self.vessel_navigator.waypoints:
                    current_wp = self.vessel_navigator.waypoints[self.vessel_navigator.current_waypoint_index]
                    # Waypoint data is already converted to decimal degrees.
                    target_lat = float(current_wp.latitude)
                    target_lon = float(current_wp.longitude)
                    self.target_pos_str.set(f"Target: {target_lat:.6f}, {target_lon:.6f}")
                    
                    distance = waypoints.get_distance([current_lat, current_lon], [target_lat, target_lon])
                    bearing = waypoints.get_bearing([current_lat, current_lon], [target_lat, target_lon])
                    
                    self.waypoint_number.set(f"Waypoint: {current_wp.name}")
                    self.distance_to_waypoint.set(f"Distance: {distance:.2f} m")
                    self.bearing_to_waypoint.set(f"Bearing: {bearing:.2f}°")
                    
                    # Compute heading error.
                    desired_heading = bearing
                    current_heading = msg.true_course if msg.true_course else 0
                    heading_error = ((desired_heading - current_heading + 180) % 360) - 180
                    self.desired_heading_str.set(f"Desired Heading: {desired_heading:.2f}°")
                    self.heading_error_str.set(f"Heading Error: {heading_error:.2f}°")
                    
                self.nav_data.set(f"Time: {received_data['TIME']}\n{received_data['GPRMC']}")
            elif msg.sentence_type == "ROT":
                received_data["SPROT"] = f"Rate of Turn: {msg.rate_of_turn}°/min"
            elif msg.sentence_type == "HDT":
                received_data["SPHDT"] = f"Heading: {msg.heading}°"
            
            if all(received_data.values()):
                self.nav_data.set(f"Time: {received_data['TIME']}\n" +
                                  "\n".join(received_data.values()))
        except pynmea2.nmea.ParseError:
            pass  # Ignore invalid messages

    def format_dmm(self, decimal_degrees, direction):
        """Converts a decimal degree value to DMM (degrees and decimal minutes) format."""
        try:
            degrees = int(decimal_degrees)
            minutes = (abs(decimal_degrees) - abs(degrees)) * 60
            return f"{abs(degrees)}° {minutes:.3f}' {direction}"
        except (ValueError, TypeError):
            return "N/A"

if __name__ == "__main__":
    root = tk.Tk()
    app = NMEA_GUI(root)
    root.mainloop()
