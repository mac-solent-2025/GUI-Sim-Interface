import serial
import tkinter as tk
from tkinter import ttk, messagebox
import time
from datetime import datetime
import waypoints  # Module with get_distance and get_bearing
from waypoint_navigation import VesselNavigator  # Centralizes NMEA parsing
from auto_control import AutoController

# Tunable Parameters for the GUI and serial communication
SERIAL_PORT = "COM4"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1            # in seconds
BASE_THRUST = 70              # Initial base thrust for auto control
GUI_UPDATE_INTERVAL = 100     # in milliseconds
class NMEA_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("NMEA Navigation & PID Debug Display")
        
        # Main Navigation Data Display
        self.nav_data = tk.StringVar(value="Waiting for navigation data...")
        ttk.Label(root, textvariable=self.nav_data, wraplength=600, justify="left")\
            .grid(row=0, column=0, columnspan=2, pady=5)
        
        # Debug labels for heading information
        self.current_heading_str = tk.StringVar(value="Current Heading: N/A")
        ttk.Label(root, textvariable=self.current_heading_str, wraplength=400, justify="left")\
            .grid(row=1, column=0, columnspan=2, pady=2)
        self.desired_heading_str = tk.StringVar(value="Desired Heading: N/A")
        ttk.Label(root, textvariable=self.desired_heading_str, wraplength=400, justify="left")\
            .grid(row=2, column=0, columnspan=2, pady=2)
        self.heading_error_str = tk.StringVar(value="Heading Error: N/A")
        ttk.Label(root, textvariable=self.heading_error_str, wraplength=400, justify="left")\
            .grid(row=3, column=0, columnspan=2, pady=2)
        
        # Waypoint and distance display
        self.waypoint_label = tk.StringVar(value="Waypoint: N/A")
        ttk.Label(root, textvariable=self.waypoint_label, wraplength=400, justify="left")\
            .grid(row=4, column=0, columnspan=2, pady=2)
        self.distance_label = tk.StringVar(value="Distance: N/A")
        ttk.Label(root, textvariable=self.distance_label, wraplength=400, justify="left")\
            .grid(row=5, column=0, columnspan=2, pady=2)
        self.bearing_label = tk.StringVar(value="Bearing: N/A")
        ttk.Label(root, textvariable=self.bearing_label, wraplength=400, justify="left")\
            .grid(row=6, column=0, columnspan=2, pady=2)
        
        # PID Debug Output
        self.pid_debug = tk.StringVar(value="PID Debug: N/A")
        ttk.Label(root, textvariable=self.pid_debug, wraplength=400, justify="left", foreground="blue")\
            .grid(row=7, column=0, columnspan=2, pady=5)
        
        # Control Buttons
        self.start_btn = ttk.Button(root, text="Start Navigation", command=self.start_navigation)
        self.start_btn.grid(row=8, column=0, pady=5)
        self.stop_btn = ttk.Button(root, text="Stop Navigation", command=self.stop_navigation)
        self.stop_btn.grid(row=8, column=1, pady=5)
        
        # Open serial port and set up VesselNavigator.
        self.serial_connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        self.vessel_navigator = VesselNavigator(serial_connection=self.serial_connection)
        self.vessel_navigator.setup()           # Loads waypoints and starts internal NMEA parsing
        self.vessel_navigator.start_parsing()     # Ensure VesselNavigator is actively parsing NMEA data
        
        # Create an instance of AutoController.
        self.auto_controller = AutoController(base_thrust=BASE_THRUST)
        
        self.running = True
        self.update_gui()
    
    def update_gui(self):
        # Retrieve current navigation status from VesselNavigator.
        status = self.vessel_navigator.get_status()  # Expected keys: "lat", "lon", "heading", "speed"
        if status["lat"] is not None and status["lon"] is not None:
            pos_str = f"Pos: {status['lat']:.6f}, {status['lon']:.6f}"
        else:
            pos_str = "Pos: N/A"
        heading = status["heading"] if status["heading"] is not None else 0.0
        speed = status["speed"] if status["speed"] is not None else 0.0
        self.nav_data.set(f"{pos_str} | Heading: {heading:.2f}° | Speed: {speed:.2f} kn")
        self.current_heading_str.set(f"Current Heading: {heading:.2f}°")
        
        # If there's at least one waypoint available...
        if self.vessel_navigator.waypoints and self.vessel_navigator.current_waypoint_index < len(self.vessel_navigator.waypoints):
            wp = self.vessel_navigator.waypoints[self.vessel_navigator.current_waypoint_index]
            self.waypoint_label.set(f"Waypoint: {wp.name} ({wp.latitude:.6f}, {wp.longitude:.6f})")
            if status["lat"] is not None and status["lon"] is not None:
                distance = waypoints.get_distance([status["lat"], status["lon"]],
                                                [float(wp.latitude), float(wp.longitude)])
                bearing = waypoints.get_bearing([status["lat"], status["lon"]],
                                                [float(wp.latitude), float(wp.longitude)])
                self.distance_label.set(f"Distance: {distance:.2f} m")
                self.bearing_label.set(f"Bearing: {bearing:.2f}°")
                
                # Check if within acceptance radius: if so, advance to the next waypoint.
                if distance < self.vessel_navigator.acceptance_radius:
                    self.vessel_navigator.current_waypoint_index += 1
                    if self.vessel_navigator.current_waypoint_index < len(self.vessel_navigator.waypoints):
                        wp = self.vessel_navigator.waypoints[self.vessel_navigator.current_waypoint_index]
                        target_lat = float(wp.latitude)
                        target_lon = float(wp.longitude)
                        distance = waypoints.get_distance([status["lat"], status["lon"]],
                                                        [target_lat, target_lon])
                        bearing = waypoints.get_bearing([status["lat"], status["lon"]],
                                                        [target_lat, target_lon])
                        self.waypoint_label.set(f"Waypoint: {wp.name} ({wp.latitude:.6f}, {wp.longitude:.6f})")
                        self.distance_label.set(f"Distance: {distance:.2f} m")
                        self.bearing_label.set(f"Bearing: {bearing:.2f}°")
                    else:
                        self.waypoint_label.set("Waypoint: N/A")
                        self.distance_label.set("Distance: N/A")
                        self.bearing_label.set("Bearing: N/A")
                
                # Only update auto control if navigation is active.
                if self.vessel_navigator.navigation_active:
                    desired_heading = bearing  # Use computed bearing as desired heading
                    self.desired_heading_str.set(f"Desired Heading: {desired_heading:.2f}°")
                    error = self.auto_controller.compute_circular_error(desired_heading, heading)
                    self.heading_error_str.set(f"Heading Error: {error:.2f}°")
                    err, corr, base, port, starboard = self.auto_controller.update_auto_control(
                        desired_heading, heading, speed, self.vessel_navigator.send_nmea_command)
                    self.pid_debug.set(f"PID: Err {err:.2f}, Corr {corr:.2f}, Base {base:.2f}")
                else:
                    self.pid_debug.set("Navigation stopped")
        
        else:
            self.waypoint_label.set("Waypoint: N/A")
            self.distance_label.set("Distance: N/A")
            self.bearing_label.set("Bearing: N/A")
        
        self.root.after(GUI_UPDATE_INTERVAL, self.update_gui)



    def start_navigation(self):
        self.vessel_navigator.start_navigation()
        self.running = True
    
    def stop_navigation(self):
        self.running = False
        self.vessel_navigator.stop_navigation()
        self.vessel_navigator.send_nmea_command("$CCMCO,0.0,0.00,0.00")

if __name__ == "__main__":
    root = tk.Tk()
    app = NMEA_GUI(root)
    root.mainloop()
