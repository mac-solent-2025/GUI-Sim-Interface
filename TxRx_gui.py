from typing import List, Tuple
import serial
import tkinter as tk
from tkinter import ttk, messagebox
from waypoint_navigation import VesselNavigator

class NMEA_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("NMEA Engine & Navigation Control")
        
        # Serial Port Selection
        self.serial_port = tk.StringVar()
        ttk.Label(root, text="Select COM Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_menu = ttk.Combobox(root, textvariable=self.serial_port, values=["COM4", "COM4", "COM5", "COM6"])
        self.port_menu.grid(row=0, column=1, padx=5, pady=5)
        self.port_menu.current(0)
        
        # Engine Controls
        ttk.Label(root, text="Port Engine (-100 to +100):").grid(row=1, column=0, padx=5, pady=5)
        self.port_thrust = tk.DoubleVar()
        ttk.Entry(root, textvariable=self.port_thrust).grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Label(root, text="Starboard Engine (-100 to +100):").grid(row=2, column=0, padx=5, pady=5)
        self.starboard_thrust = tk.DoubleVar()
        ttk.Entry(root, textvariable=self.starboard_thrust).grid(row=2, column=1, padx=5, pady=5)
        
        # self.send_button = ttk.Button(root, text="Send Engine Command", command=self.send_engine_command)
        # self.send_button.grid(row=3, column=0, columnspan=2, pady=5)
        
        self.all_stop_button = ttk.Button(root, text="All Stop", command=self.all_stop)
        self.all_stop_button.grid(row=4, column=0, columnspan=2, pady=5)
        
        # Navigation Data Display
        self.nav_data = tk.StringVar(value="Waiting for navigation data...")
        ttk.Label(root, textvariable=self.nav_data, wraplength=400, justify="left").grid(row=5, column=0, columnspan=2, pady=5)
        
        self.start_nav_button = ttk.Button(root, text="Start Navigation Data", command=self.start_navigation)
        self.start_nav_button.grid(row=6, column=0, pady=5)
        
        self.stop_nav_button = ttk.Button(root, text="Stop Navigation Data", command=self.stop_navigation)
        self.stop_nav_button.grid(row=6, column=1, pady=5)


        self.vessel_navigator: VesselNavigator = VesselNavigator(
            serial_connection = serial.Serial("COM4", 115200, timeout=1)
        )

        self.vessel_navigator.setup()

    def all_stop(self):
        self.vessel_navigator.send_nmea_command("$CCMCO,0.0,0.00,0.00")
        messagebox.showinfo("All Stop", "All engines set to 0.")
        
    def start_navigation(self):
        self.vessel_navigator.start_navigation()
        
    def stop_navigation(self):
        self.vessel_navigator.stop_navigation()

    def format_dmm(self, decimal_degrees, direction):
        """ Converts decimal degrees to degrees and decimal minutes (DMM) format """
        degrees = int(decimal_degrees)
        minutes = (abs(decimal_degrees) - abs(degrees)) * 60
        return f"{abs(degrees)}° {minutes:.3f}' {direction}"

if __name__ == "__main__":
    root = tk.Tk()
    app = NMEA_GUI(root)
    root.mainloop()
