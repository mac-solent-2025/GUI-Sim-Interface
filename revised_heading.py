import tkinter as tk
from tkinter import ttk, messagebox
import pynmea2
import numpy as np
from datetime import datetime
from simple_pid import PID

from nmea_comm import NMEAComm  # Import the serial communication module

class NMEA_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("NMEA Engine & Navigation Control")
        
        # Create an instance of the NMEA communication class.
        self.nmea = NMEAComm(port="COM4")
        
        # Navigation Data Display (Heading and Speed)
        self.nav_data = tk.StringVar(value="Waiting for navigation data...")
        ttk.Label(root, textvariable=self.nav_data, wraplength=400, justify="left")\
            .grid(row=0, column=0, columnspan=2, pady=5)
        
        # Auto Heading and Speed Control Inputs
        ttk.Label(root, text="Desired Heading (°):")\
            .grid(row=1, column=0, padx=5, pady=5)
        self.desired_heading = tk.DoubleVar(value=0.0)
        ttk.Entry(root, textvariable=self.desired_heading)\
            .grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Label(root, text="Desired Speed (kn):")\
            .grid(row=2, column=0, padx=5, pady=5)
        self.desired_speed = tk.DoubleVar(value=0.0)
        ttk.Entry(root, textvariable=self.desired_speed)\
            .grid(row=2, column=1, padx=5, pady=5)
        
        self.start_auto_button = ttk.Button(root, text="Start Auto Control", command=self.start_auto_control)
        self.start_auto_button.grid(row=3, column=0, pady=5)
        
        self.stop_auto_button = ttk.Button(root, text="Stop Auto Control", command=self.stop_auto_control)
        self.stop_auto_button.grid(row=3, column=1, pady=5)
        
        # Variables to store current navigation data.
        self.current_heading = None  # From HDT sentences.
        self.current_speed = None    # From RMC sentences.
        
        # Flags for auto control.
        self.auto_control_running = False
        self.running = True  # Always poll navigation data.
        
        # Initialize PID controllers.
        # For the heading PID, set the setpoint to 0 (we drive the circular error to zero).
        self.heading_pid = PID(2.0, 0.1, 0.5, setpoint=0, sample_time=0.1)
        # For the speed PID, the setpoint is in knots; its output will be scaled,
        # and we set output_limits so that negative output never goes below -30.
        self.speed_pid = PID(30.0, 0.0, 0.8, setpoint=self.desired_speed.get(), sample_time=0.1)
        self.speed_pid.output_limits = (-30, 100)
        
        # Start receiving navigation data automatically.
        self.start_navigation()
        
    def start_navigation(self):
        try:
            self.nmea.send_command("$CCNVO,2,1.0,0,0.0")
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return
        self.poll_navigation_data()
        
    def poll_navigation_data(self):
        if self.running:
            if self.nmea.available():
                while self.nmea.available():
                    response = self.nmea.read_line()
                    # Process HDT (heading) and RMC (speed) sentences.
                    if response.startswith("$SPHDT") or response.startswith("$GPRMC"):
                        print(f"Received: {response}")
                        self.parse_nmea_sentence(response)
            # Poll more frequently for a quicker response (every 100 ms).
            self.root.after(100, self.poll_navigation_data)
                        
    def parse_nmea_sentence(self, sentence):
        try:
            msg = pynmea2.parse(sentence)
            if msg.sentence_type == "HDT":
                try:
                    heading = float(msg.heading)
                except ValueError:
                    heading = None
                if heading is not None:
                    self.current_heading = heading
            elif msg.sentence_type == "RMC":
                try:
                    speed = float(msg.spd_over_grnd)
                except ValueError:
                    speed = None
                if speed is not None:
                    self.current_speed = speed
            self.update_nav_display()
        except pynmea2.nmea.ParseError:
            pass  # Ignore invalid messages
            
    def update_nav_display(self):
        heading_str = f"Heading: {self.current_heading}°" if self.current_heading is not None else "Heading: N/A"
        speed_str = f"Speed: {self.current_speed} kn" if self.current_speed is not None else "Speed: N/A"
        self.nav_data.set(f"{heading_str}\n{speed_str}")
        
    def start_auto_control(self):
        # Validate user inputs.
        try:
            heading_val = float(self.desired_heading.get())
            speed_val = float(self.desired_speed.get())
        except ValueError:
            messagebox.showerror("Input Error", "Desired Heading and Speed must be numbers.")
            return
        
        if not (0 <= heading_val <= 360):
            messagebox.showerror("Input Error", "Heading must be between 0 and 360 degrees.")
            return
        
        if not (0 <= speed_val <= 7):
            messagebox.showerror("Input Error", "Speed must be between 0 and 7 knots.")
            return
        
        self.auto_control_running = True
        # Update PID setpoints from validated inputs.
        self.heading_pid.setpoint = 0  # Our PID will drive the circular error to zero.
        self.speed_pid.setpoint = speed_val
        self.update_auto_control()
        
    def stop_auto_control(self):
        self.auto_control_running = False
        # When stopping auto control, send a command to zero both engines.
        try:
            self.nmea.send_command("$CCMCO,0.0,0.00,0.00")
        except Exception as e:
            messagebox.showerror("Error", str(e))
            
    def update_auto_control(self):
        if not self.auto_control_running:
            return
                
        heading_correction = 0
        base_thrust = 0
        circular_error = None
                
        if self.current_heading is not None:
            desired = self.desired_heading.get()
            circular_error = self.compute_circular_error(desired, self.current_heading)
            print(f"Circular Error: {circular_error}")
                    
            # Dynamically adjust the PID gains based on the size of the error.
            if abs(circular_error) < 20:
                self.heading_pid.tunings = (4.0, 0.1, 0.8)
            else:
                self.heading_pid.tunings = (2.0, 0.1, 1.6)
                    
            # Feed the (negative) circular error into the PID (setpoint is 0).
            heading_correction = self.heading_pid(-circular_error)
            print(f"Heading Correction (PID Output): {heading_correction}")
                    
        if self.current_speed is not None:
            # If the heading error is greater than 45°, ignore speed correction.
            if circular_error is not None and abs(circular_error) > 45:
                base_thrust = 0
                print("Heading error > 45°; ignoring speed correction (base thrust set to 0).")
            else:
                base_thrust = self.speed_pid(self.current_speed)
                print(f"Speed Correction (PID Output): {base_thrust}")
                    
        # Combine the corrections: base thrust drives forward motion,
        # while the heading correction provides differential turning.
        port = max(min(base_thrust + heading_correction, 100), -20)
        starboard = max(min(base_thrust - heading_correction, 100), -20)
        command = f"$CCMCO,0.0,{port:.2f},{starboard:.2f}"
        try:
            self.nmea.send_command(command)
        except Exception as e:
            messagebox.showerror("Error", str(e))
                    
        self.root.after(100, self.update_auto_control)

    def compute_circular_error(self, desired, current):
        # Calculate the difference in radians.
        diff_rad = np.deg2rad(desired - current)
        # Use arctan2 to get the smallest signed angle difference, then convert back to degrees.
        circular_error = np.rad2deg(np.arctan2(np.sin(diff_rad), np.cos(diff_rad)))
        return circular_error
        
if __name__ == "__main__":
    root = tk.Tk()
    app = NMEA_GUI(root)
    root.mainloop()
