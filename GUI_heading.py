import tkinter as tk
from tkinter import ttk, messagebox
import threading
# No need to import DynauticsController here, it's imported below
import pynmea2
from datetime import datetime
import queue  # Import the queue module


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    print(f"Normalized Angle: {angle}°")  # Debugging
    return angle

class SteeringGUI:
    def __init__(self, root, controller):  # <--- Accept controller as an argument
        self.root = root
        self.root.title("Autonomous Steering Control")

        # --- GUI ELEMENTS --- (Same as before, but organized)

        # Desired Heading Input
        ttk.Label(root, text="Desired Heading (°):").grid(row=0, column=0, padx=5, pady=5)
        self.desired_heading = tk.DoubleVar()
        ttk.Entry(root, textvariable=self.desired_heading).grid(row=0, column=1, padx=5, pady=5)

        # Throttle Input
        ttk.Label(root, text="Throttle (-100 to +100):").grid(row=1, column=0, padx=5, pady=5)
        self.throttle = tk.DoubleVar()
        ttk.Entry(root, textvariable=self.throttle).grid(row=1, column=1, padx=5, pady=5)

        # Send Heading and Throttle Button
        self.send_heading_button = ttk.Button(root, text="Send Heading & Throttle", command=self.update_motor_speeds)
        self.send_heading_button.grid(row=1, column=2, padx=5, pady=5)

        # Buttons
        self.start_button = ttk.Button(root, text="Start", command=self.start_control)
        self.start_button.grid(row=2, column=0, pady=5)

        self.stop_button = ttk.Button(root, text="Stop", command=self.stop_control)
        self.stop_button.grid(row=2, column=1, pady=5)

        # --- Navigation Data Display ---
        self.latest_nav_data = {
            "Computer Time": tk.StringVar(value="N/A"),
            "GPS Time": tk.StringVar(value="N/A"),
            "Latitude": tk.StringVar(value="N/A"),
            "Longitude": tk.StringVar(value="N/A"),
            "Course": tk.StringVar(value="N/A"),
            "Speed (Ground)": tk.StringVar(value="N/A"),
            "Speed (Water)": tk.StringVar(value="N/A"),
            "Heading": tk.StringVar(value="N/A"),
            "ROT": tk.StringVar(value="N/A"),
        }
        row_index = 3
        for key, var in self.latest_nav_data.items():
            ttk.Label(root, text=f"{key}:").grid(row=row_index, column=0, sticky="e", padx=5, pady=2)
            ttk.Label(root, textvariable=var, relief="sunken", width=25).grid(row=row_index, column=1, sticky="w", padx=5, pady=2)
            row_index += 1

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # --- Controller and State ---
        self.controller = controller  # <--- Use the passed-in controller
        self.running = False
        self.proportional_gain = 90.0  #  Control gain
        self.deadband = 2.0  # Deadband in degrees


    def on_closing(self):
        """Gracefully stop threads and close the GUI"""
        self.running = False
        if hasattr(self, "controller"):
            self.controller.stop()
        self.root.destroy()

    def update_navigation_display(self, message):
        """Updates the GUI with the latest navigation data."""
        try:
            msg = pynmea2.parse(message)
            if msg.sentence_type == "RMC":
                self.latest_nav_data["Computer Time"].set(datetime.utcnow().strftime("%H:%M:%S UTC"))
                self.latest_nav_data["GPS Time"].set(msg.timestamp.strftime("%H:%M:%S UTC"))
                self.latest_nav_data["Latitude"].set(msg.latitude)
                self.latest_nav_data["Longitude"].set(msg.longitude)
                self.latest_nav_data["Course"].set(f"{msg.true_course}°")
                self.latest_nav_data["Speed (Ground)"].set(f"{msg.spd_over_grnd} kn")
            elif msg.sentence_type == "ROT":
                self.latest_nav_data["ROT"].set(f"{msg.rate_of_turn}°/min")
            elif msg.sentence_type == "HDT":
                self.latest_nav_data["Heading"].set(f"{msg.heading}°")
            elif msg.sentence_type == "VBW":
                self.latest_nav_data["Speed (Water)"].set(f"{msg.data[0]} kn")  # Access data correctly
        except pynmea2.ParseError as e:
            print(f"Parse error: {e}")  # Log parse errors
        except Exception as e:
            print(f"Error updating display: {e}") # Catch other potential errors


    def process_nmea_queue(self):
        """Processes messages from the NMEA queue."""
        try:
            while True:  # Keep checking until the queue is empty
                message = self.controller.nmea_queue.get_nowait()
                self.update_navigation_display(message)
        except queue.Empty:
            pass  # Queue is empty, no more messages to process

        if self.running:
          self.root.after(100, self.process_nmea_queue)  # Check again in 100ms


    def update_motor_speeds(self):
        """Calculates and sends motor commands based on heading error."""
        if not self.running:
            return

        try:
            target_heading = self.desired_heading.get()
            current_heading_str = self.latest_nav_data["Heading"].get()

            # Check if current_heading_str is valid before converting to float
            if current_heading_str == "N/A" or "°" not in current_heading_str:
                print("Waiting for valid heading data...")
                return  # Exit if data is not yet valid

            current_heading = float(current_heading_str.replace("°", ""))
            throttle = self.throttle.get()

            error = normalize_angle(target_heading - current_heading)
            print(f"Target: {target_heading}°, Current: {current_heading}°, Error: {error}°")

            if abs(error) < self.deadband:
                print("Within deadband, no correction needed.")
                self.controller.send_motor_command(int(throttle), int(throttle))  # Apply throttle only
                self.process_nmea_queue() # Process any new data
                return

            base_speed = throttle
            adjustment = (abs(error) / self.proportional_gain) * min(50, abs(throttle))

            if error > 0:  # Need to turn starboard (right)
                port_thrust = base_speed + adjustment
                starboard_thrust = base_speed - adjustment
            else:  # Need to turn port (left)
                port_thrust = base_speed - adjustment
                starboard_thrust = base_speed + adjustment

            port_thrust = int(max(min(port_thrust, 100), -100))  # Clamp and cast to int
            starboard_thrust = int(max(min(starboard_thrust, 100), -100))  # Clamp and cast to int

            print(f"Sending motor command: Port={port_thrust}, Starboard={starboard_thrust}")
            self.controller.send_motor_command(port_thrust, starboard_thrust)
            self.process_nmea_queue() # Process the queue

        except Exception as e:
            print(f"Error updating motor speeds: {e}")


    def start_control(self):
        """Starts the autonomous control loop."""
        self.running = True
        self.controller.start_reading()
        self.process_nmea_queue()  # Start processing the queue


    def stop_control(self):
        """Stops the autonomous control."""
        self.running = False
        self.controller.send_motor_command(0, 0)  # Stop the motors
        # self.controller.stop() # Do not stop the controller here. Stop it on close.
        for key in self.latest_nav_data:
            self.latest_nav_data[key].set("N/A")

if __name__ == "__main__":
    root = tk.Tk()
    controller = DynauticsController(port='COM4', baudrate=115200) # Create controller here
    app = SteeringGUI(root, controller)  # Pass controller to the GUI
    root.mainloop()
