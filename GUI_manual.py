import tkinter as tk
from tkinter import ttk, messagebox
import threading
from dynautics_interpreter import DynauticsController
import pynmea2
from datetime import datetime

class DynauticsGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Dynautics Manual Controller")

        # Initialize the controller
        self.controller = DynauticsController(port="COM4", baudrate=115200)
        self.controller.register_nmea_callback(self.update_navigation_display)

        # Start reading NMEA data after initialization
        self.controller.start_reading()

        # Engine Controls
        ttk.Label(root, text="Port Engine (-100 to +100):").grid(row=0, column=0, padx=5, pady=5)
        self.port_thrust = tk.DoubleVar()
        ttk.Entry(root, textvariable=self.port_thrust).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(root, text="Starboard Engine (-100 to +100):").grid(row=1, column=0, padx=5, pady=5)
        self.starboard_thrust = tk.DoubleVar()
        ttk.Entry(root, textvariable=self.starboard_thrust).grid(row=1, column=1, padx=5, pady=5)

        self.send_button = ttk.Button(root, text="Send Engine Command", command=self.send_engine_command)
        self.send_button.grid(row=2, column=0, columnspan=2, pady=5)

        self.all_stop_button = ttk.Button(root, text="All Stop", command=self.all_stop)
        self.all_stop_button.grid(row=3, column=0, columnspan=2, pady=5)

        # Navigation Data Display
        ttk.Label(root, text="Latest Navigation Data:", font=("Arial", 10, "bold")).grid(row=4, column=0, columnspan=2, pady=5)

        self.latest_nav_data = {
            "Computer Time": tk.StringVar(value="N/A"),  # System time
            "GPS Time": tk.StringVar(value="N/A"),  # GPS time from RMC
            "Latitude": tk.StringVar(value="N/A"),
            "Longitude": tk.StringVar(value="N/A"),
            "Course": tk.StringVar(value="N/A"),
            "Speed (Ground)": tk.StringVar(value="N/A"),
            "Speed (Water)": tk.StringVar(value="N/A"),
            "Heading": tk.StringVar(value="N/A"),
            "ROT": tk.StringVar(value="N/A"),
        }


        row_index = 5
        for key, var in self.latest_nav_data.items():
            ttk.Label(root, text=f"{key}:").grid(row=row_index, column=0, sticky="e", padx=5, pady=2)
            ttk.Label(root, textvariable=var, relief="sunken", width=25).grid(row=row_index, column=1, sticky="w", padx=5, pady=2)
            row_index += 1

        # Box for last 3 non-navigation NMEA sentences
        ttk.Label(root, text="Last 3 Other NMEA Sentences:", font=("Arial", 10, "bold")).grid(row=row_index, column=0, columnspan=2, pady=5)
        
        self.nmea_log = tk.Text(root, height=4, width=50, state=tk.DISABLED)
        self.nmea_log.grid(row=row_index + 1, column=0, columnspan=2, padx=5, pady=5)

        # Start/Stop Navigation Buttons
        self.start_nav_button = ttk.Button(root, text="Start Navigation Data", command=self.start_navigation)
        self.start_nav_button.grid(row=row_index + 2, column=0, pady=5)

        self.stop_nav_button = ttk.Button(root, text="Stop Navigation Data", command=self.stop_navigation)
        self.stop_nav_button.grid(row=row_index + 2, column=1, pady=5)

        # Store last 3 non-navigation messages
        self.last_nmea_messages = []
        self.awaiting_ack = False  # Track if we are waiting for an ACK

    def update_nmea_log(self, message):
        """
        Stores the last 3 non-navigation NMEA sentences and updates the display.
        Excludes RMC, ROT, HDT, VBW, and ACK messages.
        """
        # Ignore known navigation messages
        if any(tag in message for tag in ["RMC", "ROT", "HDT", "VBW", "ACK"]):
            return  

        # Keep only the last 3 messages
        self.last_nmea_messages.append(message)
        if len(self.last_nmea_messages) > 3:
            self.last_nmea_messages.pop(0)

        # Update log display
        self.nmea_log.config(state=tk.NORMAL)
        self.nmea_log.delete("1.0", tk.END)
        self.nmea_log.insert(tk.END, "\n".join(self.last_nmea_messages) + "\n")
        self.nmea_log.config(state=tk.DISABLED)


    def send_engine_command(self):
        port = self.port_thrust.get()
        starboard = self.starboard_thrust.get()
        if not (-100 <= port <= 100 and -100 <= starboard <= 100):
            return
        self.controller.send_motor_command(port, starboard)

    def all_stop(self):
        self.controller.send_motor_command(0, 0)

    def start_navigation(self):
        """
        Starts the navigation stream and listens for an ACK.
        Runs in a separate thread to prevent freezing.
        """
        threading.Thread(target=self.send_nvo_command, args=("CCNVO,2,1.0,0,0.0",), daemon=True).start()

    def stop_navigation(self):
        """
        Stops the navigation stream without waiting for an ACK.
        Runs in a separate thread to prevent freezing.
        """
        threading.Thread(target=self.send_nvo_command, args=("CCNVO,0,1.0,0,0.0",), daemon=True).start()

    def send_nvo_command(self, command_body):
        """
        Sends an NVO command and waits for an ACK, but only if it is a start command.
        Retries once if no ACK is received.
        """
        is_start_command = "CCNVO,2," in command_body  # Only check ACK for start command

        self.controller.send_nmea_command(command_body)

        if not is_start_command:  
            return  # No need to check for ACK when stopping the stream

        self.awaiting_ack = True
        self.root.after(2000, self.check_ack_status, command_body)  # Check after 2 seconds

    def check_ack_status(self, command_body):
        """
        Checks if an ACK has been received after sending NVO.
        Retries once if no ACK is received.
        """
        if not self.awaiting_ack:
            return  # ACK was received, no need to retry

        print("No ACK received, retrying NVO start command...")
        self.controller.send_nmea_command(command_body)
        self.awaiting_ack = True
        self.root.after(2000, self.final_ack_check)

    def final_ack_check(self):
        """
        Final check after second NVO attempt.
        If no ACK is received, show an error message.
        """
        if not self.awaiting_ack:
            return  # ACK received, no need to show error

        print("Error: No ACK received after retry!")
        messagebox.showerror("NVO Error", "No ACK received for NVO start command!")

    def update_navigation_display(self, message):
        try:
            msg = pynmea2.parse(message)

            if "ACK" in message:
                print(f"Received ACK: {message}")
                self.awaiting_ack = False
                return

            if msg.sentence_type == "RMC":
                # Get system (computer) time
                computer_time = datetime.utcnow().strftime("%H:%M:%S UTC")
                self.latest_nav_data["Computer Time"].set(computer_time)

                # Convert NMEA time to HH:MM:SS format
                gps_time = msg.timestamp.strftime("%H:%M:%S UTC")
                self.latest_nav_data["GPS Time"].set(gps_time)

                self.latest_nav_data["Latitude"].set(msg.latitude)
                self.latest_nav_data["Longitude"].set(msg.longitude)
                self.latest_nav_data["Course"].set(f"{msg.true_course}°")
                self.latest_nav_data["Speed (Ground)"].set(f"{msg.spd_over_grnd} kn")
            elif msg.sentence_type == "ROT":
                self.latest_nav_data["ROT"].set(f"{msg.rate_of_turn}°/min")
            elif msg.sentence_type == "HDT":
                self.latest_nav_data["Heading"].set(f"{msg.heading}°")
            elif msg.sentence_type == "VBW":
                self.latest_nav_data["Speed (Water)"].set(f"{msg.data[0]} kn")
            else:
                self.update_nmea_log(message)
        except pynmea2.ParseError:
            self.update_nmea_log(message)
            
if __name__ == "__main__":
    root = tk.Tk()
    app = DynauticsGUI(root)
    root.mainloop()  # Starts the Tkinter event loop


