import tkinter as tk
from tkinter import ttk, messagebox
import pynmea2
from datetime import datetime

from nmea_comm import NMEAComm  # Import the serial communication module

class NMEA_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("NMEA Engine & Navigation Control")
        
        # Create an instance of the NMEA communication class.
        self.nmea = NMEAComm(port="COM4")
        
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
        self.nav_data = tk.StringVar(value="Waiting for navigation data...")
        ttk.Label(root, textvariable=self.nav_data, wraplength=400, justify="left").grid(row=4, column=0, columnspan=2, pady=5)
        
        self.start_nav_button = ttk.Button(root, text="Start Navigation Data", command=self.start_navigation)
        self.start_nav_button.grid(row=5, column=0, pady=5)
        
        self.stop_nav_button = ttk.Button(root, text="Stop Navigation Data", command=self.stop_navigation)
        self.stop_nav_button.grid(row=5, column=1, pady=5)
        
        self.running = False
        self.received_data = {"GPRMC": None, "SPROT": None, "SPHDT": None, "TIME": None}
        
    def send_engine_command(self):
        port = self.port_thrust.get()
        starboard = self.starboard_thrust.get()
        if not (-100 <= port <= 100 and -100 <= starboard <= 100):
            messagebox.showerror("Error", "Engine values must be between -100 and +100")
            return
        command = f"$CCMCO,0.0,{port:.2f},{starboard:.2f}"
        try:
            self.nmea.send_command(command)
            messagebox.showinfo("Success", "Engine command sent.")
        except Exception as e:
            messagebox.showerror("Error", str(e))
        
    def all_stop(self):
        try:
            self.nmea.send_command("$CCMCO,0.0,0.00,0.00")
            messagebox.showinfo("All Stop", "All engines set to 0.")
        except Exception as e:
            messagebox.showerror("Error", str(e))
        
    def start_navigation(self):
        try:
            self.nmea.send_command("$CCNVO,2,1.0,0,0.0")
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return
        self.running = True
        # Reset received data for the new navigation session.
        self.received_data = {"GPRMC": None, "SPROT": None, "SPHDT": None, "TIME": None}
        self.poll_navigation_data()
        
    def stop_navigation(self):
        self.running = False
        try:
            self.nmea.send_command("$CCNVO,0,1.0,0,0.0")
        except Exception as e:
            messagebox.showerror("Error", str(e))
        
    def poll_navigation_data(self):
        if self.running:
            if self.nmea.available():
                while self.nmea.available():
                    response = self.nmea.read_line()
                    if response.startswith("$GPRMC") or response.startswith("$SPROT") or response.startswith("$SPHDT"):
                        print(f"Received: {response}")
                        self.parse_nmea_sentence(response)
            # Schedule the next poll after 500 milliseconds.
            self.root.after(500, self.poll_navigation_data)
            
    def parse_nmea_sentence(self, sentence):
        try:
            msg = pynmea2.parse(sentence)
            if msg.sentence_type == "RMC":
                self.received_data["TIME"] = datetime.utcnow().strftime("%H:%M:%S UTC")
                lat_dmm = self.format_dmm(msg.latitude, msg.lat_dir)
                lon_dmm = self.format_dmm(msg.longitude, msg.lon_dir)
                self.received_data["GPRMC"] = (
                    f"Lat: {lat_dmm}, Long: {lon_dmm}, "
                    f"Speed: {msg.spd_over_grnd} kn, Course: {msg.true_course}°"
                )
            elif msg.sentence_type == "ROT":
                self.received_data["SPROT"] = f"Rate of Turn: {msg.rate_of_turn}°/min"
            elif msg.sentence_type == "HDT":
                self.received_data["SPHDT"] = f"Heading: {msg.heading}°"
            
            if all(self.received_data.values()):
                self.nav_data.set(
                    f"Time: {self.received_data['TIME']}\n"
                    f"{self.received_data['GPRMC']}\n"
                    f"{self.received_data['SPROT']}\n"
                    f"{self.received_data['SPHDT']}"
                )
        except pynmea2.nmea.ParseError:
            pass  # Ignore invalid messages
        
    def format_dmm(self, decimal_degrees, direction):
        """ Converts decimal degrees to degrees and decimal minutes (DMM) format """
        degrees = int(decimal_degrees)
        minutes = (abs(decimal_degrees) - abs(degrees)) * 60
        return f"{abs(degrees)}° {minutes:.3f}' {direction}"

if __name__ == "__main__":
    root = tk.Tk()
    app = NMEA_GUI(root)
    root.mainloop()
