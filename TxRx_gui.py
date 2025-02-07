import serial
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import pynmea2
import time
from datetime import datetime

class NMEA_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("NMEA Engine & Navigation Control")
        
        # Serial Port Selection
        self.serial_port = tk.StringVar()
        ttk.Label(root, text="Select COM Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_menu = ttk.Combobox(root, textvariable=self.serial_port, values=["COM3", "COM4", "COM5", "COM6"])
        self.port_menu.grid(row=0, column=1, padx=5, pady=5)
        self.port_menu.current(0)
        
        # Engine Controls
        ttk.Label(root, text="Port Engine (-100 to +100):").grid(row=1, column=0, padx=5, pady=5)
        self.port_thrust = tk.DoubleVar()
        ttk.Entry(root, textvariable=self.port_thrust).grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Label(root, text="Starboard Engine (-100 to +100):").grid(row=2, column=0, padx=5, pady=5)
        self.starboard_thrust = tk.DoubleVar()
        ttk.Entry(root, textvariable=self.starboard_thrust).grid(row=2, column=1, padx=5, pady=5)
        
        self.send_button = ttk.Button(root, text="Send Engine Command", command=self.send_engine_command)
        self.send_button.grid(row=3, column=0, columnspan=2, pady=5)
        
        self.all_stop_button = ttk.Button(root, text="All Stop", command=self.all_stop)
        self.all_stop_button.grid(row=4, column=0, columnspan=2, pady=5)
        
        # Navigation Data Display
        self.nav_data = tk.StringVar(value="Waiting for navigation data...")
        ttk.Label(root, textvariable=self.nav_data, wraplength=400, justify="left").grid(row=5, column=0, columnspan=2, pady=5)
        
        self.start_nav_button = ttk.Button(root, text="Start Navigation Data", command=self.start_navigation)
        self.start_nav_button.grid(row=6, column=0, pady=5)
        
        self.stop_nav_button = ttk.Button(root, text="Stop Navigation Data", command=self.stop_navigation)
        self.stop_nav_button.grid(row=6, column=1, pady=5)
        
        self.ser = None
        self.running = False
        
    def open_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port.get(), 115200, timeout=1)
        except serial.SerialException:
            messagebox.showerror("Error", "Failed to open serial port.")
            return False
        return True
        
    def send_nmea_command(self, command):
        if not self.ser:
            if not self.open_serial():
                return
        checksum = 0
        for char in command[1:]:
            checksum ^= ord(char)
        command_str = f"{command}*{checksum:02X}\r\n"
        self.ser.write(command_str.encode())
        print(f"Sent: {command_str.strip()}")

    def send_engine_command(self):
        port = self.port_thrust.get()
        starboard = self.starboard_thrust.get()
        if not (-100 <= port <= 100 and -100 <= starboard <= 100):
            messagebox.showerror("Error", "Engine values must be between -100 and +100")
            return
        command = f"$CCMCO,0.0,{port:.2f},{starboard:.2f}"
        self.send_nmea_command(command)
        messagebox.showinfo("Success", "Engine command sent.")
        
    def all_stop(self):
        self.send_nmea_command("$CCMCO,0.0,0.00,0.00")
        messagebox.showinfo("All Stop", "All engines set to 0.")
        
    def start_navigation(self):
        self.send_nmea_command("$CCNVO,2,1.0,0,0.0")
        self.running = True
        threading.Thread(target=self.receive_navigation_data, daemon=True).start()
        
    def stop_navigation(self):
        self.running = False
        self.send_nmea_command("$CCNVO,0,1.0,0,0.0")
        
    def receive_navigation_data(self):
        received_data = {"GPRMC": None, "SPROT": None, "SPHDT": None, "TIME": None}
        while self.running:
            if self.ser and self.ser.in_waiting > 0:
                response = self.ser.readline().decode(errors='ignore').strip()
                if response.startswith("$GPRMC") or response.startswith("$SPROT") or response.startswith("$SPHDT"):
                    print(f"Received: {response}")
                    self.parse_nmea_sentence(response, received_data)
            time.sleep(0.5)
        
    def parse_nmea_sentence(self, sentence, received_data):
        try:
            msg = pynmea2.parse(sentence)
            if msg.sentence_type == "RMC":
                received_data["TIME"] = datetime.utcnow().strftime("%H:%M:%S UTC")
                lat_dmm = self.format_dmm(msg.latitude, msg.lat_dir)
                lon_dmm = self.format_dmm(msg.longitude, msg.lon_dir)
                received_data["GPRMC"] = f"Lat: {lat_dmm}, Long: {lon_dmm}, Speed: {msg.spd_over_grnd} kn, Course: {msg.true_course}°"
            elif msg.sentence_type == "ROT":
                received_data["SPROT"] = f"Rate of Turn: {msg.rate_of_turn}°/min"
            elif msg.sentence_type == "HDT":
                received_data["SPHDT"] = f"Heading: {msg.heading}°"
            
            if all(received_data.values()):
                self.nav_data.set(f"Time: {received_data['TIME']}\n" + "\n".join(received_data.values()))
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
