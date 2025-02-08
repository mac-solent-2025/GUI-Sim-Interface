import tkinter as tk
from tkinter import ttk, messagebox
import threading
from dynautics_interpreter import DynauticsController  # Ensure this is in the same directory or properly installed

class DynauticsGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Dynautics Controller Interface")

        # Initialize the controller
        self.controller = DynauticsController(port="COM4", baudrate=115200)

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

        # Register for NMEA messages
        self.controller.register_nmea_callback(self.update_navigation_display)

    def send_engine_command(self):
        port = self.port_thrust.get()
        starboard = self.starboard_thrust.get()
        if not (-100 <= port <= 100 and -100 <= starboard <= 100):
            messagebox.showerror("Error", "Engine values must be between -100 and 100")
            return
        self.controller.send_motor_command(port, starboard)
        messagebox.showinfo("Success", "Engine command sent.")

    def all_stop(self):
        self.controller.send_motor_command(0, 0)
        messagebox.showinfo("All Stop", "All engines set to 0.")

    def start_navigation(self):
        """
        Starts the navigation data stream from the Dynautics controller.
        """
        self.controller.send_nmea_command("CCNVO,2,1.0,0,0.0")  # Now includes checksum
        self.controller.running = True
        threading.Thread(target=self.controller._read_loop, daemon=True).start()


    def stop_navigation(self):
        """
        Stops the navigation data stream from the Dynautics controller.
        """
        self.controller.send_nmea_command("CCNVO,0,1.0,0,0.0")  # Now includes checksum
        self.controller.running = False


    def update_navigation_display(self, message):
        """ Update the GUI with the latest NMEA data """
        self.nav_data.set(f"Latest NMEA: {message}")

    def on_close(self):
        """ Cleanup on closing the GUI """
        self.controller.stop()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DynauticsGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)  # Ensure clean shutdown
    root.mainloop()
