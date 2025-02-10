import serial
import threading
import time

class DynauticsController:
    def __init__(self, port='COM4', baudrate=115200, timeout=1):
        # Open the serial port; adjust baudrate/timeout as needed
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"Opened serial port {port} at {baudrate} baud.")
        except Exception as e:
            print(f"Error opening serial port {port}: {e}")
            raise

        # For thread-safe data sharing
        self.nmea_callbacks = []
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)  # ✅ Initialize `read_thread`


    def start_reading(self):
        """ Starts the NMEA read loop in a separate thread. """
        if not hasattr(self, "read_thread") or not self.read_thread.is_alive():  # ✅ Check if thread exists
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

        
    def send_nmea_command(self, command_body):
        """
        Send an NMEA command to the Dynautics controller.
        The checksum is calculated and appended automatically.
        """
        checksum = self.calculate_nmea_checksum(command_body)
        command_str = f"${command_body}*{checksum:02X}\r\n"

        try:
            self.ser.write(command_str.encode('ascii'))
            self.ser.flush()  # Ensures immediate transmission
            print(f"Sent NMEA command: {command_str.strip()}")
        except Exception as e:
            print(f"Error sending NMEA command: {e}")



    def send_motor_command(self, port_val, starboard_val):
        """
        Send motor commands to the Dynautics controller in NMEA format.
        Format: $CCMCO,0.0,<port_val>,<starboard_val>*<checksum>
        """
        if not (-100 <= port_val <= 100 and -100 <= starboard_val <= 100):
            print("⚠️ ERROR: Motor values out of range")
            return

        # Format the command as required: $CCMCO,0.0,50.00,-50.00
        command_body = f"CCMCO,0.0,{port_val:.2f},{starboard_val:.2f}"

        # Calculate checksum correctly
        checksum = self.calculate_nmea_checksum(command_body)  

        # Format the full NMEA command
        command_str = f"${command_body}*{checksum:02X}\r\n"

        try:
            self.ser.write(command_str.encode('ascii'))
            self.ser.flush()  # Ensure data is sent immediately
            print(f"Sent command: {command_str.strip()}")  # Debug print
        except Exception as e:
            print(f"Error sending command: {e}")

    def calculate_nmea_checksum(self, sentence):
        """
        Calculate NMEA checksum (XOR of all characters in the message, excluding '$' and '*')
        """
        checksum = 0
        for char in sentence:  # Loops through characters in the sentence
            checksum ^= ord(char)
        return checksum

    def _read_loop(self):
        """Continuously read from the serial port and process incoming data."""
        buffer = ""
        while self.running:
            try:
                # Read data from the serial port (non-blocking read)
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode('ascii', errors='ignore')
                    buffer += data

                    # Process complete lines (assuming \n terminated messages)
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._handle_nmea_message(line)
                else:
                    # Sleep briefly to yield control
                    time.sleep(0.1)
            except Exception as e:
                print(f"Error reading from serial port: {e}")
                time.sleep(1)

    def _handle_nmea_message(self, message):
        """
        Handle the received NMEA (or proprietary) message.
        Here you can parse it and notify any subscribers.
        """
        print(f"Received NMEA message: {message}")
        # Notify all registered callback functions
        for callback in self.nmea_callbacks:
            try:
                callback(message)
            except Exception as e:
                print(f"Error in callback: {e}")

    def register_nmea_callback(self, callback):
        """
        Register a callback to receive NMEA messages.
        The callback should accept one parameter (the message string).
        """
        self.nmea_callbacks.append(callback)

    def stop(self):
        """Stop the read loop and close the serial port."""
        self.running = False
        if hasattr(self, "read_thread") and self.read_thread.is_alive():
            self.read_thread.join(timeout=1)  # Avoid indefinite blocking
        self.ser.close()
        print("Serial port closed.")

# Example usage:
if __name__ == '__main__':
    # Instantiate the controller interface
    controller = DynauticsController(port='COM4', baudrate=115200, timeout=1)
    

    # Define a simple callback to process NMEA messages
    def process_nmea(message):
        # You can add custom parsing or pass this on to another module
        print(f"Callback processing message: {message}")

    controller.register_nmea_callback(process_nmea)

    try:
        # Example: send a motor command for port and starboard
        controller.send_motor_command(50, -50)
        
        # Run for a while to capture messages (simulate continuous operation)
        time.sleep(10)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        controller.stop()
        

