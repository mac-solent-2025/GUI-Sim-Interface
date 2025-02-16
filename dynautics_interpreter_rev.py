import serial
import threading
import time

class DynauticsController:
    def __init__(self, port='COM4', baudrate=115200, timeout=1):
        """ Initialize the serial connection and setup thread handling. """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"Opened serial port {port} at {baudrate} baud.")
        except Exception as e:
            print(f"Error opening serial port {port}: {e}")
            raise

        self.nmea_callbacks = []
        self.running = False  # Ensure the system is initially stopped
        self.read_thread = None  # Initialize thread variable

    def start_reading(self):
        """ Start the NMEA read loop in a separate thread. """
        if self.read_thread is None or not self.read_thread.is_alive():
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            print("Started NMEA read thread.")

    def send_nmea_command(self, command_body):
        """ 
        Send an NMEA command to the Dynautics controller.
        The checksum is calculated and appended automatically.
        """
        checksum = self.calculate_nmea_checksum(command_body)
        command_str = f"${command_body}*{checksum:02X}\r\n"

        try:
            self.ser.write(command_str.encode('ascii'))
            self.ser.flush()  # Ensure immediate transmission
            print(f"Sent NMEA command: {command_str.strip()}")
        except Exception as e:
            print(f"Error sending NMEA command: {e}")

    def send_motor_command(self, port_val, starboard_val):
        """
        Send motor commands to the Dynautics controller in NMEA format.
        Format: $CCMCO,0.0,<port_val>,<starboard_val>*<checksum>
        """
        if not (-100 <= port_val <= 100 and -100 <= starboard_val <= 100):
            print("ERROR: Motor values out of range (-100 to 100)")
            return

        command_body = f"CCMCO,0.0,{port_val:.2f},{starboard_val:.2f}"
        checksum = self.calculate_nmea_checksum(command_body)  
        command_str = f"${command_body}*{checksum:02X}\r\n"

        try:
            self.ser.write(command_str.encode('ascii'))
            self.ser.flush()
            print(f"Sent motor command: {command_str.strip()}")
        except Exception as e:
            print(f"Error sending motor command: {e}")

    def calculate_nmea_checksum(self, sentence):
        """ Calculate NMEA checksum (XOR of all characters excluding '$' and '*'). """
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return checksum

    def _read_loop(self):
        """ Continuously read from the serial port and process incoming data. """
        buffer = ""
        while self.running:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode('ascii', errors='ignore')
                    buffer += data

                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._handle_nmea_message(line)
                else:
                    time.sleep(0.1)  # Yield control
            except Exception as e:
                print(f"Error reading from serial port: {e}")
                time.sleep(1)

    def _handle_nmea_message(self, message):
        """ Handle the received NMEA message and notify subscribers. """
        print(f"Received NMEA message: {message}")  # Debugging
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
        """ Stop the read loop and close the serial port. """
        print("Stopping DynauticsController...")
        self.running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1)  # Avoid indefinite blocking
        if self.ser.is_open:
            self.ser.close()
        print("Serial port closed.")

# Example usage (for debugging)
if __name__ == '__main__':
    controller = DynauticsController(port='COM4', baudrate=115200, timeout=1)

    def process_nmea(message):
        print(f"Callback processing message: {message}")

    controller.register_nmea_callback(process_nmea)

    try:
        controller.start_reading()
        controller.send_motor_command(50, -50)
        time.sleep(10)  # Run for a while to capture messages
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        controller.stop()
