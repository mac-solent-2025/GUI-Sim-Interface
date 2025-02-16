import serial
import threading
import time
import sys
import queue  # Import the queue module

class DynauticsController:
    def __init__(self, port='COM4', baudrate=115200, timeout=1, retries=3, retry_delay=1):
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.retries = retries
        self.retry_delay = retry_delay

        for attempt in range(self.retries):
            try:
                self.ser = serial.Serial(port, baudrate, timeout=timeout)
                print(f"Opened serial port {port} at {baudrate} baud.")
                break  # Exit the loop if successful
            except Exception as e:
                print(f"Error opening serial port {port} (attempt {attempt + 1}/{retries}): {e}")
                if attempt < retries - 1:
                    print(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                else:
                    print(f"Failed to open serial port {port} after multiple retries.  Exiting.")
                    sys.exit(1)

        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.nmea_queue = queue.Queue()  # Create the queue for NMEA messages


    def start_reading(self):
        """ Starts the NMEA read loop in a separate thread. """
        if not self.running:  # Simpler check
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

    def send_command(self, command_body):
        """
        Send a NMEA command to the Dynautics controller.
        The checksum is calculated and appended automatically.
        """
        checksum = self.calculate_nmea_checksum(command_body)
        command_str = f"${command_body}*{checksum:02X}\r\n"

        try:
            self.ser.write(command_str.encode('ascii'))
            self.ser.flush()  # Ensures immediate transmission
            print(f"Sent NMEA command: {command_str.strip()}")
            return True # Indicate Success
        except Exception as e:
            print(f"Error sending NMEA command: {e}")
            return False # Indicate Failure

    def send_motor_command(self, port_val, starboard_val):
        """
        Send motor commands to the Dynautics controller in NMEA format.
        Format: $CCMCO,0.0,<port_val>,<starboard_val>*<checksum>
        """
        if not (-100 <= port_val <= 100 and -100 <= starboard_val <= 100):
            print("⚠️ ERROR: Motor values out of range")
            return False

        command_body = f"CCMCO,0.0,{port_val:.2f},{starboard_val:.2f}"
        return self.send_command(command_body)


    def calculate_nmea_checksum(self, sentence):
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return checksum

    def _read_loop(self):
        """Continuously read from the serial port and process incoming data."""
        buffer = ""
        while self.running:
            try:
                # Read data from the serial port (non-blocking read)
                if self.ser.in_waiting:  # Check if data is available
                    data = self.ser.read(self.ser.in_waiting).decode('ascii', errors='ignore')
                    buffer += data

                    # Process complete lines (assuming \r\n terminated messages)
                    while '\r\n' in buffer:
                        line, buffer = buffer.split('\r\n', 1)
                        line = line.strip()
                        if line:
                            self._handle_nmea_message(line)

            except serial.SerialException as e:  # Catch Serial-specific exceptions
                print(f"Serial port error: {e}")
                self.running = False  # Stop the loop on serial error
                break  # Exit the loop
            except Exception as e:
                print(f"Error reading from serial port: {e}")
                time.sleep(0.1)  # Short delay on non-serial errors
            time.sleep(0.01)


        
    def _handle_nmea_message(self, message):
        print(f"Received NMEA message: {message}")
        if "HDT" in message:  # Check if the message contains "HDT"
            print("!!! Received an HDT message !!!")
        self.nmea_queue.put(message)

    def stop(self):
        """Stop the read loop and close the serial port."""
        self.running = False
        if hasattr(self, "read_thread") and self.read_thread.is_alive():
            self.read_thread.join(timeout=1)  # Avoid indefinite blocking
        if self.ser: # check if self.ser has been initialised
            self.ser.close()
        print("Serial port closed.")

if __name__ == '__main__':
    # --- Example Usage (for testing DynauticsController independently) ---
    controller = DynauticsController(port='COM4', baudrate=115200, timeout=1)
    controller.start_reading()  #  Start the reading thread!

    try:
        # Send a test motor command
        controller.send_motor_command(20, -20)
        time.sleep(1) #Give it a second to send

        # Keep the main thread alive to allow the reading thread to run
        # You can add more test commands here, or just wait.
        time.sleep(60)  # Run for 60 seconds

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        controller.stop() # Ensure serial port is closed