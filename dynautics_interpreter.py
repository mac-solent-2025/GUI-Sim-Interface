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

        # Start a background thread for reading NMEA messages
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def send_motor_command(self, port_val, starboard_val):
        """
        Send motor commands to the Dynautics controller.
        Values should be between -100 and 100.
        """
        # Validate input values
        if not (-100 <= port_val <= 100 and -100 <= starboard_val <= 100):
            raise ValueError("Motor values must be between -100 and 100.")
        
        # Format the command as required by your controller protocol.
        # This example assumes a simple comma-separated command.
        command_str = f"MOTOR,{port_val},{starboard_val}\n"
        try:
            self.ser.write(command_str.encode('ascii'))
            print(f"Sent command: {command_str.strip()}")
        except Exception as e:
            print(f"Error sending command: {e}")

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
        if self.read_thread.is_alive():
            self.read_thread.join()
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
