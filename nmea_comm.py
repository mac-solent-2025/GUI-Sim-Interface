import serial

class NMEAComm:
    def __init__(self, port="COM4", baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def open(self):
        if self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                # Send the manual control command upon opening the port.
                self.send_command("$CCAPM,0,32,0,80")
            except serial.SerialException as e:
                raise Exception(f"Failed to open serial port {self.port}: {e}")

    def send_command(self, command):
        if self.ser is None:
            self.open()
        checksum = 0
        for char in command[1:]:
            checksum ^= ord(char)
        command_str = f"{command}*{checksum:02X}\r\n"
        self.ser.write(command_str.encode())
        print(f"Sent: {command_str.strip()}")

    def available(self):
        if self.ser:
            return self.ser.in_waiting > 0
        return False

    def read_line(self):
        if self.ser:
            return self.ser.readline().decode(errors='ignore').strip()
        return None

    def read_all(self):
        lines = []
        while self.available():
            line = self.read_line()
            if line:
                lines.append(line)
        return lines
