# MAC Simulator Controller

MAC Simulator Controller serves as the initial interface between the Dynautics autopilot device and ShipSim. It enables control of the simulated vessel without the use of H-Term by directly communicating with the Dynautics device over a serial connection.

The application is designed to work with a serial connection using port COM4 at a baud rate of 115200. The Dynautics controller, connected via this serial link, in turn communicates with the ShipSim simulators, sending simulated navigation data back to the application.

## Features

- **Graphical User Interface (GUI):** An intuitive interface built with Tkinter for ease of control.
- **Serial Communication:** Directly sends commands to the Dynautics autopilot device.
- **Engine Control:** Allows setting thrust values for the port and starboard engines (values between -100 and +100).
- **Navigation Data:** Starts and stops the stream of simulated navigation data, and parses key NMEA sentences for display.

## Dependencies

This project requires Python along with the following packages:
- [pyserial](https://pypi.org/project/pyserial/)
- [pynmea2](https://pypi.org/project/pynmea2/)

Please refer to the [requirements.txt](requirements.txt) file for the complete list.

## Installation

1. **Clone the repository:**

   ```bash
   git clone https://github.com/yourusername/MAC-Simulator-Controller.git
Navigate to the project directory:

bash
Copy
cd MAC-Simulator-Controller
Install the required dependencies:

bash
Copy
pip install -r requirements.txt
Usage
Launch the application using Python. The GUI will open, allowing you to select the desired serial port (default is COM4) and input engine thrust values. The application also enables starting and stopping the navigation data stream from the ShipSim simulators.

To run the application, execute:

bash
Copy
python main.py
Note: Replace main.py with the appropriate filename if different.

## Contributing
Contributions are welcome. Please adhere to the repository guidelines and submit a pull request for any improvements or fixes.

## License
A licensing section will be added at a later stage.
