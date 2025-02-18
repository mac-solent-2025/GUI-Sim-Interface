from simple_pid import PID
import numpy as np

class AutoController:
    def __init__(self,
                 heading_pid_tunings=(2.0, 0.1, 0.5),
                 speed_pid_tunings=(30.0, 0.0, 0.8),
                 speed_pid_output_limits=(-30, 100),
                 base_thrust=10,
                 sample_time=0.05):
        """
        Initialize the auto control module with separate PID controllers for heading and speed.
        
        heading_pid_tunings: tuple (Kp, Ki, Kd) for heading control.
        speed_pid_tunings: tuple (Kp, Ki, Kd) for speed control.
        speed_pid_output_limits: output limits for the speed controller.
        base_thrust: default forward thrust.
        sample_time: PID sample time.
        """
        # The heading PID drives the circular error (desired - current) to zero.
        self.heading_pid = PID(*heading_pid_tunings, setpoint=0, sample_time=sample_time)
        # The speed PID uses the desired speed as its setpoint.
        self.speed_pid = PID(*speed_pid_tunings, setpoint=0, sample_time=sample_time)
        self.speed_pid.output_limits = speed_pid_output_limits
        self.base_thrust = base_thrust
        self.auto_running = False

    def start_auto_control(self):
        # Set a desired speed (for instance, from a user input field)
        desired_speed = 2.0  # Replace with your desired value or input reading.
        self.auto_controller.start_auto_control(desired_speed)
        self.auto_control_running = True


    def stop_auto_control(self):
        """
        Stop automatic control.
        """
        self.auto_running = False

    def compute_circular_error(self, desired, current):
        """
        Compute the smallest signed angle difference (in degrees) between desired and current headings.
        The result is in the range [-180, +180].
        """
        diff_rad = np.deg2rad(desired - current)
        error = np.rad2deg(np.arctan2(np.sin(diff_rad), np.cos(diff_rad)))
        return error

    def update_auto_control(self, desired_heading, current_heading, current_speed, send_command_callback):
        error = self.compute_circular_error(desired_heading, current_heading)
        
        # Optionally adjust PID tunings based on error magnitude.
        # (You can tweak these values based on testing.)
        if abs(error) < 20:
            self.heading_pid.tunings = (1, 0, 0)
        else:
            self.heading_pid.tunings = (1, 0, 0)
        
        correction = self.heading_pid(-error)
        
        # Instead of setting base thrust to 0 if error > 45, use a minimum forward thrust.
        # For example, always use the speed PID output if available, or a fixed base thrust.
        base = self.speed_pid(current_speed) if current_speed is not None else self.base_thrust
        # Alternatively, ignore the error condition:
        # base = self.base_thrust
        
        # You might want to impose a lower bound on base thrust (e.g., 30) so the boat keeps moving.
        if base < 30:
            base = 30
        
        port_thrust = max(min(base + correction, 50), -50)
        starboard_thrust = max(min(base - correction, 50), -50)
        command = f"$CCMCO,0.0,{port_thrust:.2f},{starboard_thrust:.2f}"
        
        send_command_callback(command)
        
        return error, correction, base, port_thrust, starboard_thrust
