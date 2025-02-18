from simple_pid import PID
import numpy as np

# PID Controller Parameters (Kp, Ki, Kd):
#   Kp (Proportional Gain): Determines how strongly the controller reacts to the current error.
#   Ki (Integral Gain): Addresses accumulated past errors by integrating the error over time.
#   Kd (Derivative Gain): Predicts future error based on the rate of change, helping to dampen oscillations.


# Tunable Parameters for AutoController
DEFAULT_HEADING_PID_TUNINGS = (2.0, 0.1, 0.5)
DEFAULT_SPEED_PID_TUNINGS = (30.0, 0.0, 0.8)
DEFAULT_SPEED_PID_OUTPUT_LIMITS = (-30, 100)
DEFAULT_BASE_THRUST = 70
DEFAULT_SAMPLE_TIME = 0.05

# Optional PID tuning adjustment parameters
HEADING_ERROR_THRESHOLD = 70      # degrees threshold for tuning adjustment
MINIMUM_BASE_THRUST = 10          # Minimum forward thrust
MAX_THRUST = 100                   # Maximum thrust for each side
MIN_THRUST = -100                  # Minimum thrust for each side

# New global PID tunings for heading control based on error magnitude.
# Use milder gains when error is small and more aggressive gains when error is large.
HEADING_PID_TUNINGS_INSIDE_THRESHOLD = (2.0, 0.0, 1.0)    # For errors below the threshold
HEADING_PID_TUNINGS_OUTSIDE_THRESHOLD = (4.0, 0.0, 1.5)   # For errors above the threshold

DESIRED_SPEED = 5.0 


class AutoController:
    def __init__(self,
                 heading_pid_tunings=DEFAULT_HEADING_PID_TUNINGS,
                 speed_pid_tunings=DEFAULT_SPEED_PID_TUNINGS,
                 speed_pid_output_limits=DEFAULT_SPEED_PID_OUTPUT_LIMITS,
                 base_thrust=DEFAULT_BASE_THRUST,
                 sample_time=DEFAULT_SAMPLE_TIME):
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
        
        # Adjust PID tunings based on error magnitude.
        if abs(error) < HEADING_ERROR_THRESHOLD:
            self.heading_pid.tunings = HEADING_PID_TUNINGS_INSIDE_THRESHOLD
        else:
            self.heading_pid.tunings = HEADING_PID_TUNINGS_OUTSIDE_THRESHOLD
        
        correction = self.heading_pid(-error)
        
        # *** Set the speed PID setpoint to your desired speed ***
        self.speed_pid.setpoint = DESIRED_SPEED
        
        # Use speed PID output if available, or fall back to the base thrust.
        base = self.speed_pid(current_speed) if current_speed is not None else self.base_thrust
        
        # Impose a minimum forward thrust so the vessel keeps moving.
        if base < MINIMUM_BASE_THRUST:
            base = MINIMUM_BASE_THRUST
        
        port_thrust = max(min(base + correction, MAX_THRUST), MIN_THRUST)
        starboard_thrust = max(min(base - correction, MAX_THRUST), MIN_THRUST)
        command = f"$CCMCO,0.0,{port_thrust:.2f},{starboard_thrust:.2f}"
        
        send_command_callback(command)
        
        return error, correction, base, port_thrust, starboard_thrust

