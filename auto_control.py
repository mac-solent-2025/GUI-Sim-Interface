from simple_pid import PID
import numpy as np

# PID Controller Parameters (Kp, Ki, Kd):
#   Kp (Proportional Gain): Determines how strongly the controller reacts to the current error.
#   Ki (Integral Gain): Addresses accumulated past errors by integrating the error over time.
#   Kd (Derivative Gain): Predicts future error based on the rate of change, helping to dampen oscillations.

# Tunable Parameters for AutoController
DEFAULT_HEADING_PID_TUNINGS = (2.0, 0.1, 0.5)
DEFAULT_SPEED_PID_TUNINGS = (20.0, 0.0, 0.8)
DEFAULT_SPEED_PID_OUTPUT_LIMITS = (-30, 100)
DEFAULT_BASE_THRUST = 100
DEFAULT_SAMPLE_TIME = 0.05

# Optional PID tuning adjustment parameters
HEADING_ERROR_THRESHOLD = 70      # degrees threshold for tuning adjustment
MINIMUM_BASE_THRUST = 0          # Minimum forward thrust
MAX_THRUST = 100                  # Maximum thrust for each side
MIN_THRUST = -100                 # Minimum thrust for each side

# New global PID tunings for heading control based on error magnitude.
# Use milder gains when error is small and more aggressive gains when error is large.
HEADING_PID_TUNINGS_INSIDE_THRESHOLD = (2.0, 0.0, 1.2)    # For errors below the threshold
HEADING_PID_TUNINGS_OUTSIDE_THRESHOLD = (3.0, 0.0, 1.7)   # For errors above the threshold

# Global Speed Control Parameters
CRUISE_SPEED = 5.0         # Desired speed when not near a waypoint (in knots)
SLOWING_DISTANCE = 12.0    # Distance (in meters) within which to slow down
SLOWING_SPEED = 2.5        # Desired speed (in knots) when within SLOWING_DISTANCE
APPROACHING_DISTANCE = 5.0 # Distance (in meters) within which to further reduce speed
APPROACHING_SPEED = 1.0    # Desired speed (in knots) when within APPROACHING_DISTANCE

class AutoController:
    def __init__(self,
                 heading_pid_tunings=DEFAULT_HEADING_PID_TUNINGS,
                 speed_pid_tunings=DEFAULT_SPEED_PID_TUNINGS,
                 speed_pid_output_limits=DEFAULT_SPEED_PID_OUTPUT_LIMITS,
                 base_thrust=DEFAULT_BASE_THRUST,
                 sample_time=DEFAULT_SAMPLE_TIME):
        """
        Initialize the auto control module with separate PID controllers for heading and speed.
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

    def update_auto_control(self, desired_heading, current_heading, current_speed, distance_to_waypoint, send_command_callback):
        """
        Update control commands based on current headings, speed, and distance to the next waypoint.
        
        Parameters:
            desired_heading (float): Target heading in degrees.
            current_heading (float): Current heading in degrees.
            current_speed (float): Current speed in knots.
            distance_to_waypoint (float): Distance (in meters) to the next waypoint.
            send_command_callback (callable): Function to send the control command.
        """
        error = self.compute_circular_error(desired_heading, current_heading)
        
        # Adjust heading PID tunings based on error magnitude.
        if abs(error) < HEADING_ERROR_THRESHOLD:
            self.heading_pid.tunings = HEADING_PID_TUNINGS_INSIDE_THRESHOLD
        else:
            self.heading_pid.tunings = HEADING_PID_TUNINGS_OUTSIDE_THRESHOLD
        
        correction = self.heading_pid(-error)
        
        # Determine desired speed based on the distance to the next waypoint.
        if distance_to_waypoint <= APPROACHING_DISTANCE:
            desired_speed = APPROACHING_SPEED
        elif distance_to_waypoint <= SLOWING_DISTANCE:
            desired_speed = SLOWING_SPEED
        else:
            desired_speed = CRUISE_SPEED
        
        # Set the speed PID setpoint to the desired speed.
        self.speed_pid.setpoint = desired_speed
        
        # Compute the base thrust using the speed PID.
        base = self.speed_pid(current_speed) if current_speed is not None else self.base_thrust
        
        # Impose a minimum forward thrust so the vessel keeps moving.
        if base < MINIMUM_BASE_THRUST:
            base = MINIMUM_BASE_THRUST
        
        # Calculate individual thrust commands for port and starboard sides.
        port_thrust = max(min(base + correction, MAX_THRUST), MIN_THRUST)
        starboard_thrust = max(min(base - correction, MAX_THRUST), MIN_THRUST)
        command = f"$CCMCO,0.0,{port_thrust:.2f},{starboard_thrust:.2f}"
        
        send_command_callback(command)
        
        return error, correction, base, port_thrust, starboard_thrust
