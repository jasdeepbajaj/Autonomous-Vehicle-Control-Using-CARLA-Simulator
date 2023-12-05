from collections import deque
import numpy as np
import math

def get_speed(vehicle):
    """
    Get the speed of a vehicle.

    Args:
        vehicle (carla.Vehicle): Vehicle object.

    Returns:
        float: Speed of the vehicle in km/h.
    """
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

# Class for PID Longitudinal Controller
class PIDLongitudinalController():
    """
    PID Longitudinal Controller for vehicle speed control.
    """
    def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Initialize PID controller parameters and error buffer.

        Args:
            vehicle (carla.Vehicle): Vehicle object.
            K_P (float, optional): Proportional gain. Defaults to 1.0.
            K_I (float, optional): Integral gain. Defaults to 0.0.
            K_D (float, optional): Derivative gain. Defaults to 0.0.
            dt (float, optional): Time step. Defaults to 0.03.
        """
        # Initializing PID controller parameters and error buffer
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = deque(maxlen=10)

    def run_step(self, target_speed, debug=False):
        """
        Execute a step of PID control based on target speed and current speed.

        Args:
            target_speed (float): Target speed in km/h.
            debug (bool, optional): Debug mode flag. Defaults to False.

        Returns:
            float: Control action output.
        """
        # Running PID control step based on target speed and current speed
        current_speed = get_speed(self._vehicle)
        if debug:
            print('Current speed = {}'.format(current_speed))
        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
        # Calculating PID control action
        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)