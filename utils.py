import math
import numpy as np
import carla
from longitudinal_controller import PIDLongitudinalController
from lateral_controller import PurePursuitController



# Function to calculate distance between vehicle location and a target
def find_dist_veh(vehicle_loc, target):
    """
    Calculate distance between vehicle location and a target.

    Args:
        vehicle_loc (carla.Location): Vehicle's location.
        target (carla.Actor): Target actor.

    Returns:
        float: Distance between vehicle and target.
    """
    dist = math.sqrt((target.transform.location.x - vehicle_loc.x)**2 + (target.transform.location.y - vehicle_loc.y)**2)
    return dist

# Function to get the speed of a vehicle
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

def vector(location_1, location_2):
    """
    Returns the unit vector from location_1 to location_2

        :param location_1, location_2: carla.Location objects
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]

# Function to generate control signal for the vehicle
def control_signal(vehicle, waypoint_list, longitudinal_controller: PIDLongitudinalController, lateral_controller: PurePursuitController, speed, waypoint, past_steering):
    """
    Generate control signal for the vehicle.

    Args:
        vehicle (carla.Vehicle): Vehicle object.
        waypoint_list (list): List of waypoints.
        longitudinal_controller (PIDLongitudinalController): Longitudinal controller object.
        lateral_controller (PurePursuitController): Lateral controller object.
        speed (float): Current speed of the vehicle.
        waypoint (carla.Waypoint): Current waypoint.
        past_steering (float): Previous steering angle.

    Returns:
        carla.VehicleControl: Control commands for the vehicle.
    """
    # Setting maximum values for throttle, brake, and steering
    max_throttle = 0.75
    max_brake = 0.8
    max_steering = 0.8
    control = carla.VehicleControl()

    # Calculating acceleration based on longitudinal controller output
    acceleration = longitudinal_controller.run_step(speed)
    

    # Calculating vehicle properties
    veh_transform = vehicle.get_transform()
    veh_location = vehicle.get_location()
    veh_vel = vehicle.get_velocity()
    vf = np.sqrt(veh_vel.x**2 + veh_vel.y**2)
    vf = np.fmax(np.fmin(vf, 2.5), 0.1)

    # Finding target waypoint index and calculating lookahead distance
    min_index, tx, ty, dist = lateral_controller.get_target_wp_index(veh_location, waypoint_list)
    ld = lateral_controller.get_lookahead_dist(vf, min_index, waypoint_list, dist)

    # Calculating alpha and lateral error
    yaw = np.radians(veh_transform.rotation.yaw)
    alpha = math.atan2(ty - veh_location.y, tx - veh_location.x) - yaw

    if math.isnan(alpha):
        alpha = alpha_prev
    else:
        alpha_prev = alpha

    e = np.sin(alpha) * ld

    # # Calculating current steering angle
    # current_steering, R = lateral_controller.calc_steering_angle(alpha, ld) #new

    # if acceleration >= 0.0:
    #     control.throttle = min(acceleration, max_throttle)
    #     control.brake = 0.0
    # else:
    #     control.throttle = 0.0
    #     control.brake = min(abs(acceleration), max_brake)



    #new from here
    # Assume you have access to the current_steering and R values
    current_steering, R = lateral_controller.calc_steering_angle(alpha, ld)  # new

    # Define threshold values for determining a turn
    straight_road_threshold = 100  # Adjust this value according to your scenario

    if 1/abs(R) > straight_road_threshold:
        # Vehicle is on a straight road
        print(f'Curvature when moving on straight line {R = }')
        if acceleration >= 0.0:
            control.throttle = min(acceleration, max_throttle)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = 0.20#min(abs(acceleration), max_brake)
    else:
        # Vehicle is on a turn
        # Adjust throttle and brake accordingly
        # You might want to decrease throttle or apply brakes based on the turn radius
        # Example: reduce throttle or apply brakes if R is less than a certain value
        print(f'Curvature when moving on curve {R = }')
        if acceleration >= 0.0:
            control.throttle = min(acceleration, max_throttle * 0.50)  # Adjust the factor according to your scenario
            control.brake =  max_brake * 0.40
        else:
            control.throttle = 0.05
            control.brake = min(abs(acceleration), max_brake * 0.90)  # Adjust the factor according to your scenario

    #to here

    # Applying constraints to steering angle changes
    if current_steering > past_steering + 0.1:
        current_steering = past_steering + 0.1
    elif current_steering < past_steering - 0.1:
        current_steering = past_steering - 0.1

    if current_steering >= 0:
        steering = min(max_steering, current_steering)
    else:
        steering = max(-max_steering, current_steering)

    control.steer = steering

    # Setting other control properties
    control.hand_brake = False
    control.manual_gear_shift = False

    return (control, R) #new