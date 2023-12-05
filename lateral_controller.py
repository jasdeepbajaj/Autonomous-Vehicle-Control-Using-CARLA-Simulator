import math
import numpy as np



# Class for Pure Pursuit Controller
class PurePursuitController():
    """
    Pure Pursuit Controller for lateral control.
    """
    def __init__(self, L, Kdd):
        """
        Initialize Pure Pursuit controller parameters.

        Args:
            L (float): Vehicle's wheelbase.
            Kdd (float): Gain for calculating lookahead distance.
        """
        # Initializing Pure Pursuit controller parameters
        self.L = L
        self.Kdd = Kdd
        self.delta_prev = 0

    def calc_steering_angle(self, alpha, ld):
        # Calculating steering angle based on alpha and lookahead distance
        R = ld/2*np.sin(alpha) #new
        delta = math.atan2(2 * self.L * np.sin(alpha), ld)
        delta = np.fmax(np.fmin(delta, 1.0), -1.0)
        if math.isnan(delta):
            delta = self.delta_prev
        else:
            self.delta_prev = delta
        return delta, R #new

    def get_target_wp_index(self, veh_location, waypoint_list):
        # Finding the index of the target waypoint in the list
        dxl, dyl = [], []
        for i in range(len(waypoint_list)):
            dx = abs(veh_location.x - waypoint_list[i][0])
            dxl.append(dx)
            dy = abs(veh_location.y - waypoint_list[i][1])
            dyl.append(dy)

        dist = np.hypot(dxl, dyl)
        idx = np.argmin(dist) + 15

        if idx < len(waypoint_list):
            tx = waypoint_list[idx][0]
            ty = waypoint_list[idx][1]
        else:
            tx = waypoint_list[-1][0]
            ty = waypoint_list[-1][1]

        return idx, tx, ty, dist

    def get_lookahead_dist(self, vf, idx, waypoint_list, dist):
        # Calculating lookahead distance
        ld = self.Kdd * vf
        return ld