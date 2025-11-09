"""
dynamics.py

This module provides vehicles' dynamics and decisions based on IDM + MOBIL models.
Author: Júnior Anderson Rodrigues da Silva
Email: juniorars@gmail.com
Date: 2025-05-01
"""
from __future__ import annotations
import numpy as np
from dataclasses import dataclass
from self_driving_sim.config import Config

@dataclass
class Waypoint:
    def __init__(self, x, y, theta, kappa, v, s):
        self.x = x
        self.y = y
        self.theta = theta
        self.kappa = kappa
        self.v = v
        self.s = s

"""
PolynomialTrajectory class
This class computes the polynomial trajectory for a vehicle based on the starting
waypoint, the time duration T, and the acceleration. The trajectory is computed
based on the polynomial parameters.
"""
class PolynomialTrajectory:
    def __init__(self, waypoint_start: Waypoint, acc: float,
                 delta_y: float, time_step: float):
        self.waypoint_start = waypoint_start
        self.T = 3.0
        self.acc = acc
        self.delta_y = delta_y
        self.control_idx = 0
        self.is_finished = False
        self.traj = self.comp_trajectory(time_step)

    def compute_poly_params(self,  acceleration: float, delta_y: float) -> dict:
        """
        Computes the polynomial parameters for the trajectory between two waypoints.
        The trajectory is computed based on the starting and ending waypoints, the
        time duration T, and the acceleration.

        Args:
            waypoint_start (Waypoint): The starting waypoint.
            waypoint_end (Waypoint)  : The ending waypoint.
            T (float)               : The time duration for the trajectory.
            acceleration (float)    : The acceleration for the trajectory.

        Returns:
            dict: The polynomial parameters for the trajectory.
        """
        acc0 = 0.0
        v0 = self.waypoint_start.v
        vf = v0 + acceleration * (1)
        accf = 0.0
        a0 = 0.0
        a1 = v0
        a2 = acc0/2
        a3 = (3*vf - accf*self.T - 3*a1 - 4*a2*self.T)/(3*self.T**2)
        a4 = (accf - 2*a2 - 6*a3*self.T)/(12*self.T**2)

        vu0 = 0.0
        vuf = 0.0
        acc_u0 = 0.0
        uf = delta_y
        b0 = 0.0
        b1 = vu0
        b2 = acc_u0/2
        b5 = (-6*b0/self.T -3*b1 - b2*self.T - 3*vuf + 6*uf/self.T)/(self.T**4)
        b4 = (-vuf + b1 + b2*self.T -5*b5*self.T**4)/(2*self.T**3)
        b3 = (-2*b2 - 12*b4*self.T**2 - 20*b5*self.T**3)/(6*self.T)
        
        poly_param = {"a0" : a0, "a1" : a1, "a2" : a2, "a3" : a3, "a4" : a4,
                    "b0" : b0, "b1" : b1, "b2" : b2, "b3" : b3, "b4" : b4, "b5" : b5}
        return poly_param
    
    def get_arclength_curvature(self, path):
        """
        Calculates the curvature and arclength based on the point locations.
        The curvature is calculated using the heading and magnitude of the path vectors.
        The arclength is calculated using the cumulative sum of the magnitudes.

        Args:
            path (numpy.ndarray): X and Y position of points, where each line is a (X, Y) pair.

        Returns:
            tuple: kappa (curvature) and arclength (arclength from first point to each point).
        """
        # Get vector between adjacent points
        vector = np.diff(path[:, 0:2], axis=0)

        # Get heading and magnitude of path vectors
        theta = np.arctan2(vector[:,1], vector[:,0])
        magnitude = np.sqrt(((vector[:,0]**2 + vector[:,1]**2)))

        # Get heading variation
        dtheta = np.diff(theta);

        # Clip between -pi and pi
        dtheta = np.mod(dtheta + np.pi, 2 * np.pi) - np.pi

        # Calculate curvature
        kappa_mag = np.sqrt(magnitude[0:len(magnitude)-1] * magnitude[1:len(magnitude)])
        kappa = 2 * np.sin(dtheta / 2) / kappa_mag

        # Calculate arc length
        arclength = np.concatenate(( [0], np.cumsum(magnitude) ))

        # Initial and end curvature calculation
        #     Initial: Solve for kappa and dkappa using 2nd and 3rd points
        A = ([1, 0],\
            [1, magnitude[1]])
        b = kappa[0:2]
        kappa_1 = np.array([1, -magnitude[0]]).dot(np.linalg.lstsq(A,b,rcond=None)[0])

        #     Final: Solve for kappa and dkappa using the two last available points
        A = ([1, -magnitude[len(magnitude)-2]],\
            [1, 0])
        b = kappa[len(kappa)-2:len(kappa)]
        kappa_end = np.array([1, magnitude[len(magnitude)-1]]).dot( np.linalg.lstsq(A,b,rcond=None)[0])

        #     Concatenate them into one vector
        kappa = np.concatenate(( ([kappa_1]), kappa, ([kappa_end]) ))

        return arclength, kappa

    def comp_trajectory(self, time_step: float) -> dict:
        """
        Computes the controls input for the trajectory based on the polynomial parameters
        and the time step. The controls input is computed based on the polynomial parameters.

        Args:
            poly_param (list)      : The polynomial parameters for the trajectory.
            time_step (float)      : The time step for the trajectory.

        Returns:
            dict                   : The controls input for the trajectory.
        """
        self.traj = []
        poly_param = self.compute_poly_params(self.acc, self.delta_y)
        pos = []
        x_acc = []
        for t in np.arange(0, self.T, time_step):
            x_pos = poly_param["a0"] + poly_param["a1"]*t + poly_param["a2"]*t**2 + \
                poly_param["a3"]*t**3 + poly_param["a4"]*t**4
            y_pos = poly_param["b0"] + poly_param["b1"]*t + poly_param["b2"]*t**2 + \
                poly_param["b3"]*t**3 + poly_param["b4"]*t**4 + poly_param["b5"]*t**5
            pos.append([x_pos, y_pos])
            x_acc.append(2 * poly_param["a2"] + 6*poly_param["a3"]*t + 12*poly_param["a4"]*t**2)

        _, curvature = self.get_arclength_curvature(np.array(pos))
        for i in range(len(pos)):
            self.traj.append({"acc": x_acc[i], "curvature": curvature[i]})
        return self.traj
    
    def get_lc_controls(self):
        """
        Returns the control input for the trajectory. The control input is computed
        based on the polynomial parameters and the time step.

        Returns:
            dict: The control input for the trajectory.
        """
        if self.control_idx < len(self.traj):
            control_input = self.traj[self.control_idx]
            self.control_idx += 1
        else:
            self.is_finished = True
            control_input = {"acc": 0.0, "curvature": 0.0}
        return control_input

class Vehicle:
    block_all_lc = False
    number_of_vehicles = 0
    controlled_vehicle_id = 0
    def __init__(self, cfg: Config, vehicle_type: str, vehicle_params: dict = None):
        Vehicle.number_of_vehicles += 1
        self.id              = Vehicle.number_of_vehicles
        self.lane_width      = cfg.lane_width
        self.number_of_lanes = cfg.number_of_lanes
        self.time_step       = cfg.time_step
        self.speed_limit     = cfg.speed_limit
        self.lon_pos_min     = cfg.x_min
        self.lon_pos_max     = cfg.x_max
        Vehicle.controlled_vehicle_id = cfg.controlled_vehicle_id

        self.is_changing_lane= False
        if vehicle_params is not None:
            self.features, self.idm_params, self.mobil_params, state = \
                self.customize_vehicle(vehicle_params)
        else:
            self.features        = self.features_factory(vehicle_type)
            self.idm_params      = self.idm_params_factory(vehicle_type)
            self.mobil_params    = self.mobil_params_factory(vehicle_type)
            state = self.state_factory()
        self.features["track_id"] = self.id
        self.surr_vehicles   = self.create_surroundings()
        self.lon_pos         = state["lon_pos"]
        self.lat_pos         = state["lat_pos"]
        self.heading         = state["heading"]
        self.curvature       = state["curvature"]
        self.speed           = state["speed"]
        self.lane            = state["lane"]

    @staticmethod    
    def restart_static_vars():
        """
        Restart static variables of Vehicle class.
        """
        Vehicle.number_of_vehicles = 0
        Vehicle.block_all_lc = False

    def features_factory(self, vehicle_type: str) -> dict:
        """
        Creates the vehicle features based on the vehicle type. The features are:
            - track_id: The track id of the vehicle.
            - length: The length of the vehicle.
            - width: The width of the vehicle.
        The length and width are generated based on the vehicle type.

        Args:
            vehicle_type (str): The type of the vehicle. The options are:
                - car
                - bus
                - truck
                - motorcycle

        Returns:
            dict: The features of the vehicle.
        """
        if vehicle_type == "car":
            length = 3.0 + np.random.uniform(0., 1.0)
            width = 1.5 + np.random.uniform(0., 0.5)
        elif vehicle_type == "bus":
            length = 10.0 + np.random.uniform(0., 2.0)
            width = 2.5 + np.random.uniform(-0.3, 0.3)
        elif vehicle_type == "truck":
            length = 8.0 + np.random.uniform(0., 2.0)
            width = 2.5 + np.random.uniform(-0.3, 0.3)
        elif vehicle_type == "motorcycle":
            length = 1.5
            width = 0.5
        features = {
            "type": vehicle_type,
            "length": length,
            "width": width,
        }
        return features
    
    def idm_params_factory(self, vehicle_type: str) -> dict:
        """
        Creates the IDM parameters based on the vehicle type. The parameters are:
            - desired_speed: The desired speed of the vehicle.
            - jam_distance: The jam distance of the vehicle.
            - desired_time_gap: The desired time gap of the vehicle.
            - max_acceleration: The maximum acceleration of the vehicle.
            - desired_deceleration: The desired deceleration of the vehicle.
            - delta: The delta of the vehicle.
            - safe_brake: The safe brake of the vehicle.
        The parameters are generated based on the vehicle type.

        Args:
            vehicle_type (str): The type of the vehicle. The options are:
                - car
                - bus
                - truck
                - motorcycle

        Returns:
            dict: The IDM parameters of the vehicle.
        """
        if vehicle_type == "car" or vehicle_type == "motorcycle":
            DESIRED_SPEED = self.speed_limit - np.random.uniform(0., 2.0)
            MAX_ACCELERATION = np.random.uniform(2.0, 3.0)
            DESIRED_DECELERATION = 3.0
            SAFE_BRAKE = 4.0
        elif vehicle_type == "bus" or vehicle_type == "truck":
            DESIRED_SPEED = self.speed_limit * 0.7 - np.random.uniform(0., 3.0)
            MAX_ACCELERATION = np.random.uniform(1.0, 2.0)
            DESIRED_DECELERATION = 2.5
            SAFE_BRAKE = 3.0

        JAM_DISTANCE = np.random.uniform(1.0, 2.0)
        DESIRED_TIME_GAP = np.random.uniform(1.0, 3.0)
        DELTA = 4
        
        idm_params = {
            "desired_speed" : DESIRED_SPEED,
            "jam_distance" : JAM_DISTANCE,
            "desired_time_gap" : DESIRED_TIME_GAP,
            "max_acceleration" : MAX_ACCELERATION,
            "desired_deceleration" : DESIRED_DECELERATION,
            "delta" : DELTA,
            "safe_brake" : SAFE_BRAKE
        }
        return idm_params
    
    def mobil_params_factory(self, vehicle_type: str) -> dict:
        """
        Creates the MOBIL parameters based on the vehicle type. The parameters are:
            - politeness: The politeness of the vehicle.
            - acc_threshold: The acceleration threshold of the vehicle.
            - is_lc_allowed: boolean indicating if lane change is allowed.
        The parameters are generated based on the vehicle type.

        Args:
            vehicle_type (str): The type of the vehicle. The options are:
                - car
                - bus
                - truck
                - motorcycle

        Returns:
            dict: The MOBIL parameters of the vehicle.
        """
        if vehicle_type == "car" or vehicle_type == "motorcycle":
            POLITENESS = np.random.uniform(0., 1.0)
            ACC_THRESHOLD = np.random.uniform(0., 0.5)
        elif vehicle_type == "bus" or vehicle_type == "truck":
            POLITENESS = np.random.uniform(0., 1.0)
            ACC_THRESHOLD = np.random.uniform(0., 0.25)
        mobil_params = {
            "politeness" : POLITENESS,
            "acc_threshold" : ACC_THRESHOLD,
            "is_lc_allowed" : True
        }
        return mobil_params

    def state_factory(self) -> dict:
        """
        Creates the initial state of the vehicle. The state is generated based on
        the vehicle type. The state is generated based on the lane width and the
        number of lanes.
        The state is generated based on the following parameters:
            - lon_pos: The longitudinal position of the vehicle.
            - lat_pos: The lateral position of the vehicle.
            - heading: The heading of the vehicle.
            - speed: The speed of the vehicle.
            - lane: The lane of the vehicle.
        
        Args:
            vehicle_type (str): The type of the vehicle. The options are:
                - car
                - bus
                - truck
                - motorcycle
        Returns:
            dict: The state of the vehicle.
        """
        lane = np.random.choice(np.arange(0, self.number_of_lanes))
        state = {
            "lon_pos": np.random.uniform(self.lon_pos_min, self.lon_pos_max),
            "lat_pos": lane * self.lane_width,
            "heading": 0.0,
            "curvature": 0.0,
            "speed": self.idm_params["desired_speed"] + np.random.normal(loc=0.0, scale=1.0),
            "lane": lane
        }
        return state
    
    def customize_vehicle(self, vehicle_params:dict) -> tuple:
        """
        Creates a customized vehicle based on the vehicle parameters. The parameters
        are generated based on the vehicle type. The parameters are:
            - features: The features of the vehicle.
            - idm_params: The IDM parameters of the vehicle.
            - mobil_params: The MOBIL parameters of the vehicle.
            - state: The state of the vehicle.
        
        Args:
            vehicle_params (dict): The parameters of the vehicle.
        
        Returns:
            tuple: The features, idm_params, mobil_params and state of the vehicle.
        """
        features = vehicle_params["features"]
        idm_params = vehicle_params["idm"]
        mobil_params = vehicle_params["mobil"]
        state_params = vehicle_params["state"]
        return features, idm_params, mobil_params, state_params
    
    def create_surroundings(self) -> dict:
        """
        Creates the vehicle surroundings. Each key in surr_vehicles represents
        a vehicle in the surroundings. The value is the track_id of the vehicle.
        The keys are:
            - front_same_lane
            - rear_same_lane
            - front_left_lane
            - rear_left_lane
            - front_right_lane
            - rear_right_lane
        The values are the track_id of the vehicle in that position. If there is no
        vehicle in that position, the value is -1.
        """
        surr_vehicles = {
            "front_same_lane": -1,
            "rear_same_lane": -1,
            "front_left_lane": -1,
            "rear_left_lane": -1,
            "front_right_lane": -1,
            "rear_right_lane": -1
        }
        return surr_vehicles
         
    def get_current_lane(self, lat_pos: float) -> int:
        """
        Returns the lane of a vehicle based on its lateral position.
        In this representation, lane 0 is the leftmost lane.

        Args:
            lat_pos (float): Lateral position of the vehicle.

        Returns:
            int: The vehicles' current lane.            
        """
        lanes_centers = np.arange(0,
                                  self.number_of_lanes * self.lane_width,
                                  self.lane_width)
        return np.argmin(abs(lanes_centers - lat_pos))

    def apply_control_inputs(self, curvature: float, accel: float) -> None:
        """
        Returns the next state of the vehicle based on model inputs.

        Args:
            curvature (float)       : Curvature.
            accel (float)           : Acceleration.
            time_step (float)       : Time step.

        Returns:
            dictionay: The updated state of the vehicle.
        """
        self.lon_pos     = self.lon_pos + self.speed * np.cos(self.heading) * self.time_step
        self.lat_pos     = self.lat_pos + self.speed * np.sin(self.heading) * self.time_step
        self.heading     = self.heading + curvature * self.speed * self.time_step
        self.speed       = max(0., self.speed + accel * self.time_step)
        self.lane        = self.get_current_lane(self.lat_pos)

    def apply_ego_control_inputs(self, curv_derivative: float, accel: float) -> None:
        """
        Returns the next state of the vehicle based on model inputs.

        Args:
            curvature (float)       : Curvature.
            accel (float)           : Acceleration.
            time_step (float)       : Time step.

        Returns:
            dictionay: The updated state of the vehicle.
        """
        self.lon_pos     = self.lon_pos + self.speed * np.cos(self.heading) * self.time_step
        self.lat_pos     = self.lat_pos + self.speed * np.sin(self.heading) * self.time_step
        self.heading     = self.heading + self.curvature * self.speed * self.time_step
        self.curvature   = self.curvature + curv_derivative * self.time_step
        self.speed       = max(0., self.speed + accel * self.time_step)
        self.lane        = self.get_current_lane(self.lat_pos)

    def make_step(self, vehicles_dict: dict, lanes: list):
        """
        Applies the control inputs to the vehicle based on the IDM and MOBIL models.
        The control inputs are computed based on the vehicle's surroundings and
        the vehicle's state.

        Args:
            vehicles_dict (dict)   : The dict containing all vehicles objects.
            lanes (list)           : The list containing all lanes.
        """
        if self.id != Vehicle.controlled_vehicle_id:
            if not self.is_changing_lane:
                next_lane, acc = mobil_model(self, vehicles_dict, lanes)
                if next_lane == "stay" or Vehicle.block_all_lc:
                    acc = idm_model(self, vehicles_dict, lanes, self.surr_vehicles["front_same_lane"])
                    curvature = 0.0
                    self.apply_control_inputs(curvature, acc)
                    return
                else:
                    self.lc_trajectory = self.make_lane_change(next_lane, acc)
                    self.is_changing_lane = True
                    Vehicle.block_all_lc = True

            if self.is_changing_lane:
                lc_controls = self.lc_trajectory.get_lc_controls()
                acc = lc_controls["acc"]
                curvature = lc_controls["curvature"]
                if self.lc_trajectory.is_finished:
                    self.is_changing_lane = False
                    Vehicle.block_all_lc = False
                    self.lc_trajectory = None

            self.apply_control_inputs(curvature, acc)

    def make_lane_change(self, target_lane: str, acc: float):
        """
        Returns the control input for a lane change based on the target lane,
        the acceleration, and the time step. The lane change is computed based
        on the polynomial trajectory.

        Args:
            target_lane (str)      : The target lane for lane change ('left' or 'right').
            acc (float)            : The acceleration for the lane change.
            time_step (float)      : The time step for the lane change.

        Returns:
            PolynomialTrajectory   : The polynomial trajectory for the lane change.
        """
        waypoint_start = Waypoint(self.lon_pos, self.lat_pos,
                                  self.heading, 0.0, self.speed, 0.0)
        delta_y = self.lane_width if target_lane == "left" else -self.lane_width
        return PolynomialTrajectory(waypoint_start, acc,
                                          delta_y, self.time_step)
        
    @staticmethod
    def update_surr_vehicles(vehicles_dict: dict) -> dict:
        """
        Updates the surroundings of all vehicles in the vehicles_dict.
        
        Args:
            vehicles_dict (dict) : The dict containing all vehicles objects.
        Returns:
            vehicles_dict (dict) : The updated dict containing all vehicles objects.
        """
        #Update the surroundings based on surroundings in front (same, left and right lanes)
        vehicles_dict = update_surr_vehicles_in_front(vehicles_dict, False)
        vehicles_dict = update_surr_vehicles_in_front(vehicles_dict, True)
        return vehicles_dict
    
    @staticmethod
    def is_collision(vehicle:Vehicle, other_vehicle:Vehicle,
                 collision_thr_lon: float, collision_thr_lat: float) -> bool: 
        if vehicle != other_vehicle:
            # Check for collision
            if abs(vehicle.lon_pos - other_vehicle.lon_pos) < \
                collision_thr_lon + (vehicle.features["length"] + other_vehicle.features["length"]) / 2 and \
                abs(vehicle.lat_pos - other_vehicle.lat_pos) < \
                collision_thr_lat + (vehicle.features["width"] + other_vehicle.features["width"]) / 2:
                # Collision detected
                return True
        return False

class ControlledVehicle(Vehicle):
    def __init__(self, cfg: Config, vehicle_id: int, vehicle_type: str, vehicle_params: dict = None):
        super().__init__(cfg, vehicle_id, vehicle_type, vehicle_params)
        self.is_changing_lane = False
        self.lc_trajectory = None

        self.m = cfg.m
        self.I_z = cfg.I_z
        self.l_f = cfg.l_f
        self.l_r = cfg.l_r
        self.C_f = cfg.C_f
        self.C_r = cfg.C_r

    def apply_control_inputs(self, curvature, accel):
        #TODO Implement the apply_control_inputs method
        pass

    def tire_forces(self, v_x, v_y, r, delta):
        # Avoid division by zero
        if v_x < 0.1:
            v_x = 0.1

        alpha_f = delta - (v_y + self.l_f * r) / v_x
        alpha_r = - (v_y - self.l_r * r) / v_x

        F_yf = self.C_f * alpha_f
        F_yr = self.C_r * alpha_r

        return F_yf, F_yr

    def derivatives(self, state, delta, F_x):
        """
        state = [v_x, v_y, r]
        delta = steering angle [rad]
        F_x = longitudinal force at CoG [N]
        """
        v_x, v_y, r = state

        F_yf, F_yr = self.tire_forces(v_x, v_y, r, delta)

        # Dynamics
        dv_x = (F_x - F_yf * np.sin(delta) + self.m * v_y * r) / self.m
        dv_y = (F_yf * np.cos(delta) + F_yr) / self.m - v_x * r
        dr = (self.l_f * F_yf * np.cos(delta) - self.l_r * F_yr) / self.I_z

        return np.array([dv_x, dv_y, dr])
    
def idm_model(vehicle: Vehicle, vehicles_dict: dict, lanes: list, leader_key: int) -> float:
    """
    Returns the acceleration of the vehicle based on IDM.

    Args:
        vehicle (Vehicle)       : The vehicle in consideration.
        vehicle_dict (dict)    : The dict containing all vehicles objects.
        lanes (list)            : The list containing all lanes.

    Returns:
        acc (float)             : The computed acceleration.
    """
    s = vehicle.lon_pos
    v = vehicle.speed
    length = vehicle.features["length"]
    #IDM must be applied in case of reaching the end of lane.
    if leader_key == -1:
        lane = vehicle.lane
        lane_length = lanes[lane]["length"]
        delta_s = lane_length - s
        delta_v = v
    #Following leader behavior.
    else:    
        leader = vehicles_dict[leader_key]
        s_leader = leader.lon_pos
        v_leader = leader.speed
        leader_length = leader.features["length"]

        length_aver = (leader_length + length)/2
        delta_s = s_leader - s - length_aver
        delta_v = v - v_leader
    
    DESIRED_SPEED = vehicle.idm_params["desired_speed"]
    JAM_DISTANCE = vehicle.idm_params["jam_distance"]
    DESIRED_TIME_GAP = vehicle.idm_params["desired_time_gap"]
    MAX_ACCELERATION = vehicle.idm_params["max_acceleration"]
    DESIRED_DECELERATION = vehicle.idm_params["desired_deceleration"]
    DELTA = vehicle.idm_params["delta"]
    SAFE_BRAKE = vehicle.idm_params["safe_brake"]

    s_prime = JAM_DISTANCE + max(0., v * DESIRED_TIME_GAP + v * delta_v/ \
                                    (2 * np.sqrt(MAX_ACCELERATION * DESIRED_DECELERATION)))
    acc = MAX_ACCELERATION*(1 - (v / DESIRED_SPEED) ** DELTA - \
                            (s_prime / delta_s) ** 2)
    acc = max(-SAFE_BRAKE, acc)

    return acc



def is_beneficial(vehicle: Vehicle, vehicles_dict: dict, target_lane: str) -> bool:
    """
    Returns whether a lane change is beneficial, i.e., the vehicle is going to
    gain some advantage.

    Args:
        vehicle (Vehicle)       : The vehicle in consideration.
        vehicle_dict (dict)     : The dict containing all vehicles objects.
        target_lane (list)      : The target lane for lane change ('left' or 'right').

    Returns:
        (bool)                  : Return True if beneficial, and False otherwise.
    """
    leader_key = vehicle.surr_vehicles["front_same_lane"]
    if leader_key == -1:
        return False
    
    new_leader_key = vehicle.surr_vehicles["front_" + target_lane + "_lane"]
    if new_leader_key == -1:
        return True if vehicle.speed < vehicle.idm_params["desired_speed"] - 1.0 else False
    
    #TODO make time to collision a function
    leader = vehicles_dict[leader_key]
    delta_leader = leader.lon_pos - vehicle.lon_pos - \
        (leader.features["length"] + vehicle.features["length"]) / 2
    ttc = delta_leader / (vehicle.speed - leader.speed)
    if ttc < 0.:
        return False
    
    new_leader = vehicles_dict[new_leader_key]
    delta_new_leader = new_leader.lon_pos - vehicle.lon_pos - \
        (new_leader.features["length"] + vehicle.features["length"]) / 2
    ttc_new = delta_new_leader / (vehicle.speed - new_leader.speed)
    if ttc_new < 0.:
        return True if vehicle.speed < vehicle.idm_params["desired_speed"] - 1.0 else False
    
    return True if ttc_new > (ttc + 1.0) and \
        (vehicle.speed < vehicle.idm_params["desired_speed"] - 1.0) else False

def is_feasible(vehicle: Vehicle, vehicles_dict: dict, target_lane: str, lanes: list) -> bool:
    """
    Returns whether a lane change is feasible.

    Args:
        vehicle (Vehicle)       : The vehicle in consideration.
        vehicles_dict (dict)    : The dict containing all vehicles objects.
        target_lane (list)      : The target lane for lane change ('left' or 'right').
        lanes (list)            : The list containing all lanes.

    Returns:
        (bool)                  : Return True if feasible, and False otherwise.
    """
    #Check for minimum speed
    #The lane change is not feasible if the vehicle is not fast enough
    if vehicle.speed < 3.0:
        return False
    
    #Check for collision with vehicles in the target lane
    vehicle_in_front_same_key = vehicle.surr_vehicles["front_same_lane"]
    vehicle_in_front_right_key = vehicle.surr_vehicles["front_" + target_lane + "_lane"]
    vehicle_in_front_left_key = vehicle.surr_vehicles["rear_" + target_lane + "_lane"]
    if vehicle_in_front_same_key != -1:
        vehicle_in_front_same = vehicles_dict[vehicle_in_front_same_key]
        #TODO Implement time to collision to vehicle in front same lane
        if Vehicle.is_collision(vehicle, vehicle_in_front_same, 1., 0.):
            return False
    if vehicle_in_front_right_key != -1:
        vehicle_in_front_right = vehicles_dict[vehicle_in_front_right_key]
        if Vehicle.is_collision(vehicle, vehicle_in_front_right, 0.5, vehicle.lane_width):
            return False
    if vehicle_in_front_left_key != -1:
        vehicle_in_front_left = vehicles_dict[vehicle_in_front_left_key]
        if Vehicle.is_collision(vehicle, vehicle_in_front_left, 0.5, vehicle.lane_width):
            return False
   
    #Check if the lane change is possible
    #The lane change is possible if the new lane is not out of bounds       
    lane_dir = 1 if target_lane == "left" else -1
    new_lane = vehicle.lane + lane_dir
    return True if (len(lanes) > new_lane) and (new_lane > 0) else False

            
def mobil_model(vehicle: Vehicle, vehicles_dict: dict, lanes: list) -> tuple:
    """
    Returns destination lane and acceleration for a lane change based on 
    MOBIL model. The lane change must be feasible and beneficial.

    Args:
        vehicle (Vehicle)       : The vehicle in consideration.
        vehicle_dict (dict)     : The dict containing all vehicles objects.
        lanes (list)            : The list containing all lanes.

    Returns:
        (tuple)                 : Destination lane ("left", "right", "stay") and acceleration.
                                  "stay" means the lane change is not possible.
    """
    is_lane_change_allowed = vehicle.mobil_params["is_lc_allowed"]
    target_lanes = ["left", "right"]
    np.random.shuffle(target_lanes)
    #We shuffle target_lanes to give some variation in the chosen lane.
    for target_lane in target_lanes:
        if is_lane_change_allowed and \
            is_feasible(vehicle, vehicles_dict, target_lane, lanes) and \
            is_beneficial(vehicle, vehicles_dict, target_lane):

            new_rear_key = vehicle.surr_vehicles["rear_" + target_lane + "_lane"]
            if new_rear_key == -1:
                #change lane
                return (target_lane, 0.)

            #Apply MOBIL conditions for lane change
            #Compute deceleration for new successor
            new_rear = vehicles_dict[new_rear_key]
            new_rear_acc = idm_model(new_rear, vehicles_dict, lanes, vehicle.id)   
            #Execute lane change
            POLITENESS = vehicle.mobil_params["politeness"]
            ACC_THRESHOLD = vehicle.mobil_params["acc_threshold"]
            MAX_ACCELERATION = vehicle.idm_params["max_acceleration"]
            acc = POLITENESS * (-new_rear_acc) + ACC_THRESHOLD
            
            next_lane = target_lane if (acc < MAX_ACCELERATION) else "stay"
            return (next_lane, acc)
        
    return ("stay", None)



def sort_vehicles(vehicles_dict: dict) -> list:
    """
    Sorts the vehicles in the vehicles_dict based on their longitudinal position.

    Args:
        vehicles_dict (dict)        : The dict containing all vehicles objects.

    Returns:
        ordered_vehicles_ids (list) : The sorted list of vehicle ids.
    """
    s_list = []
    for vehicle in vehicles_dict.values():
        s_list.append(vehicle.lon_pos)
    idx = np.arange(len(vehicles_dict.keys()))
    ordered_vehicles = sorted(idx, key=lambda k: s_list[k])
    ordered_vehicles_ids = np.fromiter(vehicles_dict.keys(), dtype=int)[ordered_vehicles]
    return ordered_vehicles_ids

def update_surr_vehicles_in_front(vehicles_dict: dict, is_reversed: bool) -> dict:
    """
    Updates the surroundings of each vehicle in the vehicles_dict.
    The surroundings are updated based on the vehicles' longitudinal position
    and lane. The vehicles are sorted based on their longitudinal position.
    
    Args:
        vehicles_dict (dict) : The dict containing all vehicles objects.
        is_reversed (bool)    : If True, the vehicles are sorted in reverse order. Required for
                              rear vehicles.
    Returns:
        vehicles_dict (dict) : The updated dict containing all vehicles objects.
    """
    direction = "rear" if is_reversed else "front"
    sorted_vehicles_ids = sort_vehicles(vehicles_dict)
    #Reverse the order of vehicles if we are looking for rear vehicles
    if is_reversed:
        sorted_vehicles_ids = sorted_vehicles_ids[::-1]
    number_of_vehicles = len(sorted_vehicles_ids)
    
    for idx, vehicle_id in enumerate(sorted_vehicles_ids):
        v = vehicles_dict[vehicle_id]
        v_surr = v.surr_vehicles
        #Flags to check if there is a vehicle in front
        is_vehicle_in_front_same = False
        is_vehicle_in_front_left = False
        is_vehicle_in_front_right = False
        #Interate over front vehicles
        for vehicle_in_front_key in sorted_vehicles_ids[idx+1:number_of_vehicles]:
            vf = vehicles_dict[vehicle_in_front_key] #vf => vehicle in front

            #Update the surroundings based on surroundings in front (same, left and right lanes)
            if not is_vehicle_in_front_same and vf.lane == v.lane:
                is_vehicle_in_front_same = True
                v_surr[f"{direction}_same_lane"] = vehicle_in_front_key
            elif not is_vehicle_in_front_right and vf.lane == v.lane - 1:
                is_vehicle_in_front_right = True
                v_surr[f"{direction}_right_lane"] = vehicle_in_front_key
            elif not is_vehicle_in_front_left and vf.lane == v.lane + 1:
                is_vehicle_in_front_left = True
                v_surr[f"{direction}_left_lane"] = vehicle_in_front_key
        
        #There is no vehicle in front, so we set the surroundings to -1
        if not is_vehicle_in_front_same:
            v_surr[f"{direction}_same_lane"] = -1
        if not is_vehicle_in_front_left:    
            v_surr[f"{direction}_left_lane"] = -1
        if not is_vehicle_in_front_right:
            v_surr[f"{direction}_right_lane"] = -1
    
    return vehicles_dict

