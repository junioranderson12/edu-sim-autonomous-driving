"""
dynamics.py

This module provides vehicles' dynamics and decisions based on IDM + MOBIL models.
Author: Júnior Anderson Rodrigues da Silva
Email: juniorars@gmail.com
Date: 2025-05-01
"""

import numpy as np

class Vehicle:
    def __init__(self, state: dict, features: dict, 
                    idm_params: dict, mobil_params: dict,
                    lane_width: float, number_of_lanes: int):
        self.features        = features
        self.idm_params      = idm_params
        self.mobil_params    = mobil_params
        self.lon_pos         = state["lon_pos"]
        self.lat_pos         = state["lat_pos"]
        self.heading         = state["heading"]
        self.speed           = state["speed"]
        self.lane            = state["lane"]
        self.lane_width      = lane_width
        self.number_of_lanes = number_of_lanes

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

    def make_step(self, curvature: float, accel: float, time_step: float):
        """
        Returns the next state of the vehicle based on model inputs.

        Args:
            curvature (float)       : Curvature.
            accel (float)           : Acceleration.
            time_step (float)       : Time step.

        Returns:
            dictionay: The updated state of the vehicle.
        """
        self.lon_pos = self.lon_pos + self.speed * np.cos(self.heading) * time_step
        self.lat_pos = self.lat_pos + self.speed * np.sin(self.heading) * time_step
        self.heading = self.heading + curvature * self.speed * time_step
        self.speed   = self.speed + accel * time_step
        self.lane    = self.get_current_lane(self.lat_pos)


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
        lane_length = lanes[lane]["lane_length"]
        delta_s = lane_length - s
        delta_v = v
    #Following leader behavior.
    else:    
        leader = vehicles_dict[leader_key]
        s_leader = leader.lon_pos
        v_leader = leader.speed
        leader_length = leader["length"]

        length_aver = (leader_length + length)/2
        delta_s = s_leader - s - length_aver
        delta_v = v - v_leader
    
    DESIRED_SPEED = vehicle.idm_params["DESIRED_SPEED"]
    JAM_DISTANCE = vehicle.idm_params["JAM_DISTANCE"]
    DESIRED_TIME_GAP = vehicle.idm_params["DESIRED_TIME_GAP"]
    MAX_ACCELERATION = vehicle.idm_params["MAX_ACCELERATION"]
    DESIRED_DECELERATION = vehicle.idm_params["DESIRED_DECELERATION"]
    DELTA = vehicle.idm_params["DELTA"]
    SAFE_BRAKE = vehicle.idm_params["SAFE_BRAKE"]

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
    
    leader = vehicles_dict[leader_key]
    new_leader_key = vehicle.surr_vehicles["front_" + target_lane + "_lane"]
    new_leader = vehicles_dict[new_leader_key]

    delta_leader = leader.lon_pos - vehicle.lon_pos
    delta_new_leader = new_leader.lon_pos- vehicle.lon_pos
    if new_leader_key == -1:
        #TODO Implement a penalization for lane change based on TTC
        return True

    DIST_AHEAD_THR = vehicle.mobil_params["DIST_AHEAD_THR"]
    return True if delta_new_leader > (delta_leader * DIST_AHEAD_THR) else True

def is_feasible(vehicle: Vehicle, target_lane: str, lanes: list) -> bool:
    """
    Returns whether a lane change is feasible.

    Args:
        vehicle (Vehicle)       : The vehicle in consideration.
        target_lane (list)      : The target lane for lane change ('left' or 'right').
        lanes (list)            : The list containing all lanes.

    Returns:
        (bool)                  : Return True if feasible, and False otherwise.
    """
    #TODO Implement minimum speed for feasible lane change
    lane_dir = 1 if target_lane == "left" else -1
    new_lane = vehicle.lane + lane_dir
    return True if (len(lanes) >= new_lane) and (new_lane > 0) else False

            
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
    target_lanes = ["left", "right"]
    #We shuffle target_lanes to give some variation in the chosen lane.
    for target_lane in np.random.shuffle(target_lanes):
        if is_feasible(vehicle, target_lane, lanes) and \
            is_beneficial(vehicle,vehicles_dict, target_lane):

            new_rear_key = vehicle.surr_vehicles["rear_" + target_lane + "_lane"]
            if new_rear_key == -1:
                #change lane
                pass

            #Apply MOBIL conditions for lane change
            #Compute deceleration for new successor
            new_rear = vehicles_dict[new_rear_key]
            new_rear_acc = idm_model(new_rear, vehicles_dict, lanes, vehicle.features["track_id"])   

            #Execute lane change
            POLITENESS = vehicle.mobil_params["POLITENESS"]
            ACC_THRESHOLD = vehicle.mobil_params["ACC_THRESHOLD"]
            SAFE_BRAKE = vehicle.mobil_params["SAFE_BRAKE"]
            MAX_ACCELERATION = vehicle.idm_params["MAX_ACCELERATION"]
            acc = POLITENESS * (-new_rear_acc) + ACC_THRESHOLD
            next_lane = target_lane if (new_rear_acc > -SAFE_BRAKE) and \
                (acc < MAX_ACCELERATION) else "stay"
            return (next_lane, acc)
        else:
            return ("stay", 0.)
