"""
simulator.py

This file contains the Simulator class, which is responsible for simulating the
vehicles in the environment.

Author: Júnior Anderson Rodrigues da Silva
Email: juniorars@gmail.com
Date: 2025-05-09
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import _pylab_helpers
import time
from dynamics import Vehicle, update_surr_vehicles, is_collision

class Simulator:
    def __init__(self, lane_width: float, number_of_lanes: int, time_step: float):
        self.ax = plt.figure(num=None, figsize=(10, 2)).gca()
        plt.ion()
        plt.axis("equal")
        plt.draw()
        plt.pause(0.1)
        #plt.show()
        self.lane_width = lane_width
        self.number_of_lanes = number_of_lanes
        self.time_step = time_step
        self.X_VIEW_MIN = -100.0
        self.X_VIEW_MAX = 100.0
        self.Y_VIEW_MIN = -5.0
        self.Y_VIEW_MAX = number_of_lanes * lane_width + 5.0
        self.vehicle_dict = {}
        self.time_stamp = 0.0
        self.frame = 0

    def check_collision(self, vehicle_dict: dict, vehicle: Vehicle, collision_thr: float) -> bool:
        """
        Check if the vehicle collides with any other vehicles in the vehicle_dict.

        Args:
            vehicle_dict (dict): Dictionary of vehicles in the simulation.
            vehicle (Vehicle): The vehicle to check for collisions.
            collision_thr (float): Collision threshold.

        Returns:
            bool: True if there is a collision, False otherwise.
        """
        for other_vehicle in vehicle_dict.values():
            if is_collision(vehicle, other_vehicle, 5.0, 0.0):
                return True
        return False

    def spaw_vehicle(self, vehicle_type: str, vehicle_params: dict = None) -> Vehicle:
        """
        Spawn a vehicle in the simulation.
        
        Args:
            vehicle_type (str): The type of vehicle to spawn (e.g., "car", "bus", "truck").
            vehicle_params (dict): The parameters for the vehicle.

        Returns:
            Vehicle: The spawned vehicle object.
        """
        is_collision = True
        vehicle_id = Vehicle.number_of_vehicles

        # Spawn a vehicle
        while is_collision:
            vehicle = Vehicle(vehicle_id, vehicle_type, self.lane_width,
                              self.number_of_lanes, self.time_step, vehicle_params)
            # Check if the vehicle collides with any other vehicles in the vehicle_dict
            collision_lon_thr = 5.0
            is_collision = self.check_collision(self.vehicle_dict, vehicle, collision_lon_thr)
            
        # Add the vehicle to the vehicle_dict
        self.vehicle_dict[vehicle_id] = vehicle

    def visualize_vehicles(self, vehicle_dict):
        """
        Visualize the vehicles in the simulation.

        Args:
            vehicle_dict (dict): Dictionary of vehicles in the simulation.
        """
        # Clear the previous plot
        rect_list = []
        for vehicle in vehicle_dict.values():
            # Create a Rectangle patch
            abs_psi = np.abs(vehicle.heading)
            if abs_psi > np.pi/2:
                psi = vehicle.heading + np.pi
            else:
                psi = vehicle.heading

            #Vehicles color
            facecolor = 'blue'
            scale = 1.0
            
            rect = patches.Rectangle((vehicle.lon_pos - (vehicle.features["LENGTH"]/scale)/2.0, 
                                        vehicle.lat_pos - (vehicle.features["WIDTH"]/scale)/2.0),
                                        vehicle.features["LENGTH"]/scale,
                                        vehicle.features["WIDTH"]/scale,
                                        angle=psi*180/np.pi, 
                                        facecolor=facecolor,
                                        edgecolor="black")

            # Add the patch to the Axes
            self.ax.add_patch(rect)
            rect_list.append(rect)

        plt.xlim([self.X_VIEW_MIN, self.X_VIEW_MAX])
        #plt.ylim([self.Y_VIEW_MIN, self.Y_VIEW_MAX])
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title(f'$t$={self.time_stamp:.1f}s')
        plt.draw()
        plt.pause(0.09)
            
        #self.pause(self.time_step, focus_figure=False)
        #time.sleep(self.time_step)
        # Remove the rectangles from the plot
        for rect in rect_list:
            rect.remove()

if __name__ == "__main__":
    # Create a simulator object
    lane_width = 3.5
    number_of_lanes = 3
    time_step = 0.1
    simulator = Simulator(lane_width, number_of_lanes, time_step)
    print("Simulator initialized")

    vehicles_params1 = {
        "features": {"TRACK_ID": 1, "LENGTH": 4.5, "WIDTH": 2.0},
        "idm": {
            "DESIRED_SPEED" : 15.0,
            "JAM_DISTANCE" : 1.0,
            "DESIRED_TIME_GAP" : 1.0,
            "MAX_ACCELERATION" : 2.0,
            "DESIRED_DECELERATION" : 3.0,
            "DELTA" : 4,
            "SAFE_BRAKE" : 4},
        "mobil": {
            "POLITENESS" : 0.5,
            "ACC_THRESHOLD" : 0.3,
            "DIST_AHEAD_THR" : 1.0,
            "is_lc_allowed" : True},
        "state": {
            "lon_pos": -100.0,
            "lat_pos": lane_width,
            "heading": 0.0,
            "speed": 15.0,
            "lane": 1}
    }
    vehicles_params2 = {
        "features": {"TRACK_ID": 2, "LENGTH": 4.5, "WIDTH": 2.0},
        "idm": {
            "DESIRED_SPEED" : 15.0,
            "JAM_DISTANCE" : 2.0,
            "DESIRED_TIME_GAP" : 1.0,
            "MAX_ACCELERATION" : 2.0,
            "DESIRED_DECELERATION" : 3.0,
            "DELTA" : 4,
            "SAFE_BRAKE" : 4},
        "mobil": {
            "POLITENESS" : 0.,
            "ACC_THRESHOLD" : 0.3,
            "DIST_AHEAD_THR" : 1.0,
            "is_lc_allowed" : True},
        "state": {
            "lon_pos": -30.0,
            "lat_pos": 0.0,
            "heading": 0.0,
            "speed": 15.0,
            "lane": 0}
    }
    vehicles_params3 = {
        "features": {"TRACK_ID": 3, "LENGTH": 4.5, "WIDTH": 2.0},
        "idm": {
            "DESIRED_SPEED" : 10.0,
            "JAM_DISTANCE" : 2.0,
            "DESIRED_TIME_GAP" : 1.0,
            "MAX_ACCELERATION" : 2.0,
            "DESIRED_DECELERATION" : 3.0,
            "DELTA" : 4,
            "SAFE_BRAKE" : 4},
        "mobil": {
            "POLITENESS" : 0.5,
            "ACC_THRESHOLD" : 0.3,
            "DIST_AHEAD_THR" : 1.0,
            "is_lc_allowed" : False},
        "state": {
            "lon_pos": -20.0,
            "lat_pos": 0.0,
            "heading": 0.0,
            "speed": 10.0,
            "lane": 0}
    }
    vehicles_params4= {
        "features": {"TRACK_ID": 4, "LENGTH": 4.5, "WIDTH": 2.0},
        "idm": {
            "DESIRED_SPEED" : 10.0,
            "JAM_DISTANCE" : 2.0,
            "DESIRED_TIME_GAP" : 1.0,
            "MAX_ACCELERATION" : 2.0,
            "DESIRED_DECELERATION" : 3.0,
            "DELTA" : 4,
            "SAFE_BRAKE" : 4},
        "mobil": {
            "POLITENESS" : 0.5,
            "ACC_THRESHOLD" : 0.3,
            "DIST_AHEAD_THR" : 1.0,
            "is_lc_allowed" : False},
        "state": {
            "lon_pos": -0.0,
            "lat_pos": lane_width,
            "heading": 0.0,
            "speed": 10.0,
            "lane": 1}
    }
    vehicles_params = [vehicles_params1, vehicles_params2, vehicles_params3, vehicles_params4]
    # Spawn vehicles
    for i in range(10):
        vehicle_type = np.random.choice(["car", "bus", "truck", "motorcycle"], 
                                        p=[0.6, 0.1, 0.1, 0.2])
        simulator.spaw_vehicle(vehicle_type)
        # vehicle_type = "car"
        # simulator.spaw_vehicle(vehicle_type, vehicles_params[i])
    print("Vehicles spawned")

    # Simulate the vehicles
    lanes = [{"lane_id": i, "length":np.inf, "width": lane_width} for i in range(number_of_lanes)]
    
    # Simulate the vehicles
    while simulator.time_stamp < 15.0:
        simulator.visualize_vehicles(simulator.vehicle_dict)
        simulator.time_stamp += simulator.time_step
        simulator.vehicle_dict = Vehicle.update_surr_vehicles(simulator.vehicle_dict)
        for vehicle in simulator.vehicle_dict.values(): 
            vehicle.make_step(simulator.vehicle_dict, lanes)
            if vehicle.ID == 1:
                print(vehicle.speed)