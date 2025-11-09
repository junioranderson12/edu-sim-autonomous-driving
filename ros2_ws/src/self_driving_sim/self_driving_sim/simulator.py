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
from pathlib import Path
import toml
import os
from ament_index_python.packages import get_package_share_directory

from self_driving_sim.dynamics import Vehicle
from self_driving_sim.config import Config

class Simulator:
    def __init__(self, package_directory):
        self.package_directory = get_package_share_directory(package_directory)
        cfg = Config(self.package_directory, 'simulator_config.toml')
        self.cfg = cfg
        self.ax = plt.figure(num=None, figsize=(10, 2)).gca()
        plt.ion()
        plt.axis("equal")
        plt.draw()
        plt.pause(0.1)
        #plt.show()
        self.lane_width = cfg.lane_width
        self.number_of_lanes = cfg.number_of_lanes
        self.time_step = cfg.time_step
        self.number_of_vehicles = cfg.number_of_vehicles
        self.surr_config_path = cfg.surr_config_path
        self.user_surr_vehicles_params = cfg.user_surr_vehicles_params
        self.X_VIEW_MIN = -100.0
        self.X_VIEW_MAX = 100.0
        self.Y_VIEW_MIN = -5.0
        self.Y_VIEW_MAX = self.number_of_lanes * self.lane_width + 5.0
        self.vehicle_dict = {}
        self.time_stamp = 0.0
        self.frame = 0
        self.episode = 0
        self.frames_per_episode = cfg.frames_per_episode
        self.max_episodes = cfg.max_episodes
        self.number_of_vehicles = cfg.number_of_vehicles
        self.lanes = [{"lane_id": i, "length":np.inf, "width": self.lane_width} for i in range(self.number_of_lanes)]

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
            if Vehicle.is_collision(vehicle, other_vehicle, 5.0, 0.0):
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

        # Spawn a vehicle
        while is_collision:
            vehicle = Vehicle(self.cfg, vehicle_type, vehicle_params)
            # Check if the vehicle collides with any other vehicles in the vehicle_dict
            collision_lon_thr = 5.0
            is_collision = self.check_collision(self.vehicle_dict, vehicle, collision_lon_thr)
            
        # Add the vehicle to the vehicle_dict
        self.vehicle_dict[vehicle.id] = vehicle

    def spaw_vechiles_from_user_params(self, surrs_params):
        """
        Spawn all vehicles in the scene based on user parameters.

        Args:
            scenes_params (list): The parameters of all vehicles loaded from user config.
        """
        for params in surrs_params:
            vehicle_type = params["features"]["type"]
            self.spaw_vehicle(vehicle_type, params)
    
    def spaw_vehicles_from_random_params(self):
        """
        Spawn all vehicles in the scene based on random parameters.
        """
        for _ in range(self.number_of_vehicles):
            vehicle_type = np.random.choice(["car", "bus", "truck", "motorcycle"], 
                                            p=[self.cfg.car_perc, self.cfg.bus_perc, 
                                               self.cfg.truck_perc, self.cfg.motorcycle_perc])
            self.spaw_vehicle(vehicle_type)

    def spaw_all_vehicles(self, scenes_params:list=None) -> None:
        """
        Spawn all vehicles in the scene.

        Args:
            scenes_params (list): The parameters of all vehicles loaded from user config.
        """
        if scenes_params:
            number_of_user_scenes = len(scenes_params)
            params = scenes_params[self.episode % number_of_user_scenes]
            self.spaw_vechiles_from_user_params(params)
        else:
            self.spaw_vehicles_from_random_params()

    def load_surr_vehicles_params(self, path: str) -> list:
        """
        Load vehicles parameters passed by the user. This is very useful in customized simulation or
        to achieve reproducable results using different planners.

        Args:
            path (str): The path containing the user configuration files.
        """
        #TODO Put this function in Vehicle class
        # Load the surrounding vehicles parameters from the config file
        scenes_params = []
        config_file_path = os.path.join(self.package_directory, 'config', self.surr_config_path)
        for file in Path(config_file_path).iterdir():
            scene = file.name
            if (not self.cfg.use_all_scenes) and (scene not in self.cfg.scenes):
                continue
            
            raw = toml.load(file)
            surrs_params = []
            for vehicle in raw.keys():
                params = {
                    "features": {
                        "type": raw[vehicle]["type"],
                        "length": raw[vehicle]["length"],
                        "width": raw[vehicle]["width"],
                    },
                    "idm": {
                        "desired_speed" : raw[vehicle]["desired_speed"],
                        "jam_distance" : raw[vehicle]["jam_distance"],
                        "desired_time_gap" : raw[vehicle]["desired_time_gap"],
                        "max_acceleration" : raw[vehicle]["max_acceleration"],
                        "desired_deceleration" : raw[vehicle]["desired_deceleration"],
                        "delta" : raw[vehicle]["delta"],
                        "safe_brake" : raw[vehicle]["safe_brake"]
                    },
                    "mobil": {
                        "politeness" : raw[vehicle]["politeness"],
                        "acc_threshold" : raw[vehicle]["acc_threshold"],
                        "is_lc_allowed" : raw[vehicle]["is_lc_allowed"]
                    },
                    "state": {
                        "lon_pos": raw[vehicle]["lon_pos"],
                        "lat_pos": raw[vehicle]["lat_pos"],
                        "heading": raw[vehicle]["heading"],
                        "curvature": raw[vehicle]["curvature"],
                        "speed": raw[vehicle]["speed"],
                        "lane": raw[vehicle]["lane"]
                    }
                }
                surrs_params.append(params)
            scenes_params.append(surrs_params)
        return scenes_params

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
            if vehicle.id == Vehicle.controlled_vehicle_id:
                facecolor = 'red'
            else:
                facecolor = 'blue'
            scale = 1.0
            
            rect = patches.Rectangle((vehicle.lon_pos - (vehicle.features["length"]/scale)/2.0, 
                                        vehicle.lat_pos - (vehicle.features["width"]/scale)/2.0),
                                        vehicle.features["length"]/scale,
                                        vehicle.features["width"]/scale,
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

    def run_frame(self, curv_derivative: float=0.0, acceleration: float=0.0) -> None:
        # Simulate the vehicles
        self.visualize_vehicles(self.vehicle_dict)
        self.vehicle_dict = Vehicle.update_surr_vehicles(self.vehicle_dict)
        for vehicle in self.vehicle_dict.values():
            if vehicle.id == Vehicle.controlled_vehicle_id:
                vehicle.apply_ego_control_inputs(curv_derivative, acceleration)
            else:
                vehicle.make_step(self.vehicle_dict, self.lanes)
        self.time_stamp += self.time_step
        self.frame += 1
        
    def initialize_episode(self):
        if self.user_surr_vehicles_params:
            user_params = self.load_surr_vehicles_params(self.cfg.surr_config_path)
        else:
            user_params = None
        self.vehicle_dict = {}
        Vehicle.restart_static_vars()
        self.frame = 0
        self.time_stamp = 0.
        self.spaw_all_vehicles(user_params)