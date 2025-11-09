import toml
from dataclasses import dataclass
import os

@dataclass
class Config:
    def __init__(self, package_directory: str, path: str):

        config_file_path = os.path.join(package_directory, 'config', path)
        self._raw = toml.load(config_file_path)

        #Controlled vehicle parameters
        self.m = self._raw["controlled_vehicle"]["m"]
        self.I_z = self._raw["controlled_vehicle"]["I_z"]
        self.l_f = self._raw["controlled_vehicle"]["l_f"]
        self.l_r = self._raw["controlled_vehicle"]["l_r"]
        self.C_f = self._raw["controlled_vehicle"]["C_f"]
        self.C_r = self._raw["controlled_vehicle"]["C_r"]
        self.lon_pos = self._raw["controlled_vehicle"]["lon_pos"]
        self.lat_pos = self._raw["controlled_vehicle"]["lat_pos"]
        self.heading = self._raw["controlled_vehicle"]["heading"]
        self.curvature = self._raw["controlled_vehicle"]["curvature"]
        self.speed = self._raw["controlled_vehicle"]["speed"]
        self.lane = self._raw["controlled_vehicle"]["lane"]

        #Simulator parameters
        self.lane_width = self._raw["simulator"]["lane_width"]
        self.number_of_lanes = self._raw["simulator"]["number_of_lanes"]
        self.time_step = self._raw["simulator"]["time_step"]
        self.car_perc = self._raw["simulator"]["car_perc"]
        self.truck_perc = self._raw["simulator"]["truck_perc"]
        self.bus_perc = self._raw["simulator"]["bus_perc"]
        self.motorcycle_perc = self._raw["simulator"]["motorcycle_perc"]
        self.x_max = self._raw["simulator"]["x_max"]
        self.x_min = self._raw["simulator"]["x_min"]
        self.speed_limit = self._raw["simulator"]["speed_limit"]
        self.number_of_vehicles = self._raw["simulator"]["number_of_vehicles"]
        self.user_surr_vehicles_params = self._raw["simulator"]["user_surr_vehicles_params"]
        self.max_episodes = self._raw["simulator"]["max_episodes"]
        self.frames_per_episode = self._raw["simulator"]["frames_per_episode"]
        self.controlled_vehicle_id = self._raw["simulator"]["controlled_vehicle_id"]

        #Surrounding vehicles parameters
        self.surr_config_path = self._raw["surrounding_vehicles"]["config_path"]
        self.scenes = self._raw["surrounding_vehicles"]["scenes_config"]
        self.use_all_scenes = self._raw["surrounding_vehicles"]["use_all_scenes"]   