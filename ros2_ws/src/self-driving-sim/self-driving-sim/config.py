import toml
from dataclasses import dataclass

@dataclass
class Config:
    def __init__(self, path):
        self._raw = toml.load(path)

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
