import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import time

from self_driving_sim.simulator import Simulator
from self_driving_sim.dynamics import Vehicle
from custom_interfaces.msg import ControlInputs, VehicleState, VehicleStates

class SimulatorNode(Node):

    def __init__(self, simulator: Simulator):
        super().__init__('simulator_node')
        self.simulator = simulator
        self.clock_publisher_ = self.create_publisher(Clock, '/clock', 10)
        self.marker_array_publisher_ = self.create_publisher(MarkerArray, 'visualization_markers', 10)
        self.vehicle_states_publisher_ = self.create_publisher(VehicleStates, 'vehicle_states', 10)
        self.ego_vehicle_state_publisher_ = self.create_publisher(VehicleState, 'ego_vehicle_state', 10)
        self.control_inputs_subscription = self.create_subscription(ControlInputs,'control_inputs', self.control_callback, 10)
        self.control_acc = 0.0
        self.control_curv_derivative = 0.0
        self.is_updated = False
        self.get_logger().info('Node created')

    def publish_clock(self, time_stamp):
        clock_msg = Clock()
        clock_msg.clock = Time()
        clock_msg.clock.sec = int(time_stamp)
        clock_msg.clock.nanosec = int(1e9 * (time_stamp - int(time_stamp)))
        self.clock_publisher_.publish(clock_msg)
        return clock_msg
    
    def publish_ego_vehicle_state(self, vehicles: dict):
        ego_vehicle = vehicles[Vehicle.controlled_vehicle_id] # Assuming vehicle with id 1 is the ego vehicle
        ego_vehicle_state_msg = VehicleState()
        ego_vehicle_state_msg.id = int(ego_vehicle.id)
        ego_vehicle_state_msg.lon_pos = float(ego_vehicle.lon_pos)
        ego_vehicle_state_msg.lat_pos = float(ego_vehicle.lat_pos)
        ego_vehicle_state_msg.heading = float(ego_vehicle.heading)
        ego_vehicle_state_msg.curvature = float(ego_vehicle.curvature)
        ego_vehicle_state_msg.speed = float(ego_vehicle.speed)
        ego_vehicle_state_msg.length = float(ego_vehicle.features['length'])
        ego_vehicle_state_msg.width = float(ego_vehicle.features['width'])
        ego_vehicle_state_msg.lane = int(ego_vehicle.lane)
        self.ego_vehicle_state_publisher_.publish(ego_vehicle_state_msg)
    
    def publish_vehicle_states(self, vehicles: dict):
        vehicle_states_msg = VehicleStates()
        for vehicle in vehicles.values():
            if vehicle.id == Vehicle.controlled_vehicle_id:
                continue  # Skip ego vehicle
            vehicle_state_msg = VehicleState()
            vehicle_state_msg.id = int(vehicle.id)
            vehicle_state_msg.lon_pos = float(vehicle.lon_pos)
            vehicle_state_msg.lat_pos = float(vehicle.lat_pos)
            vehicle_state_msg.heading = float(vehicle.heading)
            vehicle_state_msg.speed = float(vehicle.speed)
            vehicle_state_msg.length = float(vehicle.features['length'])
            vehicle_state_msg.width = float(vehicle.features['width'])
            vehicle_state_msg.lane = int(vehicle.lane)
        
            vehicle_states_msg.states.append(vehicle_state_msg)
        self.vehicle_states_publisher_.publish(vehicle_states_msg)

    def publish_markers(self, vehicles: dict):
        marker_array = MarkerArray()
        for vehicle in vehicles.values():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "vehicle"
            
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Size of the rectangle (width x height x thickness)
            marker.scale.x = vehicle.features["length"]
            marker.scale.y = vehicle.features["width"]
            marker.scale.z = 1.5  # Thin height to make it look flat

            # Color (opaque blue)
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.id = vehicle.id

            # Position of the center of the rectangle
            marker.pose.position.x = vehicle.lon_pos
            marker.pose.position.y = vehicle.lat_pos
            marker.pose.position.z = 0.01  # Slightly above the ground to be visible

            qx, qy, qz, qw = self.yaw_to_quaternion(vehicle.heading)
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            
            marker_array.markers.append(marker)
        self.marker_array_publisher_.publish(marker_array)
    
    def control_callback(self, msg):
        self.control_acc = msg.acceleration
        self.control_curv_derivative = msg.curv_derivative
        if self.simulator.frame < 2:
            self.control_acc = 0.0
            self.control_curv_derivative = 0.0
        self.get_logger().info(f'acc: {self.control_acc:.3f}, curv_deriv: {self.control_curv_derivative:.3f}')

    def run_episode(self):
        # Simulate the vehicles
        #TODO Fix rclpy.spin_once
        while self.simulator.frame < self.simulator.frames_per_episode:
            self.publish_clock(self.simulator.time_stamp)
            self.publish_markers(self.simulator.vehicle_dict)
            self.publish_ego_vehicle_state(self.simulator.vehicle_dict)
            self.publish_vehicle_states(self.simulator.vehicle_dict)
            
            rclpy.spin_once(self)
            #time.sleep(0.5)
            self.simulator.run_frame(self.control_curv_derivative, self.control_acc)
            
            

    def run(self):
        while self.simulator.episode < self.simulator.max_episodes:
            self.simulator.initialize_episode()
            self.run_episode()
            self.get_logger().info('Episode %d completed' % self.simulator.episode)
            self.simulator.episode += 1

    def yaw_to_quaternion(self, yaw):
        half = yaw / 2.0
        return 0.0, 0.0, np.sin(half), np.cos(half)

def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator('self_driving_sim')
    
    simulator_node = SimulatorNode(simulator)
    time.sleep(3.0)
    simulator_node.run()
    simulator_node.get_logger().info('Finished')
    #rclpy.spin(simulator_node)

    

if __name__ == '__main__':
    main()
