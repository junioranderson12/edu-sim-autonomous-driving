#include "clothoid_motion_planner/clothoid_motion_planner.hpp"
#include <cmath>
#include <tuple>
#include <vector>
#include <algorithm>
#include <iostream>


namespace clothoid{

Lane ClothoidMotionPlanner::MakeLane(int lane_id) {
  Lane lane;
  for (std::size_t i = 0; i < 1000; ++i) {
    Waypoint waypoint;
    waypoint.x = kXMin + i;
    waypoint.y = lane_id * kLaneWidth; // y-coordinate remains constant for a straight lane
    waypoint.theta = 0.0; // Orientation is zero for a straight lane
    waypoint.kappa = 0.0; // Curvature is zero for a straight lane
    waypoint.c = 0.0; // Sharpness is zero for a straight lane
    waypoint.s = static_cast<double>(i); // Distance along the path
    lane.waypoints.push_back(waypoint); 
  }
  lane.id = lane_id;
  return lane;
}

Road ClothoidMotionPlanner::MakeRoad(int number_of_lanes) {
  Road road;
  for (int lane_id = 0; lane_id < number_of_lanes; ++lane_id) {
      Lane lane = MakeLane(lane_id);
      road.lanes.push_back(lane);
  }
  road.number_of_lanes = number_of_lanes;
  return road;
}

Path ClothoidMotionPlanner::MakePath(State init_state, State final_state, float ds) {
  Pose init_pose = {init_state.x, init_state.y, init_state.theta};
  Pose final_pose = {final_state.x, final_state.y, final_state.theta};
  double init_curvature = init_state.kappa;
  double final_curvature = final_state.kappa;
  return clothoid_path.GetOptimalPath(init_pose, init_curvature,
                                      final_pose, final_curvature, ds);
}   

double ClothoidMotionPlanner::GetDeltaX(double v0, double vf, double delta_y){
  double delta_s = 0.5 * kT * (v0 + vf);
  return sqrt(pow(delta_s, 2) - pow(delta_y, 2));
}

Trajectory ClothoidMotionPlanner::MakeTrajectory(State init_state, double yf, double vf) {
  double v0 = init_state.v;
  double acc = (vf - v0) / kT;
  double delta_x = GetDeltaX(v0, vf, init_state.y-yf);
  State final_state = {init_state.x + delta_x, yf, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0};
  Path path = MakePath(init_state, final_state, 0.1);
  double path_length = path.waypoints[path.waypoints.size() - 1].s;

  double s = 0;
  double v = v0;
  Trajectory trajectory;
  while (s < path_length) {
    // Find the corresponding waypoint on the path
    int s_idx = static_cast<int>(std::round(s * 10.0));
    Waypoint waypoint = path.waypoints[s_idx];
    State state = {waypoint.x, waypoint.y, waypoint.theta,
                   waypoint.kappa, waypoint.c, v, s, acc};
    trajectory.states.push_back(state);
    v = std::max(0.0, acc * dt + v);
    s += ((v0 + v) / 2) * dt; // Distance over dt
    v0 = v;  
  }
  return trajectory;
}

std::vector<Trajectory> ClothoidMotionPlanner::MakeTrajectories(const State& init_state,
                                                  const std::vector<double>& yf,
                                                  const std::vector<double>& vfs) {
  std::vector<Trajectory> trajectories;
  for (std::size_t i = 0; i < yf.size(); ++i) {
    for (std::size_t j = 0; j < vfs.size(); ++j) {
      Trajectory trajectory = MakeTrajectory(init_state, yf[i], vfs[j]);
      trajectories.push_back(trajectory);
    }
  }
  return trajectories;
}

std::vector<Trajectory> ClothoidMotionPlanner::SampleTrajectories(
  const custom_interfaces::msg::VehicleState& ego_vehicle_state){
  State init_state = {ego_vehicle_state.lon_pos,
                      ego_vehicle_state.lat_pos,
                      ego_vehicle_state.heading,
                      ego_vehicle_state.curvature,
                      0.0,
                      ego_vehicle_state.speed,
                      0.0,
                      0.0};
  int lane_id = ego_vehicle_state.lane;
  std::vector<double> yf;
  double current_lane_position = lane_id * kLaneWidth;
  if (lane_id == 0){
    yf = {current_lane_position, -kLaneWidth};
  } else if (lane_id == road.number_of_lanes - 1){
    yf = {current_lane_position, current_lane_position + kLaneWidth};
  } else {
    yf = {current_lane_position, 
            current_lane_position - kLaneWidth,
            current_lane_position + kLaneWidth};
  }
  double v0 = ego_vehicle_state.speed;
  std::vector<double> vfs = { std::max(v0 - 5.0, 0.),
                              std::max(v0 - 4.0, 0.),
                              std::max(v0 - 3.0, 0.),
                              std::max(v0 - 2.0, 0.),
                              std::max(v0 - 1.0, 0.),
                              v0,
                              v0 + 1.0,
                              v0 + 2.0,
                              v0 + 3.0,
                              v0 + 4.0,
                              v0 + 5.0};
  return MakeTrajectories(init_state, yf, vfs);
}

std::vector<Trajectory> ClothoidMotionPlanner::CheckCollisions(
  std::vector<Trajectory>& trajectories, 
  const std::vector<custom_interfaces::msg::VehicleState>& vehicles_states) {
  // Placeholder for collision checking logic
  for (auto& vehicle_state : vehicles_states) {
    for (auto& trajectory : trajectories) {
      double x = vehicle_state.lon_pos;
      double y = vehicle_state.lat_pos;
      double length = vehicle_state.length;
      double width = vehicle_state.width;
      double speed = vehicle_state.speed;
      for (const auto& state : trajectory.states) {
        //Constant speed assumption for other vehicles
        //TODO : improve with interaction model
        x = x + speed * dt;
        //Assumes no collision if the other vehicle is behind ego vehicle
        if (x < state.x){
          continue;
        }
        // Simple bounding box collision check
        if (std::abs(state.x - x) < length / 2 && std::abs(state.y - y) < width / 2) {
          trajectory.is_colliding = true;
          break;
        }
      }
    }
  }
  return trajectories;
}

Trajectory ClothoidMotionPlanner::GetOptimaTrajectory(
  const custom_interfaces::msg::VehicleState& ego_vehicle_state,
  const std::vector<custom_interfaces::msg::VehicleState>& vehicles_states) {
  std::vector<Trajectory> trajectories = SampleTrajectories(ego_vehicle_state);
  trajectories = CheckCollisions(trajectories, vehicles_states);
  double min_cost = std::numeric_limits<double>::max();
  Trajectory optimal_trajectory;
  for (const auto& trajectory : trajectories) {
    double cost = GetTrajCost(trajectory);
    if ((cost < min_cost) && (!trajectory.is_colliding)) {
      min_cost = cost;
      optimal_trajectory = trajectory;
    }
  }
  for (const auto& state : optimal_trajectory.states) {
    std::cerr << "Optimal Trajectory State: x=" << state.x << ", y=" << state.y << ", v=" << state.v << ", kappa=" << state.kappa << std::endl;
  }
  return optimal_trajectory;
}

double ClothoidMotionPlanner::GetTrajCost(const Trajectory& trajectory) {
  double cost = 0.0;
  for (auto &state : trajectory.states) {
    cost = cost + GetStateCost(state);
    if (trajectory.is_colliding) {
      return 1e6; // High cost for colliding trajectories
    }
  }
  return cost;
}

double ClothoidMotionPlanner::GetStateCost(const State& state) {
  return  state.kappa * abs(kCostCurvature) + 
    (std::pow(kMaxSpeed - state.v, 2)) * kCostVelocity;
}

std::tuple<double, double> ClothoidMotionPlanner::GetOptimalAction(
  const Trajectory& optimal_trajectory) {

  double curv_derivative = optimal_trajectory.states.front().c;
  double acc = optimal_trajectory.states.back().acc;
  return std::make_tuple(curv_derivative, acc);
}

}; // namespace clothoid