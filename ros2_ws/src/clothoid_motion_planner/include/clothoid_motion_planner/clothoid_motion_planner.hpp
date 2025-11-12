#ifndef CLOTHOIDMOTIONPLANNER_HPP
#define CLOTHOIDMOTIONPLANNER_HPP

#include "clothoid_motion_planner/clothoid_path.hpp"
#include "custom_interfaces/msg/vehicle_states.hpp"

namespace clothoid{

struct Lane {
  std::vector<Waypoint> waypoints; // List of waypoints defining the lane
  int id; // Lane identifier
};

struct Road {
  std::vector<Lane> lanes; // List of lanes in the road
  int number_of_lanes;
  std::vector<double> lane_centers;
};

struct State {
  double x;       // x-coordinate
  double y;       // y-coordinate
  double theta;   // orientation (angle)
  double kappa;   // curvature
  double c;       // sharpness
  double v;       // velocity
  double s;       // distance along the path
  double acc;     // acceleration
};

struct Trajectory {
  std::vector<State> states; // List of states defining the trajectory
  bool is_colliding = false; // Collision flag
};

class ClothoidMotionPlanner {
  public:
    ClothoidMotionPlanner() = default;
    Lane MakeLane(int lane_id);
    Road MakeRoad(int number_of_lanes);
    Path MakePath(State init_state, State final_state, float ds);
    double GetDeltaX(double v0, double vf, double dy);
    Trajectory MakeTrajectory(State init_state, double yf, double vf);
    std::vector<Trajectory> MakeTrajectories(const State& init_state,
                              const std::vector<double>& yf,
                              const std::vector<double>& vfs);
    std::vector<Trajectory> SampleTrajectories(
            const custom_interfaces::msg::VehicleState& ego_vehicle_state);
    int GetClosestLaneIndex(double lat_pos,
            const std::vector<double>& lane_centers);
    std::vector<Trajectory> CheckCollisions(
            std::vector<Trajectory>& trajectories, 
            const std::vector<custom_interfaces::msg::VehicleState>& vehicles_states);
    Trajectory GetOptimaTrajectory(
            const custom_interfaces::msg::VehicleState& ego_vehicle_state,
            const std::vector<custom_interfaces::msg::VehicleState>& vehicles_states);
    std::tuple<double, double> GetOptimalAction(
            const Trajectory& optimal_trajectory);   
    double GetTrajCost(const Trajectory& trajectory);
    double GetStateCost(const State& state);
    const double dt = 0.5; // Time interval for trajectory points
    
    const double kXMin = -100.0; // Minimum x-coordinate for the lane
    const double kLaneWidth = 3.5; // Width of the lane
    const double kCostCurvature = 5.0;
    const double kCostHeading = 15.0;
    const double kCostVelocity = 5.0;
    const double kCostDeltaY = 0.0;
    const double kMaxSpeed = 20.0;
    const double kT = 5.0; //horizon time in seconds

    ClothoidPath clothoid_path;
    Road road = MakeRoad(3); // Default road with 3 lanes
};
} // namespace clothoid
#endif // CLOTHOIDMOTIONPLANNER_HPP