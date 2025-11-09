#include <chrono>
#include <memory>
#include <string>

#include "clothoid_motion_planner/clothoid_motion_planner.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/vehicle_state.hpp"
#include "custom_interfaces/msg/vehicle_states.hpp"
#include "custom_interfaces/msg/control_inputs.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

class Clot_Motion_Planner_Node : public rclcpp::Node
{
public:
  Clot_Motion_Planner_Node()
  : Node("clot_motion_planner_node")
  {
    auto ego_vehicle_state_callback =
      [this](const custom_interfaces::msg::VehicleState::SharedPtr msg) -> void {
         ego_vehicle_state = *msg;
         RCLCPP_INFO(this->get_logger(), "Received ego vehicle state: curv=%f, speed=%f",
                     ego_vehicle_state.curvature,
                     ego_vehicle_state.speed);
      };
    auto vehicle_states_callback =
      [this](const custom_interfaces::msg::VehicleStates::SharedPtr msg) -> void {
        vehicles_states = msg->states;
        //RCLCPP_INFO(this->get_logger(), "Received %zu vehicle states", vehicles_states.size());
      };
    
    auto timer_callback =
      [this]() -> void {
        if (vehicles_states.size() == 0) {
          RCLCPP_WARN(this->get_logger(), "No vehicle states received yet. Skipping control input computation.");
          return;
        }
        //RCLCPP_INFO(this->get_logger(), "Received %zu vehicle states", vehicles_states.size());
        auto optimal_trajectory = planner.GetOptimaTrajectory(ego_vehicle_state, vehicles_states);
        auto [curv_derivative, acc] = planner.GetOptimalAction(optimal_trajectory);
        auto msg = custom_interfaces::msg::ControlInputs();
        msg.curv_derivative = curv_derivative;
        msg.acceleration = acc;
        RCLCPP_INFO(this->get_logger(), "Publishing control inputs: curv_deriv=%f, acc=%f",
                        msg.curv_derivative, msg.acceleration);
        RCLCPP_INFO(this->get_logger(),
                  "Current time: %.6f s",
                  this->now().seconds());
        this->control_publisher_->publish(msg);
      };
      
    timer_ = this->create_wall_timer(100ms, timer_callback);


    // auto clock = this->get_clock();  // respeita use_sim_time = true

    // timer_ = rclcpp::create_timer(
    //     this->get_node_base_interface(),
    //     this->get_node_timers_interface(),
    //     clock,
    //     std::chrono::milliseconds(500),
    //     [this]() {
    //       RCLCPP_INFO(this->get_logger(),
    //                   "Timer fired at sim time: %.3f s",
    //                   this->now().seconds());
    //     });

    ego_vehicle_state_subscription_ =
      this->create_subscription<custom_interfaces::msg::VehicleState>("ego_vehicle_state", 10, 
                                                        ego_vehicle_state_callback);
    vehicle_states_subscription_ =
      this->create_subscription<custom_interfaces::msg::VehicleStates>("vehicle_states", 10, 
                                                        vehicle_states_callback);
    control_publisher_ = this->create_publisher<custom_interfaces::msg::ControlInputs>("control_inputs", 10);
  }

private:
  rclcpp::Subscription<custom_interfaces::msg::VehicleState>::SharedPtr ego_vehicle_state_subscription_;
  rclcpp::Subscription<custom_interfaces::msg::VehicleStates>::SharedPtr vehicle_states_subscription_;
  rclcpp::Publisher<custom_interfaces::msg::ControlInputs>::SharedPtr control_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  custom_interfaces::msg::VehicleState ego_vehicle_state;
  std::vector<custom_interfaces::msg::VehicleState> vehicles_states;
  clothoid::ClothoidMotionPlanner planner;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Clot_Motion_Planner_Node>());
  rclcpp::shutdown();
  return 0;
}