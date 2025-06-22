#include <iostream>
#include "clothoid_planner/clothoid_path.hpp"

#include <tuple>
#include <cmath>
#include <chrono>
using namespace clothoid;

int main(){
    ClothoidPath clot_path;
    double x = clot_path.NormalizeAngle(6.02); // Example usage of CosC
    std::cout << "x: " << x << std::endl;
    Pose init_pose = {-2.0, 0.0, 0.0};
    Pose final_pose = {10.0, 10.0, M_PI / 2};
    double kappa_max = 10.0;
    ClothoidPathParams path_params = clot_path.CCTurn(init_pose, final_pose, kappa_max);
    /*
    init_pose = {0.0, 0.0, 0.0};
    final_pose = {10.0, 3.5, 0.2};
    kappa_max = 10.0;
    path_params = clot_path.CCSshape(init_pose, final_pose, kappa_max);
    */
    std::cout << "Clothoid Path Parameters:" << std::endl;
    std::cout << "x0: " << path_params.x0 << ", y0: " << path_params.y0 
              << ", theta0: " << path_params.theta0 << ", kappa0: " << path_params.kappa0 << std::endl;
    std::cout << "c: ";
    for (const auto& c : path_params.c) {
        std::cout << c << " ";
    }
    std::cout << "\ns: ";
    for (const auto& s : path_params.s) {
        std::cout << s << " ";
    }
    std::cout << std::endl;

    auto clothoid_path = clot_path.GetClothoidPath(path_params, 0.5);
    
    std::cout << "Clothoid Path Waypoints:" << std::endl;
    for (const auto& waypoint : clothoid_path) {
        std::cout << "x: " << waypoint.x0 << ", y: " << waypoint.y0 
                  << ", theta: " << waypoint.theta0 << ", kappa: " << waypoint.kappa0 
                  << ", c: " << waypoint.c << ", s: " << waypoint.s << std::endl;
    }
    
    ClothoidParams init_params = {-2.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Pose final_pose2 = {10.0, 10.0, M_PI / 2};
    double kappaf = 0.0;
    auto opt_path = clot_path.GetOptimalManeuver(init_params, final_pose2, kappaf);
    std::cout << "Optimal Clothoid Path Parameters:" << std::endl;
    std::cout << "x0: " << opt_path.x0 << ", y0: " << opt_path.y0 
              << ", theta0: " << opt_path.theta0 << ", kappa0: " << opt_path.kappa0 << std::endl;
    std::cout << "c: ";
    for (const auto& c : opt_path.c) {
        std::cout << c << " ";
    }
    std::cout << "\ns: ";
    for (const auto& s : opt_path.s) {
        std::cout << s << " ";
    }
    std::cout << std::endl;

    clothoid_path = clot_path.GetClothoidPath(opt_path, 0.5);
    
    std::cout << "Clothoid Path Waypoints:" << std::endl;
    for (const auto& waypoint : clothoid_path) {
        std::cout << "x: " << waypoint.x0 << ", y: " << waypoint.y0 
                  << ", theta: " << waypoint.theta0 << ", kappa: " << waypoint.kappa0 
                  << ", c: " << waypoint.c << ", s: " << waypoint.s << std::endl;
    }
    
    std::vector<double> arr_x = {20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0};
    std::vector<double> arr_y = {-3.5, 0.0, 3.5};
    auto start = std::chrono::high_resolution_clock::now();
    for (auto i : arr_x) {
        for (auto j : arr_y) {
            init_params = {0.0, 0.0, -0.1, -0.01, 0.0, 0.0};
            final_pose2 = {i, j, 0.0};
            kappaf = 0.02;
            opt_path = clot_path.GetOptimalManeuver(init_params, final_pose2, kappaf);
            clothoid_path = clot_path.GetClothoidPath(opt_path, 1.0);
            /*
            std::cout << "c: ";
            for (const auto& c : opt_path.c) {
                std::cout << c << " ";
            }
            std::cout << "\ns: ";
            for (const auto& s : opt_path.s) {
                std::cout << s << " ";
            }
            std::cout << std::endl;
            */
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;
    return 0;
}