#include "clothoid_planner/clothoid_path.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <tuple>

namespace clothoid{

ClothoidPath::ClothoidPath() : 
  XlYl(LoadXLYLConfigParams()) {
}

std::pair<std::vector<double>, std::vector<double>> ClothoidPath::LoadXLYLConfigParams() {
  std::ifstream file("../src/Clothoid_LT.txt");
  if (!file.is_open()) {
    std::cerr << "Error opening file!" << std::endl;
  }

  std::vector<double> x, y;

  std::string line;
  while (std::getline(file, line)) {
      std::istringstream iss(line);
      double x_val, y_val;
      if (iss >> x_val >> y_val) {
          x.push_back(x_val);
          y.push_back(y_val);
      }
  }
  file.close();
  return std::make_pair(x, y);
}

std::pair<double, double> ClothoidPath::GetBasicClothoidCoords(double s){
  /*
  @brief
  Calculates clothoid basic coordinates from a lookup
  table with previous computed coordinates. These coordinates are used later
  to compute general clothoid coordinates.

  Given the travelled distance along the basic clothoid, this
  function returns the basic clothoid coordinates. The pseudocode was
  implemented by Misel Brezak and Ivan Petrovic and it was described
  in the paper Real-time Approximation of Clothoids With Bounded Error
  for Path Planning Applications, IEEE Transactions on Robotics, 2014.
  IEEE, v. 30, n. 2, p. 507–515, 2014.

  @param s: distance travelled along the clothoid

  @return: a pair of vectors containing the abscissa and ordinate coordinates
  */

  int k = static_cast<int>(std::abs(s) / kDeltaSL);
  double sk = k * kDeltaSL;
  double rk = 1.0 / (kCcL * (sk + 0.5 * kDeltaSL));
  double theta_lk = 0.5 * kCcL * sk * sk;
  double x_lk = kXL[k];
  double y_lk = kYL[k];
  double ds = std::abs(s) - sk;
  double x_lb = x_lk + 2 * rk * std::cos(theta_lk + ds / (2 * rk)) * 
                 std::sin(ds / (2 * rk));
  double y_lb = y_lk + 2 * rk * std::sin(theta_lk + ds / (2 * rk)) * 
                 std::sin(ds / (2 * rk));
  if (s < 0) {
    x_lb = -x_lb;
    y_lb = -y_lb;
  }
  return std::make_pair(x_lb, y_lb);
}

std::pair<double, double> ClothoidPath::GetGeneralClothoidCoords(double x0, double y0,
                                                    double theta0, double kappa0,
                                                    double c, double s){
  /*
  @brief
  Calculates clothoid general coordinates.

  Given the clothoid initial parameters as well as the curvature
  derivate and travelled distance along the clothoid, this function
  computes abscissa and ordinate coordinates. The pseudocode was
  implemented by Miˇsel Brezak and Ivan Petrovi´c and it was described
  in the paper Real-time Approximation of Clothoids With Bounded Error
  for Path Planning Applications, IEEE Transactions on Robotics, 2014.
  IEEE, v. 30, n. 2, p. 507–515, 2014.

  @param x0: initial abscissa coordinate
  @param y0: initial ordinate coordinate
  @param theta0: initial angle
  @param kappa0: initial curvature derivate
  @param c: curvature derivate constant
  @param s: distance travelled along the clothoid

  @return: a pair of abscissa and ordinate coordinates in meters
  */
  
  theta0 = theta0 + 3 * M_PI / 2 + std::copysign(1.0, c) * M_PI / 2;
  const double kC = 1. / (std::sqrt(std::abs(c)));                                                
  const double kK = kappa0 * kC;
  double s1 = kCL * (s / kC + kK);
  double s2 = kCL * kK;                                               
  double x, y;                                                    
  if ((kC < kCMin) && (std::abs(s1) > kSL || std::abs(s2) > kSL)) {
    x = x0;
    y = y0;
  }
  else if ((kappa0 == 0) && (kC > kCMax)){
    x = x0 + std::cos(theta0) * s;
    y = y0 + std::sin(theta0) * s;
  }
  else if(std::abs(kK) > kKMax){
    x = x0 + (1. / kappa0) * (-std::sin(theta0) + std::sin(theta0 + kappa0 * s));
    y = y0 + (1. / kappa0) * (std::cos(theta0) - std::cos(theta0 + kappa0 * s));
  }     
  else{
    double theta_rot = (-std::pow(kappa0, 2) / 2) / c + theta0;
    double x1, y1, x2, y2;
    std::tie(x1, y1) = GetBasicClothoidCoords(s1);
    std::tie(x2, y2) = GetBasicClothoidCoords(s2);

    double r11 = std::cos(theta_rot);
    double r22 = r11;
    double r12 = -std::sin(theta_rot);
    double r21 = -r12;

    double xz = (x1 - x2) * kC / kCL;
    double yz = (y1 - y2) * (kC / kCL) * std::copysign(1.0, c);
    x = x0 + r11 * xz + r12 * yz;
    y = y0 + r21 * xz + r22 * yz;
  }  
  return std::make_pair(x, y);
}

double ClothoidPath::cosC(double delta) {
  /*
  @brief
  Calculates the "clothoid cossine" of delta

  Description: Given the angle delta, this function computes the
  "clothoid cossine" of delta. The pseudocode was implemented by Doran
  K. Wilde and it was described in the paper Computing clothoid segments
  for trajectory generation. In: IEEE. 2009 IEEE/RSJ International
  Conference on Intelligent Robots and Systems. 20010. p. 2440–2445.

  @param delta: input angle (rad)

  @return: "clothoid cossine" (rad)
  */
  double t1 = 0.0;
  double t2 = std::sqrt(2 * std::abs(delta) / M_PI);
  double c_c, s_c;
  if (delta > 0){
    std::tie(c_c, s_c) = GetGeneralClothoidCoords(0, 0, 0, 0, M_PI, (t2 - t1));
    return (std::cos(delta) * c_c + std::sin(delta) * s_c) / t2;
  }
  else if (delta == 0){
    return 1.0;
  }
  else{
    std::tie(c_c, s_c) = GetGeneralClothoidCoords(0, 0, 0, 0, -M_PI, (t2 - t1));
    return (std::cos(-delta) * c_c + std::sin(-delta) * s_c) / t2;
  }
}

double ClothoidPath::sinC(double delta) {
  /*
  @brief
  Calculates the "clothoid sine" of delta

  Description: Given the angle delta, this function computes the
  "clothoid sine" of delta. The pseudocode was implemented by Doran
  K. Wilde and it was described in the paper Computing clothoid segments
  for trajectory generation. In: IEEE. 2009 IEEE/RSJ International
  Conference on Intelligent Robots and Systems. 20010. p. 2440–2445.

  @param delta: input angle (rad)

  @return: "clothoid sine" (rad)
  */
  double t1 = 0.0;
  double t2 = std::sqrt(2 * std::abs(delta) / M_PI);
  double c_c, s_c;
  if (delta > 0){
    std::tie(c_c, s_c) = GetGeneralClothoidCoords(0, 0, 0, 0, M_PI, (t2 - t1));
    return (std::sin(delta) * c_c - std::cos(delta) * s_c) / t2;
  }
  else if (delta == 0){
    return 0.0;
  }
  else{
    std::tie(c_c, s_c) = GetGeneralClothoidCoords(0, 0, 0, 0, -M_PI, (t2 - t1));
    return (std::sin(-delta) * c_c - std::cos(-delta) * s_c) / t2;
  }
}

} // namespace clothoid