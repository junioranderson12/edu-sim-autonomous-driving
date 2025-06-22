#include "clothoid_planner/clothoid_path.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <tuple>
#include <numeric>
#include <Eigen/Dense>

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

double ClothoidPath::CosC(double delta) {
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
    std::tie(c_c, s_c) = GetGeneralClothoidCoords(0.0, 0.0, 0.0, 0.0, M_PI, (t2 - t1));
    return (std::cos(delta) * c_c + std::sin(delta) * s_c) / t2;
  }
  else if (delta == 0){
    return 1.0;
  }
  else{
    std::tie(c_c, s_c) = GetGeneralClothoidCoords(0.0, 0.0, 0.0, 0.0, -M_PI, (t2 - t1));
    return (std::cos(-delta) * c_c + std::sin(-delta) * s_c) / t2;
  }
}

double ClothoidPath::SinC(double delta) {
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
    std::tie(c_c, s_c) = GetGeneralClothoidCoords(0.0, 0.0, 0.0, 0.0, M_PI, (t2 - t1));
    return (-std::sin(delta) * c_c + std::cos(delta) * s_c) / t2;
  }
  else if (delta == 0){
    return 0.0;
  }
  else{
    std::tie(c_c, s_c) = GetGeneralClothoidCoords(0.0, 0.0, 0.0, 0.0, -M_PI, (t2 - t1));
    return (std::sin(-delta) * c_c - std::cos(-delta) * s_c) / t2;
  }
}

std::tuple<double, double, double> ClothoidPath::MaxValues1(double delta, double x) {
  /*
  @brief
  Calculates the maximum curvature, curvature derivative and arc
  length required in a turn using a single clothoid.

  Description: Given the defleciotn angle delta and the longiturnal travelled
  distance, this function computes the length, maximum curvature and
  curvature derivative of the clothoid. These parameters are used later
  to compute the clothoid path. The pseudocode was implemented by Doran
  K. Wilde and it was described in the paper Computing clothoid segments
  for trajectory generation. In: IEEE. 2009 IEEE/RSJ International
  Conference on Intelligent Robots and Systems. 20010. p. 2440–2445.

  @param delta: input angle (rad)
  @param x: longitudinal travelled distance (m)

  @return
  tuple (L, kappa_max, sigma_max):
  [0] L             - clothoid arc length.
  [1] kappa_max     - maximum curvature (1/m).
  [2] sigma_max     - curvature derivative (constante by definition) (1/m^2)
  */
  if (delta < 0) {
    delta = -delta;
  }
  double t1 = 0.0;
  double t2 = std::sqrt(2 * delta / M_PI);
  double c_c, s_c;
  std::tie(c_c, s_c) = GetGeneralClothoidCoords(0, 0, 0, 0, M_PI, (t2 - t1));
  double cosc_delta = CosC(delta);
  double l = x / cosc_delta;
  double kappa_max = 2 * delta / l;
  double sigma_max = kappa_max / l;
  return std::make_tuple(l, kappa_max, sigma_max);
}

std::tuple<double, double, double> ClothoidPath::MaxValues2(double delta, double kappa_max, double x) {
  /*
  @brief
  Calculates the clothoid length, circular arc length and curvature
  derivative required in a turn with bounded curvature.

  Description: Given the deflection angle delta, the longiturnal travelled
  distance and the maximum curvature allowed, this function computes
  the length, maximum curvature and curvature derivative of the clothoid.
  These parameters are used later to compute the clothoid path. The
  pseudocode was implemented by Doran K. Wilde and it was described in
  the paper Computing clothoid segments for trajectory generation. In:
  IEEE. 2009 IEEE/RSJ International Conference on Intelligent Robots and
  Systems. 20010. p. 2440–2445.

  @param delta: input angle (rad)
  @param kappa_max: maximum curvature (1/m)
  @param x: longitudinal travelled distance (m)

  @return
  tuple (lc, l_arc, sigma_max):
  [0] Lc            - clothoid arc length (m).
  [1] Larc          - circular arc length (m).
  [2] sigma_max     - curvature derivative (constante by definition) (1/m^2)
  */
  if (delta < 0) {
    delta = -delta;
  }
  double lambda = delta / 2;
  double lambda_prev = 10;
  while (std::abs(lambda - lambda_prev) > 1e-6) { //Convergence criteria: 1e-6 rad
    lambda_prev = lambda;
    lambda = lambda - ((2 * lambda - 2 * delta) + 
             ((x * kappa_max - std::sin(lambda)) / 
             (CosC(delta - lambda) * std::cos(lambda) + 
             SinC(delta - lambda) * std::sin(lambda))));
  }
  double delta_c = delta - lambda;
  double lc = 2 * (delta_c) / kappa_max;
  double l_arc = lambda / kappa_max;
  double sigma_max = kappa_max / lc;
  return std::make_tuple(lc, l_arc, sigma_max);
}

double ClothoidPath::NormalizeAngle(double theta) {
  /*
  @brief
  Calculates the normalized angle (-pi to pi) of theta

  Description: This function computes the normalized angle (-pi to pi) of a
  given angle theta. For example, the normalizad angle of 3*pi/2 is
  -pi/2.

  @param angle: input angle in radians

  @return: normalized angle in radians
  */
  if (theta > M_PI) {
    theta = theta - 2 * M_PI;
  } else if (theta < -M_PI) {
    theta = 2 * M_PI + theta;
  }
  return theta;
}

ClothoidParams ClothoidPath::GetKinkPoints(ClothoidPathParams clothoid_path_params) {

  /*
  @brief
  Calculates the final configuration (x, y, theta and
  kappa) of a path constituded of clothoids, circular arc and straight
  lines. Also returns the initial configuration.

  Description: The final configuration is computed from the initial configuration
  x0, y0, theta0 and kappa0 and from all curvature derivative and length of
  all curve segments present in the path. As a piecewise linear path, the
  curvature derivative is constant along all segments.

  @param clothoid_path_params: a struct containing the initial configuration
  (x0, y0, theta0, kappa0) and vectors of curvature derivative and length
  of all segments in the path.

  @return: a tuple containing two ClothoidParams structs, the first one
  representing the initial configuration and the second one representing
  the final configuration of the path.
  */

  // Load initial parameters
  double x0 = clothoid_path_params.x0;
  double y0 = clothoid_path_params.y0;
  double theta0 = clothoid_path_params.theta0;
  double kappa0 = clothoid_path_params.kappa0;

  for (size_t i = 0; i < clothoid_path_params.c.size(); ++i) {
    double sigma = clothoid_path_params.c[i];
    double s = clothoid_path_params.s[i];

    if (std::abs(sigma) >= 1e-10) { // Compute clothoid
      sigma < 0 ? s = -s : s = s;
      std::tie(x0, y0) =
          GetGeneralClothoidCoords(x0, y0, theta0, kappa0, sigma, s);
    } else if (std::abs(kappa0) > 1e-6) { // Compute circular arc
      double alpha = kappa0 * s;
      int sign_alfa = (alpha > 0) ? 1 : -1;
        x0 += (1 / kappa0) * std::cos(theta0 + M_PI / 2) +
            std::cos(theta0 + alpha - sign_alfa * M_PI / 2) / std::abs(kappa0);
        y0 += (1 / kappa0) * std::sin(theta0 + M_PI / 2) +
            std::sin(theta0 + alpha - sign_alfa * M_PI / 2) / std::abs(kappa0);
    } else { // Compute straight line
      x0 += s * std::cos(theta0);
      y0 += s * std::sin(theta0);
    }
    s = std::abs(s);
    theta0 += kappa0 * s + (sigma * s * s) / 2;
    kappa0 += sigma * s;
  }
  //TODO Create struct ClothoidWayPoints to represent kink points
  return ClothoidParams{x0, y0, theta0, kappa0, 0.0, 0.0};
}

ClothoidPathParams ClothoidPath::CCTurn(Pose init_pose, Pose final_pose, double kappa_max) {
  /*
  @brief
  Calculates the paremeters required to perfom a turn with G2 continuity.

  Given the start and goal configurations, this function computes
  the curvature derivatives and lengths of all segments required to
  perform the configuration change. Also, the maneuver is made with
  bounded curvature passed as input.

  @param init_pose: a struct containing the initial pose (x0, y0, theta0)
  @param final_pose: a struct containing the final pose (x1, y1, theta1)

  @return: a struct containing the configuration of clothoid path.
  */

  double x0 = init_pose.x;
  double y0 = init_pose.y;
  double theta0 = init_pose.theta;
  double kappa0 = 0.0; // Initial curvature is zero
  double x1 = final_pose.x;
  double y1 = final_pose.y;
  double theta1 = final_pose.theta;

  ClothoidPathParams clothoid_path_params;
  clothoid_path_params.x0 = x0;
  clothoid_path_params.y0 = y0;
  clothoid_path_params.theta0 = theta0;
  clothoid_path_params.kappa0 = kappa0;
  
  double mod_v1 = std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
  double v1_ang = NormalizeAngle(std::atan2((y1 - y0) / mod_v1, (x1 - x0) / mod_v1));
  double alpha1 = NormalizeAngle(v1_ang - theta0);
  double alpha2 = NormalizeAngle(theta1 - v1_ang);
  double alpha = std::abs(NormalizeAngle(theta1 - theta0) / 2);
  double alfa_aux = (std::abs(alpha1) >= std::abs(alpha2)) ? alpha1 : alpha2;

  double z = std::sin(std::abs(alfa_aux) - alpha) * mod_v1;
  double y = z / (1e-8 + std::tan(alpha));
  double x = (z != 0) ? (z - y * std::tan(std::abs(alfa_aux) - alpha)) / 
    std::tan(std::abs(alfa_aux) - alpha) : mod_v1;
  
  double delta = alpha1 + alpha2;
  bool flag = false;
  x /= 2;
  delta /= 2;
  if (delta < 0) {
    delta = -delta;
    flag = true;
  }

  auto [l, kappa, sigma] = MaxValues1(delta, x);
  if (kappa > kappa_max) {
    std::tuple<double, double, double> max_val2 = MaxValues2(delta, kappa_max, x);
    sigma = std::get<2>(max_val2);
    double delta_min = kappa_max * kappa_max / sigma;
    double lc = kappa_max / sigma;
    flag ? sigma = -sigma : sigma = sigma;
    clothoid_path_params.c.insert(clothoid_path_params.c.end(),
      {sigma, 0.0, -sigma});
    clothoid_path_params.s.insert(clothoid_path_params.s.end(), 
      {lc, (2 * delta - delta_min) / kappa_max, lc});
  } else {
    flag ? sigma = -sigma : sigma = sigma;
    clothoid_path_params.c.insert(clothoid_path_params.c.end(),
      {sigma, -sigma});
    clothoid_path_params.s.insert(clothoid_path_params.s.end(), 
      {l, l});
  }

  //TODO : make this part another function
  // Compute the angles of the straight lines
  ClothoidParams final_params = GetKinkPoints(clothoid_path_params);
  double xf = final_params.x0;
  double yf = final_params.y0;
  double xp = std::abs(NormalizeAngle(theta0)) != M_PI / 2 ? 
    (y1 - yf + std::tan(theta0) * xf - std::tan(theta1) * x1) /
    (std::tan(theta0) - std::tan(theta1) + 1e-8) : xf;
  double yp = std::abs(NormalizeAngle(theta1)) != M_PI / 2 ?
    (std::tan(theta1) * (xp - x1) + y1) : yf;

  double r1 = std::sqrt(std::pow(xp - xf, 2) + std::pow(yp - yf, 2)); //length of first straight line
  double r2 = std::sqrt(std::pow(xp - x1, 2) + std::pow(yp - y1, 2)); //length of second straight line
  if ((r1 < kMinStraightLength) && (r2 > kMinStraightLength)) {
    clothoid_path_params.c.push_back(0.0);
    clothoid_path_params.s.push_back(r2);
  } else if ((r1 > kMinStraightLength) && (r2 < kMinStraightLength)) {
    //TODO : Optimize this case to avoid O(n) complexity
    clothoid_path_params.c.insert(clothoid_path_params.c.begin(), 0.0);
    clothoid_path_params.s.insert(clothoid_path_params.s.begin(), r1);
  } else if ((r1 > kMinStraightLength) && (r2 > kMinStraightLength)) {
    clothoid_path_params.c.insert(clothoid_path_params.c.begin(), 0.0);
    clothoid_path_params.s.insert(clothoid_path_params.s.begin(), r1);
    clothoid_path_params.c.push_back(0.0);
    clothoid_path_params.s.push_back(r2);
  }
  
  return clothoid_path_params;
}

ClothoidPathParams ClothoidPath::CCSshape(Pose init_pose, Pose final_pose, double kappa_max) {
  /*
  @brief
  Calculates the parameters required to perfom a S-shape maneuver with G2 continuity.

  Given the start and goal configurations, this function computes
  the curvature derivatives and lengths of all segments required to
  perform S-shape interpolation. Also, the maneuver is made with
  bounded curvature passed as input.

  @param init_pose: a struct containing the initial pose (x0, y0, theta0)
  @param final_pose: a struct containing the final pose (x1, y1, theta1)

  @return: a struct containing the configuration of clothoid path.
  */

  double x0 = init_pose.x;
  double y0 = init_pose.y;
  double theta0 = init_pose.theta;
  double kappa0 = 0.0; // Initial curvature is zero
  double x1 = final_pose.x;
  double y1 = final_pose.y;
  double theta1 = final_pose.theta;
  
  double mod_v1 = std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
  double v1_ang = NormalizeAngle(std::atan2((y1 - y0) / mod_v1, (x1 - x0) / mod_v1));
  double alpha1 = NormalizeAngle(v1_ang - theta0);
  double alpha2 = NormalizeAngle(theta1 - v1_ang);
  double alpha = std::abs(NormalizeAngle(theta1 - theta0) / 2);

  ClothoidPathParams clothoid_path_params;
  int sign_alpha1 = (alpha1 > 0) ? 1 : -1;
  int sign_alpha2 = (alpha2 > 0) ? 1 : -1;

  if ((sign_alpha1 == sign_alpha2) && (std::abs(alpha1) > 0) && (std::abs(alpha2) > 0)) {
    clothoid_path_params = CCTurn(init_pose, final_pose, kappa_max);
  } else if (alpha1 > 1e-6 || alpha1 < -1e-6) {
    int sign_alfa1 = alpha1 > 0 ? 1 : -1;
    double phi1 = (3 * std::abs(alpha1) + std::abs(alpha2)) / 4;
    double phi2 = -(std::abs(alpha1) - std::abs(alpha2) - 2 * std::abs(phi1)) / 2;
    double l3 = mod_v1 / (std::cos(std::abs(phi1) - std::abs(alpha1)) + 
                         std::cos(std::abs(alpha2) - std::abs(phi2)));
    double npoint_x = x0 + l3 * std::cos(theta0 + sign_alfa1 * std::abs(phi1));
    double npoint_y = y0 + l3 * std::sin(theta0 + sign_alfa1 * std::abs(phi1));
    Pose npoint = {npoint_x, npoint_y, theta0 + sign_alfa1 * 2 * std::abs(phi1)};
    
    clothoid_path_params = CCTurn(init_pose, npoint, kappa_max);
    ClothoidPathParams path_params2 = CCTurn(npoint, final_pose, kappa_max);
    clothoid_path_params.c.insert(clothoid_path_params.c.end(), 
                        path_params2.c.begin(), path_params2.c.end());
    clothoid_path_params.s.insert(clothoid_path_params.s.end(), 
                        path_params2.s.begin(), path_params2.s.end());
  } else {
    clothoid_path_params.x0 = x0;
    clothoid_path_params.y0 = y0;
    clothoid_path_params.theta0 = theta0;
    clothoid_path_params.kappa0 = kappa0;
    clothoid_path_params.c.push_back(0.0);
    clothoid_path_params.s.push_back(mod_v1);
  }
  return clothoid_path_params;

}

std::vector<ClothoidParams> ClothoidPath::GetClothoidPath(
  ClothoidPathParams clothoid_path_params, double ds) {
  /*
  @brief
  Calculates a path made of clothoids, circular arcs and straigh
  straight lines with a given step.

  The path is computed from the initial configuration
  x0, y0, theta0 and kappa0 and from all curvature derivative and length of
  all curve segments present in the path. As a piecewise linear path, the
  curvature derivative is constant along all segments. The step, i.e.,
  the distance between each point is passed as input.

  @param clothoid_path_params: a struct containing the initial configuration
  (x0, y0, theta0, kappa0) and vectors of curvature derivative and length
  of all segments in the path.
  @param ds: step to compute the path

  @return: a vector of ClothoidParams structs containing the path waypoints.
  */

  //Start and stop criteria with respect to length
  double l_initial = 0.0;
  const double kLfinal = std::accumulate(clothoid_path_params.s.begin(),
                                   clothoid_path_params.s.end(), 0.0);
  double l = l_initial;
  
  //Initiate parameters
  double x0 = clothoid_path_params.x0;
  double y0 = clothoid_path_params.y0;
  double theta0 = clothoid_path_params.theta0;
  double kappa0 = clothoid_path_params.kappa0;
  double c = clothoid_path_params.c[0];
  double s = 0.0;

  //Initiate counters
  int k = 0; //parameters counter
  int m = 0; //way points counter

  //Adjust step according to sharpness
  if (c != 0) {
    int sign_c = c > 0 ? 1 : -1;
    ds = abs(ds) * sign_c;
    s = abs(s) * sign_c;
  }
  
  double xc, yc, thetac, kappac, sigmac, sc;
  std::vector<ClothoidParams> clothoid_waypoints;
  while (l <= kLfinal) {
    if (c != 0) {
      std::tie(xc, yc) = GetGeneralClothoidCoords(x0, y0, theta0, kappa0, c, s);
    } else if (std::abs(kappa0) > 1e-6) {
      double alpha = kappa0 * s;
      int sign_alfa = (alpha > 0) ? 1 : -1;
      xc = (1 / kappa0) * std::cos(theta0 + M_PI / 2) +
           std::cos(theta0 + alpha - sign_alfa * M_PI / 2) / std::abs(kappa0) + x0;
      yc = (1 / kappa0) * std::sin(theta0 + M_PI / 2) +
           std::sin(theta0 + alpha - sign_alfa * M_PI / 2) / std::abs(kappa0) + y0;
    } else {
      xc = x0 + s * std::cos(theta0);
      yc = y0 + s * std::sin(theta0);
    }
    thetac = theta0 + kappa0 * std::abs(s) + (c * s * s) / 2;
    kappac = kappa0 + c * std::abs(s);
    sigmac = c;
    sc = l;
    clothoid_waypoints.push_back({xc, yc, thetac, kappac, sigmac, sc});

    s += ds;
    l += std::abs(ds);
    if (l + std::abs(ds) > kLfinal) {
      ClothoidPathParams clot_path_params_aux = {x0, y0, theta0, kappa0, {c}, 
                                        {clothoid_path_params.s[k]}};
      ClothoidParams clot_params_aux = GetKinkPoints(clot_path_params_aux);
      xc = clot_params_aux.x0;
      yc = clot_params_aux.y0;
      thetac = clot_params_aux.theta0;
      kappac = clot_params_aux.kappa0;
      sigmac = c;
      sc = l;
      clothoid_waypoints.push_back({xc, yc, thetac, kappac, sigmac, sc});
      break;
    }

    while (std::abs(s) > clothoid_path_params.s[k]) {
      //Intial parameters of the next segment
      ClothoidPathParams clot_path_params_aux = {x0, y0, theta0, kappa0, {c}, 
                                        {clothoid_path_params.s[k]}};
      ClothoidParams clot_params_aux = GetKinkPoints(clot_path_params_aux);
      x0 = clot_params_aux.x0;
      y0 = clot_params_aux.y0;
      theta0 = clot_params_aux.theta0;
      kappa0 = clot_params_aux.kappa0;
      
      //Uptade length, sharpness and step
      s = std::abs(s) - clothoid_path_params.s[k];
      k += 1;
      c = clothoid_path_params.c[k];
      
      //Adjust step according to sharpness
      if (c != 0) {
        int sign_c = c > 0 ? 1 : -1;
        ds = abs(ds) * sign_c;
        s = abs(s) * sign_c;
      } else {
        ds = std::abs(ds);
      }
    }
    m += 1;
  }
  return clothoid_waypoints;
}

OptimalClothoidParams ClothoidPath::GetOptimalClothoidParams(ClothoidParams init_params,
                                                    Pose final_pose,
                                                    double kappaf,
                                                    OptimalClothoidParams init_solution) {
  /*

  */
  Eigen::Vector3d delta_Xf;
  Pose diff_pose = {1e10, 1e10, 1e10}; // Initialize with a large value
  OptimalClothoidParams optimal_params = init_solution;
  Eigen::Vector3d P, P_prev;                                   
  double k = 0; //Iteration counter                                                    
  while ((std::abs(diff_pose.x) > 0.01) && (std::abs(diff_pose.y) > 0.01) && 
          (std::abs(diff_pose.theta) > 0.01)) {
    double sigma1_opt = optimal_params.sigma1;
    double sigma2_opt = optimal_params.sigma2;
    double s_sum_opt = optimal_params.s_sum;
    
    double kappa_middle = init_params.kappa0 + sigma1_opt * s_sum_opt / div_s1 +
                          sigma2_opt * s_sum_opt / div_s2;
    double sigma3 = (kappaf - kappa_middle) / (s_sum_opt / div_s3);

    auto get_clothoid_path_params = [&](double sigma1_opt, double sigma2_opt,
                                          double s_sum_opt) {                            
      ClothoidPathParams clothoid_path_params = {
        init_params.x0, init_params.y0, init_params.theta0, init_params.kappa0,
        {sigma1_opt, sigma2_opt, sigma3},
        {s_sum_opt / div_s1, s_sum_opt / div_s2, s_sum_opt / div_s3}
      };
      return clothoid_path_params;
    };
    auto get_d_dvar = [&](double delta_sigma1, double delta_sigma2, double delta_s_sum) { 
      ClothoidPathParams clothoid_path_params1 = 
        get_clothoid_path_params(sigma1_opt + delta_sigma1,
                                  sigma2_opt + delta_sigma2,
                                  s_sum_opt + delta_s_sum);
      ClothoidPathParams clothoid_path_params2 =
        get_clothoid_path_params(sigma1_opt - delta_sigma1,
                                  sigma2_opt - delta_sigma2,
                                  s_sum_opt - delta_s_sum);
      ClothoidParams clot_params1 = GetKinkPoints(clothoid_path_params1);
      ClothoidParams clot_params2 = GetKinkPoints(clothoid_path_params2);
      ClothoidParams dparams_dvar = clot_params1 - clot_params2;
      double dx_dvar = dparams_dvar.x0 / (2 * kDeltaLim);
      double dy_dvar = dparams_dvar.y0 / (2 * kDeltaLim);
      double dtheta_dvar = dparams_dvar.theta0 / (2 * kDeltaLim);
      return std::make_tuple(dx_dvar, dy_dvar, dtheta_dvar);
    };

    auto [dx_dsig1, dy_dsig1, dtheta_dsig1] = get_d_dvar(kDeltaLim, 0.0, 0.0);
    auto [dx_dsig2, dy_dsig2, dtheta_dsig2] = get_d_dvar(0.0, kDeltaLim, 0.0);
    auto [dx_ds_sum, dy_ds_sum, dtheta_ds_sum] = get_d_dvar(0.0, 0.0, kDeltaLim);
    
    ClothoidPathParams clothoid_path_params =
      get_clothoid_path_params(sigma1_opt, sigma2_opt, s_sum_opt);
    ClothoidParams clot_params = GetKinkPoints(clothoid_path_params);
    Pose current_pose = {clot_params.x0,
                         clot_params.y0,
                         clot_params.theta0};
    diff_pose = final_pose - current_pose;
    delta_Xf << diff_pose.x, diff_pose.y, NormalizeAngle(diff_pose.theta);
    P_prev << sigma1_opt, sigma2_opt, s_sum_opt;
    Eigen::Matrix3d Jacobian;
    Jacobian << dx_dsig1, dx_dsig2, dx_ds_sum,
         dy_dsig1, dy_dsig2, dy_ds_sum,
         dtheta_dsig1, dtheta_dsig2, dtheta_ds_sum;
    Eigen::Matrix3d Jacobian_inv = Jacobian.inverse();
    P = - Jacobian_inv * -delta_Xf + P_prev;  
    optimal_params.sigma1 = P(0);
    optimal_params.sigma2 = P(1);
    optimal_params.s_sum = P(2);
    k += 1;
    if (k > kMaxIterations) {
      std::cout << "Maximum number of iterations reached. "
                << "Returning the last solution." << std::endl;
      break;
    }
  }
  return optimal_params;
}

ClothoidPathParams ClothoidPath::GetOptimalManeuver(ClothoidParams init_params,
                                                      Pose final_pose,
                                                      double kappaf) {
  /*

  */
  Pose init_pose = {init_params.x0, init_params.y0, init_params.theta0};
  ClothoidPathParams clot_path_params;

  //Compute initial parameters to speed up the optimization process
  if (std::abs(NormalizeAngle(final_pose.theta - init_pose.theta)) < 0.36) {
    clot_path_params = CCSshape(init_pose, final_pose, 10.0);
    div_s1 = 4.0;
    div_s2 = 2.0;
    div_s3 = 4.0;
  } else {
    clot_path_params = CCTurn(init_pose, final_pose, 10.0);
    div_s1 = 2.5;
    div_s2 = 5.0;
    div_s3 = 2.5;
  }
  double sigma1 = clot_path_params.c[0];
  double sigma2 = clot_path_params.c[1];
  double s_sum = std::accumulate(clot_path_params.s.begin(),
                                  clot_path_params.s.end(), 0.0);
  OptimalClothoidParams solution = {sigma1, sigma2, s_sum};
  solution =  GetOptimalClothoidParams(init_params, final_pose, kappaf, solution);

  //We need to compute sigma3 to get the whole path
  double kappa2 = init_params.kappa0 + solution.sigma1 * solution.s_sum / div_s1 +
                          solution.sigma2 * solution.s_sum / div_s2;
  double sigma3 = (kappaf - kappa2) / (s_sum / div_s3);

  ClothoidPathParams optimal_clothoid_path_params = {
    init_params.x0, init_params.y0, init_params.theta0, init_params.kappa0,
    {solution.sigma1, solution.sigma2, sigma3},
    {solution.s_sum / div_s1, solution.s_sum / div_s2, s_sum / div_s3}
  };
  return optimal_clothoid_path_params;
}

} // namespace clothoid