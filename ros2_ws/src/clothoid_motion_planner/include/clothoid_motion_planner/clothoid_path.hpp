#ifndef CLOTHOIDPATH_HPP
#define CLOTHOIDPATH_HPP

#include <utility>
#include <vector>

namespace clothoid{

struct Pose {
  double x;
  double y;
  double theta;

  Pose operator-(const Pose& other) const {
    return Pose{x - other.x,
                y - other.y,
                theta - other.theta};
  }
};

struct ClothoidParams {
  double x0;      // Initial x-coordinate
  double y0;      // Initial y-coordinate
  double theta0;  // Initial orientation (angle)
  double kappa0;  // Initial curvature
  double c;       // Clothoid sharpness
  double s;       // Distance along the clothoid

  ClothoidParams operator-(const ClothoidParams& other) const {
    return ClothoidParams{x0 - other.x0,
                          y0 - other.y0,
                          theta0 - other.theta0,
                          kappa0 - other.kappa0,
                          c - other.c,
                          s - other.s};
  }
};

struct ClothoidPathParams {
  double x0;      // Initial x-coordinate
  double y0;      // Initial y-coordinate
  double theta0;  // Initial orientation (angle)
  double kappa0;  // Initial curvature
  std::vector<double> c;       // Clothoid sharpness
  std::vector<double> s;       // Distance along the clothoid
};

struct OptimalClothoidParams {
  double sigma1;      
  double sigma2;     
  double s_sum;
};


class ClothoidPath {
  public:
    ClothoidPath();
    std::pair<std::vector<double>, std::vector<double>> LoadXLYLConfigParams();
    std::pair<double, double> GetBasicClothoidCoords(double s);
    std::pair<double, double> GetGeneralClothoidCoords(double x0, double y0,
                                                        double theta0, double kappa0,
                                                        double c, double s);
    double CosC(double delta);
    double SinC(double delta);
    std::tuple<double, double, double> MaxValues1(double delta, double x);
    std::tuple<double, double, double> MaxValues2(double delta, double kappa_max, double x);
    double NormalizeAngle(double angle);
    ClothoidParams GetKinkPoints(ClothoidPathParams clothoid_path_params);
    ClothoidPathParams CCTurn(Pose init_pose, Pose final_pose, double kappa_max);
    ClothoidPathParams CCSshape(Pose init_pose, Pose final_pose, double kappa_max);
    std::vector<ClothoidParams> GetClothoidPath(ClothoidPathParams clothoid_path_params,
                                                double ds);
    OptimalClothoidParams GetOptimalClothoidParams(ClothoidParams init_params,
                                                    Pose final_pose,
                                                    double kappaf,
                                                    OptimalClothoidParams init_solution);
    ClothoidPathParams GetOptimalManeuver(ClothoidParams init_params,
                                                    Pose final_pose,
                                                    double kappaf);
    std::pair<std::vector<double>, std::vector<double>> XlYl;
    const double kCMin = 5.944748225238571e-04;
    const double kCMax = 10.237604306862353e+03;
    const double kKMax = 1.196835466593837e+02;
    const double kCL = 1.;
    const double kSL = 1.223430214110182e+02;
    const double kDeltaSL = 0.002660093525200;
    const double kCcL = 1.;
    const double kMinStraightLength = 1e-6;
    const std::vector<double> kXL = XlYl.first;
    const std::vector<double> kYL = XlYl.second;
    const double kDeltaLim = 1e-5;
    const int kMaxIterations = 20;
    double div_s1, div_s2, div_s3;
};
} // namespace clothoid_path
#endif // CLOTHOIDPATH_HPP