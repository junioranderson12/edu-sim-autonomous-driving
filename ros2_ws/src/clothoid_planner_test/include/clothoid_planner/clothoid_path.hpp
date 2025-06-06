#ifndef CLOTHOIDPATH_HPP
#define CLOTHOIDPATH_HPP

#include <utility>
#include <vector>

namespace clothoid{

struct ClothoidParams {
  double x0;      // Initial x-coordinate
  double y0;      // Initial y-coordinate
  double theta0;  // Initial orientation (angle)
  double kappa0;  // Initial curvature
  double c;       // Clothoid sharpness
  double s;       // Distance along the clothoid
};

class ClothoidPath {
  public:
    ClothoidPath();
    std::pair<std::vector<double>, std::vector<double>> LoadXLYLConfigParams();
    std::pair<double, double> GetBasicClothoidCoords(double s);
    std::pair<double, double> GetGeneralClothoidCoords(double x0, double y0,
                                                        double theta0, double kappa0,
                                                        double c, double s);
    double cosC(double delta);
    double sinC(double delta);

    std::pair<std::vector<double>, std::vector<double>> XlYl;
    const double kCMin = 5.944748225238571e-04;
    const double kCMax = 10.237604306862353e+03;
    const double kKMax = 1.196835466593837e+02;
    const double kCL = 1.;
    const double kSL = 1.223430214110182e+02;
    const double kDeltaSL = 0.002660093525200;
    const double kCcL = 1.;
    const std::vector<double> kXL = XlYl.first;
    const std::vector<double> kYL = XlYl.second;
};
} // namespace clothoid_path
#endif // CLOTHOIDPATH_HPP