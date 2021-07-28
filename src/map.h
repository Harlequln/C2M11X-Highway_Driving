#ifndef MAP_H
#define MAP_H

#include "spline.h"

#include <vector>
#include <string>

class Map {
public:
  std::string m_map;
  Map(std::string map="../data/highway_map.csv");
  ~Map() {};
  std::vector<double> transform_sd2xy(double s, double d);
  double curvature(double s);

  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;
  tk::spline spline_theta;
  double max_curvature;

  // Unprocessed waypoints of the map with 30 meter distance in between
  std::vector<double> coarse_wpts_x;
  std::vector<double> coarse_wpts_y;
  std::vector<double> coarse_wpts_s;
  std::vector<double> coarse_wpts_dx;
  std::vector<double> coarse_wpts_dy;
  // Interpolated waypoints of the map with 1 meter distance
  std::vector<double> fine_wpts_x;
  std::vector<double> fine_wpts_y;
  std::vector<double> fine_wpts_s;
  std::vector<double> fine_wpts_dx;
  std::vector<double> fine_wpts_dy;
  std::vector<double> fine_wpts_theta;
};

#endif //MAP_H