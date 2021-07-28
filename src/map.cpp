#include "spline.h"
#include "parameters.h"
#include "map.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>


/** Highway map class. 
 * 
 * The highway is represented by its center line and is defined by the 
 * waypoints in the map file which hold 
 * 
 *    - the global x and y coordinates 
 *    - the Frenet s coordinate 
 *    - and the components of the Frenet unit normal vector n_r = (dx, dy) 
 * 
 * However, the raw waypoints are approximatly 30 meters apart from each other 
 * which is too coarse to be used directly for the several sub tasks of the 
 * planning problem. We will therefore use splines to fill the gaps. This will 
 * be beneficial for the generation of jerk-minimal polynomial trajectories of 
 * 
 * @param map_file the file containing the map data
*/
Map::Map(std::string map_file) {
  // Read the raw waypoints from the file 
  std::ifstream map_stream(map_file.c_str(), std::ifstream::in);
  std::string line;
  while (getline(map_stream, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    double s;
    double dx;
    double dy;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;
    coarse_wpts_x.push_back(x);
    coarse_wpts_y.push_back(y);
    coarse_wpts_s.push_back(s);
    coarse_wpts_dx.push_back(dx);
    coarse_wpts_dy.push_back(dy);
  }

  // Reuse the first waypoint as the last to get a decent spline approximation 
  // as the track circuit closes
  coarse_wpts_s.push_back(SMAX);
  coarse_wpts_x.push_back(coarse_wpts_x[0]);
  coarse_wpts_y.push_back(coarse_wpts_y[0]);
  coarse_wpts_dx.push_back(coarse_wpts_dx[0]);
  coarse_wpts_dy.push_back(coarse_wpts_dy[0]);

  // Compute splines for the x, y and d unit normal vector in dependence of the 
  // Frenet s-coordinate
  spline_x.set_points(coarse_wpts_s, coarse_wpts_x);
  spline_y.set_points(coarse_wpts_s, coarse_wpts_y);
  spline_dx.set_points(coarse_wpts_s, coarse_wpts_dx);
  spline_dy.set_points(coarse_wpts_s, coarse_wpts_dy);

  // Interpolate the new waypoints with a higher density using the splines 
  for (double s = 0; s <= floor(SMAX); s++) {
    fine_wpts_x.push_back(spline_x(s));
    fine_wpts_y.push_back(spline_y(s));
    fine_wpts_s.push_back(s);
    fine_wpts_dx.push_back(spline_dx(s));
    fine_wpts_dy.push_back(spline_dy(s));
    // The s unit normal vector is perpendicular to d unit normal vector 
    double sx = -fine_wpts_dy.back();
    double sy = +fine_wpts_dx.back();
    // Compute the orientation of the highway center line 
    double theta = atan2(sy, sx);  
    fine_wpts_theta.push_back(theta);
  }

  // Reuse the first waypoint as the last 
  fine_wpts_s.push_back(SMAX);
  fine_wpts_x.push_back(fine_wpts_x[0]);
  fine_wpts_y.push_back(fine_wpts_y[0]);
  fine_wpts_dx.push_back(fine_wpts_dx[0]);
  fine_wpts_dy.push_back(fine_wpts_dy[0]);
  fine_wpts_theta.push_back(fine_wpts_theta[0]);

  // Remove phase shifts from the theta sequence to get a continuous curve
  for (int i = 0; i < fine_wpts_theta.size(); ++i) {
    double theta = fmod(fine_wpts_theta[i] + 2 * M_PI, 2 * M_PI) - 2 * M_PI;
    double theta_previous = i > 0 ? fine_wpts_theta[i-1] : theta;
    if (fabs(theta_previous - theta) > 6) {
      theta = theta < theta_previous ? theta + 2 * M_PI : 2 * M_PI - theta;
    } 
    fine_wpts_theta[i] = theta;
  } 

  // Compute the theta spline based on the waypoints with higher density
  spline_theta.set_points(fine_wpts_s, fine_wpts_theta);

  // Determine max curvature on highway
  max_curvature = 0;
  for (auto s: fine_wpts_s) {
    double curvature = this->curvature(s);
    max_curvature = fabs(curvature) > fabs(max_curvature) 
                    ? curvature : max_curvature;
  }
}

/** Transformation of s-/d- to x-/y-coordinates.
 * 
 * @param s s coordinate
 * @param d d coordinate
 * @returns a vector with the {x, y} coordinates
 */
std::vector<double> Map::transform_sd2xy(double s, double d) {
  s = fmod(s, SMAX);
  double x = this->spline_x(s) + d * this->spline_dx(s);
  double y = this->spline_y(s) + d * this->spline_dy(s);
  return {x, y};
};

/** Compute the curvature at location s.
 * 
 * @param s s coordinate
 * @returns the signed curvature
 */
double Map::curvature(double s) {
  // Minus sign to achieve a positive curvature for a bending into the 
  // normal (dx|dy) direction of the reference curve. This leads to the
  // same curvature sign as in the paper.
  return -this->spline_theta.deriv(1, s);
};

