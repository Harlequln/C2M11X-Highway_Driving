#include "parameters.h"
#include "utils.h"

#include <cmath>


// Convertions between miles per hour and meters per second
double mph2ms(double v) { return v / 2.237; }
double ms2mph(double v) { return v * 2.237; }

// Convertions between rad and degree
double deg2rad(double phi) { 
  while (phi < 0) { phi += 360; }
  return fmod(phi * M_PI / 180, 2 * M_PI); 
}
double rad2deg(double phi) { 
  while (phi < 0) { phi += 2 * M_PI; }
  return fmod(phi * 180 / M_PI, 360); }

/** 
 * Compute the distance of two points.
 * 
 * @param x1 x coordinate of 1st point
 * @param y1 y coordniate of 1st point
 * @param x2 x coordinate of 2nd point
 * @param y2 y coordniate of 2nd point
 */
double distance(double x1, double y1, double x2, double y2) {
  return std::sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

/** Compute the distance between two s locations in the Frenet system. 
 * 
 * @param s1 s coordniate of 1st point
 * @param s2 s coordinate of 2nd point
 */
double s_distance(double s1, double s2) {
  // Handle finished laps on the highway map
  double s1_dist_to_origin = s1 < SMAX / 2 ? s1 : SMAX - s1;
  double s2_dist_to_origin = s2 < SMAX / 2 ? s2 : SMAX - s2;
  return fmin(fabs(s1 - s2), s1_dist_to_origin + s2_dist_to_origin);
}

// Return the lane identifier for the given d coordinate. 
int lane(double d) { 
  return (int)(d / LANE_WIDTH); 
}

// Return the lane identifier of a possible vehicle lane change destination. 
int possible_adjacent_lane(double d) {
  double delta = 1.0;
  int possible_right = lane(d) != lane(d + delta); 
  int possible_left = lane(d) != lane(d - delta); 
  if (possible_right) { return lane(d + delta); } 
  else if (possible_left) { return lane(d - delta); }
  else { return lane(d); }
}

/** Evenly spaced numbers over a specified interval.
 * @param start start value of interval
 * @param stop end value of interval, will be included
 * @param num number of samples to generate
 */
std::vector<double> linspace(double start, double stop, int num) {
  std::vector<double> result;
  double step = (stop - start) / (num - 1);
  for (int i = 0; i < num; ++i) {
    result.push_back(start + step * i);
  }
  return result;  
}
