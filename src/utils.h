#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <algorithm>

double mph2ms(double v);
double ms2mph(double v);
double deg2rad(double phi);
double rad2deg(double phi);
double distance(double x1, double y1, double x2, double y2);
double s_distance(double s1, double s2);

int lane(double d);
int possible_adjacent_lane(double d);

std::vector<double> linspace(double start, double stop, int num);

// Return the index of the maximum value in the given vector. 
template <class T> 
int maxloc(std::vector<T> &v) {
    return std::max_element(v.begin(), v.end()) - v.begin();
};

// Return the index of the minimum value in the given vector. 
template <class T> 
int minloc(std::vector<T> &v) {
    return std::min_element(v.begin(), v.end()) - v.begin();
};

// Return the minimum value of the given vector. 
template <class T> 
T minval(std::vector<T> &v) {
    return *std::min_element(v.begin(), v.end());
};

// Return the maximum value of the given vector. 
template <class T> 
T maxval(std::vector<T> &v) {
    return *std::max_element(v.begin(), v.end());
};

#endif  // UTILS_H

