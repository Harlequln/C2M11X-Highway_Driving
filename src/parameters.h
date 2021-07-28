#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <vector>
#include <string>

// Dynamic thresholds
const double VMAX = 22.2;  // [m/s], maximum lane velocity (50 mph)
const double VLIM = 21.9;  // [m/s], decelerate above this velocity
const double AMAX = 10;  // [m/s^2], maximal feasible acceleration 
const double JMAX = 100;  // [m/s^3], maximal feasible jerk 

// Behaviour parameters
// Lateral successor state enumaration
struct LAT { enum SS {DOUBLE_LEFT=-2, SINGLE_LEFT=-1, 
                      KEEP=0, 
                      SINGLE_RIGHT=1, DOUBLE_RIGHT=2}; };
// Longitudinal successor state enumaration
struct LON { enum SS {KEEP=0, FOLLOW=1, FALLBACK=2}; };
// Driving scenario enumration
struct SCN { enum CASE {STANDARD=0, RECOVERY=1, EMERGENCY=2}; };
const double VEHICLE_LENGTH = 7;  // [m], for collision check (with buffer)
const double VEHICLE_WIDTH = 2.8;  // [m], for collison check (circle diameter)
const double LEAD_DETECT_DIST = 100;  // [m], leading vehicle detection horizon
const double NEARBY_DIST = 40;  // [m], nearby vehicles detection horizon
const double LANE_COST_DIST = 60;  // [m], activate lane cost horizon 
const double LEAD_FOLLOW_TIME = 1.3;  // [s], activate follow behaviour horizon
const double SAFETY_TIME = 0.5;  // [s], safety buffer behind leading vehicle
const double BLOCK_DIST = 5;  // [m], lane change blocking distance threshold 
const double DEADLOCK_GAP = 15;  // [m], maximum gap threshold for deadlock
const double LANE_COST_BONUS = 0.90;  // [-], lane cost bonus factor 
const double LANE_COST_MALUS = 2.0;  // [-], lane cost malus factor
// Trajectory parameters
const double DT = 0.02;  // [s], time step between two waypoints (= 20 ms)
const int NUM_TRAJECTORY_WPTS = 150;  // [-], full trajectory waypoint number
const int REPLAN_THRESHOLD = 10;  // [-], wpts to be consumed before replaning
const int NUM_TRANSITION_WPTS = 25;  // [-], wpts to keep for smooth transition
const int NUM_VARIATIONS = 3;  // [-], end state variations for each duration
// Weighting factors for the trajectory cost functionials
const double KJ = 0.2;  // jerk weight
const double KS = 1;  // longitudinal position residual weight
const double KD = 20;  // lateral position residual weight
const double KV = 1;  // velocity residual weight
const double KT = 20;  // trajectory completion duration weight
// const double KT = 10;  // trajectory completion duration weight
const double KLON = 1;  // longitudinal trajectory weight
const double KLAT = 1;  // lateral trajectory weight
// Trajectory Enumarations
struct FRE { enum DOF {S=0, D=1}; };  // Frenet degree of freedom
struct DEG { enum POLYNOMIAL {QUINTIC=5, QUARTIC=4}; };  // Polynomial degree

// Highway parameters 
const double SMAX = 6945.554;  // [m], length of the highway track 
const double LANE_WIDTH = 4;  // [m], lane width 
const std::vector<double> LANE_CENTER {2, 6, 9.7};  // [m], lane center
                                                    // 9.7 due to simulator bug
struct LANE { enum ID {LEFT=0, CENTER=1, RIGHT=2}; };  // lane enumaration
const std::vector<std::string> LANE_STRING {"left", "center", "right"};  // log

#endif //PARAMETERS_H
