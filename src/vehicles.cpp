#include "parameters.h"
#include "vehicles.h"
#include "utils.h"

#include <cmath>
#include <sstream>
#include <iostream>
#include <cstdio>


/** Vehicle class for each of the other vehicles on the highway.
 * 
 * @param id vehicle id
 * @param x_pos x location
 * @param y_pos y location
 * @param x_vel x velocity 
 * @param y_vel y velocity 
 * @param dist global distance to ego vehicle
 * @param s_pos s location
 * @param d_pos d location
 * @param s_vel s velocity 
 * @param d_vel d velocity 
 * @param s_dist Frenet s distance to the ego vehicle
 * @param current_lane current lane id
 */
Vehicle::Vehicle(int id, 
                 double x_pos, double y_pos, 
                 double x_vel, double y_vel, 
                 double dist,
                 double s_pos, double d_pos, 
                 double s_vel, double d_vel,
                 double s_dist, 
                 int current_lane) 
  : id(id), 
    x_pos(x_pos), y_pos(y_pos), x_vel(x_vel), y_vel(y_vel), dist(dist),
    s_pos(s_pos), d_pos(d_pos), s_vel(s_vel), d_vel(d_vel), s_dist(s_dist),
    current_lane(current_lane) {
  
  // Compute the safety distance to the vehicle. The safety distance is the 
  // distance required to come to a complete halt if the front vehicle does so.
  // It is defined by the constant time gap law.
  this->safety_dist = VEHICLE_LENGTH + SAFETY_TIME * this->s_vel;

  // Initialize flags valid for all vehicles
  this->is_present = this->id == -1 ? false : true;
  this->in_left_lane = this->current_lane == LANE::LEFT ? true : false;
  this->in_center_lane = this->current_lane == LANE::CENTER ? true : false;
  this->in_right_lane = this->current_lane == LANE::RIGHT ? true : false;

  // Initialize flags valid only for leading vehicles
  this->is_followable = false;
  this->to_close = false;
}

// Constant velocity prediction for location s at time t
double Vehicle::s_position(double t) { 
  return this->s_pos + this->s_vel * t;  
}


// Check if a vehicle is on the left side of the given lane
bool Vehicle::is_leftside_of(int lane) {
  return this->current_lane == lane - 1 ? true : false;
}


// Check if a vehicle is on the right side of the given lane
bool Vehicle::is_rightside_of(int lane) {
  return this->current_lane == lane + 1 ? true : false;
}


// Check if vehicle is in the front of an other vehicle
bool Vehicle::in_front_of(Vehicle &other) {
  // Just a value to handle completed highway laps
  double eps = 10;
  // s distance between this vehicle and the other vehicle
  double delta_s = this->s_pos - other.s_pos;
  // vehicle is in front
  bool in_front = delta_s > 0;
  // vehicle is in front (completed highway lap)
  double s_dist = s_distance(this->s_pos, other.s_pos);
  bool in_front_lap = delta_s < 0 && abs(delta_s) > s_dist + eps;
  return in_front || in_front_lap;
}


// Check if vehicle is in the rear of an other vehicle
bool Vehicle::in_rear_of(Vehicle &other) {
  return !in_front_of(other);
}


/** EgoVehicle class.
 * 
 * @param map the highway Map object
 * @param x_pos x location
 * @param y_pos y location
 * @param s_pos s location
 * @param d_pos d location
 * @param yaw yaw [rad]
 * @param speed velocity magnitude [mph]
 */
EgoVehicle::EgoVehicle(Map map, 
                      double x_pos, double y_pos, 
                      double s_pos, double d_pos, 
                      double yaw, double speed)
  : map(map), x_pos(x_pos), y_pos(y_pos), 
    s_pos(s_pos), d_pos(d_pos), yaw(yaw), speed(speed) {
  
  // Initialize flags
  this->current_lane = lane(this->d_pos);
  this->to_fast = false;
  this->is_deadlocked = false;
  this->has_free_opposite = false;
  }


/** Update ego vehicle state with new data of the simulator.
 * 
 * @param x_pos x location
 * @param y_pos y location
 * @param s_pos s location
 * @param d_pos d location
 * @param yaw yaw [rad]
 * @param speed velocity magnitude [mph]
 * @param traffic collector of the other vehicles on the highway 
 */
void EgoVehicle::update_state(double x_pos, double y_pos, 
                              double s_pos, double d_pos, 
                              double yaw, double speed, 
                              vector<Vehicle> &traffic) {
  this->x_pos = x_pos;
  this->y_pos = y_pos;
  this->s_pos = s_pos;
  this->d_pos = d_pos;
  this->yaw = yaw;
  this->speed = speed;
  // Determine longitudinal component of global speed limit in Frenet space 
  this->s_vel_limit = get_lon_speed_limit();  // [m/s]
  // Check if the ego vehicle is to fast and requires to be decelerated 
  this->to_fast = this->speed > this->s_vel_limit ? true : false;
  // Update the current lane
  this->current_lane = lane(this->d_pos);
  // Search for nearby vehicles
  this->nearby_vehicles = search_nearby_vehicles(traffic);
  // Search for closest leading vehicle on each lane.
  this->leading_vehicles = search_leading_vehicles(traffic);
  // Compute a cost for each lane depending on surrounding vehicles closeness
  this->lane_cost = get_lane_cost(); 
  // Check if the ego vehicle is deadlocked
  this->is_deadlocked = check_deadlock();
  // Check if the opposite lane is free
  this->has_free_opposite = check_opposite_lane();
}


/** Initialize the ego vehicle's trajectory at the beginning of the simulation. 
 * 
 * The starting state is the initial ego vehicle's position without movement: 
 * 
 *    s_start = {s_init, 0, 0} 
 *    d_start = {d_init, 0, 0} 
 * 
 * The end state is the longitudinal velocity limit keeping the starting lane: 
 * 
 *    s_end = {s_vel_limit, 0} 
 *    d_end = {d_init, 0, 0} 
 * 
 * The initial maneveur has a duration T of 4 sec to achieve a smooth start: 
 * 
 *    T = 4
 * 
 * However, the initialized trajectory will be updated in the first replaning 
 * cycle anyways. 
 */
void EgoVehicle::init_trajectory() {

  // Definition of the boundary states at start and end
  vector<double> s_start{this->s_pos, 0, 0};
  vector<double> d_start{this->d_pos, 0, 0};
  vector<double> s_end{this->s_vel_limit, 0};
  vector<double> d_end{this->d_pos, 0, 0};
  double T = 4;

  // Initialize the longitudinal (quartic) and lateral (quintic) trajectories 
  Polynomial s_quartic, d_quintic;
  s_quartic = Polynomial(s_start, s_end, T, DEG::QUARTIC, FRE::S);
  d_quintic = Polynomial(d_start, d_end, T, DEG::QUINTIC, FRE::D);

  // Compute the polynomial costs
  s_quartic.compute_cost();
  d_quintic.compute_cost(this->d_pos, this->lane_cost);

  // Compute discrete waypoints for the polynomials in Frenet space
  // The initial polynomials are evaluated for the complete planning horizon
  s_quartic.evaluate(NUM_TRAJECTORY_WPTS);
  d_quintic.evaluate(NUM_TRAJECTORY_WPTS);
  
  // Initialize the vehicles's composite trajectory and its waypoints
  this->trajectory = Trajectory(s_quartic, d_quintic, this->map);
}


/** Update the trajectory based on the current simulator data. 
 * 
 * The replaning will take place in the moment the simulator has consumed  
 * num_consumed_wpts > REPLAN_THRESHOLD waypoints. Since the computation of 
 * the updated trajectory will take some time in which the vehicle will keep 
 * moving and consuming waypoints, the starting state of the updated trajectory 
 * will be defined a few timesteps in the future. This will allow a smooth 
 * transition from the current to the updated trajectory. 
 * 
 * @param num_consumed_wpts number of consumed waypoints up to now  
 */
void EgoVehicle::update_trajectory(int num_consumed_wpts) {
  
  // Define polynomial starting states
  // The update of the trajectory polynomials will begin NUM_TRANSITION_WPTS 
  // in the future, starting to count from the first waypoint not jet consumed 
  // when the replaning began. 
  int starting_wpt = num_consumed_wpts + NUM_TRANSITION_WPTS;
  vector<double> s_start{
    this->trajectory.s_pos_wpts[starting_wpt],
    this->trajectory.s_vel_wpts[starting_wpt],
    this->trajectory.s_acc_wpts[starting_wpt]};
  vector<double> d_start{
    this->trajectory.d_pos_wpts[starting_wpt],
    this->trajectory.d_vel_wpts[starting_wpt],
    this->trajectory.d_acc_wpts[starting_wpt]};

  // Some logging
  printf("Reference curve (theta|kappa):    %6.2f %9.5f \n",
         rad2deg(this->map.spline_theta(this->s_pos)),
         this->map.curvature(this->s_pos));
  printf("Current longitudinal speed limit: %6.2f \n", this->s_vel_limit);
  printf("Current lane: %26s \n", LANE_STRING[this->current_lane].c_str());
  printf("\n");
  printf("Nearby vehicles s dist  (left|center|right): ");
  for (auto lane: {LANE::LEFT, LANE::CENTER, LANE::RIGHT}) {
    int num_vehicles = (int)this->nearby_vehicles[lane].size();
    if (num_vehicles == 0) {
      printf("%16d", 0); 
    } else {
      printf("%*s", (4 - num_vehicles) * 4, "");
    }
    for (auto vehicle: this->nearby_vehicles[lane]) {
      double dist = in_rear_of(vehicle) ? vehicle.s_dist : -vehicle.s_dist;
      printf("%4d", (int)dist);
    }
  }
  printf("\n");
  printf("Leading vehicles s dist (left|center|right): %16d%16d%16d \n",
         (int)this->leading_vehicles[LANE::LEFT].s_dist,
         (int)this->leading_vehicles[LANE::CENTER].s_dist,
         (int)this->leading_vehicles[LANE::RIGHT].s_dist);
  printf("Lane cost               (left|center|right): %16.2f%16.2f%16.2f\n",
         this->lane_cost[LANE::LEFT], 
         this->lane_cost[LANE::CENTER], 
         this->lane_cost[LANE::RIGHT]);
  printf("\n");
  printf("Waypoint states: \n");
  printf("Current location (x|y|yaw):    %8.2f %8.2f %6.2f \n", 
         this->trajectory.x_pos_wpts[num_consumed_wpts-1],
         this->trajectory.y_pos_wpts[num_consumed_wpts-1],
         rad2deg(this->trajectory.yaw_wpts[num_consumed_wpts-1]));
  printf("Current state s (pos|vel|acc): %8.2f %8.2f %6.2f \n", 
         this->trajectory.s_pos_wpts[num_consumed_wpts-1],
         this->trajectory.s_vel_wpts[num_consumed_wpts-1],
         this->trajectory.s_acc_wpts[num_consumed_wpts-1]);
  printf("Current state d (pos|vel|acc): %8.2f %8.2f %6.2f \n\n", 
         this->trajectory.d_pos_wpts[num_consumed_wpts-1],
         this->trajectory.d_vel_wpts[num_consumed_wpts-1],
         this->trajectory.d_acc_wpts[num_consumed_wpts-1]);
  printf("Start state s (pos|vel|acc):   %8.2f %6.2f %6.2f \n", 
          s_start[0], s_start[1], s_start[2]);
  printf("Start state d (pos|vel|acc):   %8.2f %6.2f %6.2f \n\n", 
          d_start[0], d_start[1], d_start[2]);

  // Initialize the optimal trajectory as empty trajectory
  Trajectory optimal_trajectory;

  // Determine the possible successors states
  vector<Successor> successors = get_successor_states();

  // Standard driving scenario: 
  // Determine the optimal trajectory out of all successor state trajectories
  optimal_trajectory = get_optimal_trajectory(
    s_start, d_start, successors, num_consumed_wpts, SCN::STANDARD);

  // Recovery driving scenario:
  // If for some reason (collisions/feasibility) no optimal trajectory could be 
  // generated, try to recover the state with eased feasibility constraints
  if (optimal_trajectory.is_empty) {
    printf("Creating recovery trajectory.\n");
    optimal_trajectory = get_optimal_trajectory(
      s_start, d_start, successors, num_consumed_wpts, SCN::RECOVERY);
  }

  // Emergency driving scenario:
  // If the recovery was not successful, start an emergency maneuver. The 
  // feasibility constraints are eased and the collision check is cost based 
  // instead of a hierarchical zero/one decision.
  if (optimal_trajectory.is_empty) {
    printf("Creating emergency trajectory.\n");
    optimal_trajectory = get_optimal_trajectory(
      s_start, d_start, successors, num_consumed_wpts, SCN::EMERGENCY);
  }

  printf("\n");
  if (!optimal_trajectory.is_empty) {

    // Replace the current trajectory by the new optimal one
    this->trajectory = optimal_trajectory;

    printf("Chosen %s lane as best successor with cost "
            "(klon=%.1f, klat=%.1f): %6.2f \n",
            LANE_STRING[optimal_trajectory.target_lane].c_str(),
            KLON, KLAT, optimal_trajectory.cost);
    printf("s costs (jrk|dur|vel|pos): %6.2f %6.2f %6.2f %6.2f \n", 
            optimal_trajectory.s_poly.cost_jerk, 
            optimal_trajectory.s_poly.cost_duration, 
            optimal_trajectory.s_poly.cost_velocity, 
            optimal_trajectory.s_poly.cost_position);
    printf("d costs (jrk|dur|vel|pos): %6.2f %6.2f %6.2f %6.2f \n", 
            optimal_trajectory.d_poly.cost_jerk, 
            optimal_trajectory.d_poly.cost_duration, 
            optimal_trajectory.d_poly.cost_velocity, 
            optimal_trajectory.d_poly.cost_position);
  } else {
    printf("No optimal trajectory available. \n");
  }
  printf("\n");
}


/** Determine the possible successor states. 
 * 
 * The successor states are determined based on the ego vehicle's current lane 
 * and the nearby and/or leading vehicles. Combinations of the following 
 * lateral and longitudinal successor states are possible: 
 * 
 * Lateral successor states: 
 *  a) keeping the lane 
 *  b) single lane changes to the left or right 
 *  c) double lane changes to the left or right 
 * 
 * Each lateral successor state has a specific target lane for which the 
 * corresponding longitudinal successor state will be determined.
 * 
 * Longitudinal successor states: 
 *  a) keep velocity if no leading vehicle is in the target lane 
 *  a) follow a leading vehicle in the target lane
 *  c) fallback if deadlocked and the opposite lane is free 
 * 
 * Returns a vector of possible successor state objects. Each successor state 
 * object is defined by a combination of lateral and longitudinal successor 
 * states, e.g. Successor(LAT::KEEP, LON::FOLLOW) defining a follow while 
 * keeping lane successor state.
*/
vector<Successor> EgoVehicle::get_successor_states() {

  // Initialize lane availability flags
  bool left_free = true;
  bool right_free = true;  
  bool left_free_for_double = true;
  bool right_free_for_double = true;

  // Loop over all nearby vehicles
  for (auto &vehicle: flatten(this->nearby_vehicles)) {
    // Adjacent lanes are not available if the vehicles there are to close or 
    // front/rear vehicles are to slow/fast for their distance to ego vehicle 
    if (single_blocked_by(vehicle)) {
      if (vehicle.is_leftside_of(this->current_lane)) { 
        left_free = false;
        left_free_for_double = false;
        printf("Single lane change to the left not "
               "available due to nearby vehicle \n"
               "Nearby vehicle (id|s_pos|d_pos|s_vel|d_vel|s_dist): "
               "%4d %8.2f %6.2f %6.2f %6.2f %6.2f \n", 
               vehicle.id, vehicle.s_pos, vehicle.d_pos, 
               vehicle.s_vel, vehicle.d_vel,
               this->in_rear_of(vehicle) ? vehicle.s_dist : -vehicle.s_dist);
      } 
      else if (vehicle.is_rightside_of(this->current_lane)) {
        right_free = false; 
        right_free_for_double = false; 
        printf("Single lane change to the right not "
               "available due to nearby vehicle \n"
               "Nearby vehicle (id|s_pos|d_pos|s_vel|d_vel|s_dist): "
               "%4d %8.2f %6.2f %6.2f %6.2f %6.2f \n", 
               vehicle.id, vehicle.s_pos, vehicle.d_pos, 
               vehicle.s_vel, vehicle.d_vel,
               this->in_rear_of(vehicle) ? vehicle.s_dist : -vehicle.s_dist);
      }
    }
    
    // Double lane changes are not allowed if the vehicles on the adjacent 
    // or target lanes are to close, whereby front vehicles must be more 
    // distant then rear vehicles
    if (double_blocked_by(vehicle)) {
      if (vehicle.is_leftside_of(this->current_lane) || vehicle.in_left_lane) { 
        left_free_for_double = false; 
      } 
      else if (vehicle.is_rightside_of(this->current_lane) || vehicle.in_right_lane) { 
        right_free_for_double = false; 
      }
    }
  }

  // Define the successor states
  vector<Successor> successors;
  int lat, lon;  // lateral end longitudinal successor state component

  // Keeping the lane is always a valid lateral successor state
  lat = LAT::KEEP;
  // The longitudinal successor state depends on the surrounding vehicles
  lon = get_lon_state(this->current_lane);
  // Append the defined successor state to the successor state collector
  successors.push_back(Successor(lat, lon));

  // Due to a simulator bug false sensor fusion data is delivered at the end 
  // of a lap. Keeping the lane is the only valid successor state in that zone.
  if (SMAX - this->s_pos < 100 || this->s_pos < 100) { return successors; }
  
  // Get the leading vehicle in the current lane 
  Vehicle cur_lead = this->leading_vehicles[this->current_lane];

  // Check for valid lane changes from the left lane
  if (this->current_lane == LANE::LEFT) {
    // Single lane change to the right 
    if (right_free) {
      lat = LAT::SINGLE_RIGHT;
      lon = get_lon_state(LANE::CENTER);
      successors.push_back(Successor(lat, lon));
    }
    // Double lane change to the right if it is free and there is a leading 
    // vehicle in the current lane 
    if (cur_lead.is_present && right_free_for_double) {
      lat = LAT::DOUBLE_RIGHT;
      lon = LON::KEEP;
      successors.push_back(Successor(lat, lon));
      printf("Free right lane. Double lane change from the "
             "left lane will be checked. \n");
    }
  }

  // Check for valid lane changes from the right lane
  else if (this->current_lane == LANE::RIGHT) {
    // Single lane change to the left
    if (left_free) {
      lat = LAT::SINGLE_LEFT;
      lon = get_lon_state(LANE::CENTER);
      successors.push_back(Successor(lat, lon));
    }
    // Double lane change to the left if it is free and there is a leading 
    // vehicle in the current lane 
    if (cur_lead.is_present && left_free_for_double) {
      lat = LAT::DOUBLE_LEFT;
      lon = LON::KEEP;
      successors.push_back(Successor(lat, lon));
      printf("Free left lane. Double lane change from the "
             "right lane will be checked. \n");
    }
  }

  // Check for valid lane changes from the center lane
  else if (this->current_lane == LANE::CENTER) {
    // Single lane change to the left
    if (left_free) {
      lat = LAT::SINGLE_LEFT;
      lon = get_lon_state(LANE::LEFT);
      successors.push_back(Successor(lat, lon));
    }
    // Single lane change to the right
    if (right_free) {
      lat = LAT::SINGLE_RIGHT;
      lon = get_lon_state(LANE::RIGHT);
      successors.push_back(Successor(lat, lon));
    }
  }
  return successors;
}


/** Determine the minimum cost successor trajectory. 
 * 
 * For each of the provided successor states a set of longitudinal and 
 * lateral polynomial candidates is created. Composite trajectory candidates  
 * are defined by combining each of these polynomials. The best of these 
 * candidates is determined and returned as optimal trajectory. 
 * 
 * @param s_start longitudinal polynomials starting state in Frenet system
 * @param d_start lateral polynomials starting state in Frenet system
 * @param successors vector of possible successor states
 * @param num_consumed_wpts number of consumed waypoints up to now  
 * @param scenario driving scenario case (standard, recovery or emergency)
 */
Trajectory EgoVehicle::get_optimal_trajectory(
  vector<double> s_start, 
  vector<double> d_start, 
  vector<Successor> &successors,
  int num_consumed_wpts,
  int scenario) {

  // Optimal trajectory for current driving scenario
  Trajectory optimal_trajectory;  
  // Polynomial candidates for the optimal trajectory determination
  vector<Polynomial> d_polys, s_polys;
  // Collector for the best trajectory candidate of each successor state
  vector<Trajectory> best_successor_trajectories;  

  // Loop over all successor states
  for (Successor successor: successors) {
    
    // Reset best successor trajectory
    Trajectory best_successor_trajectory;

    // Some logging
    // Target lane of the current successor state
    int target_lane = this->current_lane + successor.lat;
    const char *lane_str = LANE_STRING[target_lane].c_str();
    // Leading vehicle in the target lane
    Vehicle leading_vehicle = this->leading_vehicles[target_lane];
    printf("\n\n");
    printf("Checking %s lane for availability\n", lane_str);
    if (leading_vehicle.is_present) {
      printf("Leading vehicle (id|s_pos|d_pos|s_vel|d_vel|s_dist): "
             "%4d %8.2f %6.2f %6.2f %6.2f %4d \n", 
             leading_vehicle.id, 
             leading_vehicle.s_pos, leading_vehicle.d_pos, 
             leading_vehicle.s_vel, leading_vehicle.d_vel, 
             (int)leading_vehicle.s_dist);
    } else {
      printf("No leading vehicle detected in %s lane. \n", lane_str);
    }

    // Generate a set of trajectory traversal durations
    // Standard: T = {1.5, 2.0, 2.5}
    // Recovery: T = {1.25, 1.75, 2.25}
    // Emergency: T = {1.25, 1.75, 2.25}
    vector<double> T;

    if (scenario == SCN::STANDARD) {T = linspace(1.5, 2.5, 3);} 
    // Faster options with eased feasibility checks for recovery scenarios
    else if (scenario == SCN::RECOVERY) { T = linspace(1.25, 2.25, 3); }
    // Additionally eased feasibility for emergencies
    else if (scenario == SCN::EMERGENCY) { T = linspace(1.25, 2.25, 3); }

    // Generate lateral and longitudinal polynomial candidates for the current 
    // successor state and all durations 
    d_polys = this->get_d_polys(d_start, T, successor, scenario);
    s_polys = this->get_s_polys(s_start, T, successor, scenario);

    best_successor_trajectory = this->get_best_candidate(
      s_polys, d_polys, num_consumed_wpts, scenario);

    // Store the best successor trajectory if it exists
    if (!best_successor_trajectory.is_empty) {
      best_successor_trajectories.push_back(best_successor_trajectory);
      
      printf("\n");
      printf("Best successor trajectory cost (klon=%.1f, klat=%.1f): %6.2f \n",
            KLON, KLAT, best_successor_trajectory.cost);
      printf("s costs (jrk|dur|vel|pos): %6.2f %6.2f %6.2f %6.2f \n", 
            best_successor_trajectory.s_poly.cost_jerk, 
            best_successor_trajectory.s_poly.cost_duration, 
            best_successor_trajectory.s_poly.cost_velocity, 
            best_successor_trajectory.s_poly.cost_position);
      printf("d costs (jrk|dur|vel|pos): %6.2f %6.2f %6.2f %6.2f \n", 
            best_successor_trajectory.d_poly.cost_jerk, 
            best_successor_trajectory.d_poly.cost_duration, 
            best_successor_trajectory.d_poly.cost_velocity, 
            best_successor_trajectory.d_poly.cost_position);
    } else { 
      printf("No successor trajectory available. \n\n\n"); 
    }
  }

  // Determine the optimal trajectory of the set of best successor trajectories
  if (!best_successor_trajectories.empty()) {
    optimal_trajectory = minval(best_successor_trajectories);
  }
  
  return optimal_trajectory;
}


/** Compose and determine best trajectory candidate for the given polynomials. 
 * 
 * Composite trajectory candidates are defined by combining each of the given 
 * lateral and longitudinal polynomials. The evaluated waypoints of the 
 * trajectory candidates are checked for feasibility and collisions.  In the 
 * case of recovery and emergency scenarios the feasibility checks are eased. 
 * The "surviving" minimum cost trajectory is returned. 
 * 
 * @param s_polys set of longitudinal polynomials
 * @param d_polys set of lateral polynomials
 * @param num_consumed_wpts number of consumed waypoints up to now  
 * @param scenario driving scenario case (standard, recovery or emergency)
 */
Trajectory EgoVehicle::get_best_candidate(vector<Polynomial> s_polys, 
                                          vector<Polynomial> d_polys, 
                                          int num_consumed_wpts, 
                                          int scenario) {

  printf("\n");
  printf("Found %lu feasible d-polynomials \n", d_polys.size());
  printf("Found %lu feasible s-polynomials \n", s_polys.size());

  int num_combinations = (int)d_polys.size() * (int)s_polys.size();
  printf("Searching best trajectory candidate of %d combinations \n", 
          num_combinations);

  // Flattened version (for successor state determination and collision checks)
  vector<Vehicle> nearby_vehicles_1D = flatten(this->nearby_vehicles);

  int num_collisions = 0;  // for logging 
  int num_unfeasible = 0;  // for logging 
  double min_cost = 1e9;  // Some big initial cost
  Trajectory best_candidate;  // Best candidate trajectory
  
  // Combine all longitudinal and lateral polynomial trajectory components
  for (auto &s_poly: s_polys) {
    for (auto &d_poly: d_polys) {
      // Create a new candidate by copying the current trajectory
      Trajectory candidate = this->trajectory;
      // Update the candidte by replacing the replaned trajectory waypoints
      candidate.update(s_poly, d_poly, num_consumed_wpts, this->map, scenario);
      // If the candidate is not feasible continue with the next one
      if (!candidate.is_feasible) { ++num_unfeasible; continue; }
      // Check if the candidate is collision free 
      candidate.check_collision(nearby_vehicles_1D, this->map, scenario);
      if (candidate.is_collided) {++num_collisions;}
      if (!candidate.is_collided && candidate.cost < min_cost) {
        min_cost = candidate.cost;
        best_candidate = candidate;
      }
    }
  }
  printf("Number of trajectories exceeding allowed velocity: %d \n", num_unfeasible);
  printf("Number of trajectories with collisions: %d \n", num_collisions);

  return best_candidate;
}
  

/** Create a set of lateral polynomials. 
 * 
 * All polynomial candidates in the set will have the same starting state which 
 * is NUM_TRANSITION_WPTS in the future, beginning to count at the waypoint the 
 * replanning began. 
 *
 *        d_start = {d_pos, d_vel, d_acc}. 
 * 
 * The set of polynomials is created for all durations T_j in T_set and with 
 * varying end locations d_i perpendicular to the target lane's center. The 
 * lateral end velocities and accelerations are defined to be zero: 
 * 
 *        d_end = {d_i, 0, 0}. 
 * 
 * For instance, if the target lane is the left lane (d = 2) the varying end 
 * locations are d_i = {1.5, 2., 2.5}. The target lane is determined from the 
 * lateral successor state. 
 * 
 * Each polynomial will be evaluated for all waypoints to be replanned. In 
 * case of recovery and emergency scenarios the feasibility checks are eased.
 * 
 * @param d_start starting state for all polynomials in the set
 * @param T_set set of durations to create the polynomials for 
 * @param successor successor state object with longitudinal/lateral targets
 * @param scenario driving scenario case (standard, recovery or emergency)
 */
vector<Polynomial> EgoVehicle::get_d_polys(
  vector<double> d_start, 
  vector<double> T_set, 
  Successor successor,
  int scenario) {

  // Deduce the target lane from the successor state
  int target_lane = this->current_lane + successor.lat;
  // The lateral target location is the center of the target lane
  double d_pos_target = LANE_CENTER[target_lane];
  // Create end location variations
  vector<double> delta_d_pos = linspace(-0.5, +0.5, NUM_VARIATIONS);
  // The end velocity and acceleration shall be zero
  double d_vel_end = 0; 
  double d_acc_end = 0; 

  printf("\n");

  // Create the polynomial end states with varying end locations 
  vector<vector<double>> d_end_set;  // set of lateral end states
  for (auto i = 0; i < delta_d_pos.size(); ++i) {
    // End position variation
    double d_pos_end = d_pos_target + delta_d_pos[i];
    // End state variation
    vector<double> d_end = {d_pos_end, d_vel_end, d_acc_end};
    d_end_set.push_back(d_end);

    printf("End state variation d (pos|vel|acc): %4.2f %.1f %.1f \n", 
          d_end[0], d_end[1], d_end[2]);
  }
  printf("\n");

  // Create lateral polynomials for each duration and end state variation
  // Number of waypoints the trajectory needs to be replanned
  int num_replaning_wpts = NUM_TRAJECTORY_WPTS - NUM_TRANSITION_WPTS;
  Polynomial poly;  // polynomial candidate
  vector<Polynomial> polys;  // collector of polynomial candidates  
  for (auto T: T_set) {
    printf("Duration T: %.2f \n", T);
    for (auto d_end: d_end_set) {
      // Create a quintic polynomial for the current end state and duration
      poly = Polynomial(d_start, d_end, T, DEG::QUINTIC, FRE::D);
      // Compute the polynomials cost
      poly.compute_cost(d_pos_target, this->lane_cost);
      // Compute discrete waypoints of the polynomial and check its feasibility
      poly.evaluate(num_replaning_wpts, scenario);
      // If it is feasible store it
      if (poly.is_feasible) { 
        polys.push_back(poly);
      }
    }
  }
  return polys;
}


/** Create a set of longitudinal polynomials. 
 * 
 * All polynomial candidates in the set will have the same starting state which 
 * is NUM_TRANSITION_WPTS in the future beginning to count at the waypoint 
 * the replanning began. 
 *
 *        s_start = {s_pos, s_vel, s_acc}. 
 * 
 * The polynomials will be created for all durations T_j in T_set and with 
 * varying end states depending on the longitudinal successor state. 
 * 
 * LON::KEEP: 
 * If the target lane is free, a set of quartic polynomials will be created,
 * in order to keep the velocity. The end states will 
 *
 *        a) leave the location as a degree of freedom, 
 *        b) contain variations of velocities (s_vel_i) varying up to the 
 *           velocity limit and
 *        c) have zero acceleration (s_acc = 0). 
 *
 *        >>> s_end = {s_vel_i, 0}
 * 
 * LON::FOLLOW: 
 * If there is a leading vehicle in the target lane and it is driving slower 
 * than the maximum allowed velocity, a set of quintic following polynomials 
 * will be created. The end states will have  
 *
 *        a) varying end locations (s_pos_i) of which the closest one is at 
 *           a safety distance behind the leading vehicle, 
 *        b) a speed equal to the leading vehicle's one (s_vel_lead) and 
 *        c) zero acceleration (s_acc = 0). 
 *
 *        >>> s_end = {s_pos_i, s_vel_lead, 0} 
 * 
 * LON::FALLBACK: 
 * If the ego vehicle is deadlocked in its current lane and its opposite lane 
 * is free, a set of quintic following polynomials will be created. The end 
 * states will have  
 *
 *        a) varying end locations (s_pos_i) keeping larger distances to the  
 *           leading vehicle in the current lane, than in the follow maneuver 
 *        b) a speed equal to the deadlock causing vehicle in the adjacent 
 *           lane (s_vel_adjacent) 
 *        c) zero acceleration (s_acc = 0). 
 *
 *        >>> s_end = {s_pos_i, s_vel_adjacent, 0} 
 *    
 * 
 * Each polynomial will be evaluated for all waypoints to be replanned. In 
 * case of recovery and emergency scenarios the feasibility checks are eased.
 * 
 * @param s_start starting state for all polynomials in the set
 * @param T_set set of durations to create the polynomials for
 * @param successor successor state object with longitudinal/lateral targets 
 * @param scenario driving scenario case (standard, recovery or emergency)
 */
vector<Polynomial> EgoVehicle::get_s_polys(
  vector<double> s_start, 
  vector<double> T_set, 
  Successor successor, 
  int scenario) {

  Polynomial poly;  // polynomial candidate
  vector<Polynomial> polys;  // collector of polynomial candidates
  double s_pos_end, s_pos_target, s_pos_min;  // position
  double s_vel_end, s_vel_target, s_vel_min;  // velocity
  double s_acc_end;  // acceleration
  vector<double> delta_s_pos, delta_s_vel;  // End state variations
  vector<double> s_end;  // End state variant
  
  // The previous trajectory is followed up to the start state of the 
  // replanning, which is reached after the transition time
  double T_transition = NUM_TRANSITION_WPTS * DT;
  // Number of waypoints the trajectory needs to be replanned
  int num_replaning_wpts = NUM_TRAJECTORY_WPTS - NUM_TRANSITION_WPTS;

  // Since the fallback maneuver is kind of a comfort maneuver, it is 
  // converted into a follow manuever in case of emergency scenarios. 
  // Safety before comfort.
  if (successor.lon == LON::FALLBACK && scenario == SCN::EMERGENCY) {
    successor.lon = LON::FOLLOW; 
  }

  printf("\n");
  printf("Chosen longitudinal mode: ");
  if (successor.lon == LON::KEEP) { printf("Keep velocity \n"); }
  else if (successor.lon == LON::FOLLOW) { printf("Follow \n"); }
  else if (successor.lon == LON::FALLBACK) { printf("Fallback \n"); }
  
  // Loop over all trajectory durations
  for (auto T: T_set) {

    // Collector of end state variants for current duration
    vector<vector<double>> s_end_set;  

    printf("\n");
    printf("Duration T: %.2f \n", T);
  
    // If no leading vehicle exists or it's far away, keep the velocity
    if (successor.lon == LON::KEEP) {
      
      // The target velocity is the minimum of
      // - the speed limit or 
      // - the speed achievable with AMAX in time t = T_j
      double s_vel_start = s_start[1];
      s_vel_target = fmin(this->s_vel_limit, s_vel_start + AMAX * T);
      // The minimum velocity is below the target speed
      // Im emergencies allow full stops even if free drive is possible
      s_vel_min = scenario == SCN::EMERGENCY ? 0 : s_vel_target - 1.0;
      // Create end velocity variations
      delta_s_vel = linspace(0, s_vel_target - s_vel_min, NUM_VARIATIONS);
      // The end acceleration shall be zero
      s_acc_end = 0; 

      // Loop over all end velocity variations
      for (auto i = 0; i < delta_s_vel.size(); ++i) {
        // End velocity variant
        s_vel_end = s_vel_target - delta_s_vel[i];
        // End state variant
        s_end = {s_vel_end, s_acc_end};
        s_end_set.push_back(s_end);
        printf("End state variation s (vel|acc): %6.2f %6.2f\n", 
               s_end[0], s_end[1]);
      }
    }

    // Follow and Fallback maneuvers
    else if (successor.lon == LON::FOLLOW || successor.lon == LON::FALLBACK) {

      // Deduce the target lane from the successor state
      int target_lane = this->current_lane + successor.lat;
      // Leading vehicle in target lane
      Vehicle leading_vehicle = leading_vehicles[target_lane];
      // The safety distance is defined by the constant time gap law.
      double safety_dist = leading_vehicle.safety_dist;

      // Since the sensor fusion only provides information about the leading 
      // vehicle's speed, it is simplifying assumed that it will keep moving 
      // without acceleration. To compute the leading vehicle's position at
      // the end of the trajectory candidate, we need to consider the 
      // transition time, since the start state of the replanning is shifted
      // NUM_TRANSITION_WPTS into the future. The leading vehicle will keep 
      // moving from the moment the replanning began, thus for the transition 
      // time plus the trajectory duration.
      // The leading vehicle's position at the end of the planned trajectory
      double s_pos_lead = leading_vehicle.s_position(T_transition + T);
      // Adjust the leading vehicle's end location for completed highway laps
      if (leading_vehicle.s_pos < this->s_pos) {s_pos_lead += SMAX;}
      
      // Follow maneuver
      if (successor.lon == LON::FOLLOW) {
        // The ego vehicle's target location is at a safety distance behind 
        // the leading vehicle's position at t = T_j 
        s_pos_target = s_pos_lead - 1.0 * safety_dist;
        // Determine the minimum end location 
        s_pos_min = s_pos_target - 1.0 * safety_dist;
        // The target speed is the leading vehicle's current speed
        s_vel_target = leading_vehicle.s_vel;
      }

      // Fallback maneuver
      else if (successor.lon == LON::FALLBACK) {
        // For fallback maneuvers the target location is at a larger distance 
        // behind the leading vehicle
        s_pos_target = s_pos_lead - 1.5 * safety_dist;
        // Determine the minimum end location
        s_pos_min = s_pos_target - 1.0 * safety_dist;
        // The target speed is below the leading vehicle's velocity
        s_vel_target = fmin(leading_vehicle.s_vel - 2.5, 0);
      }

      // Create end position variations
      delta_s_pos = linspace(0, s_pos_target - s_pos_min, NUM_VARIATIONS);
      // Since the leading vehicle's acceleration is assumed to be zero, we 
      // let the ego vehicle's end acceleration be zero, too. 
      s_acc_end = 0;
    
      // Loop over all end velocity variations
      for (auto i = 0; i < delta_s_pos.size(); ++i) {
        // End position variation
        s_pos_end = s_pos_target - delta_s_pos[i];
        // End state variation
        s_end = {s_pos_end, s_vel_target, s_acc_end};
        s_end_set.push_back(s_end);

        printf("End state variation s (pos|vel|acc): %6.2f %6.2f %6.2f\n", 
               s_end[0], s_end[1], s_end[2]);
      }
    }

    for (auto s_end: s_end_set) {

      // Keep velocity: Create a quartic polynomial and compute its cost
      if (successor.lon == LON::KEEP) {
        poly = Polynomial(s_start, s_end, T, DEG::QUARTIC, FRE::S);
        poly.compute_cost();
      } 
      // Follow/Fallback: Create a quintic polynomial and compute its cost
      else if (successor.lon == LON::FOLLOW || successor.lon == LON::FALLBACK) {
        poly = Polynomial(s_start, s_end, T, DEG::QUINTIC, FRE::S);
        poly.compute_cost(s_pos_target);
      }

      // Compute discrete waypoints of the polynomial and check its feasibility 
      poly.evaluate(num_replaning_wpts, scenario);
      // If it is feasible store it
      if (poly.is_feasible) { 
        polys.push_back(poly); 
      }  
    }
  }

  return polys;
}


/** Determine the longitudinal successor state for the given target lane. 
 * 
 * Returns one of the following states if the corresponding rules apply. 
 * 
 * LON::FOLLOW 
 *  The leading vehicle in the target lane is in a followable distance. 
 * 
 * LON::FALLBACK 
 *  The ego vehicle is deadlocked but has an free opposite lane. 
 * 
 * LON::KEEP 
 *  In all other cases keep the velocity. 
 * 
 * @param target_lane target lane to get the longitudinal successor state for 
 */
int EgoVehicle::get_lon_state(int target_lane) {
    
    // Standard longitudinal successor
    int lon = LON::KEEP;

    // Fallback behind the leading vehicle in the current lane if the ego 
    // vehicle is deadlocked and the opposite lane is free 
    if (target_lane == this->current_lane && 
        this->current_lane == maxloc(this->lane_cost) && 
        this->is_deadlocked && this->has_free_opposite) {
      lon = LON::FALLBACK;
    } 
    // Follow the leading vehicle in the target lane if it is followable
    else if (this->leading_vehicles[target_lane].is_followable) { 
      lon = LON::FOLLOW;
    }

    return lon;
}


/** Search the closest leading vehicle in each lane (in Frenet s coordinates). 
 *
 * Each vehicle will be searched within LEAD_DETECT_DIST in the front of the 
 * ego vehicle. 
 * 
 * Additionally to the leading vehicle search some important properties will 
 * be set, dependening on the vehicle's velocity and its distance to the ego 
 * vehicle. 
 * 
 * Properties: is_present, is_followable, to_close 
 * 
 * The returned leading vehicles vector is indexed by the lane ids, for 
 * instance leading_vehicles[LANE::LEFT] is the leading vehicle on the left 
 * lane. 
 * 
 * @param traffic set of vehicles based on the simulator's sensor_fusion data 
 * @returns a vector of three vehicle objects, one for each lane. 
 */
vector<Vehicle> EgoVehicle::search_leading_vehicles(vector<Vehicle> &traffic) {
  
  // Initialize vector with empty vehicles
  Vehicle v;
  vector<Vehicle> leading_vehicles = {v, v, v};
  
  // Search the leading vehicle for each lane
  for (auto target_lane: {LANE::LEFT, LANE::CENTER, LANE::RIGHT}) {

    // detection horizon
    double closest_dist = LEAD_DETECT_DIST;
    // at this distance a leading vehicle could be followable
    double follow_dist = LEAD_FOLLOW_TIME * this->speed;

    for (auto &vehicle: traffic) {
     
      // vehicle is on target lane
      bool on_target_lane = target_lane == vehicle.current_lane;
      // vehicle is the closest one (in Frenet s coordinates)
      bool closest = vehicle.s_dist < closest_dist;
      // safety distance
      double safety_dist = vehicle.safety_dist;

      // Store the closest leading vehicle and adjust its properties 
      if (on_target_lane && in_rear_of(vehicle) && closest) {
        closest_dist = vehicle.s_dist;
        vehicle.is_present = true;
        vehicle.is_followable = vehicle.s_dist < follow_dist && 
                                vehicle.s_vel < VMAX ? true : false;
        vehicle.to_close = vehicle.s_dist < 0.5 * safety_dist ? true : false;
        leading_vehicles[target_lane] = vehicle;
      }
    }
  }
  return leading_vehicles;
}


/** Search for nearby vehicles (in Frenet s coordinates). 
 *
 * The vehicles are searched in NEARBY_DIST around the ego vehicle. 
 * 
 * The returned nearby vehicles vector is indexed by the lane ids, for 
 * instance nearby_vehicles[LANE::LEFT] contains all the nearby vehicles found 
 * in the left lane sorted by their distance to the ego vehicle. 
 * 
 * @param traffic set of vehicles based on the simulator's sensor_fusion data 
 * @returns the nearby vehicles on each lane (2D vector). 
 */
vector<vector<Vehicle>> EgoVehicle::search_nearby_vehicles(
  vector<Vehicle> &traffic) {

  // Initialize vector 
  vector<vector<Vehicle>> nearby_vehicles {{}, {}, {}};
  // Loop over all detected vehicles
  for (auto &vehicle: traffic) {  
    if (vehicle.dist < NEARBY_DIST) {
      nearby_vehicles[vehicle.current_lane].push_back(vehicle);
    }
  }
  // Sort the vehicles on each lane by their distance to the ego vehicle
  for (auto &nearby_vehicles_by_lane: nearby_vehicles) {
    std::sort(nearby_vehicles_by_lane.begin(), nearby_vehicles_by_lane.end());
  }
  return nearby_vehicles;
}


/** Flatten the nearby vehicles per lane vector. 
 * 
 * @param nearby_vehicles nearby vehicles on each lane (2D vector)
 * @returns flattend nearby vehicles vector (1D vector)
*/
vector<Vehicle> EgoVehicle::flatten(vector<vector<Vehicle>> &nearby_vehicles) {
  vector<Vehicle> nearby_vehicles_1D;
  for (int lane: {LANE::LEFT, LANE::CENTER, LANE::RIGHT}) {
    nearby_vehicles_1D.insert(nearby_vehicles_1D.end(), 
                              nearby_vehicles[lane].begin(), 
                              nearby_vehicles[lane].end());
  }
  return nearby_vehicles_1D;
}


/** Compute a lane cost depending on the closeness of the surrounding vehicles. 
 * 
 * The following rules apply: 
 *  a) Lanes without leading vehicle's have zero cost 
 *  b) The cost increases linearly with the leading vehicle's closeness 
 *  c) The ego vehicle's current lane is favored and gets a lower cost 
 *  d) If the ego vehicle is deadlocked its current lane gets an increased cost 
 */
vector<double> EgoVehicle::get_lane_cost() {
  
  // Initialize the cost on each lane
  vector<double> lane_cost = {0, 0, 0};
  // Only continue if the closest leading vehicle is below a threshold distance
  // This allows gathering information also on other lanes
  double min_dist = 1e9;
  for (auto vehicle: this->leading_vehicles) {
    if (vehicle.is_present && vehicle.s_dist < min_dist) {
      min_dist = vehicle.s_dist;
    }
  }
  if (min_dist > LANE_COST_DIST) { return lane_cost; }
  
  // The intial maximum cost is the ego vehicle's detection range 
  double max_cost = LEAD_DETECT_DIST;
  // Determine the cost for each lane 
  for (auto lane: {LANE::LEFT, LANE::CENTER, LANE::RIGHT}) {
    Vehicle vehicle = this->leading_vehicles[lane];
    // The larger the dist between leading and ego vehicle the lower the cost
    double cost = vehicle.is_present ? max_cost - vehicle.s_dist : 0;
    // Special treating of current lane 
    if (lane == this->current_lane) { 
      // The current lane is favoured and gets a reduced cost
      cost *= LANE_COST_BONUS;
      // If the ego vehicle is deadlocked and the opposite lane is free the 
      // current lane gets an increased cost
      if (this->is_deadlocked && this->has_free_opposite) {
        cost *= LANE_COST_MALUS; }
    }
    // Store the total cost of the lane
    lane_cost[lane] = cost;
  }
  // Prevent overhasty lane changes, if a side lane has no leading vehicle but
  // is currently blocked.
  if ((lane_cost[LANE::LEFT] == 0 && lane_cost[LANE::RIGHT] > 0) || 
      (lane_cost[LANE::RIGHT] == 0 && lane_cost[LANE::LEFT] > 0)) {
        lane_cost[this->current_lane] *= LANE_COST_BONUS;
      }
  return lane_cost;
}


/** Determine the longitudinal speed limit for the current location on the map. 
 * 
 * The longitudinal speed limit is determined by converting the global speed 
 * limit into the Frenet space considering the highways curvature. It is 
 * required for the definition of the end states of the longitudinal polynomial 
 * candidates. 
 */
double EgoVehicle::get_lon_speed_limit() {
  // Return the unconverted global speed limit for trajectory initialization
  if (this->trajectory.is_empty) {return VLIM;}
  // Maximum Frenet lateral speed of the current trajectory 
  double d_vel_max = maxval(this->trajectory.d_vel_wpts);
  // Convert the global speed limit into the longitudinal component of the 
  // speed in Frenet space
  double numerator = sqrt(VLIM * VLIM - d_vel_max * d_vel_max);
  double denominator = 1 - this->map.curvature(this->s_pos) * this->d_pos;
  return  numerator / denominator;
}


/** Check if the ego vehicle is deadlocked between other vehicles. 
 * 
 * The ego vehicle is declared deadlocked if 
 *  a) it is on a side lane, 
 *  b) the leading vehicle in the current lane is in an appropriate distance 
 *  c) the gap between the leading and the closest adjacent vehicle is small 
 */
bool EgoVehicle::check_deadlock() {

  // The case in which the ego vehicle is in the center lane must not be 
  // handled here since it is implicity covered by the successor states
  if (this->current_lane == LANE::CENTER) {return false;}
  
  // Initialize deadlock flag
  bool deadlocked = false;

  // Determine if the leading vehicle in the current lane is close enough to
  // be considered as blocking the lane
  Vehicle cur_lead = this->leading_vehicles[this->current_lane];
  bool leading_vehicle_close = cur_lead.is_present && !cur_lead.to_close &&
                               cur_lead.s_dist < 2 * DEADLOCK_GAP; 

  // Determine if a deadlock causing adjacent vehicle exists
  if (leading_vehicle_close) {
      Vehicle vehicle = get_deadlock_causing_vehicle(cur_lead);
      if (vehicle.is_present) {
        deadlocked = true; 
        this->deadlock_causing_vehicle = vehicle;
      }
  }
  return deadlocked;
}


/** Determine the deadlock causing vehicle in the center lane. 
 * 
 * The nearby vehicle in the center lane is considered as causing a deadlock if 
 *    a) it is in the current leading vehicle's rear 
 *    b) the gap formed by it and the current leading vehicle is smaller than 
 *       DEADLOCK_GAP
 * 
 * @param cur_lead the leading vehicle in the current lane
 */
Vehicle EgoVehicle::get_deadlock_causing_vehicle(Vehicle &cur_lead) {

  // Initialize empty vehicle
  Vehicle deadlock_causing_vehicle;
  // Return the deadlocked causing vehicle in the center lane.
  for (Vehicle center_nearby: nearby_vehicles[LANE::CENTER]) {
    if (cur_lead.in_front_of(center_nearby)) {
      double s_dist = s_distance(cur_lead.s_pos, center_nearby.s_pos);
      if (s_dist < DEADLOCK_GAP) {
        deadlock_causing_vehicle = center_nearby;
      }
    }
  }
  return deadlock_causing_vehicle;
}


/** Check if the opposite lane is free. 
 * 
 * The left lane's opposite is the right lane and vice versa. 
 * The center lane has no opposite lane. 
 * 
 * The opposite lane is free if 
 *  a) the leading vehicle there is in an appropriate distance and in front of 
 *     the leading vehicle in the current lane 
 *  b) the closest rear vehicle there is in an appropriate distance and its 
 *     gap with the closest vehicle on the center lane is big enough
 */
bool EgoVehicle::check_opposite_lane() {

  // Determine the opposite lane
  int opposite_lane;
  if (this->current_lane == LANE::RIGHT) { opposite_lane = LANE::LEFT; }
  else if (this->current_lane == LANE::LEFT) { opposite_lane = LANE::RIGHT; }
  else { return false; }  // Center lane has no opposite lane 

  // Determine the leading vehicles on the current and opposite lanes 
  Vehicle cur_lead = this->leading_vehicles[this->current_lane];
  Vehicle opp_lead = this->leading_vehicles[opposite_lane];
  // Determine the closest rear vehicle on the opposite lane
  Vehicle opp_rear;
  for (Vehicle opp_nearby: this->nearby_vehicles[opposite_lane]) {
    if (this->in_front_of(opp_nearby)) {
      opp_rear = opp_nearby;
      break;
    }
  }
  // Determine the closest vehicle in the center lane
  Vehicle center_nearby;
  if (!this->nearby_vehicles[LANE::CENTER].empty()) {
    center_nearby = this->nearby_vehicles[LANE::CENTER][0];
  }

  // The opposite lane is not free if 
  // a) the leading vehicle in the opposite lane is to close or 
  //    in the rear of the leading vehicle in the current lane
  if (opp_lead.is_present && 
     (opp_lead.s_dist < DEADLOCK_GAP || opp_lead.in_rear_of(cur_lead))) {
    return false;
  }
  // b) the rear vehicle on the opposite lane is to close or
  //    its gap with the closest vehicle in the center lane is not big enough 
  if (opp_rear.is_present) {
    bool opp_rear_to_close = opp_rear.s_dist < 0.5 * DEADLOCK_GAP;
    bool gap_to_small = center_nearby.is_present 
      && s_distance(center_nearby.s_pos, opp_rear.s_pos) < DEADLOCK_GAP;
    if (opp_rear_to_close || gap_to_small) {
      return false;
    }
  }
  
  return true;
}


/** Check if adjacent lanes are blocked for single lane changes. 
 * 
 * Adjacent lanes are not available if  
 * 
 *    a) the vehicles there are to close or 
 *    b) front vehicles there are to slow for their distance 
 *    c) rear vehicles there are to fast for their distance 
 * 
 * @param vehicle a nearby vehicle 
 */
bool EgoVehicle::single_blocked_by(Vehicle &vehicle) {
  
  double hard_thres = 1 * BLOCK_DIST;
  double soft_thres = 4 * BLOCK_DIST;
  double tolerable_s_vel_diff = (vehicle.s_dist - hard_thres);
  
  // Adjacent lanes are not available if the vehicles there are to close
  if (vehicle.s_dist < hard_thres) {
    return true;
  } 
  // or if front vehicles there are to slow for their distance
  else if (this->in_rear_of(vehicle)) {
    return vehicle.s_dist < soft_thres && 
           this->speed - vehicle.s_vel > tolerable_s_vel_diff * 0.3;
  } 
  // or if rear vehicles there are to fast for their distance
  else { 
    return vehicle.s_dist < soft_thres && 
           vehicle.s_vel - this->speed > tolerable_s_vel_diff * 0.5;
  }
}


/** Check if adjacent lanes are blocked for double lane changes. 
 * 
 * Adjacent lanes are not available if  
 * 
 *    a) the vehicles there are to close or 
 *    b) front vehicles there are to slow for their distance 
 *    c) rear vehicles there are to fast for their distance 
 * 
 * @param vehicle a nearby vehicle 
 */
bool EgoVehicle::double_blocked_by(Vehicle &vehicle) {
  
  double hard_thres = 3 * BLOCK_DIST;
  double soft_thres = 5 * BLOCK_DIST;
  double tolerable_s_vel_diff = (vehicle.s_dist - hard_thres);
  
  // Lane changes are not available if the vehicles there are to close
  if (vehicle.s_dist < hard_thres) {
    return true;
  } 
  // or if front vehicles there are to slow for their distance
  else if (this->in_rear_of(vehicle)) {
    return vehicle.s_dist < soft_thres && 
           this->speed - vehicle.s_vel > tolerable_s_vel_diff * 1.0;
  } 
  // or if rear vehicles there are to fast for their distance
  else { 
    return vehicle.s_dist < soft_thres && 
           vehicle.s_vel - this->speed > tolerable_s_vel_diff * 1.0;
  }
}


// Check if the ego vehicle is in the front of an other vehicle
bool EgoVehicle::in_front_of(Vehicle &vehicle) {
  // Just a value to handle completed highway laps
  double eps = 10;
  // s distance between ego vehicle and other vehicle
  double delta_s = this->s_pos - vehicle.s_pos;
  // ego vehicle is in front
  bool in_front = delta_s > 0;
  // vehicle is in front (completed highway lap)
  double s_dist = s_distance(this->s_pos, vehicle.s_pos);
  bool in_front_lap = delta_s < 0 && abs(delta_s) > s_dist + eps;
  return in_front || in_front_lap;
}


// Check if the ego vehicle is in the rear of an other vehicle
bool EgoVehicle::in_rear_of(Vehicle &vehicle) {
  return !this->in_front_of(vehicle);
}


/** Initialize a trajectory by its polynomial components.
 * 
 * The longitudinal and lateral polynomials are used to compute the 
 * trajectory's waypoints in the Frenet and global systems. 
 * 
 * @param s_poly longitudinal s polynomial 
 * @param d_poly lateral d polynomial 
 * @param map highway map class
 */
Trajectory::Trajectory(Polynomial s_poly, Polynomial d_poly, Map &map) {

  this->s_poly = s_poly;
  this->d_poly = d_poly;

  // Compute the trajectory's total cost
  this->cost = KLON * this->s_poly.cost + KLAT * this->d_poly.cost;
 
  // Set feasibility flag
  if (s_poly.is_feasible && d_poly.is_feasible) {
    this->is_feasible = true;
  } else {
    this->is_feasible = false;
  }

  // Initialize the trajectory using the waypoints of its polynomial components
  this->s_pos_wpts = this->s_poly.pos_wpts;
  this->s_vel_wpts = this->s_poly.vel_wpts;
  this->s_acc_wpts = this->s_poly.acc_wpts;
  this->d_pos_wpts = this->d_poly.pos_wpts;
  this->d_vel_wpts = this->d_poly.vel_wpts;
  this->d_acc_wpts = this->d_poly.acc_wpts;

  // Compute the waypoints in Cartesian x, y coordinates and the yaw waypoints
  this->x_pos_wpts.clear();
  this->y_pos_wpts.clear();
  this->yaw_wpts.clear();
  for (size_t i = 0; i < this->s_pos_wpts.size(); ++i) {
    double s = this->s_pos_wpts[i];
    double d = this->d_pos_wpts[i];
    vector<double> xy = map.transform_sd2xy(s, d);
    this->x_pos_wpts.push_back(xy[0]);
    this->y_pos_wpts.push_back(xy[1]);

    // Initialize the trajectory's yaw waypoints
    // This will be used in the update step to compute the collision circles
    double yaw = i == 0 ? 0 : atan2(xy[1] - this->y_pos_wpts[i-1], 
                                    xy[0] - this->x_pos_wpts[i-1]);
    this->yaw_wpts.push_back(yaw);
  
  // Update trajectory target lane (used only for logging)
  this->target_lane = lane(this->d_pos_wpts.back());

  // Set empty flag to false
  this->is_empty=false;
  } 
}


/** Update trajectory with new polynomials and recompute its waypoints. 
 * 
 * The consumed waypoints of the current trajectory will be removed. The 
 * remaining waypoints will be kept up to the transition point in the future. 
 * From there on the new polynomial trajectory waypoints will be integrated.
 * The global trajectory is checked for its feasibility.
 * 
 * @param s_poly longitudinal s polynomial 
 * @param d_poly lateral d polynomial 
 * @param num_consumed_wpts number of already consumed trajectroy waypoints
 * @param map highway map class
 * @param scenario driving scenario case (standard, recovery or emergency)
 */
void Trajectory::update(Polynomial s_poly, Polynomial d_poly,
                        int num_consumed_wpts, Map &map, int scenario) {
  
  this->s_poly = s_poly;
  this->d_poly = d_poly;

  // Compute the trajectory's total cost
  this->cost = KLON * this->s_poly.cost + KLAT * this->d_poly.cost;
 
  // Set feasibility flag
  if (s_poly.is_feasible && d_poly.is_feasible) {
    this->is_feasible = true;
  } else {
    this->is_feasible = false;
  }

  // Update the waypoints from the previous cycles
  vector<double> s_pos, s_vel, s_acc, 
                 d_pos, d_vel, d_acc, 
                 x_pos, y_pos, yaw;

  // Keep the waypoints that were not yet consumed by the simulator 
  // up to the transition point in the future
  int start_wpt = num_consumed_wpts + NUM_TRANSITION_WPTS;
  for (int i = num_consumed_wpts; i < start_wpt; ++i) {
    s_pos.push_back(this->s_pos_wpts[i]);
    s_vel.push_back(this->s_vel_wpts[i]);
    s_acc.push_back(this->s_acc_wpts[i]);
    d_pos.push_back(this->d_pos_wpts[i]);
    d_vel.push_back(this->d_vel_wpts[i]);
    d_acc.push_back(this->d_acc_wpts[i]);
    x_pos.push_back(this->x_pos_wpts[i]);
    y_pos.push_back(this->y_pos_wpts[i]);
    yaw.push_back(this->yaw_wpts[i]);
  }

  // Concatenate the kept waypoints with the new updated ones coming from 
  // the polynomial trajectories (only Frenet)
  s_pos.insert(s_pos.end(), s_poly.pos_wpts.begin(), s_poly.pos_wpts.end());
  s_vel.insert(s_vel.end(), s_poly.vel_wpts.begin(), s_poly.vel_wpts.end());
  s_acc.insert(s_acc.end(), s_poly.acc_wpts.begin(), s_poly.acc_wpts.end());
  d_pos.insert(d_pos.end(), d_poly.pos_wpts.begin(), d_poly.pos_wpts.end());
  d_vel.insert(d_vel.end(), d_poly.vel_wpts.begin(), d_poly.vel_wpts.end());
  d_acc.insert(d_acc.end(), d_poly.acc_wpts.begin(), d_poly.acc_wpts.end());

  // Store the updated trajectory (Frenet) waypoints
  this->s_pos_wpts = s_pos;
  this->s_vel_wpts = s_vel;
  this->s_acc_wpts = s_acc;
  this->d_pos_wpts = d_pos;
  this->d_vel_wpts = d_vel;
  this->d_acc_wpts = d_acc;
  
  // Recompute the Cartesian x, y and (derived) yaw waypoints
  this->x_pos_wpts = x_pos;
  this->y_pos_wpts = y_pos;
  this->yaw_wpts = yaw;

  // Loop over the replaned polynomial trajectory waypoints
  for (size_t i = 0; i < this->s_poly.pos_wpts.size(); ++i) {
    double s = this->s_poly.pos_wpts[i];
    double d = this->d_poly.pos_wpts[i];

    // Transform the replaned Frenet into Cartesian coordinates
    vector<double> xy = map.transform_sd2xy(s, d);
    this->x_pos_wpts.push_back(xy[0]);
    this->y_pos_wpts.push_back(xy[1]);    

    // Derive the ego vehicle's yaw by two adjacent waypoints
    int j = NUM_TRANSITION_WPTS + i; 
    double yaw = atan2(xy[1] - this->y_pos_wpts[j-1], 
                       xy[0] - this->x_pos_wpts[j-1]);
    this->yaw_wpts.push_back(yaw);

    // Check if the global speed is feasible (not for emergencies)
    double x_vel = (xy[0] - this->x_pos_wpts[j-1]) / DT;
    double y_vel = (xy[1] - this->y_pos_wpts[j-1]) / DT;
    double speed = hypot(x_vel, y_vel);
    if (!(scenario == SCN::EMERGENCY) & speed > VMAX) {
      this->is_feasible = false;
      return;
    }
  }

  // Update trajectory target lane (used only for logging)
  this->target_lane = std::max(0, std::min(lane(this->d_pos_wpts.back()), 2));

  // Set empty flag to false
  this->is_empty=false;
}


/** Check the trajectory for collisions with the nearby vehicles. 
 * 
 * Simple circle-to-circle collision check with three circles defining the 
 * vehicle contour. The ego vehicles circles are shifted somewhat to the 
 * front to provide an additional safety buffer. 
 * 
 * @param nearby_vehicles the nearby vehicles (1D vector)
 * @param scenario driving scenario case (standard, recovery or emergency)
 */
void Trajectory::check_collision(vector<Vehicle> &nearby_vehicles,
                                 Map &map, int scenario) {

  // Intialize collision flag
  this->is_collided = false;

  vector<double> circle_front, circle_center, circle_rear;
  vector<vector<double>> circles_other, circles_ego;
  
  // Offset of the front and rear circle midpoints
  double offset = 0.5 * (VEHICLE_LENGTH - VEHICLE_WIDTH);

  // Collision checks
  // Loop over all nearby vehicles
  for (auto &vehicle: nearby_vehicles) {
    // Loop over the trajectory time intervall
    for (size_t i = 0; i < this->x_pos_wpts.size(); ++i) {
      // The ego vehicles position and yaw 
      double x_ego = this->x_pos_wpts[i];
      double y_ego = this->y_pos_wpts[i];
      double yaw = this->yaw_wpts[i];
      // The ego vehicle circles will get an offset to the front to provide an 
      // additional buffer there. On the back the following vehicle is 
      // responsible to avoid a collision. 
      double x_ego_center = x_ego + offset / 7 * cos(yaw);
      double y_ego_center = y_ego + offset / 7 * sin(yaw);
      circle_center = {x_ego_center, y_ego_center};
      double x_ego_front = x_ego_center + offset * cos(yaw);
      double y_ego_front = y_ego_center + offset * sin(yaw);
      circle_front = {x_ego_front, y_ego_front};
      double x_ego_rear = x_ego_center - offset * cos(yaw);
      double y_ego_rear = y_ego_center - offset * sin(yaw);
      circle_rear = {x_ego_rear, y_ego_rear};
      
      // Ego vehicle collision detection circles
      circles_ego = {circle_front, circle_center, circle_rear};
      
      // The other vehicles position
      double t = i * DT;
      double s_other = vehicle.s_position(t);
      double d_other = vehicle.d_pos;
      circle_center = map.transform_sd2xy(s_other, d_other);
      // The front and rear circles of the other vehicle are determined
      // by making the simplified assumption that it will keep its current lane
      double s_other_front = s_other + offset;
      circle_front = map.transform_sd2xy(s_other_front, d_other);
      double s_other_rear = s_other - offset;
      circle_rear = map.transform_sd2xy(s_other_rear, d_other);
      
      // Other vehicle collision detection circles
      circles_other = {circle_front, circle_center, circle_rear};
      
      // Check all circle combinations for collisions
      for (auto circle_ego: circles_ego) {
        for (auto circle_other: circles_other) {
          // A collision occurs if two circles overlap. It is assumed that all
          // vehicles have the same dimensions. The VEHICLE_WIDTH is defined to
          // be circle diameter. The circles overlap if the distance of their 
          // centers is smaller than the sum of their radii, that is the 
          // vehicle width.
          double dist = distance(circle_ego[0], circle_ego[1], 
                                 circle_other[0], circle_other[1]);
          if (dist < VEHICLE_WIDTH) {  
            // In case of emergencies there shall be no hard exclusion of 
            // trajectories. It is better to have a defined collision then
            // no trajectory at all. To assure the collision has a low impact,
            // the trajectory cost is increased with each collision event, 
            // whereby the cost increases with the degree of overlap and the
            // velocity difference of the involved vehicles.
            if (scenario == SCN::EMERGENCY) {
              double s_vel_diff = this->s_vel_wpts[i] - vehicle.s_vel;
              this->cost += 100 * (VEHICLE_WIDTH - dist) * pow(s_vel_diff, 2);
            } else {
              this->is_collided = true;
              return;
            }
          }
        }
      }
    }
  }
}
