#include "polynomials.h"
#include "utils.h"

#include <cmath>
#include <iostream>
#include <cstdio>

using namespace Eigen;


/** Create a jerk minimal trajectory by solving a quintic or quartic polynomial. 
 * 
 * The determined trajectory describes the movement of the ego vehicle in one 
 * of the Frenet s- or d- dimensions from an initial state to a final state 
 * during the time T. The start and end states define the boundary conditions 
 * at the beginning and the end of the trajectory. Two kinds of polynomials are 
 * considered in this implementation: quintics and quartics. 
 * 
 * Quintics: 
 * Quintics are 5th degree polynomials with six unknown coefficients describing 
 * the ego vehicle's jerk minimized motion in time. 
 * 
 *    x = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
 * 
 * In order to be able to solve for the six unknown coefficients six variables 
 * need to be defined as boundary conditions: the ego vehicle's position, 
 * velocity and acceleration at the trajectory's start and end.
 * 
 *    start_state = {start_pos, start_vel, start_acc}
 *    final_state = {final_pos, final_vel, final_acc}
 * 
 * Quartics: 
 * In many situations, such as driving with no vehicles directly ahead, the 
 * ego vehicle does not necessarily have to be at a certain position but needs 
 * to adapt to a desired velocity given by the behavioral level. That's why the 
 * final location of the car is not required in the boundary condition at the 
 * end of trajectory. With one degree of freedom less a quartic polynomial is 
 * sufficient to describe the jerk minimized motion in time. 
 * 
 *    x = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4
 * 
 * with boundary states
 * 
 *    start_state = {start_pos, start_vel, start_acc}
 *    final_state = {final_vel, final_acc}
 * 
 * @param start inital state vector of the vehicle, e.g. {s_pos, s_vel, s_acc}
 * @param end final state vector of the vehicle, e.g. {s_vel, s_acc}
 * @param T duration in seconds required to travers the trajectory
 * @param degree polynomial degree, DEG::QUINTIC (5) or DEG::QUARTIC (4)
 * @param dof degree of freedom of trajectory, FRE::S (0) or FRE::D (1)
 */
Polynomial::Polynomial(const std::vector<double> start, 
                       const std::vector<double> end, 
                       double T, int degree, int dof) 
  : start(start), end(end), T(T), degree(degree), dof(dof), 
    coeff(6), time_pars(6) {
  
  // Determine the polynomial's coefficients
  if (degree == DEG::QUINTIC) { 
    solve_quintic(start, end, T);
  } 
  else if (degree == DEG::QUARTIC) {
    solve_quartic(start, end, T);
  }
}


/** Compute discrete waypoints of the trajectory and check its feasibility. 
 * 
 * Waypoints will be computed for the position, velocity, acceleration and 
 * jerk of the polynomial trajectory. The evaluation will begin at time t=0 
 * and continue until num_waypoints were computed. The time delta bewtween 
 * each pair of waypoints is 0.02 seconds. Each waypoint will be checked for 
 * its feasibility: 
 * 
 *  a) the ego vehicles velocity is below the maximum allowed velocity, 
 *  b) the accelerations and jerks are below their thresholds, 
 *  c) the ego cars location is within the drivable highway area. 
 * 
 * For recovery and emergency driving scenarios the feasibility checks are 
 * eased: safety before comfort. If any of the feasibility checks fails the 
 * complete polynomial trajectory is declared as non-feasible.
 * 
 * @param num_waypoints number of waypoints to generate/replan
 * @param scenario driving scenario case (standard, recovery or emergency)
 */
void Polynomial::evaluate(int num_waypoints, int scenario) {
  
  // Reset the waypoint collectors
  this->pos_wpts.clear();
  this->vel_wpts.clear();
  this->acc_wpts.clear();
  this->jrk_wtps.clear();
  
  // Initialize feasibility flag
  this->is_feasible = true;  
  // Initilize driving scenario flags
  bool standard = scenario == SCN::STANDARD;
  bool recovery = scenario == SCN::RECOVERY;
  bool emergency = scenario == SCN::EMERGENCY;

  for (int i = 0; i < num_waypoints; ++i) {

    // Compute waypoints
    double t = i * DT;
    // Treat completed highway track laps
    double pos = fmod(this->eval_position(t), SMAX);
    double vel = this->eval_velocity(t);
    double acc = this->eval_acceleration(t);
    double jrk = this->eval_jerk(t);

    // Check for each timestep if the trajectory exceeds the feasible maximum.
    // In case of recovery and emergency scenarios the feasibility (comfort) 
    // checks are eased. However, in a real world application feasibility would 
    // need to be checked in all cases to respect physical feasibility.
    double acc_lim = emergency ? AMAX * 5 : recovery ? AMAX * 3 : AMAX;
    double jrk_lim = emergency ? JMAX * 5 : recovery ? JMAX * 3 : JMAX;
    double vel_lim = emergency ? VMAX + 5 : recovery ? VMAX + 3 : VMAX;
    bool acc_flag = abs(acc) > acc_lim;
    bool jrk_flag = abs(jrk) > jrk_lim;
    bool vel_flag = vel > vel_lim;
    // For lateral quintic polynomials additionally check if the trajectory 
    // stays inside the drivable highway area
    bool pos_flag = degree == DEG::QUINTIC && 
                    dof == FRE::D && 
                    (pos < 0.5 || pos > 11.00);
    // Check feasibility if the waypoint doesn't largely extend the duration T
    if ((standard || t < T + 0.5) && 
        (vel_flag || acc_flag || jrk_flag || pos_flag)) {
      this->is_feasible = false;
      
      printf("Not feasible at t=%4.2f due to (vel|acc|jrk|pos): "
            "%d%d%d%d (%6.2f %6.2f %7.2f %8.2f) \n", 
            t,
            vel_flag, acc_flag, jrk_flag, pos_flag, 
            vel, acc, jrk, pos);

      break;
    }

    // Store waypoints
    this->pos_wpts.push_back(pos);
    this->vel_wpts.push_back(vel);
    this->acc_wpts.push_back(acc);
    this->jrk_wtps.push_back(jrk);
  }
}


/** Evaluate the position on the trajectory at time t.
 * Quintic: p = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5 
 * Quartic (a5=0): p = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4
 */
double Polynomial::eval_position(double t) {
  time_pars << 1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5);
  double pos = coeff.dot(time_pars);
  return pos;
}


/** Evaluate the velocity (1st derivative) on the trajectory at time t.
 * Quintic: v = a1 + 2 * a2 * t + 3 * a3 * t^2 + 4 * a4 * t^3 + 5 * a5 * t^4
 * Quartic (a5=0): v = a1 + 2 * a2 * t + 3 * a3 * t^2 + 4 * a4 * t^3 
 */
double Polynomial::eval_velocity(double t) {
  time_pars << 0, 1, 2*t, 3*pow(t,2), 4*pow(t,3), 5*pow(t,4);
  return coeff.dot(time_pars);
}


/** Evaluate the acceleration (2nd derivative) on the trajectory at time t.
 * Quintic: a = 2 * a2 + 6 * a3 * t + 12 * a4 * t^2 + 20 * a5 * t^3 
 * Quartic (a5=0): a = 2 * a2 + 6 * a3 * t + 12 * a4 * t^2
 */
double Polynomial::eval_acceleration(double t) {
  time_pars << 0, 0, 2, 6*t, 12*pow(t,2), 20*pow(t,3);
  return coeff.dot(time_pars);
}


/** Evaluate the jerk (3rd derivative) on the trajectory at time t.
 * Quintic: j = 6 * a3 + 24 * a4 * t + 60 * a5 * t^2 
 * Quartic (a5=0): j = 6 * a3 + 24 * a4 * t
 */
double Polynomial::eval_jerk(double t) {
  time_pars << 0, 0, 0, 6, 24*t, 60*pow(t,2);
  return coeff.dot(time_pars);
}


/** Solve the quintic polynomial for its coefficients. 
 * 
 * @param start inital state vector of the vehicle {pos, vel, acc}
 * @param end final state vector of the vehicle {pos, vel, acc}
 * @param T duration in seconds required to travers the trajectory
 */
void Polynomial::solve_quintic(const std::vector<double> &start,
                               const std::vector<double> &end, double T) {

  Eigen::Matrix3d A;
  Eigen::Vector3d b, x;

  A <<     pow(T, 3),      pow(T, 4),      pow(T, 5), 
       3 * pow(T, 2),  4 * pow(T, 3),  5 * pow(T, 4), 
       6 * T,         12 * pow(T, 2), 20 * pow(T, 3);

  b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * pow(T, 2)), 
       end[1] - (start[1] + start[2] * T),
       end[2] - start[2];

  x = A.colPivHouseholderQr().solve(b);

  // Store polynomial coefficients and duration
  // coeff << a0, a1, a2, a3, a4, a5
  coeff << start[0], start[1], 0.5 * start[2], x(0), x(1), x(2);
  this->T = T;
}


/** Solve the quartic polynomial for its coefficients. 
 * 
 * The final location of the car is not required for the quartic polynomial. 
 * 
 * @param start inital state vector of the vehicle {pos, vel, acc}
 * @param end final state vector of the vehicle {vel, acc}
 * @param T duration in seconds required to travers the trajectory
 */
void Polynomial::solve_quartic(const std::vector<double> &start,
                               const std::vector<double> &end, double T) {
  
  Eigen::Matrix2d A;
  Eigen::Vector2d b, x;

  A << 3 * pow(T, 2),  4 * pow(T, 3),  
       6 * T,         12 * pow(T, 2);

  b << end[0] - (start[1] + start[2] * T), 
       end[1] - start[2];

  x = A.colPivHouseholderQr().solve(b);

  // Store polynomial coefficients and duration
  // (a5 is set to zero since it is not used for quartic polynomials)
  // coeff << a0, a1, a2, a3, a4, 0
  coeff << start[0], start[1], 0.5 * start[2], x(0), x(1), 0;
  this->T = T;
}


/** Compute the polynomial trajectory's cost. 
 *  
 * The cost will increase with 
 * 
 *    high jerks, 
 *    high durations, 
 *    high deviations from the desired end location (DEG::QUINTIC), 
 *    high deviations from the maximum allowed lane velocity (FRE::S) 
 * 
 * The following parameters are only required for quintic polynomials 
 * 
 * @param goal_pos Desired goal position at the end of the trajectory. Only 
 *                 required for quintic polynomials (DEG::QUINTIC).
 * @param lane_cost Vector with a jam cost for each lane. Only required for 
 *                  lateral quintic polynomials (DEG::QUINTIC, FRE:D)
 */
void Polynomial::compute_cost(double goal_pos, 
                              const std::vector<double> &lane_cost) {

  /** Jerk cost
   * Integral of the squared jerk with respect to t evaluated for [0,T]. 
   * 
   * Quintic:
   * j^2 = 36 * a3^2 
   *     + 288 * a3 * a4 * t 
   *     + 720 * a3 * a5 * t^2 
   *     + 576 * a4^2 * t^2 
   *     + 2880 * a4 * a5 * t^3 
   *     + 3600 * a5^2 * t^4
   * 
   * integral(j^2) | 0,T = 36 * a3^2 * T 
   *                     + 144 * a3 * a4 * T^2 
   *                     + (240 * a3 * a5 + 192 * a4^2) * T^3 
   *                     + 720 * a4 * a5 * t^4 
   *                     + 720 * a5^2 * t^5
   * 
   * Quartic:
   * Same as Quintic with a5=0
   */
  cost_jerk = 36 * coeff(3) * coeff(3) * T 
            + 144 * coeff(3) * coeff(4) * T*T 
            + (240 * coeff(3) * coeff(5) + 192 * coeff(4) * coeff(4)) * T*T*T
            + 720 * coeff(4) * coeff(5) * T*T*T*T
            + 720 * coeff(5) * coeff(5) * T*T*T*T*T;
  cost_jerk *= KJ;  // weighted by KJ

  // Duration cost
  cost_duration = T * KT;  // weighted by KT

  // Position cost 
  double kp = (dof == FRE::S) ? KS : KD;  // position cost weight
  cost_position = 0;
  if (degree == DEG::QUINTIC) {
    // Quintic polynomials have a defined end position (e.g. the center of the 
    // target lane or at a safety distance behind a leading vehicle)
    // The cost is the squared residual of the defined end position and the 
    // desired goal position 
    cost_position = pow(eval_position(T) - goal_pos, 2);
    // For lateral polynomials also add a cost term for jammed lanes
    if (dof == FRE::D) { cost_position += lane_cost[lane(goal_pos)]; }
  }
  cost_position *= kp;  // weighted by position cost weight

  // Velocity cost (only for longitudinal trajectories)
  // Squared residual of final velocity and maximum lane velocity
  cost_velocity = 0;
  if (dof == FRE::S) {
    cost_velocity = pow(eval_velocity(T) - VMAX, 2) * KV;  // weighted by KV
  }

  // Compute the final weighted cost
  cost = cost_jerk + cost_duration + cost_position + cost_velocity;
}
