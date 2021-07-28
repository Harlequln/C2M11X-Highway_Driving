#ifndef POLYNOMIALS_H
#define POLYNOMIALS_H

#include "parameters.h"

#include "Eigen-3.3/Eigen/Dense"

#include <vector>
#include <string>

class Polynomial {

public:
  Polynomial() {is_feasible=false;};
  Polynomial(const std::vector<double> start, 
             const std::vector<double> end, 
             double T, int degree, int dof);
  ~Polynomial() {};

  void compute_cost(double goal_pos=0, 
                    const std::vector<double> &lane_cost=std::vector<double>());
  void evaluate(int num_waypoints, int scenario=SCN::STANDARD);
  
  double cost;
  double cost_jerk;
  double cost_duration;
  double cost_velocity;
  double cost_position;
  bool is_feasible;
  double T;
  std::vector<double> start;
  std::vector<double> end;
  std::vector<double> pos_wpts;
  std::vector<double> vel_wpts;
  std::vector<double> acc_wpts; 
  std::vector<double> jrk_wtps;

private:
  double eval_position(double t);
  double eval_velocity(double t);
  double eval_acceleration(double t);
  double eval_jerk(double t);
  void solve_quintic(const std::vector<double> &start, 
                     const std::vector<double> &end, 
                     double T);
  void solve_quartic(const std::vector<double> &start, 
                     const std::vector<double> &end, 
                     double T);

  int degree;
  int dof;
  double goal;
  Eigen::VectorXd coeff;
  Eigen::VectorXd time_pars;
};

#endif //POLYNOMIALS_H