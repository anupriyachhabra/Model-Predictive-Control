#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  vector<double> x_vals;
  vector<double> y_vals;
  vector<double> psi_vals;
  vector<double> v_vals;
  vector<double> cte_vals;
  vector<double> epsi_vals;
  vector<double> delta_vals = {};
  vector<double> a_vals = {};
  vector<double> next_x_vals = {};
  vector<double> next_y_vals = {};

};

#endif /* MPC_H */
