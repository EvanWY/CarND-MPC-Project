#ifndef MPC_H
#define MPC_H

#include <vector>
#include <iostream>
#include <ctime>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
 private:
  std::clock_t timestamp = std::clock();
};

#endif /* MPC_H */
