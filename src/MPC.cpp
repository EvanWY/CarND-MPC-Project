#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <ctime>

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 60;
double dt = 0.015;
double delay = 0.1;
int fixed_steps = ceil(delay / dt);

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    Eigen::VectorXd coeffs_der(3);
    coeffs_der << coeffs[1], 2*coeffs[2], 3*coeffs[3];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      AD<double> x0 = vars[x_start + t -1];
      AD<double> y0 = vars[y_start + t -1];
      AD<double> psi0 = vars[psi_start + t -1];
      AD<double> v0 = vars[v_start + t -1];
      AD<double> cte0 = vars[cte_start + t -1];
      AD<double> epsi0 = vars[epsi_start + t -1];

      AD<double> delta0 = vars[delta_start + t -1];
      AD<double> a0 = vars[a_start + t -1];

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.

      // TODO: Setup the rest of the model constraints

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 / Lf * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      
      fg[1 + cte_start + t] = cte1 - y1 + (coeffs[0]+coeffs[1]*x1+coeffs[2]*x1*x1+coeffs[3]*x1*x1*x1);
      AD<double> dfx = coeffs_der[0] + coeffs_der[1]*x1 + coeffs_der[2]*x1*x1;
      fg[1 + epsi_start + t] = epsi1 - psi1 + CppAD::atan(dfx);

      // unfortunately, even if we can compute to very small time interval, the control signal can only be updated 10 times per second.
      if (t % fixed_steps != 0) {
        v1 = v0;
      }

      //fg[0] = -v1;
      fg[0] += cte1*cte1 // meter
          + (10*epsi1) * (10*epsi1) // 
          //+ (v1-50)*(v1-50) // optimize speed to as close to 50 as possible
          - v1 // optimize speed to as fast as possible
          //+ (60*delta0) * (60*delta0) // delta : [-0.49, 0.49] // optimize delta0
          //+ (2*a0) * (2*a0); // a: [-1, 1] // optimize a0
          + ((v0*v0 / Lf * delta0 * v0*v0 / Lf * delta0) + a0*a0/100) / 100 // instad of seperating delta0 with a0, I decided to combine both acceleration and limit the combined force.
          ;

      if (t < N-1) {
        AD<double> delta1 = vars[delta_start + t];
        AD<double> a1 = vars[a_start + t];
        fg[0] += (delta1 - delta0) * (delta1 - delta0) * 4 // optimize turning rate changing rate
          + (a1 - a0) * (a1 - a0) * 0.02; // optimize acceleration changing rate
      }
      //fg[0] += 2*(v1-20)*(v1-20) + delta0*delta0 + a0*a0;
      //fg[0] +=  cte1*cte1 + epsi1*epsi1 + (a0-0.5)*(a0-0.5);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  double prev_delta = state[6];
  double prev_a = state[7];

  //std::clock_t currentTimestamp = std::clock();
  //double delay = (currentTimestamp - timestamp)/ (double) CLOCKS_PER_SEC;
  //fixed_steps = min(int(N), fixed_steps);
  //timestamp = currentTimestamp;
  //std::cout<< std::clock()/ (double) CLOCKS_PER_SEC << " " << delay << " : " << fixed_steps <<std::endl;
  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  vars[delta_start] = prev_delta;
  vars[a_start] = prev_a;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  for (int i = 0; i < delta_start; i++) {
      vars_lowerbound[i] = -1.0e19;
      vars_upperbound[i] = 1.0e19;
  }
  for (int i = delta_start; i < a_start; i++) {
      vars_lowerbound[i] = -0.436332;
      vars_upperbound[i] = 0.436332;
  }
  for (int i = a_start; i < n_vars; i++) {
      vars_lowerbound[i] = -10.0;
      vars_upperbound[i] = 10.0;
  }

  // first fixed steps, the control signal are setted on last iteration
  for (int i = 0; i < fixed_steps; i++) {
      vars_lowerbound[i + delta_start] 
        = vars_upperbound[i + delta_start] 
        = vars[i + delta_start]
        = prev_delta;
      vars_lowerbound[i + a_start] 
        = vars_upperbound[i + a_start] 
        = vars[i + a_start]
        = prev_a;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = constraints_upperbound[x_start] = x;
  constraints_lowerbound[y_start] = constraints_upperbound[y_start] = y;
  constraints_lowerbound[psi_start] = constraints_upperbound[psi_start] = psi;
  constraints_lowerbound[v_start] = constraints_upperbound[v_start] = v;
  constraints_lowerbound[cte_start] = constraints_upperbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          100\n";

  //std::cout << "before run" << std::endl;

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  std::vector<double> ans = {solution.x[delta_start+fixed_steps], solution.x[a_start+fixed_steps]};
  
  //std::cout << "size :" << solution.x.size() << std::endl;
  for (int i=0; i<solution.x.size(); i++) {
    ans.push_back(solution.x[i]);
  }
  return ans;
}
