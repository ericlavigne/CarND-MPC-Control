#include "MPC.h"
#include <iostream>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;


// Evaluate a polynomial.
AD<double> poly_eval(Eigen::VectorXd coeffs, AD<double> x) {
    AD<double> result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}
double poly_eval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

Eigen::VectorXd poly_derivative(Eigen::VectorXd coeffs) {
    Eigen::VectorXd deriv(coeffs.size()-1);
    for(int i = 0; i < coeffs.size()-1; i++) {
        deriv(i) = coeffs(i+1) * (i+1);
    }
    return deriv;
}

// Need to see three seconds ahead to handle sharpest turns at 110 MPH.
// Actuation delay of 100 millis makes shorter timesteps useless.
// But in the beginning, use smaller lookahead to simplify calculations.
size_t N = 10;
double dt = 0.05;

// Cost parameters
double ref_speed = 50;
double speed_weight = 1.0;
double direction_weight = 1.0;
double road_center_weight = 6.0;
double on_road_weight = 1.0;
double minimal_steer_weight = 1.0;
double minimal_acceleration_weight = 1.0;
double steer_jerk_weight = 500.0;
double acceleration_jerk_weight = 1.0;

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

// Variety of info dumped into one big vector. Keep track of what each section means.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1; // Because there are only N-1 delta and a values


class FG_eval {
    public:
    // Fitted polynomial coefficients and corresponding derivative
    Eigen::VectorXd coeffs;
    Eigen::VectorXd deriv_coeffs;
    double previous_steer;
    double previous_throttle;

    FG_eval(Eigen::VectorXd coeffs, double previous_steer, double previous_throttle) {
      this->coeffs = coeffs;
      this->deriv_coeffs = poly_derivative(coeffs);
      this->previous_steer = previous_steer;
      this->previous_throttle = previous_throttle;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
      // First element of fg is cost.

      // Remaining elements of fg represent errors in physics.
      // For each aspect of physics that I model, I need to
      // create an expression that is expected to be equal to 0.

      // vars contains all state and actuations.
      // vars is automatically modified in order to
      // satisfy the physics constraints in fg
      // as well as to minimize the cost.

      // Start cost at 0.0 and add cost aspects later.
      fg[0] = 0.0;

      // Cost of deviating from the intended position, orientation, and speed
      for (int t = 0; t < N; t++) {
          fg[0] += direction_weight * CppAD::pow(vars[epsi_start + t], 2);
          fg[0] += speed_weight * CppAD::pow(vars[v_start + t] - ref_speed, 2);
          fg[0] += road_center_weight * CppAD::pow(CppAD::abs(vars[cte_start + t]), 2);
          fg[0] += on_road_weight * CppAD::pow(CppAD::abs(vars[cte_start + t]), 4);
      }

      // Faster is better
      //for (int t = 0; t < N-1; t++) {
      //    fg[0] -= vars[v_start + t];
      //    fg[0] += CppAD::pow(vars[a_start + t] - 1.0, 2);
      //}

      // Minimize the use of actuators.
      for (int t = 0; t < N - 1; t++) {
          fg[0] += minimal_steer_weight * CppAD::pow(vars[delta_start + t], 2);
          fg[0] += minimal_acceleration_weight * CppAD::pow(vars[a_start + t], 2);
      }

      // Minimize change in actuators (jerkiness).
      for (int t = 0; t < N - 2; t++) {
          fg[0] += steer_jerk_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
          fg[0] += acceleration_jerk_weight * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
      }
      fg[0] += steer_jerk_weight * CppAD::pow(vars[delta_start] - previous_steer, 2);
      fg[0] += acceleration_jerk_weight * CppAD::pow(vars[a_start] - previous_throttle, 2);

      // Initial values can't be changed.
      fg[1 + x_start] = vars[x_start];
      fg[1 + y_start] = vars[y_start];
      fg[1 + psi_start] = vars[psi_start];
      fg[1 + v_start] = vars[v_start];
      fg[1 + cte_start] = vars[cte_start];
      fg[1 + epsi_start] = vars[epsi_start];

      // Apply physics
      for (int t = 1; t < N; t++) {
          // The state at time t+1 .
          AD<double> x1 = vars[x_start + t];
          AD<double> y1 = vars[y_start + t];
          AD<double> psi1 = vars[psi_start + t];
          AD<double> v1 = vars[v_start + t];
          AD<double> cte1 = vars[cte_start + t];
          AD<double> epsi1 = vars[epsi_start + t];

          // The state at time t.
          AD<double> x0 = vars[x_start + t - 1];
          AD<double> y0 = vars[y_start + t - 1];
          AD<double> psi0 = vars[psi_start + t - 1];
          AD<double> v0 = vars[v_start + t - 1];
          AD<double> cte0 = vars[cte_start + t - 1];
          AD<double> epsi0 = vars[epsi_start + t - 1];

          // Only consider the actuation at time t.
          AD<double> delta0 = vars[delta_start + t - 1];
          AD<double> a0 = vars[a_start + t - 1];

          AD<double> f0 = poly_eval(coeffs,x1);
          AD<double> psides0 = CppAD::atan(poly_eval(deriv_coeffs,x1));

          // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt
          // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
          // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

          fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
          fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
          fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
          fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
          fg[1 + cte_start + t] =
                  cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
          fg[1 + epsi_start + t] =
                  epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
      }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
    this->plan_x.clear();
    this->plan_y.clear();
    for(int i = 0; i < N; i++) {
        this->plan_x.push_back(0.0);
        this->plan_y.push_back(0.0);
    }
    this->previous_steer = 0.0;
    this->previous_throttle = 0.1;
}

MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double actuation_delay) {
    typedef CPPAD_TESTVECTOR(double) Dvector;

    time_t start_time = clock();

    double px0 = state[0];
    double py0 = state[1];
    double psi0 = state[2];
    double v0 = state[3];

    Eigen::VectorXd deriv_coeffs = poly_derivative(coeffs);

    double px = px0 + v0 * actuation_delay * cos(psi0);
    double py = py0 + v0 * actuation_delay * sin(psi0);
    double psi = psi0 + previous_steer * v0 * actuation_delay / Lf;
    double v = v0 + previous_throttle * actuation_delay;
    double cte = py - poly_eval(coeffs,px);
    double epsi = psi - atan(poly_eval(deriv_coeffs,px));

    // N copies of each state variable
    // N-1 copies of each actuation (involve transitions between states)
    size_t n_vars = N * 6 + (N-1) * 2;
    // Constraints are just the parts that are not chosen, so actuations not included
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }
    vars[x_start] = px;
    vars[y_start] = py;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    // Lower and upper bounds for all variables
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // State variables will be precisely controlled by physics,
    // so no need to place upper and lower bounds.
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // Steering angle restricted to range of (-25 degrees, 25 degrees) translated to radians.
    for (int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Throttle range is -1 (full brakes) to 1 (max acceleration)
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Set lower and upper limits for state variables.
    // Use 0 in most cases to indicate "don't know"
    // and expect these to be overridden with physics
    // calculations in FG_eval.

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = px;
    constraints_lowerbound[y_start] = py;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = px;
    constraints_upperbound[y_start] = py;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs,previous_steer,previous_throttle);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // Limit 0.5 seconds to calculate solution.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    // Check if solver finished.
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;

    if(!ok) {
        cout << "Optimization failed!" << endl;
    }

    time_t end_time = clock();

    double time_elapsed = (end_time - start_time) * 1.0 / CLOCKS_PER_SEC;

    if(time_elapsed > 0.05) {
        cout << "Optimization took " << time_elapsed << " seconds." << endl;
    }

    for(int i = 0; i < N; i++) {
        plan_x[i] = solution.x[x_start+i];
        plan_y[i] = solution.x[y_start+i];
    }

    previous_steer = solution.x[delta_start];
    previous_throttle = solution.x[a_start];

    return {previous_steer,previous_throttle};
}
