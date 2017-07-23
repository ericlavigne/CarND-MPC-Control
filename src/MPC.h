#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
    public:
    MPC();

    virtual ~MPC();

    vector<double> plan_x;
    vector<double> plan_y;
    double previous_steer;
    double previous_throttle;

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double actuation_delay);
};

#endif /* MPC_H */
