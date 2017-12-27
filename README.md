# CarND MPC Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Self-Driving Car Engineer Nanodegree Program

---

The car in the following video uses Model Predictive Control (MPC) to drive at 80 MPH around a closed track.
This involves using a real physics model, combined with minimizing a cost function. For added challenge,
all steering commands are delayed by 1/10 of a second so that the model needs to predict the future
situation and determine what command would be helpful in that situation.

[![project video](https://github.com/ericlavigne/CarND-MPC-Control/raw/master/recording-thumb.png)](https://youtu.be/76V_7VvCeFM)

*Note: Find the latest version of this project on
[Github](https://github.com/ericlavigne/CarND-MPC-Control).*

## Relationship to Other Components

The controller is responsible for low-level control of the vehicle, such as steering, accelerating,
and braking. The controller receives its own location from the localization component and receives speed and
a list of waypoints from the planning component. The controller must determine an appropriate series
of actions to balance the competing requirements to stay close to the planned waypoints, maintain
the desired speed, and ensure a smooth ride.

## Plotting a Course

The controller converts waypoints into a coordinate system relative to the car's location and orientation,
then reinterprets those waypoints as a best-fit polynomial. The continuity and smoothness of a polynomial
makes it much more suitable for estimating positional and directional error. The positional error, also known
as Cross-Track Error (CTE), is the distance from the car's location to the nearest point on the polynomial
plan. The directional error is the difference in direction between the car and the nearest point on the
polynomial plan.

## Physics-Based Prediction

This MPC uses a simple physics model based on X state variables (x, y, speed, direction) and
two actuation variables (steering and acceleration). There are also two pseudo-state variables
CTE and EPSI, representing the error in position and direction, respectively.

```
        x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
        y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
        psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
        v[t+1] = v[t] + a[t] * dt
        cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
        epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

The MPC predicts the next 0.5 seconds, using 10 timesteps of 0.05 seconds each. This represents a
tradeoff between calculation time, calculation accuracy, and how far in advance the MPC can predict problems.
Increasing the number of timesteps or the size of timesteps increases calculation time. Increasing the horizon
from 0.5 seconds to 1.0 seconds, either with 10 timesteps of 0.1 seconds or 20 timesteps of 0.05 seconds,
increases the calculation time to more than 0.5 seconds, which is not affordable in a realtime context. Smaller
horizons tend to cause oversteering because the MPC can't predict that turning toward the center now will cause
an overshoot later (after the horizon).

I think that increasing the speed from 80 MPH to 100 MPH would require a prediction horizon of about 3 seconds,
so that the MPC can see how countersteering on the first turn sets the car up for a more successful second turn.
Unfortunately, 3 seconds seems to be far more than my computer can handle.

## Compensating for Delay

All commands (steering and acceleration) are delayed by 1/10 of a second for an added challenge.
This MPC compensates by predicting how the previous commands will affect the vehicle's state over
the next 1/10 of a second and using that predicted future state as the starting state for the rest of
the MPC calculation process.

## Cost Optimization

The MPC uses a cost function to balance the competing requirements to stay close to the
planned waypoints, maintain the desired speed, and ensure a smooth ride.

* Distance from center
  * Prefer to be in the center of the road.
  * Cost contribution is distance to the 4th power. This creates a small cost
    for moving towards the edge of the road and a large cost for going off-road.
* Orientation
  * Prefer orientation in the direction of the road.
* Speed
  * Prefer to drive 30 MPH.
* Minimize controls
  * Prefer to turn the steering wheel as little as possible.
  * Prefer to brake and accelerate as little as possible.
* Minimize change in controls (jerkiness)
  * Prefer steering angle to be close to the steering angle of the previous timestep.
  * Prefer acceleration to be close to the acceleration of the previous timestep.
* Constraints
  * Steering angle limited to -25 degrees to 25 degrees.
  * Acceleration limited to -1 to +1.

Ipopt and CppAD selects a series of steering and throttle values to minimize the cost
function. The MPC selects only the first pair of steering and throttle values to
control the vehicle.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
