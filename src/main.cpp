#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

Eigen::VectorXd vector_std_to_eigen(vector<double> vec) {
  Eigen::VectorXd result(vec.size());
  for(int i = 0; i < vec.size(); i++) {
    result(i) = vec[i];
  }
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          cout << endl << "x:" << px << " y:" << py << " psi:" << psi << " speed:" << v << endl;
          cout << "ptsx size:" << ptsx.size() << " first:" << ptsx[0] << " last:" << ptsx[ptsx.size()-1]  << endl;
          cout << "ptsy size:" << ptsy.size() << " first:" << ptsy[0] << " last:" << ptsy[ptsy.size()-1]  << endl << endl;


          json msgJson;

          // Convert waypoints ptsx,ptsy to vehicle coordinate system. Simulator displays as yellow line.
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          next_x_vals.clear();
          next_y_vals.clear();
          cout << "Waypoints:" << endl;
          for(int i = 0; i < ptsx.size(); i++) {
            double distance = sqrt(pow(ptsx[i]-px,2) + pow(ptsy[i]-py,2));
            double direction_abs = atan2(ptsy[i]-py,ptsx[i]-px);
            double direction_rel = direction_abs - psi;
            double x = distance * cos(direction_rel);
            double y = distance * sin(direction_rel);
            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
            cout << "  dist:" << distance << " dir_abs:" << direction_abs << " psi:" << psi << " dir_rel:" << direction_rel << endl;
          }
          cout << endl;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          // Fit polynomial to waypoints
          Eigen::VectorXd poly = polyfit(vector_std_to_eigen(next_x_vals),vector_std_to_eigen(next_y_vals),4);
          cout << "Polynomial:" << endl << poly << endl << endl;

          // Represent current state: px, py, psi, v, cte, epsi (should also have steer and throttle)
          Eigen::VectorXd state(6);
          state << 0.0, 0.0, 0.0, v, - poly[0], - atan2(poly[1],1.0);

          // Invoke MPC solver
          vector<double> actuations = mpc.Solve(state,poly);

          // Transmit solution: steering (-1,1), throttle (-1,1), and plan (mpc_x,y_vals, green line)

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].

          double steer = actuations[0] / deg2rad(25);
          if(steer > 1.0) steer = 1.0;
          if(steer < -1.0) steer = -1.0;
          double throttle = actuations[1];
          msgJson["steering_angle"] = - steer;
          msgJson["throttle"] = throttle;

          // Transmit MPC predicted trajectory (displayed as green line)
          msgJson["mpc_x"] = mpc.plan_x;
          msgJson["mpc_y"] = mpc.plan_y;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          // this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
