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
#include <fstream>
#include <iostream>
#include <sstream>

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

void transform_map_coord(vector<double>& xvals,vector<double>& yvals, double vehicle_x, double vehicle_y, double vehicle_theta) {

  vector<double> transformed_x;
  vector<double> transformed_y;
  int total_size = xvals.size();

  for (int i = 0; i < total_size; i++) {

    double new_x;
    double new_y;

    double cos_theta = cos(vehicle_theta - M_PI / 2);
    double sin_theta = sin(vehicle_theta - M_PI / 2);
    new_x = (xvals[i] - vehicle_x) * sin_theta + (yvals[i] - vehicle_y) * cos_theta;
    new_y = -(xvals[i] - vehicle_x) * cos_theta - (yvals[i] - vehicle_y) * sin_theta;

    transformed_x.push_back(new_x);
    transformed_y.push_back(new_y);
  }
  xvals= transformed_x;
  yvals = transformed_y;

  return;
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
void load_waypoints(vector<double>& xvals, vector<double>&  yvals) {

  string in_file_name_ = "../lake_track_waypoints.csv";
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);
  cout << "process file" << in_file_name_ << endl;
  if (!in_file_.is_open()) {
    cerr << "Cannot open input file: " << in_file_name_ << endl;
    exit(EXIT_FAILURE);
  }

  string line;
  bool first_line = true;
  while (getline(in_file_, line)) {
    if (first_line) {
      //pass the first line, which is label x,y
      first_line = false;
      continue;
    }
    istringstream iss(line);
    double x;

    double y;
    iss >> x;
    if (iss.peek() == ',')
      iss.ignore();
    iss >> y;
    xvals.push_back(x);
    yvals.push_back(y);

  }
}

int main() {
  uWS::Hub h;
  vector<double> next_xvals;
  vector<double> next_yvals;
  load_waypoints(next_xvals, next_yvals);
  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc, &next_xvals, &next_yvals](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          auto coeffs = polyfit(Eigen::VectorXd::Map(ptsx.data(), ptsx.size()), Eigen::VectorXd::Map(ptsy.data(), ptsy.size()), 3);
          double cte = polyeval(coeffs, px) - py;
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = -atan(coeffs[1]);
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          if (mpc.x_vals.empty()) mpc.x_vals = {px};
          if (mpc.y_vals.empty()) mpc.y_vals = {py};
          auto vars = mpc.Solve(state, coeffs);
          mpc.x_vals.push_back(vars[0]);
          mpc.y_vals.push_back(vars[1]);

          double steer_value = vars[6];
          double throttle_value = vars[7];

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = mpc.x_vals;
          vector<double> mpc_y_vals= mpc.y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          transform_map_coord(mpc_x_vals,mpc_y_vals, px, py, psi);
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          transform_map_coord(next_xvals,next_yvals, px, py, psi);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_xvals;
          msgJson["next_y"] = next_yvals;


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
          this_thread::sleep_for(chrono::milliseconds(100));
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
