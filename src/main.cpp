#include "PID.h"
#include <algorithm>
#include "json.hpp"
#include <iostream>
#include <math.h>
#include <uWS/uWS.h>

// for convenience
using json = nlohmann::json;

bool TWIDDLE = false;

enum State {
  START,
  HIGHER,
  LOWER
};

struct Twiddle {
  uint step = 0;
  double best_err = 100000;
  double total_err = 0;
  std::vector<double> dp = {.1, .0001, 1};
  uint param_index = 0;
  State state = State::START;
};

void UpdateTwiddle(Twiddle& twiddle, PID& pid) {
  std::vector<double> p = {pid.Kp, pid.Ki, pid.Kd};
  double current_err = twiddle.total_err; 
  switch(twiddle.state) {
   case State::START:
    p[twiddle.param_index] += twiddle.dp[twiddle.param_index];
    twiddle.state = State::HIGHER;
    break;
   case State::HIGHER:
    if (current_err < twiddle.best_err) {
      twiddle.best_err = current_err;
      twiddle.dp[twiddle.param_index] *= 1.1;
      twiddle.state = State::START;
      twiddle.param_index = (twiddle.param_index + 1) % p.size();
    } else {
      p[twiddle.param_index] -= 2 * twiddle.dp[twiddle.param_index];
      twiddle.state = State::LOWER; 
    }
    break;
   case State::LOWER:
    if (current_err < twiddle.best_err) {
      twiddle.best_err = current_err;
      twiddle.dp[twiddle.param_index] *= 1.1;
    } else {
      p[twiddle.param_index] += twiddle.dp[twiddle.param_index];
      twiddle.dp[twiddle.param_index] *= 0.9;
    }
    twiddle.state = State::START;
    twiddle.param_index = (twiddle.param_index + 1) & p.size();
  }
  pid.Init(std::max(0.0, p[0]),
           std::max(0.0, p[1]),
           std::max(0.0, p[2]));
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  pid.Init(.41 , .000011 , 3.89);
  int total_steps = 0;
  Twiddle twiddle;

  h.onMessage([&pid, &twiddle, &total_steps](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message
    // event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          ++total_steps;
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if (TWIDDLE && total_steps > 1000) {
            if (twiddle.step == 500) {
              UpdateTwiddle(twiddle, pid);
              twiddle.step = 0;
              twiddle.total_err = 0;
            } else {
                twiddle.total_err += std::fabs(cte);
            }
            ++twiddle.step;
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
