#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#include <sysexits.h>
#include <fstream>
#include <sstream>
#include <string>  

// for convenience
using json = nlohmann::json;

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
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char **argv)
{
  uWS::Hub h;

  bool study=false;
  double Kp;
  double Ki;
  double Kd;

  if (argc > 1) {
    study = true;
    Kp = atof(argv[1]);
    Ki = atof(argv[2]);
    Kd = atof(argv[3]);
  } else {
    Kp = 0.12;
    Ki = 0.001;
    Kd = 3.5;
  }

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(study, Kp, Ki, Kd);

  h.onMessage([&study, &pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {

          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          pid.UpdateError(cte);

          steer_value = pid.TotalError();

          if(steer_value < -1.0) {
            steer_value = -1.0;
          }

          if(steer_value > 1.0) {
            steer_value = 1.0;
          }

          if( (pid.timestep>2000 || pid.dead) && study){
            std::cout << "1000 timesteps passed or car in deadlock, next parameter" << std::endl;

            std::ofstream outfile;
            outfile.open("study_output.txt", std::ios_base::app);//std::ios_base::app
            outfile << pid.Kp << "\t" << pid.Ki << "\t" << pid.Kd << "\t" << pid.avg_cte << std::endl;

            // reset the simulator
            std::string reset_msg = "42[\"reset\", {}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

            ws.close(2001);
            return;
          }
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    switch (code) {
      case 2000:
        // The car crashed; let the caller know.
        exit(1);
      case 2001:
        // The simulator ran until our deadline; that's a success.
        exit(EX_OK);
      default:
        // If the simulator exits, we seem to get code 1006 or 0.
        std::cerr << "Disconnected: code=" << code << ":" <<
          std::string(message, length) << std::endl;
        exit(EX_UNAVAILABLE);
    }
});

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
