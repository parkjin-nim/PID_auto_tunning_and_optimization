#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <vector>
#include <numeric>

// for convenience
using nlohmann::json;
using std::string;
using namespace std;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}
//parameter optimization


int main() {
  uWS::Hub h;

  PID pid;
    bool init = true;

    //pid.Init(0.3083945237000776, 3.73066396507075, 0.009287418277686622);
    pid.Init(0.15910442248556678, 1.587110426670305, 0.005533208544239475);

    h.onMessage([&pid, &init](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value = .0;
          double throttle_value = 0.;
          
          if(init){
            pid.prev_error = cte;
            pid.prev_terror = pid.TotalError();
            init = false;
          }
            
          pid.UpdateError(cte);
            
          //Steering value is  [-1, 1].
          steer_value = pid.GetSteer();
            


          //Throttle value is  [0, 1].
          throttle_value = 0.05*(30-speed) - 0.5*fabs(steer_value) + 0.1;

            
          if(steer_value > 1){
            steer_value = 1;
          }
          else if(steer_value < -1){
            steer_value = -1;
          }
            
          if(throttle_value > 1){
            throttle_value = 1;
          }
          else if(throttle_value < 0){
            throttle_value = 0;
          }
            
          // DEBUG
          std::cout << " CTE: " << cte << " Steering Value: " << steer_value <<" Thrtl: "<< throttle_value<<" terr:" <<pid.prev_terror - pid.TotalError() << std::endl;

            
            
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
            
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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
