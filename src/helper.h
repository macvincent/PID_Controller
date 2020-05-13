#include <math.h>
#include <uWS/uWS.h>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <thread>
#include <mutex>
#include <condition_variable>


uWS::Hub h;
std::mutex mtx;
std::condition_variable cv;
PID pid;
bool updated = false;
bool reset = false;
double error = 0.0f;
// for convenience
using nlohmann::json;
using std::string;

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

void run_simlation(uWS::Hub& h, PID& pid){
    int count = 0;
// pid.Init(0.06, 0.000001, 1.5); //Initial values
pid.Init(0.285395, 0.0028573, 3.14895 ); //Values from twiddle algorithm
h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
        // "42" at the sStart of the message means there's a websocket message event.
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
            if(reset){
                reset = false;
                count = 0;
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            }
            if(count > 50){
                error += pow(cte,2);
            }
            count++;
            if(count == 1500){
                std::unique_lock<std::mutex> lck(mtx);
                error /= 1500;
                updated = true;
                cv.notify_all();
            }
            pid.UpdateError(cte);
            double steer_value = pid.TotalError();
            if(steer_value > 1)steer_value = 1;
            else if(steer_value < -1)steer_value = -1;
            // DEBUG
            // std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
            //             << std::endl;

            json msgJson;

            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.25;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
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
        // std::cout << "Connected!!!" << std::endl;
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
    } 
    h.run();
}

void twiddle(){
  std::unique_lock<std::mutex> lck(mtx);
  while (!updated)cv.wait(lck);
  lck.unlock();
  double best_error = error;
  error = 0;
  reset = true;
  updated = false;

  std::vector<double> pi = {pid.Kp, pid.Kd, pid.Ki};
  std::vector<double> dp = {0.05,0.5,0.0005};  
  int count = 0;
  while(true){
      if(count == 5)break;
        count++;
      std::cout << count << " " << best_error<< std::endl;
      std::cout << pi[0] << ' ' << pi[1] << ' ' << pi[2] << std::endl;
      std::cout << dp[0] << ' ' << dp[1] << ' ' << dp[2] << std::endl;
      for(int i = 0; i < 3; i++){
        // select errors and parameters to be updated
        pi[i] += dp[i];

        pid.Set(pi, dp);
        std::unique_lock<std::mutex> lck2(mtx);
        while (!updated)cv.wait(lck2);
        lck2.unlock();
        double temp_error = error;
        error = 0;
        reset = true;
        updated = false;

        if(temp_error < best_error){
          best_error = temp_error;
          dp[i]  *= 1.1;
        }else{
          pi[i] -= 2*dp[i];
          pid.Set(pi, dp);

          std::unique_lock<std::mutex> lck3(mtx);
          while (!updated)cv.wait(lck3);
          lck3.unlock();
          temp_error = error;
          updated = false;
          error = 0;
          reset = true;

          if(temp_error < best_error){
            best_error = temp_error;
            dp[i] *= 1.1;
          }else{
            pi[i] += dp[i];
            dp[i] *= 0.9;
          }
        }
      }
    std::cout << pi[0] << ' ' << pi[1] << ' ' << pi[2] << std::endl;
    std::cout << dp[0] << ' ' << dp[1] << ' ' << dp[2] << std::endl;
  }
}