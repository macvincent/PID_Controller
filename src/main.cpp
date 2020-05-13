#include <math.h>
#include <uWS/uWS.h>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "helper.h"
#include <thread>
PID pid;
PID pid_throt;

bool twiddle_gains = false;
using nlohmann::json;
using std::string;

void run_simlation();
void twiddle(PID &);

int main() {
  std::vector<std::thread> threads;
  twiddle_gains = false; //toggle for twiddle optimization
  //   pid.Init(0.06, 0.000001, 1.5); //Initial values before twinddle optimization
  pid_throt.Init(1.06, 1e-06, 0.5); //Values from twiddle optimization
  pid.Init(0.285395, 0.0028573, 3.14895 ); //Values from twiddle optimization

  if(twiddle_gains) 
  threads.push_back(std::thread(twiddle, std::ref(pid))); ///modify value of pid argument based on controller to be tuned
  run_simlation();
  threads[0].join();
}

void run_simlation(){
  uWS::Hub h;
  int iterations = 0;
  double desired_speed = 30;
  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                      uWS::OpCode opCode) {
          // "42" at the sStart of the message means there's a websocket message event.
          // The 4 signifies a websocket message
          // The 2 signifies a websocket event
          if (length && length > 2 && data[0] == '4' && data[1] == '2') {
          auto s = hasData(std::string(data).substr(0, length));

          if (s != "") {
              auto j = json::parse(s);

              string event = j[0].get<string>();

              if (event == "telemetry") {
                // j[1] is the data JSON object
                double cte = std::stod(j[1]["cte"].get<string>());
                double speed = std::stod(j[1]["speed"].get<std::string>());
                double speed_error = speed - desired_speed;

                if(twiddle_gains){
                  if(reset){
                      reset = false;
                      iterations = 0;
                      std::string reset_msg = "42[\"reset\",{}]";
                      ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                  }
                  if(iterations > 50){
                      error += pow(cte,2); //modify error based on what is to be tuned
                  }
                  iterations++;
                  if(iterations == 100){
                      error /= 100;
                      updated = true;
                      cv.notify_all();
                  }
                }

                pid.UpdateError(cte);
                pid_throt.UpdateError(speed_error);

                double steer_value = pid.TotalError();
                double throt_value = pid_throt.TotalError();

                if(steer_value > 1)steer_value = 1;
                else if(steer_value < -1)steer_value = -1;

                json msgJson;

                msgJson["steering_angle"] = steer_value;
                msgJson["throttle"] = throt_value;
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
      } 
      h.run();
}

void twiddle(PID& pid){
  std::vector<double> pi = {pid.Kp, pid.Kd, pid.Ki};
  std::vector<double> dp = {1,1,1};
  update_error();
  double best_error = temp_error;
  int count = 0;
  while(true){
      if(count == 9)break;
      count++;
      std::cout << count << " " << best_error<< std::endl;
      std::cout << pi[0] << ' ' << pi[1] << ' ' << pi[2] << std::endl;
      std::cout << dp[0] << ' ' << dp[1] << ' ' << dp[2] << std::endl;
      for(int i = 0; i < 3; i++){
        // select errors and parameters to be updated
        pi[i] += dp[i];

        pid.Set(pi, dp);
        update_error();
        if(temp_error < best_error){
          best_error = temp_error;
          dp[i]  *= 1.1;
        }else{
          pi[i] -= 2*dp[i];
          pid.Set(pi, dp);
          update_error();
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