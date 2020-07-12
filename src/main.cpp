#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

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

int main() {
    uWS::Hub h;
    
    PID pid;
    /**
     * TODO: Initialize the pid variable.
     */
    double p[3] = {0.06,0.0001,1.5}; // initial PID reasonable parameters
    double dp[3] = {0.01,0.0001,0.1}; // initial twiddle parameters
    int n = 0; // twiddle run numbers
    bool twiddle = true; // flag for twiddle
    double tol = 0.01; // twiddle tolerance
    double err = 0.; // twiddle error
    double best_err = 10000.; // twiddle best error
    int it = 0; // iterator for twiddle index
    bool new_run = true; // flag for new simulation cycle
    bool minus = true; // flag for twiddle deduction range
    
    h.onMessage([&pid, &p, &dp, &n, &twiddle, &tol, &err, &best_err, &it, &new_run, &minus](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
                    // double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value;
                    double throttle = 0.3;
                    /**
                     * TODO: Calculate steering value here, remember the steering value is [-1, 1].
                     */
                    if (twiddle) // Do twiddle
                    {
                        if (n==0)
                            pid.Init(p[0], p[1], p[2]);
                        
                        pid.UpdateError(cte);
                        steer_value = pid.TotalError();
                        // Adjus throttle based on speed and steering angle
                        // High speed at big steering angle may lead the car to drive off track
                        if ( (fabs(steer_value) > 0.1) && (speed > 15) )
                            throttle = -0.3;
                        
                        // If the car goes off track, reset the simulator
                        if (fabs(cte) >= 4.0){
                            std::string msg = "42[\"reset\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                        
                        err += pow(cte, 2);
                        n += 1;
                        if (n == 50){
                            err = err / n;
                            
                            // If this is a new simulation cycle, twiddle p for the first time
                            if (new_run){
                                p[it] += dp[it];
                                std::cout << "new run, p[" << it << "] is " << p[it] << std::endl;
                                new_run = false;
                            }
                            else{
                                if (err < best_err){
                                    best_err = err;
                                    dp[it] *= 1.1;
                                    std::cout << "find the new best error " << best_err << ", p[" << it << "] is " << p[it] << std::endl;
                                    it = (it + 1) % 3;
                                    new_run = true;
                                    minus = true;
                                }
                                else{
                                    if (minus){
                                        p[it] -= 2*dp[it];
                                        minus = false;
                                        std::cout << "try the minus range, p[" << it << "] is " << p[it] << std::endl;
                                    }
                                    else{
                                        p[it] += dp[it];
                                        dp[it] *= 0.9;
                                        std::cout << "minus range tried and failed, p[" << it << "] is " << p[it] << std::endl;
                                        it = (it + 1) % 3;
                                        new_run = true;
                                        minus = true;
                                    }
                                }
                            }
                            
                            // One cycle done, reset twiddle and simulator
                            err = 0.;
                            n = 0;
                            // Simulator has to be reset, otherwise it does not take in new data
                            //               std::string msg = "42[\"reset\",{}]";
                            //               ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                        // Still running, pass steering angle and throttle to the simulator
                        json msgJson;
                        msgJson["steering_angle"] = steer_value;
                        msgJson["throttle"] = throttle;
                        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        
                        if (dp[0] + dp[1] + dp[2] < tol){
                            twiddle = false;
                            pid.PrintParam();
                            ws.close();
                        }
                    }
                    else // No twiddle, normal drive mode
                    {
                        pid.Init(p[0], p[1], p[2]);
                        pid.UpdateError(cte);
                        steer_value = pid.TotalError();
                        // Adjus throttle based on speed and steering angle
                        // High speed at big steering angle may lead the car to drive off track
                        if ( (fabs(steer_value) > 0.1) && (speed > 15) )
                            throttle = -0.3;
                        
                        json msgJson;
                        msgJson["steering_angle"] = steer_value;
                        msgJson["throttle"] = throttle;
                        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                    // END TODO
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