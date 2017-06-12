#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"

#define StandardPIDImplementation 1

// for convenience
using json = nlohmann::json;

// used to calculate delta t
std::clock_t t_start, t_end;

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
// int main(int argc, char *argv[]) {
    // ...
    //double init_Kp = atof(argv[1]);
    //double init_Ki = atof(argv[2]);
    //double init_Kd = atof(argv[3]);

    uWS::Hub h;

    PID pid;

    // manually tuned parameters
    double init_Kp = 0.3;
    double init_Ki = 0.000228;
    double init_Kd = 0.000255;

    pid.Init(init_Kp, init_Ki, init_Kd);

    h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            t_end = std::clock();

            // calculate delta t
            double dt = double(t_end - t_start) / CLOCKS_PER_SEC;

#if StandardPIDImplementation == 1
            std::cout << "elapsed time : " << dt << std::endl;
#endif

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
                    /*
                    * TODO: Calcuate steering value here, remember the steering value is
                    * [-1, 1].
                    * NOTE: Feel free to play around with the throttle and speed. Maybe use
                    * another PID controller to control the speed!
                    */

                    // calculate differential error
                    pid.d_error = (cte - pid.d_error) / dt;
                    // calculate integral error
                    pid.i_error += cte;
                    // calculate steering value
                    steer_value = (-pid.Kp * cte) - (pid.Kd * pid.d_error) - (pid.Ki * pid.i_error);
                    // ... OR ...
                    // From "Self-Driving Car Project Q&A | PID Controller" video
                    // https://www.youtube.com/watch?v=YamBuzDjrs8&feature=youtu.be
                    // pid.UpdateError(cte);
                    //steer_value = pid.TotalError();

#if StandardPIDImplementation == 1
                    std::cout << "angle : " << angle << " steer_value degrees : " << rad2deg(steer_value)
                              << " final steering angle : " << steer_value << std::endl;
#endif

                // store last cte
                pid.d_error = cte;
                // normalize steering value
                steer_value = steer_value > 1.0 ? 1.0 : steer_value;
                steer_value = steer_value < -1.0 ? -1.0 : steer_value;

#if StandardPIDImplementation == 1
                // DEBUG
                std::cout << "CTE : " << cte << " steering value : " << steer_value << " speed : " << speed
                        << std::endl;
#endif
                    // send values to simulator
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = 0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
#if StandardPIDImplementation == 1
                    std::cout << msg << std::endl;
#endif
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }

            t_start = std::clock();

        }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        t_start = std::clock();

        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
