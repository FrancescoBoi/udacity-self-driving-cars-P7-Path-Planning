#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  /* Define lane and reference velocity */
  int lane = 1;
  double ref_vel = 0.; //mph
  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::cout<<"=======================================\n";
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.

          auto sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;
          size_t prev_size = previous_path_x.size();
          if (prev_size>0)
          {
              car_s = end_path_s;
          }
          bool too_close = false;
          bool safe_left_change = true;
          bool safe_right_change = true;
          static int propose_lane = lane;
          //let's discard the first meters when the car is starting:
          //if a different lane was proposed let's consider that for some time:
          for (size_t i=0; i<sensor_fusion.size(); ++i)
          {
              float d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3], vy=sensor_fusion[i][4];
              double check_speed = sqrt(pow(vx,2)+pow(vy,2));
              double check_car_s = sensor_fusion[i][5];
              //check car's s position forward in time
              check_car_s += static_cast<double>(prev_size)*0.02*check_speed;
              // Check if the car in the current lane
              if ((d<2+4*lane+2) && (d>2+4*lane-2)) //car in the same lane
              {
                  if((check_car_s>car_s)&&(check_car_s-car_s<30.))
                  {
                      //slow down
                      too_close = true;
                      propose_lane = std::max(lane-1, 0);
                  }
                  else if((check_car_s<car_s)&&(car_s-check_car_s<25.)&&
                    (propose_lane==lane))//if we have decided to overtake,
                    //we stick to that decision, otherwise we move to the right
                    // lane because there's a faster car behind us
                  {
                    propose_lane = std::min(lane+1, 2);
                  }
              }
              else if (car_s<200)
              {
                  safe_left_change = false;
                  safe_right_change = false;
              }
              else if((d<2+4*(lane-1)+2) && (d>2+4*(lane-1)-2)) //car in the left lane
              {
                  //if there is a car ahead in the left lane with slower speed
                  if ((check_car_s>car_s)&& (check_car_s-car_s<40.*ref_vel/car_speed) && (check_speed<car_speed))
                  {
                    safe_left_change = false;
                  }
                  // if the car from behind is coming with higher speed
                  else if ((check_car_s<car_s)&&(car_s-check_car_s<40.*ref_vel/car_speed))
                  {
                    safe_left_change = false;
                  }
              }
              //Consider a lane change to the right
              else if ((d<2+4*(lane+1)+2) && (d>2+4*(lane+1)-2)) //car in the right lane
              {
                  //if the car is ahead with slower speed than ours
                  if ((check_car_s>car_s)&&(check_car_s-car_s<40.*ref_vel/car_speed))
                  {
                    safe_right_change = false;;
                  }
                  // if the car from behind is coming with higher speed
                  else if ((check_car_s<car_s)&&(car_s-check_car_s<40.*ref_vel/car_speed))
                  {
                    safe_right_change = false;
                  }
              }
          }
          // 0.224 ~ 5 m/s2 (max accelaration allowed)
          if ((safe_right_change && (propose_lane==lane+1)) ||
             (safe_left_change && (propose_lane==lane-1)))
          {
              lane = propose_lane;
          }
          if (too_close)
          {
              ref_vel -= 0.2;
          }
          else
          {
              ref_vel = fmin(ref_vel+0.5, 49.9);
          }


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
           vector<double> next_x_vals;
           vector<double> next_y_vals;
           vector<double> ptsx, ptsy;
           double ref_x=car_x, ref_y=car_y, ref_yaw=deg2rad(car_yaw);
           /* If prev_size is (almost) empty (all waypoints consumed), use the cars
           as starting reference*/
           if (prev_size<2)
           {
               //use 2 points that make the path tangent to the car
               std::cout<<"No prev points\n";
               double prev_car_x = car_x - cos(ref_yaw);
               double prev_car_y = car_y - sin(ref_yaw);
               ptsx.push_back(prev_car_x);
               ptsx.push_back(car_x);
               ptsy.push_back(prev_car_y);
               ptsy.push_back(car_y);
           }
           else //use the remaining points of the prev path
           {
               ref_x = previous_path_x[prev_size-1];
               ref_y = previous_path_y[prev_size-1];
               double prev_ref_x = previous_path_x[prev_size-2];
               double prev_ref_y = previous_path_y[prev_size-2];
               ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);
               ptsx.push_back(prev_ref_x);
               ptsx.push_back(ref_x);
               ptsy.push_back(prev_ref_y);
               ptsy.push_back(ref_y);
           }
           //add evenly 30m spaced WP in Frenet coordinates
           for (size_t i=1; i<4; ++i)
           {
               vector<double> next_wp = getXY(car_s+30.*i, 2+4*lane, map_waypoints_s,
                   map_waypoints_x, map_waypoints_y);
               ptsx.push_back(next_wp[0]);
               ptsy.push_back(next_wp[1]);
           }
           //so far we have 5 points
           //Convert the points using car reference frame (i.e. car heading = 0)
           for (size_t i=0; i<ptsx.size(); ++i)
           {
               double shift_x = ptsx[i] - ref_x;
               double shift_y = ptsy[i] - ref_y;
               ptsx[i] = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
               ptsy[i] = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
           }
           //use previous points to help with transitions
           for (size_t i=0; i<prev_size; ++i)
           {
               next_x_vals.push_back(previous_path_x[i]);
               next_y_vals.push_back(previous_path_y[i]);
           }
           //create a spline
           tk::spline s;
           s.set_points(ptsx, ptsy);
           //break up the spline to travel at the reference velocity
           //target point: 30m
           double target_x = 30.;
           double target_y = s(target_x);
           double target_dist = sqrt(pow(target_x, 2)+ pow(target_y, 2));
           double x_add_on = 0.; // position of the car w.r.t. car coordinates
           double N = target_dist/(0.02*ref_vel/2.24);
           for (size_t i=0; i<50-previous_path_x.size(); ++i)
           {
               //we want points separated by 0.02 m;
               // 2.24->conversion from mph to meters/s
               double x_point = x_add_on+target_x/N;
               double y_point = s(x_point);
               x_add_on = x_point;
               double x_ref=x_point, y_ref=y_point;
               // convert to global coordinates again
               x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw) ;
               y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
               x_point += ref_x;
               y_point += ref_y;
               next_x_vals.push_back(x_point);
               next_y_vals.push_back(y_point);
           }
          //END
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
