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

using nlohmann::json;
using std::string;
using std::vector;
using std::unique_ptr;
using std::cout;

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

  const int NUM_TRAJECTORY_POINTS = 50;
  double ref_speed = 0.;  // in MPH
  int car_lane = 1;

  h.onMessage([&ref_speed, &car_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data: a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Next set of waypoints to output.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int prev_path_size = previous_path_x.size();
          if (prev_path_size > 0) {
            car_s = end_path_s;
          }
          // Push previous path's coordinates for continuity.
          for (int i=0; i < prev_path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //// Prediction: Loop through the detected vehicles, make predictions as to where they'd be by end of trajectory time.
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;
          for (int i=0; i < sensor_fusion.size(); ++i) {
            Vehicle *v = initDetectedVehicle(
                sensor_fusion[i][3],
                sensor_fusion[i][4],
                sensor_fusion[i][5],
                sensor_fusion[i][6],
                sensor_fusion[i][0]);

            // Get vehicle lane.
            int vehicle_lane = v->getVehicleLane();
            if (vehicle_lane < 0)
              continue;

            // Find projected s-coordinate of the vehicle from last planned trajectory.
            double vehicle_proj_s = v->getProjectedSCoordinate(prev_path_size);

            // Check whether the detected vehicle is ahead of us, or in left/right lane.
            if (vehicle_lane == car_lane) {  // Vehicle is ahead of us.
              car_ahead |= vehicle_proj_s > car_s && vehicle_proj_s - car_s < SAFE_FRENET_S_DISTANCE;
            } else if (vehicle_lane - car_lane == 1) {  // Vehicle to our right.
              car_right |= isNeighbourLaneVehicleNearby(car_s, vehicle_proj_s);
            } else if (vehicle_lane - car_lane == -1) {  // Vehicle to our left.
              car_left |= isNeighbourLaneVehicleNearby(car_s, vehicle_proj_s);
            }
          }

          //// Behaviour planning : Pick a safe and legal maneuver.
          double car_speed_diff = 0.;
          if (car_ahead) {
            if (!car_left && car_lane > 0) {  // Lane Change Left.
              --car_lane;
            }
            else if (!car_right && car_lane != 2) {  // Lane Change Right.
              ++car_lane;
            }
            else {  // Keep Lane, Reduce Speed.
              car_speed_diff = -MAXIMUM_ACCELERATION;
            }
          }
          else {
            if (car_lane != 1) {
              if ((car_lane == 0 && !car_right) || (car_lane == 2 && !car_left)) {  // Lane Change Centre.
                car_lane = 1;
              }
            }
            if (ref_speed < MAXIMUM_SPEED) {
              car_speed_diff = MAXIMUM_ACCELERATION;
            }
          }

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          struct Waypoints prev_waypoints = {previous_path_x, previous_path_y};

          unique_ptr<struct Waypoints> wps(getContinuingWaypoints(prev_waypoints, ref_x, ref_y, ref_yaw));
          vector<double> pts_x = wps->waypoints_x;
          vector<double> pts_y = wps->waypoints_y;

          // Set target maneuver waypoints.
          vector<double> target_wp1 = getXY(
              car_s+SAFE_FRENET_S_DISTANCE, 2+4*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> target_wp2 = getXY(
              car_s+SAFE_FRENET_S_DISTANCE*2, 2+4*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> target_wp3 = getXY(
              car_s+SAFE_FRENET_S_DISTANCE*3, 2+4*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.push_back(target_wp1[0]);
          pts_x.push_back(target_wp2[0]);
          pts_x.push_back(target_wp3[0]);
          pts_y.push_back(target_wp1[1]);
          pts_y.push_back(target_wp2[1]);
          pts_y.push_back(target_wp3[1]);

          //// Generate trajectory to achieve the target maneuver.
          // Convert waypoint coordinates to car local coordinates.
          globalToLocalCoordinateTransform(pts_x, pts_y, ref_yaw);
          tk::spline s;
          s.set_points(pts_x, pts_y);

          // Setting trajectory distance to 30m forward.
          double target_x = 30.;
          double target_y = s(target_x);
          double target_distance = get2DVectorMagnitude(target_x, target_y);

          double next_x = 0.;

          for(int i=1; i < NUM_TRAJECTORY_POINTS-prev_path_size; ++i) {
            ref_speed += car_speed_diff;
            clipCarSpeed(ref_speed);

            double speed_alpha = target_distance / (CONTROLLER_WAYPOINT_INTERVAL * ref_speed / MPH_TO_METRE_PER_SEC);
            double x_coord = next_x + target_x / speed_alpha;
            double y_coord = s(x_coord);

            next_x = x_coord;
            vector<double> global_coord = localToGlobalCoordinateTransform(x_coord, y_coord, ref_yaw);
            x_coord = ref_x + global_coord[0];
            y_coord = ref_y + global_coord[1];

            next_x_vals.push_back(x_coord);
            next_y_vals.push_back(y_coord);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
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