#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "vehicle.h"

using std::string;
using std::vector;
using std::array;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate magnitude of a 2D vector.
double get2DVectorMagnitude(double a, double b) {
  return sqrt(a*a + b*b);
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


Vehicle* initDetectedVehicle(double vx, double vy, double s, double d, int id) {
  return new Vehicle(vx, vy, s, d, id);
}

bool isNeighbourLaneVehicleNearby(double car_s, double neighbour_proj_s) {
  return
  (car_s - SAFE_FRENET_S_DISTANCE < neighbour_proj_s) &&
  (car_s + SAFE_FRENET_S_DISTANCE > neighbour_proj_s);
}

struct Waypoints* getContinuingWaypoints(
    const struct Waypoints& prev_wps, double& ref_x, double& ref_y, double& yaw) {

  Waypoints* wps = new Waypoints();
  vector<double> prev_path_x = prev_wps.waypoints_x;
  vector<double> prev_path_y = prev_wps.waypoints_y;
  int prev_path_size = prev_path_x.size();

  if (prev_path_size < 2) {
    double prev_car_x = ref_x - cos(yaw);
    double prev_car_y = ref_y - sin(yaw);

    wps->waypoints_x.push_back(prev_car_x);
    wps->waypoints_x.push_back(ref_x);
    wps->waypoints_y.push_back(prev_car_y);
    wps->waypoints_y.push_back(ref_y);
  }
  else {
    ref_x = prev_path_x[prev_path_size-1];
    ref_y = prev_path_y[prev_path_size-1];
    double prev_ref_x = prev_path_x[prev_path_size-2];
    double prev_ref_y = prev_path_y[prev_path_size-2];
    yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

    wps->waypoints_x.push_back(prev_ref_x);
    wps->waypoints_x.push_back(ref_x);
    wps->waypoints_y.push_back(prev_ref_y);
    wps->waypoints_y.push_back(ref_y);
  }
  return wps;
}


void globalToLocalCoordinateTransform(vector<double>& pts_x, vector<double>& pts_y, double yaw) {
  assert(pts_x.size() == pts_y.size());
  double start_x = pts_x[0];
  double start_y = pts_y[0];
  for (int i=0; i < pts_x.size(); ++i) {
    double dx = pts_x[i] - start_x;
    double dy = pts_y[i] - start_y;
    pts_x[i] = dx * cos(-yaw) - dy * sin(-yaw);
    pts_y[i] = dx * sin(-yaw) + dy * cos(-yaw);
  }
}

vector<double> localToGlobalCoordinateTransform(const double& x, const double& y, double yaw) {
  double x1 = x * cos(yaw) - y * sin(yaw);
  double y1 = x * sin(yaw) + y * cos(yaw);
  return vector<double>{x1, y1};
}

double clipCarSpeed(const double& inputSpeed) {
  if (inputSpeed > MAXIMUM_SPEED) {
    return MAXIMUM_SPEED;
  }
  else if (inputSpeed < MAXIMUM_ACCELERATION) {
    return MAXIMUM_ACCELERATION;
  }
  else return inputSpeed;
}


#endif  // HELPERS_H