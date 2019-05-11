#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

double const CONTROLLER_WAYPOINT_INTERVAL = 0.02;  // Time-interval for the controller to achieve successive waypoints.
double const SAFE_FRENET_S_DISTANCE = 30.0;
double const MAXIMUM_SPEED = 49.5;  // in mph.
double const MAXIMUM_ACCELERATION = 0.225;
double const MPH_TO_METRE_PER_SEC = 2.237;

struct Waypoints {
  vector<double> waypoints_x;
  vector<double> waypoints_y;
};

class Vehicle {
 public:
  Vehicle();
  Vehicle(double vx, double vy, double s, double d, int id=-1);
  virtual ~Vehicle() = default;

  enum Lane {
    LEFT_LANE,
    CENTRE_LANE,
    RIGHT_LANE,
    INVALID_LANE = -1
  };

  // Localisation and Kinematics getters.
  int getVehicleLane();
  double getVehicleSpeed();
  double getProjectedSCoordinate(int numPastWaypoints);


 private:
  int id;
  double vx;
  double vy;
  double s;
  double d;
  Lane lane;

  Vehicle::Lane computeVehicleLane(double d_coordinate);
};

#endif //PATH_PLANNING_VEHICLE_H
