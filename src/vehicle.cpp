#include "vehicle.h"
#include <math.h>

Vehicle::Vehicle(){}

Vehicle::Vehicle(double vx, double vy, double s, double d, int id) {
  this->id = id;
  this->vx = vx;
  this->vy = vy;
  this->s = s;
  this->d = d;
  this->lane = computeVehicleLane(d);
}

Vehicle::Lane Vehicle::computeVehicleLane(double d_coordinate) {
  Lane lane = INVALID_LANE;
  if (d_coordinate >= 0 && d_coordinate < 4) {
    lane = LEFT_LANE;
  }
  else if (d_coordinate >= 4 && d_coordinate < 8) {
    lane = CENTRE_LANE;
  }
  else if (d_coordinate >= 8 && d_coordinate < 12) {
    lane = RIGHT_LANE;
  }
  return lane;
}

int Vehicle::getVehicleLane() {
  return this->lane;
}

double Vehicle::getVehicleSpeed() {
  return sqrt(this->vx*this->vx + this->vy*this->vy);
}

double Vehicle::getProjectedSCoordinate(int numPastWaypoints) {
  return this->s + (numPastWaypoints * CONTROLLER_WAYPOINT_INTERVAL * getVehicleSpeed());
}
