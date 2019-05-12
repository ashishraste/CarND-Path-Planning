# Path Planning

Self-driving car path planning on a highway.

<img src="">

## Overview
This project is part of [Udacity's Self-Driving Car Nanodegree program](https://www.udacity.com/drive)
and most of boiler-plate code comes from the program's lecture notes and quizzes.

A self-driving car on a simulated highway road is tasked with planning its path. The controller provided by the simulator makes sure that the waypoints are reached perfectly. The following constraints exist for the ego-car to drive on this road.

* Speed limit : 50 mph
* Total acceleration : 10 m/s^2
* Total jerk : 10 m/s^3

## Dependencies
* Udacity simulator : https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2
* uWebSockets : Only available for Mac OS and Ubuntu linux environments.
  * Mac OS : `sh install-mac.sh`
  * Ubuntu : `sh install-ubuntu.sh`  
* Cubic Spline Library : https://kluge.in-chemnitz.de/opensource/spline/ 
* CMake and Make build systems
* Compiler supporting C++11 standard 
  

## Build and Run
1. Start the simulator. 
2. From the parent directory, run the following. It builds and runs the `path_planning` binary. 
   ```bash
    sh run-path-planning.sh
    ```    

## Directory Layout
* src : Contains source related to behaviour-planning and path-planning. Also includes
`vehicle.cpp` class which encapsulates a vehicle's position and kinematic properties like speed, Frenet coordinates, lane ID, etc.   
* data : Has the global map information of the highway with details such as map's waypoint
coordinates.

## Path Planning
Let's go through the steps that the path-planner takes to navigate around the highway maintaining
its speed, acceleration, jerk and safety constraints. Please note that rest
of this document refers to the source in `src/` directory.

### Inputs
The path-planner is given the inputs of the ego-car's localisation data and the sensor-fused data 
containing the information of detected vehicles. 

The localisation data includes the following information of the ego-car.

* Global position : Both Euclidean (x, y) and Frenet (s, d) coordinates.
* Yaw
* Speed

The sensor-fusion data includes the below information of other detected vehicles.

* Vehicle ID
* Velocity in x and y direction
* Global position in Frenet coordinates

### Output
As an output, the path-planner provides the simulator with a set of waypoints to achieve.
These waypoints are computed using the approach described in the [next section](#methodology).

### Methodology

The approach taken to plan a path is broken down into three main sections: Prediction, Behaviour Planning, and Trajectory Planning. Each of these sub-topics are described in the following sections referring to their
implementation.

![path-planning](./images/path-planning-components.png)
* Path Planning Components. Source: _Udacity Self-driving car Nanodegree_  

#### Prediction

Using the sensor-fusion input, the ego-car estimates the lane, position,
and the next position of nearby vehicles.

#### Behaviour Planning

In order to drive safely and under legal constraints, the ego-car decides 
the next _state_ that it wants to be in given current situation and state.

#### Trajectory Planning

Once we have a list of points in the selected maneuver, we create a trajectory using
Spline interpolation. 