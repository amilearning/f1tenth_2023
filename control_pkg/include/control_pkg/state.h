#ifndef STATE_HPP_
#define STATE_HPP_

#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>

// Path (x,y,psi)
using Path = std::vector<std::tuple<double, double, double>>;
// Trajectory includes Speed reference (x,y,psi,speed)
using Trajectory = std::vector<std::tuple<double, double, double, double>>;
using PathPoint = Eigen::Vector2d;
struct VehicleState {
    Eigen::Vector2d position;
    double yaw;
    double vx;  // vel in local 
    double vy; // vel in local
    double wz; // vel in local
    double accel;
    double delta;  
};


struct ControlInput {
    double accel;
    double delta;
    
};





#endif  // CONTROLLER_HPP_
