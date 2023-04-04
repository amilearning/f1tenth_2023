#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

#include <fstream>


#include "FORCESNLPsolver.h"
#include "FORCESNLPsolver_memory.h"

#include "trajectory.h"

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



class Controller{
public:
    // constructor
     Controller(const std::shared_ptr<rclcpp::Node>& node);

    // convert the odometry to vehicle state
    void odometryToVehicleState(const nav_msgs::msg::Odometry& odom, VehicleState& state);

    // solve the main controller to compute the optimal control input
    void mainControllerSolve();
    void mainControllerSolveCallback();

    // compute the predicted trajectory given the predicted control inputs
    void computePredictedTrajectory(const VehicleState& state, const std::vector<ControlInput>& control_inputs, visualization_msgs::msg::Marker& predicted_trajectory);

    // compute the backup controller to use if the main controller fails
    void backupController(const VehicleState& state, ControlInput& control_input);

    TrajectoryManager traj_manager_;

private:
    
    std::shared_ptr<rclcpp::Node> node;
    // private member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predicted_traj_pub_, target_traj_pub_;    
    
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    
    // rclcpp::CallbackGroup::SharedPtr vehicle_status_callback_group_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::CallbackGroup::SharedPtr laser_scan_callback_group_;

    rclcpp::TimerBase::SharedPtr main_controller_timer_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    

    // private callback functions for the subscribers
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // void vehicleStatusCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /////////////////////////////////////////////////////////////////////////////
    VehicleState prev_state, cur_state;
    std::string param_filename;

    double max_speed_;
    double max_steering_angle_;
    double look_ahead_distance_;

    FORCESNLPsolver_params mpc_problem;
    FORCESNLPsolver_info info;
    FORCESNLPsolver_output output;
    FORCESNLPsolver_mem * mem;
    FORCESNLPsolver_extfunc extfunc_eval = &FORCESNLPsolver_adtool2forces;
    Eigen::MatrixXd x0;
    Eigen::MatrixXd x0i;
    bool init_run;
    int N;
    int nvar;
    int neq;    
    int npar;
    int exitflag;
    int return_val;

};

#endif  // CONTROLLER_HPP_
