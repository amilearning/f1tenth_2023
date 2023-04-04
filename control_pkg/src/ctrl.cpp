#include "ctrl.h"



// constructor

// Controller::Controller(const std::shared_ptr<rclcpp::Node>& node) : traj_manager_(node) {
    Controller::Controller(const std::shared_ptr<rclcpp::Node>& node_) : node(node_), traj_manager_(node_), init_run(true){
    // create publishers and subscribers here as before
    // create publishers
    mem = FORCESNLPsolver_internal_mem(0);
    N = 10;
    nvar = 7;
    neq = 5;    
    npar = 2;
    exitflag = 0;
    return_val = 0;
    x0i.resize(nvar,1);
    x0.resize(nvar, N);


    
    //Initialize parameters
    node->declare_parameter("param_filename", "config.yaml");
    node->get_parameter("param_filename", param_filename);

    // // create publishers
    control_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("drive", 10);
    predicted_traj_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("predicted_trajectory", 10);
    target_traj_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("target_trajectory", 10);

    // // create callback groups
    odom_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto imu_options = rclcpp::SubscriptionOptions();    
    imu_options.callback_group = odom_callback_group_;
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Controller::odomCallback, this, std::placeholders::_1), imu_options);


    imu_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto imu_optins =  rclcpp::SubscriptionOptions(); 
    imu_optins.callback_group = imu_callback_group_;
    imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&Controller::imuCallback, this, std::placeholders::_1), imu_optins);
    
    
    laser_scan_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto laser_optins =  rclcpp::SubscriptionOptions(); 
    laser_optins.callback_group = laser_scan_callback_group_;
    laser_scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Controller::laserScanCallback, this, std::placeholders::_1), laser_optins);
    
    
     
    timer_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    double main_controller_freq = 10.0;  // 10 Hz
    main_controller_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Controller::mainControllerSolveCallback, this),timer_cb_group_);

    

}


void Controller::mainControllerSolve(){
    return;
}

void Controller::mainControllerSolveCallback() {
    // Get current vehicle state
    traj_manager_.display_path();
    traj_manager_.updatelookaheadPath(cur_state.position(0),cur_state.position(1),3.0);
    
    // set initial state [F, deltarate, xPos yPos vx yaw delta] input = [acceleration force F and steering rate phi]
    x0i << 0.0, 0.0, cur_state.position(0),cur_state.position(1),cur_state.vx, cur_state.yaw, cur_state.delta;
    // set initial guess
        if(init_run){
            for (int i = 0; i < N; ++i) {
            x0.col(i) = x0i;
            }
            init_run = false;
        }
    
    // set ref path 
    std::vector<std::tuple<double, double, double>> lookahead_path = traj_manager_.getlookaheadPath();
      
    if(lookahead_path.size() >= N){
        
        Eigen::MatrixXd target_path(2,N);
        
        for(int i = 0; i < N; i++) {
            target_path(0, i) = std::get<0>(lookahead_path[i]);
            target_path(1, i) = std::get<1>(lookahead_path[i]);
        }


        std::memcpy(mpc_problem.xinit, x0i.data(), x0i.rows()* x0i.cols()* sizeof(double));
        std::memcpy(mpc_problem.x0, x0.data(), x0.rows()* x0.cols()* sizeof(double));
        std::memcpy(mpc_problem.all_parameters, target_path.data(), target_path.cols()* target_path.rows()* sizeof(double));

        exitflag = FORCESNLPsolver_solve(&mpc_problem, &output, &info, mem, NULL, extfunc_eval);
        if (exitflag !=1)
        {
            std::cout<< "/n/nmyMPC did not return optimla solution)" <<std::endl;
        } 
            // std::cout <<"output force " << output.x01[0];
            std::cout <<"output deltarate " << output.x01<< std::endl;
            
    }
    
        
    return;
}
// // convert the odometry to vehicle state
void Controller::odometryToVehicleState(const nav_msgs::msg::Odometry& odom, VehicleState& state) {
    
    state.position << odom.pose.pose.position.x  , odom.pose.pose.position.y;
    
    tf2::Quaternion q(odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w);
    q.normalize();
    // Extract the yaw angle from the quaternion object    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    state.yaw = yaw;
    
    // state.yaw = tf2::getYaw(odom.pose.pose.orientation);    
    state.wz = odom.twist.twist.angular.z;
    bool twist_in_local = true;
    if (twist_in_local){
        state.vx =      odom.twist.twist.linear.x;
        state.vy =      odom.twist.twist.linear.y;        
    }else{
        tf2::Matrix3x3 rot_mat;
        rot_mat.setRPY(0.0, 0.0, state.yaw);
        tf2::Vector3 global_vel(odom.twist.twist.linear.x, odom.twist.twist.linear.y, 0.0);
        tf2::Vector3 local_vel = rot_mat * global_vel;
        state.vx =  local_vel.x();  
        state.vy =  local_vel.y();          
    }
    

}


// // compute the predicted trajectory given the predicted control inputs
// void Controller::computePredictedTrajectory(const VehicleState& state, const std::vector<ControlInput>& control_inputs, visualization_msgs::msg::Marker& predicted_trajectory) {
//         return;
//     // // clear the predicted trajectory
//     // predicted_trajectory.points.clear();

//     // // compute the predicted trajectory using the control inputs
//     // double dt = 0.1;  // time step between control inputs
//     // Eigen::Vector2d pos = state.pos;
//     // Eigen::Vector2d vel = state.vel;
//     // double theta = state.heading;
//     // for (size_t i = 0; i < control_inputs.size(); i++) {
//     //     // apply the control input to the vehicle model
//     //     double steer = control_inputs[i].steer;
//     //     double throttle = control_inputs[i].throttle;
//     //     double brake = control_inputs[i].brake;
//     //     VehicleModel::updateState(pos, vel, theta, throttle, brake, steer, dt);
//     //     pos = VehicleModel::getState().pos;
//     //     vel = VehicleModel::getState().vel;
//     //     theta = VehicleModel::getState().heading;

//     //     // create a new point for the predicted trajectory and add it to the marker
//     //     geometry_msgs::msg::Point point;
//     //     point.x = pos(0);
//     //     point.y = pos(1);
//     //     point.z = 0.0;
//     //     predicted_trajectory.points.push_back(point);
//     // }

//     // // set the properties of the marker
//     // predicted_trajectory.header.frame_id = "map";
//     // predicted_trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
//     // predicted_trajectory.action = visualization_msgs::msg::Marker::ADD;
//     // predicted_trajectory.pose.orientation.w = 1.0;
//     // predicted_trajectory.scale.x = 0.1;
//     // predicted_trajectory.color.a = 1.0;
//     // predicted_trajectory.color.r = 0.0;
//     // predicted_trajectory.color.g = 1.0;
//     // predicted_trajectory.color.b = 0.0;
// }


// // compute the backup controller to use if the main controller fails
// void Controller::backupController(const VehicleState& state, ControlInput& control_input) {
    
//     return;
//     // TODO: implement the backup controller algorithm to compute the control input
// }


// // private callback functions for the subscribers
// // private callback functions for the subscribers
void Controller::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     // convert the odometry to vehicle state
    odometryToVehicleState(*msg, cur_state);
    traj_manager_.log_odom(*msg);
    // RCLCPP_INFO(node->get_logger(), "current yaw in deg: %f", cur_state.yaw*180.0/M_PI);

}

// // void Controller::vehicleStatusCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
// //     // TODO: implement the vehicle status subscriber callback function
// //     return;
// // }

void Controller::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // TODO: implement the IMU subscriber callback function
    return;
}

void Controller::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // TODO: implement the laser scan subscriber callback function
    return;
}
