// Copyright 2023 HMCL, High-assurance Mobility and Control Laboratory in UNIST 
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



#include <algorithm>
#include <limits>
#include <utility>
#include "pure_pursuit.h"


////////////////////////////////////////////////////////////////////////////////
PurePursuit::PurePursuit(const std::shared_ptr<rclcpp::Node>& node_) : nh_(node_), dt(0.05){

    nh_->declare_parameter<double>("Pminimum_lookahead_distance", 0.2);
    nh_->declare_parameter<double>("Pmaximum_lookahead_distance", 50.0);
    nh_->declare_parameter<double>("Pspeed_to_lookahead_ratio", 1.0);
    nh_->declare_parameter<double>("Pemergency_stop_distance", 0.0);
    // nh_->declare_parameter<double>("Pspeed_thres_traveling_direction", 0.0);
    nh_->declare_parameter<double>("Pmax_acceleration", 50.0);
    nh_->declare_parameter<double>("Pdistance_front_rear_wheel", 0.33);
    nh_->declare_parameter<double>("vel_lookahead_ratio", 1.0);
    
    
    nh_->get_parameter("vel_lookahead_ratio",vel_lookahead_ratio);    
    nh_->get_parameter("Pminimum_lookahead_distance",minimum_lookahead_distance);    
    nh_->get_parameter("Pmaximum_lookahead_distance",maximum_lookahead_distance);    
    nh_->get_parameter("Pspeed_to_lookahead_ratio",speed_to_lookahead_ratio);    
    nh_->get_parameter("Pemergency_stop_distance",emergency_stop_distance);    
    // nh_->get_parameter("Pspeed_thres_traveling_direction",speed_thres_traveling_direction);    
    nh_->get_parameter("Pmax_acceleration",max_acceleration);    
    nh_->get_parameter("Pdistance_front_rear_wheel",distance_front_rear_wheel);    
    


}

////////////////////////////////////////////////////////////////////////////////

void PurePursuit::update_ref_path(const Path & ref_,const std::vector<double> & ref_speed_){

    ref_path = ref_;
    pathToTrajectory(ref_,ref_traj,ref_speed_);
        
  
}

void PurePursuit::update_ref_traj(const Trajectory & ref_){
    ref_traj = ref_;
}

void PurePursuit::pathToTrajectory(
    const Path & ref_path, 
    Trajectory & ref_traj, 
    const std::vector<double>& ref_speed
) {
    ref_traj.resize(ref_path.size());
    
    std::transform(ref_path.begin(), ref_path.end(), ref_speed.begin(), std::begin(ref_traj),
        [](const auto& tup, const double& value) {
            return std::make_tuple(
                std::get<0>(tup),
                std::get<1>(tup),
                std::get<2>(tup),
                value
            );
        }
    );
}

void PurePursuit::update_vehicleState(const VehicleState & state){
    cur_state = state;
}


ackermann_msgs::msg::AckermannDriveStamped PurePursuit::compute_command()
{
  const auto start = std::chrono::system_clock::now();
//   TrajectoryPoint current_point = current_pose.state;  // copy 32bytes
  PathPoint target_point;
  double target_speed = cur_state.vx; 
  int near_idx;
  compute_lookahead_distance(cur_state.vx);  // update m_lookahead_distance 
  const auto is_success = compute_target_point(m_lookahead_distance, target_point, near_idx); // update target_point, near_idx
  m_target_point = target_point;
  if (is_success) {
    // m_command.long_accel_mps2 = compute_command_accel_mps(current_point, false);
        cmd_msg.header.stamp = nh_->now();
        cmd_msg.header.frame_id = "ego";
        // ackermann_msg.drive.speed =  opt_vel/5.0;
        cmd_msg.drive.speed =  compute_target_speed();

        cmd_msg.drive.steering_angle = compute_steering_rad();
        std::cout << "drive.steering_angle  =  " << cmd_msg.drive.steering_angle  << std::endl;
    // Use velocity of the next immediate trajectory point as the target velocity
  } else {
        cmd_msg.header.stamp = nh_->now();
        cmd_msg.header.frame_id = "ego";        
        cmd_msg.drive.speed =  0.0;
        cmd_msg.drive.steering_angle =cur_state.delta;
    
  }
  

  return cmd_msg;
}

PathPoint PurePursuit::get_target_point(){
    return m_target_point;
} 

double PurePursuit::compute_target_speed(){
     PathPoint target_point;
     int near_idx;
     double vel_lookahed_dist = fabs(cur_state.vx*vel_lookahead_ratio);
    compute_target_point(vel_lookahed_dist, target_point, near_idx); 
    if (near_idx >= ref_traj.size()-1){
        near_idx = ref_traj.size()-1;
    }
    double target_speed = std::get<3>(ref_traj[near_idx]); 
    return target_speed;
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuit::compute_lookahead_distance(const double current_velocity)
{
  const double lookahead_distance = fabs(current_velocity * speed_to_lookahead_ratio);
  m_lookahead_distance =
    std::max(minimum_lookahead_distance,
    std::min(lookahead_distance, maximum_lookahead_distance));
}
////////////////////////////////////////////////////////////////////////////////




bool PurePursuit::compute_target_point(const double & lookahead_distance, PathPoint & target_point_, int & near_idx)
{   near_idx =0;
    PathPoint current_point;
    current_point =cur_state.position;
    
  int idx = 0;
//   uint32_t last_idx_for_noupdate = 0U;
  int last_idx_for_noupdate = 0;
  
  for (idx = 0; idx <   ref_traj.size(); ++idx) {
    PathPoint target_point;
    target_point << std::get<0>(ref_traj[idx]), std::get<1>(ref_traj[idx]);
    
    last_idx_for_noupdate = idx;

      // Search the closest point over the lookahead distance
      if (compute_points_distance_squared(current_point, target_point) >=lookahead_distance)
      {
        target_point_ = target_point;
        
        break;
      }
   
  }

  bool is_success = true;
  // If all points are within the distance threshold,
  if (idx == ref_traj.size()) {
      // use the farthest target index in the traveling direction      
      target_point_ << std::get<0>(ref_traj[last_idx_for_noupdate]), std::get<1>(ref_traj[last_idx_for_noupdate]);   
  }

    // if the target point is too close to stop within 0.2sec 
  if(compute_points_distance_squared(current_point, target_point_) < cur_state.vx*0.2){
    is_success = false;
  }
  near_idx = last_idx_for_noupdate;

  return is_success;
}
////////////////////////////////////////////////////////////////////////////////
double PurePursuit::compute_points_distance_squared(
  const PathPoint & point1,
  const PathPoint & point2)
{
    return sqrt((point1[0] - point2[0])*(point1[0] - point2[0]) + (point1[1] - point2[1])*(point1[1] - point2[1]) );
    
}
////////////////////////////////////////////////////////////////////////////////
std::pair<double, double> PurePursuit::compute_relative_xy_offset(
  const PathPoint & current,
  const PathPoint & target) const
{
  const auto diff_x = target[0] - current[0];
  const auto diff_y = target[1] - current[1];
  const auto yaw = cur_state.yaw;  
  const auto cos_pose = std::cos(yaw);
  const auto sin_pose = std::sin(yaw);
  const auto relative_x = static_cast<double>((cos_pose * diff_x) + (sin_pose * diff_y));
  const auto relative_y = static_cast<double>((-sin_pose * diff_x) + (cos_pose * diff_y));
  const std::pair<double, double> relative_xy(relative_x, relative_y);
  return relative_xy;
}
////////////////////////////////////////////////////////////////////////////////
double PurePursuit::compute_steering_rad()
{

    PathPoint current_point;
        current_point << cur_state.position[0] , cur_state.position[1];
  // Compute the steering angle by arctan(curvature * wheel_distance)
  // link: https://www.ri.cmu.edu/pub_files/2009/2/
  //       Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
  const double denominator = compute_points_distance_squared(current_point, m_target_point);
  const double numerator = compute_relative_xy_offset(current_point, m_target_point).second;
  constexpr double epsilon = 0.0001F;
  // equivalent to (2 * y) / (distance * distance) = (2 * sin(th)) / distance
  const double curvature = (denominator > epsilon) ? ((2.0F * numerator) / denominator) : 0.0F;
  const double steering_angle_rad = atanf(curvature * distance_front_rear_wheel);
  return -1*steering_angle_rad;
}
////////////////////////////////////////////////////////////////////////////////

