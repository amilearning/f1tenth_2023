/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file StateEstimator.cpp
 * @author Paul Drews <pdrews3@gatech.edu>
 * @author Edited by Hojin Lee <hojinlee@unist.ac.kr> 
 * @date May 1, 2017 / modified 2022
 * @copyright 2017 Georgia Institute of Technology
 * @brief ROS node to fuse information sources and create an accurate state estimation *
 * @details Subscribes to other pose estimate solution, GPS, IMU, and wheel odometry topics, claculates
 * an estimate of the car's current state using GTSAM, and publishes that data.
 ***********************************************/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <vector>
#include "speedEstimator.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>


 
#include "rclcpp/rclcpp.hpp"





using namespace gtsam;
// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::G; // GPS pose


// macro for getting the time stamp of a ros message
#define TIME(msg) ( (msg)->header.stamp.sec + (msg)->header.stamp.nanosec*1e-9 )  


  StateEstimator::StateEstimator(const std::shared_ptr<rclcpp::Node>& node_) : nh_(node_),
    lastImuT_(0.0),
    lastImuTgps_(0.0),
    maxQSize_(0),    
    imuOptQ_(400),
    localposeOptQ_(200),
    dual_localposeOptQ_(200),
    gotFirstFix_(false),
    gotFirstLocalPose_(false),
    gotFirstImu_(false),
    odomKey(1),
    imuKey(1),
    latestGPSKey(0)
  {
    
    nh_->declare_parameter<std::string>("toFrame", "odom");
    nh_->declare_parameter<std::string>("fromFrame", "base_link");
    nh_->get_parameter("toFrame",toFrame);    
    nh_->get_parameter("fromFrame",fromFrame );
    


    odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("/est_odom", 10);
    fix_pub_ = nh_->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);

    tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
 
        tf_buffer_2=  std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_2 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_2);


    // timer_main_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // main_timer_ = nh_->create_wall_timer( std::chrono::milliseconds(100), std::bind(&StateEstimator::mainloop, this),timer_main_group_); 
    // std::cout << " main init" << std::endl;
    
    boost::thread optimizer(&StateEstimator::tfloop,this); // main loop
     boost::thread optimizer2(&StateEstimator::fixloop,this); // main loop
    
  }

  StateEstimator::~StateEstimator()
  {}

  void StateEstimator::tfloop(){
    
      rclcpp::Rate loop_rate(20);
      
      while(rclcpp::ok()){
     geometry_msgs::msg::TransformStamped t;
      //    tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
      // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        try {
          t = tf_buffer_->lookupTransform(
            toFrame, fromFrame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(nh_->get_logger(), "Could not transform %s to %s: %s",
            toFrame.c_str(), fromFrame.c_str(), ex.what());
          continue;
        }

        nav_msgs::msg::Odometry poseNew;
        poseNew.header.stamp = t.header.stamp;

         
        poseNew.pose.pose.orientation.x = t.transform.rotation.x;
        poseNew.pose.pose.orientation.y = t.transform.rotation.y;
        poseNew.pose.pose.orientation.z = t.transform.rotation.z;
        poseNew.pose.pose.orientation.w = t.transform.rotation.w;

        poseNew.pose.pose.position.x = t.transform.translation.x;
        poseNew.pose.pose.position.y = t.transform.translation.y;
        poseNew.pose.pose.position.z = t.transform.translation.z;

        // poseNew.twist.twist.linear.x = currentPose.velocity().x();
        // poseNew.twist.twist.linear.y = currentPose.velocity().y();
        // poseNew.twist.twist.linear.z = currentPose.velocity().z();
        
        // poseNew.twist.twist.angular.x = gyro.x() + optimizedBias.gyroscope().x();
        // poseNew.twist.twist.angular.y = gyro.y() + optimizedBias.gyroscope().y();
        // poseNew.twist.twist.angular.z = gyro.z() + optimizedBias.gyroscope().z();

        poseNew.child_frame_id = "base_link";
        poseNew.header.frame_id = "odom";

        odom_pub_->publish(poseNew);
        loop_rate.sleep();
          }

        
  }


void StateEstimator::fixloop(){
    
      rclcpp::Rate loop_rate(10);
      
      while(rclcpp::ok()){
     geometry_msgs::msg::TransformStamped t;
      //    tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
      // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        try {
          t = tf_buffer_2->lookupTransform(
            toFrame, fromFrame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(nh_->get_logger(), "Could not transform %s to %s: %s",
            toFrame.c_str(), fromFrame.c_str(), ex.what());
          continue;
        }

        sensor_msgs::msg::NavSatFix fixNew;
        fixNew.header.stamp =  t.header.stamp;
        fixNew.latitude = 

         
        fixNew.latitude = t.transform.translation.x;
        fixNew.longitude = t.transform.translation.y;
        fixNew.altitude = t.transform.translation.z;


        fix_pub_->publish(fixNew);
        loop_rate.sleep();
          }

        
  }



  void StateEstimator::mainloop(){
      rclcpp::Rate loop_rate(10);

    while(rclcpp::ok()){
      
      
      loop_rate.sleep();
    }
    
}


  




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // create the ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("speed_node");


      // rclcpp::executors::MultiThreadedExecutor executor;
      // executor.add_node(node);

    // create the controller object
    StateEstimator wpt(node);

    
    // executor.spin();
   

    // run the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}