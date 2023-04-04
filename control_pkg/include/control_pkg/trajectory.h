#ifndef TRAJECTORY_MANAGER_HPP_
#define TRAJECTORY_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/empty.hpp"
// #include "trajectory_msgs/srv/set_string.hpp"
#include <fstream>
#include <mutex>
#include <vector> 
#include <cmath>

#include <tuple>
#include <algorithm>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <boost/filesystem.hpp>



#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <thread>
#include <sstream>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/color_rgba.hpp"

class PathLogger {
public:
    PathLogger(double threshold) : threshold_(threshold) {}

    visualization_msgs::msg::Marker getPathMarker(std::vector<std::tuple<double, double, double>> tmp_path, std_msgs::msg::ColorRGBA color){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = " ";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        // marker.color.r = 1.0;
        // marker.color.a = 1.0;
        marker.color = color;

        geometry_msgs::msg::Point point;
        for (const auto& p : tmp_path) {
        point.x = std::get<0>(p);
        point.y = std::get<1>(p);
        point.z = 0.0;
        marker.points.push_back(point);
        }

        return marker;
    }


   void logPath(const nav_msgs::msg::Odometry& odom) {
    double dx = odom.pose.pose.position.x - last_odom.pose.pose.position.x;
    double dy = odom.pose.pose.position.y - last_odom.pose.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    tf2::Quaternion q(odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w);
    q.normalize();
    // Extract the yaw angle from the quaternion object    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        if (distance >= threshold_) {
            path_.push_back(std::make_tuple(odom.pose.pose.position.x, odom.pose.pose.position.y, yaw));
            last_odom = odom;
        }
        // std::cout << " path length " << path_.size() << std::endl;
    }

    std::vector<std::tuple<double, double, double>> getPath() const {
        return path_;
    }

    //////////////////////////////////////////////////////////////////////
    
    void savePathToFile(const std::string& file_path) {
    // Open the file for writing

    std::size_t pos = file_path.find_last_of("/\\");
    std::string directory_path = file_path.substr(0, pos);
    if (!boost::filesystem::exists(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }


    std::ofstream ofs(file_path, std::ios::out);

    // Write the path to the file
    for (const auto& point : path_) {
        
        ofs << std::get<0>(point) << " " << std::get<1>(point) << " " << std::get<2>(point) << "\n";
    }

    // Close the file
    ofs.close();
    }

    void readPathFromFile(const std::string& file_path) {
    // Open the file for reading
    std::ifstream ifs(file_path, std::ios::in);
    if (ifs.fail()) {        
        return;
    }
    // Clear the current path
    path_.clear();
    // Read the path from the file
    double x, y, yaw;
    while (ifs >> x >> y >> yaw) {
        path_.emplace_back(x, y, yaw);
    }

    // Close the file
    ifs.close();

    }



    double threshold_;
    std::vector<std::tuple<double, double, double>> path_;
    nav_msgs::msg::Odometry last_odom;
};


class TrajectoryManager {
public:
    TrajectoryManager(const std::shared_ptr<rclcpp::Node>& node_);

    // save the path as center line in global coordinate
    void savePathCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void startRecordingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void stopRecordingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void readPathCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void log_odom(const nav_msgs::msg::Odometry& odom);
    void display_path();
    void updatelookaheadPath(const double& x, const double& y, const double& length);
    std::vector<std::tuple<double, double, double>> getlookaheadPath(); 
    // // extract a lookahead path in frenet coordinate given the current position (odometry)
    // void extractLookaheadPath(const nav_msgs::msg::Odometry& odom, nav_msgs::msg::Path& lookahead_path);

    // // save and load a path file
    // void savePathToFile(const nav_msgs::msg::Path& path);
    // nav_msgs::msg::Path loadPathFromFile();

    // Service callback functions
    
    PathLogger path_logger;

private:
    // ROS node and publishers/subscribers
    rclcpp::Node::SharedPtr node;
    
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_recording_srv_, stop_recording_srv_, save_path_srv_, read_path_srv_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub, lookahead_path_pub;


    // Path recording variables
    nav_msgs::msg::Path recorded_path_;
    bool is_recording_;
    
    // Mutex for thread safety
    std::mutex mutex_;

    std::vector<std::tuple<double, double, double>> lookahead_path, ref_path;
    
    
};







#endif  // TRAJECTORY_MANAGER_HPP_
