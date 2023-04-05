// include necessary headers


#include "trajectory.h"

TrajectoryManager::TrajectoryManager(const std::shared_ptr<rclcpp::Node>& node_) : node(node_), path_logger(0.1){
    // create publishers and subscribers
    is_recording_ = false;
    start_recording_srv_ = node->create_service<std_srvs::srv::Empty>("path_record_init", std::bind(&TrajectoryManager::startRecordingCallback, this, std::placeholders::_1, std::placeholders::_2));
    stop_recording_srv_ = node->create_service<std_srvs::srv::Empty>("path_record_stop", std::bind(&TrajectoryManager::stopRecordingCallback, this, std::placeholders::_1, std::placeholders::_2));
    save_path_srv_ = node->create_service<std_srvs::srv::Empty>("path_save", std::bind(&TrajectoryManager::savePathCallback, this, std::placeholders::_1, std::placeholders::_2));
    read_path_srv_ = node->create_service<std_srvs::srv::Empty>("path_read", std::bind(&TrajectoryManager::readPathCallback, this, std::placeholders::_1, std::placeholders::_2));

    path_pub = node->create_publisher<visualization_msgs::msg::Marker>("ego_path", 10);
    lookahead_path_pub = node->create_publisher<visualization_msgs::msg::Marker>("lookahead_path", 10);
    
    

}

std::vector<std::tuple<double, double, double>> TrajectoryManager::getlookaheadPath(){
    return lookahead_path;
}

void TrajectoryManager::updatelookaheadPath(const double& x, const double& y, const double& length) {
    ref_path = path_logger.getPath();
    // Find the closest point on the path to the current position
    double closest_dist = std::numeric_limits<double>::infinity();
    int closest_idx = -1;
    for (int i = 0; i < ref_path.size(); ++i) {
        double dist = std::sqrt(std::pow(std::get<0>(ref_path[i]) - x, 2.0) + std::pow(std::get<1>(ref_path[i]) - y, 2.0));
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_idx = i;
        }
    }
    if (closest_idx < ref_path.size()-4){
        closest_idx = closest_idx +3;
    }

    
    // Determine the start and end indices of the segment
    double total_dist = 0.0;
    int start_idx = closest_idx;
    int end_idx = closest_idx;
    while (total_dist < length) {
        
        if (end_idx < ref_path.size() - 1) {
            double dist = std::sqrt(std::pow(std::get<0>(ref_path[end_idx]) - std::get<0>(ref_path[end_idx + 1]), 2.0) + std::pow(std::get<1>(ref_path[end_idx]) - std::get<1>(ref_path[end_idx + 1]), 2.0));
            if (total_dist + dist < length) {
                total_dist += dist;
                ++end_idx;
            } else {
                break;
            }
        }else{
            break;
        }
    }
    std::vector<std::tuple<double, double, double>>::const_iterator first = ref_path.begin() + start_idx;
    std::vector<std::tuple<double, double, double>>::const_iterator last = ref_path.begin() + end_idx;
    
    std::vector<std::tuple<double, double, double>> segment(first, last);

    lookahead_path = segment;
    
    

}


int TrajectoryManager::getRefTrajSize(){
    
    if(ref_path.size()<2){
        return 0;
    }else{
        return ref_path.size();
    }
    
}

void TrajectoryManager::display_path(){
    
    if (path_logger.path_.size() > 0){
    
        std_msgs::msg::ColorRGBA ref_color, lookahead_color;
        ref_color.r = 1.0;
        ref_color.a = 0.2;
        lookahead_color.g = 1.0;
        lookahead_color.a = 1.0;
        visualization_msgs::msg::Marker path_marker = path_logger.getPathMarker(ref_path,ref_color);
        visualization_msgs::msg::Marker lookahedpath_marker = path_logger.getPathMarker(lookahead_path,lookahead_color);
        path_pub->publish(path_marker);
        lookahead_path_pub->publish(lookahedpath_marker);
    }
}
void TrajectoryManager::log_odom(const nav_msgs::msg::Odometry& odom){
    
    if(is_recording_){
        path_logger.logPath(odom);
    }
}


void TrajectoryManager::startRecordingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    is_recording_ = true;

    RCLCPP_INFO(node->get_logger(), "Path Recording Init");
}

void TrajectoryManager::stopRecordingCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    is_recording_ = false;
    RCLCPP_INFO(node->get_logger(), "Path Recording Stop");
}

void TrajectoryManager::readPathCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res){

    std::string package_path = ament_index_cpp::get_package_share_directory("control_pkg");
    std::string path_file = package_path + "/path/path.txt";
    path_logger.readPathFromFile(path_file);
    // RCLCPP_INFO(node->get_logger(), "Path Saved at");
    RCLCPP_INFO(node->get_logger(), "Path read from file: %s", path_file.c_str());
}

// save the path as center line in global coordinate
void TrajectoryManager::savePathCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    
   
    std::string package_path = ament_index_cpp::get_package_share_directory("control_pkg");
    std::string path_file = package_path + "/path/path.txt";
    path_logger.savePathToFile(path_file);
    // RCLCPP_INFO(node->get_logger(), "Path Saved at");
    RCLCPP_INFO(node->get_logger(), "Path saved to file: %s", path_file.c_str());
    // if (recorded_path_.poses.size() > 0) {
    //     std::string filename = req->data;
    //     rosbag::Bag bag;
    //     bag.open(filename, rosbag::bagmode::Write);
    //     bag.write("path", ros::Time::now(), recorded_path_);
    //     bag.close();
    //     res->message = "Recorded path saved to file: " + filename;
    //     res->success = true;
    // } else {
    //     res->message = "No recorded path to save";
    //     res->success = false;
    // }
}




// save and load a path file
    // save the path to a file
    // void savePathToFile(const nav_msgs::msg::Path& path) {
    //     std::ofstream file("path.txt");
    //     for (const auto& pose : path.poses) {
    //         file << pose.pose.position.x << "," << pose.pose.position.y << std::endl;
    //     }
    //     file.close();
    // }

//  // load the path from a file
//     nav_msgs::msg::Path loadPathFromFile() {
//         nav_msgs::msg::Path path;
//         std::ifstream file("path.txt");
//         if (!file.is_open()) {
//             return path;
//         }
//         std::string line;
//         while (std::getline(file, line)) {
//             std::istringstream iss(line);
//             std::string x_str, y_str;
//             std::getline(iss, x_str, ',');
//             std::getline(iss, y_str, ',');
//             geometry_msgs::msg::PoseStamped pose;
//             pose.header.frame_id = "map";
//             pose.header.stamp = rclcpp::Time::now();
//             pose.pose.position.x = std::stod(x_str);
//             pose.pose.position.y = std::stod(y_str);
//             path.poses.push_back(pose);
//         }
//         file.close();
//         return path;
//     }


