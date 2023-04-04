#include "rclcpp/rclcpp.hpp"
#include "ctrl.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // create the ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("my_node");

    // create the controller object
    Controller controller(node);

    // run the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
