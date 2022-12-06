/**
 * @file obsAvoid.cpp
 * @author Madhu Narra Chittibabu (madhunc117@gmail.com)
 * @brief this file contains all the implementations needed for obstacle avoidance
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "obsAvoid.h"

ObsAvoid::ObsAvoid(const std::string &node_name) : Node(node_name){
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",1,std::bind(&ObsAvoid::bot_callback,this, std::placeholders::_1));
}

void ObsAvoid::bot_callback(const sensor_msgs::msg::LaserScan::ConstPtr &msg){
     if (obs_detected(msg)) {
        // Obstacle ahead so turn
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.2;
    } else {
        // No obstacle ahead so go straight
        vel_.linear.x = 0.2;
        vel_.angular.z = 0.0;
    }
    publisher_->publish(vel_);
}

bool ObsAvoid::obs_detected(const sensor_msgs::msg::LaserScan::ConstPtr &msg){
    std::array<int, 41> degrees = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 , 11, 12,
            13, 14, 15, 16, 17, 18, 19, 20, 340, 341, 342, 343, 344, 345, 346,
            347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359};

    // If obstacle is present before 0.4 meters return true
    for (auto i : degrees) {
        if (msg->ranges[i] < 0.4) {
           RCLCPP_WARN_STREAM(
      rclcpp::get_logger("rclcpp"),
      "Obstacle detected");
            return true;
        }
    }
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("rclcpp"),
      "No Obstacle");
    return false;
}


