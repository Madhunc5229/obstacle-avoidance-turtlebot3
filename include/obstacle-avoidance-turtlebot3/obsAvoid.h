/**
 * @file obsAvoid.h
 * @author Madhu Narra CHittibabu (madhunc117@gmail.com)
 * @brief this file contains all the declarations needed for obstacle avoidance
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef INCLUDE_OBSAVOID_H_
#define INCLUDE_OBSAVOID_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


class ObsAvoid : public rclcpp::Node {
 public:

  ObsAvoid(const std::string &node_name);

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      publisher_;  //!< The pointer to the publisher topic.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_;
  geometry_msgs::msg::Twist vel_;

  void bot_callback(const sensor_msgs::msg::LaserScan::ConstPtr &msg);

  bool obs_detected(const sensor_msgs::msg::LaserScan::ConstPtr &msg);
};

#endif // INCLUDE_OBSAVOID_H_