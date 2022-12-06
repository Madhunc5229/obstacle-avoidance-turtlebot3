/**
 * @file obsAvoid.h
 * @author Madhu Narra Chittibabu (madhunc117@gmail.com)
 * @brief this file contains all the declarations needed for obstacle avoidance
 * @version 0.1
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef INCLUDE_OBSTACLE_AVOIDANCE_TURTLEBOT3_OBSAVOID_H_
#define INCLUDE_OBSTACLE_AVOIDANCE_TURTLEBOT3_OBSAVOID_H_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

/**
 * @brief this class conatins declarations for obstacle avoidance
 *
 */
class ObsAvoid : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Obs Avoid object
   *
   * @param node_name
   */
  ObsAvoid(const std::string &node_name);

 private:
  /**
   * @brief this function checks publishes cmd vel based on obstacle in front
   *
   * @param msg point cloud from /scan topic
   */
  void bot_callback(const sensor_msgs::msg::LaserScan::ConstPtr &msg);

  /**
   * @brief this function checks if there an obstacle in front
   *
   * @param msg point cloud from /scan topic
   * @return true if there is an obstacle in front
   * @return false if there is no obstacle in front
   */
  bool obs_detected(const sensor_msgs::msg::LaserScan::ConstPtr &msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      publisher_;  //!< The pointer to the publisher topic.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  geometry_msgs::msg::Twist vel_;
};

#endif  // INCLUDE_OBSTACLE_AVOIDANCE_TURTLEBOT3_OBSAVOID_H_
