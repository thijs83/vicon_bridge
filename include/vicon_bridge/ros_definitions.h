#ifndef VICON_BRIDGE_ROS_DEFINITIONS_H
#define VICON_BRIDGE_ROS_DEFINITIONS_H

//#define ROS_VERSION1
#ifdef ROS_VERSION1

#include <ros/ros.h>

#define GET_ROS_CLOCK() ros::Time::now();
typedef ros::Time ROS_TIME;
#define ROS_OK() ros::ok();
typedef ros::Rate ROS_RATE;
#define ROS_SHUTDOWN() ros::shutdown();
typedef ros::Duration ROS_DURATION;



#endif // ROS_VERSION1

#ifdef ROS_VERSION2

#include <rclcpp/rclcpp.hpp>

#define GET_ROS_CLOCK() this->get_clock()->now();
typedef rclcpp::Time ROS_TIME;
#define ROS_OK() rclcpp::ok();
typedef rclcpp::Rate ROS_RATE;
#define ROS_SHUTDOWN() rclcpp::shutdown();
typedef rclcpp::Duration ROS_DURATION;




#endif // ROS_VERSION2


#endif