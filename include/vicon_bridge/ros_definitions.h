#ifndef VICON_BRIDGE_ROS_DEFINITIONS_H
#define VICON_BRIDGE_ROS_DEFINITIONS_H

/**
 * 
 * ROS1 definitions
 * 
 */
#ifdef ROS_VERSION1

#include <ros/ros.h>

#define GET_ROS_CLOCK() ros::Time::now();
typedef ros::Time ROS_TIME;
#define ROS_OK() ros::ok();
typedef ros::Rate ROS_RATE;
#define ROS_SHUTDOWN() ros::shutdown();
typedef ros::Duration ROS_DURATION;


// create publisher object for ROS1
template <typename MSG>
class RosPublisher
{
    ros::Publisher publisher;

public:
    RosPublisher(const ros::Publisher publisher):
        publisher(publisher)
    {};

    void publishNow(MSG msg)
    {
        publisher.publish(msg);
    }
};


#endif // ROS_VERSION1

/**
 * 
 * ROS2 definitions
 * 
 */
#ifdef ROS_VERSION2

#include <rclcpp/rclcpp.hpp>

#define GET_ROS_CLOCK() this->get_clock()->now();
typedef rclcpp::Time ROS_TIME;
#define ROS_OK() rclcpp::ok();
typedef rclcpp::Rate ROS_RATE;
#define ROS_SHUTDOWN() rclcpp::shutdown();
typedef rclcpp::Duration ROS_DURATION;

//create publisher object for ROS2
template <typename MSG>
class RosPublisher
{
    rclcpp::Publisher<MSG>::SharedPtr publisher;

public:
    RosPublisher(const rclcpp::Publisher<MSG>::SharedPtr publisher):
        publisher(publisher)
    {};

    void publishNow(MSG msg)
    {
        publisher->publish(msg);
    }
};

#endif // ROS_VERSION2


#endif // VICON_BRIDGE_ROS_DEFINITIONS_H