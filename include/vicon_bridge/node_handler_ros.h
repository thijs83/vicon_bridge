#ifndef VICON_BRIDGE_NODE_HANDLER_ROS_H
#define VICON_BRIDGE_NODE_HANDLER_ROS_H

#include <tf2_ros/transform_broadcaster.h>


#ifdef ROS_VERSION1

#include <ros/ros.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include <geometry_msgs/TransformStamped.h>

typedef geometry_msgs::TransformStamped GEOMETRY_MSG_TRANSFORMSTAMPED;
typedef vicon_bridge::Markers VICON_BRIDGE_MARKERS;
typedef vicon_bridge::Marker VICON_BRIDGE_MARKER;

class NodeHandler
{
public:
    NodeHandler(std::string node_name):
        nh_priv_("~"),
        nh_(node_name)
    {
    }

    // Get parameter from the parameter server from private namespace
    template <typename T>
    void getParameter(std::string parameter_name, T &param)
    {
        nh_priv_.param(parameter_name, param, param);
    }

    template <typename MSG>
    ros::Publisher createPublisher(std::string topic, int queue_size)
    {
        return nh_.advertise<MSG>(topic, queue_size);
    }


    void createMarkerPublisher(std::string topic, int queue_size)
    {
        pub_marker_ = std::make_shared<ros::Publisher>(nh_.advertise<vicon_bridge::Markers>(topic, queue_size));
    }

    int subscribersCount()
    {
        return pub_marker_->getNumSubscribers();
    }


protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    std::shared_ptr<ros::Publisher> pub_marker_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};



#endif // ROS_VERSION1

#ifdef ROS_VERSION2

#include "rclcpp/rclcpp.hpp"
#include "vicon_bridge/msg/markers.hpp"
#include "vicon_bridge/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

typedef geometry_msgs::msg::TransformStamped GEOMETRY_MSG_TRANSFORMSTAMPED;
typedef vicon_bridge::msg::Markers VICON_BRIDGE_MARKERS;
typedef vicon_bridge::msg::Marker VICON_BRIDGE_MARKER;

class NodeHandler : public rclcpp::Node
{
public:
    NodeHandler(std::string node_name) : Node(node_name), tf_broadcaster_(this)
    {}

 
    // Get parameter from the parameter server from private namespace
    template <typename T>
    void getParameter(std::string parameter_name, T &param)
    {
        this->declare_parameter(parameter_name, param);
        this->get_parameter(parameter_name, param);
    }

    template <typename MSG>
    typename rclcpp::Publisher<MSG>::SharedPtr createPublisher(std::string topic, int queue_size)
    {
        return this->create_publisher<MSG>(topic, 10);
    }

    void createMarkerPublisher(std::string topic, int queue_size)
    {
        pub_marker_ = this->create_publisher<vicon_bridge::msg::Markers>(topic, queue_size);
    }

    int subscribersCount()
    {
        return pub_marker_->get_subscription_count();
    }

protected:
    rclcpp::Publisher<vicon_bridge::msg::Markers>::SharedPtr pub_marker_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};


#endif // ROS_VERSION2

#endif // VICON_BRIDGE_NODE_HANDLER_ROS_H
