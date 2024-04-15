#ifndef VICON_BRIDGE_NODE_HANDLER_ROS_H
#define VICON_BRIDGE_NODE_HANDLER_ROS_H



#ifdef ROS_VERSION1

#include <ros/ros.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>

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
        nh_.param(parameter_name, param, param);
    }

    template <typename MSG>
    ros::Publisher createPublisher(std::string topic, int queue_size)
    {
        return nh_.advertise<MSG>(topic, queue_size);
    }

    void createMarkerPublisher(std::string topic, int queue_size)
    {
        pub_marker_ = nh_.advertise<vicon_bridge::Markers>(topic, queue_size);
    }


protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    ros::Publisher pub_marker_;

};



#endif // ROS_VERSION1

#ifdef ROS_VERSION2

#include <rclcpp/rclcpp.hpp>

class NodeHandler : public rclcpp::Node
{
public:
    NodeHandler(std::string node_name) : Node(node_name)
    {}


    // Get parameter from the parameter server from private namespace
    template <typename T>
    void getParameter(std::string parameter_name, T &param)
    {
        rclcpp::Parameter parameter;
        parameter = this->get_parameter("~/"+parameter_name, parameter);
        param = std::static_cast<T>(parameter.get_value());
    }

    template <typename MSG>
    rclcpp::Publisher<MSG>::SharedPtr createPublisher(std::string topic, int queue_size)
    {
        return this->create_publisher<MSG>(publish_topic, 10);
    }

    void createMarkerPublisher(std::string topic, int queue_size)
    {
        pub_marker_ = nh_.advertise<vicon_bridge::msg::Markers>(topic, queue_size);
    }

protected:
    rclcpp::Publisher<vicon_bridge::msg::Markers>::SharedPtr pub_marker_;

};

#endif // ROS_VERSION2

#endif // VICON_BRIDGE_NODE_HANDLER_ROS_H
