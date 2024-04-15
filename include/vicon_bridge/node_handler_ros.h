#ifndef VICON_BRIDGE_NODE_HANDLER_ROS_H
#define VICON_BRIDGE_NODE_HANDLER_ROS_H



#ifdef ROS_VERSION1

#include <ros/ros.h>


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

    void createMarkerPublisher(std::string topic, int queue_size)
    {
        pub_marker_ = nh_.advertise<visualization_msgs::Marker>(topic, queue_size);
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
};
















#endif