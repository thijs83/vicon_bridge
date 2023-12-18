
#ifndef VICON_BRIDGE_REMAP
#define VICON_BRIDGE_REMAP

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <chrono>
#include <time.h>

using namespace std::chrono;

class ViconRemap
{
public:
    ViconRemap(std::shared_ptr<ros::NodeHandle> nh, const int frequency, const std::string topic_name_subscriber, const std::string topic_name_publisher, bool publish_pose_with_covariance_stamped);

    void setup();

    geometry_msgs::PoseStamped poseToPoseStampedMsg();
    geometry_msgs::PoseWithCovarianceStamped poseToPoseWithCovarianceStampedMsg();

    void callback_pose(const geometry_msgs::TransformStamped &msg);

private:
    /// Global node handler
    std::shared_ptr<ros::NodeHandle> _nh;

    // Period between remapping
    ros::Rate _loop_rate;

    // Publisher for the mavros topic
    ros::Publisher _pub_remap;

    // Subscriber for the IMU pose and Reset
    ros::Subscriber _sub_pose;

    // Beginning time of period (Timer<milliseconds, steady_clock>)
    geometry_msgs::TransformStamped _msg;

    // Store z offset
    double _offset_z;

    // Store whether we should publish with or without covariance
    bool _publish_pose_with_covariance_stamped;

    // If first frame is received for storing offset
    bool _first_frame = true;
};

#endif
