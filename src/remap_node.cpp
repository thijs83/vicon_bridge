
#include <vicon_bridge/vicon_remap.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{

    // double frequency = 40;

    ros::init(argc, argv, "vicon_remap");

    auto nh = std::make_shared<ros::NodeHandle>("~");

    double frequency;
    if (nh->hasParam("remap/frequency"))
    {
        nh->getParam("remap/frequency", frequency);
        ROS_INFO_STREAM("[VICON] Frequency is set to " << frequency);
    }
    else
    {
        ROS_ERROR_STREAM("[VICON] No remap value specified in the config");
    }

    std::string topic_name_subscriber;
    std::string topic_name_publisher;
    bool publish_pose_with_covariance_stamped;

    if (nh->hasParam("remap/topic_name_subscriber"))
    {
        nh->getParam("remap/topic_name_subscriber", topic_name_subscriber);
        // ROS_INFO_STREAM("[VICON] Topic name is set to " << topic_name_subscriber);
    }
    else
    {
        ROS_ERROR_STREAM("[VICON] No topic name specified in the config");
    }

    if (nh->hasParam("remap/topic_name_publisher"))
    {
        nh->getParam("remap/topic_name_publisher", topic_name_publisher);
        // ROS_INFO_STREAM("[VICON] Topic name is set to " << topic_name_publisher);
    }
    else
    {
        ROS_ERROR_STREAM("[VICON] No topic name specified in the config");
    }

    if (nh->hasParam("remap/publish_pose_with_covariance_stamped"))
    {
        nh->getParam("remap/publish_pose_with_covariance_stamped", publish_pose_with_covariance_stamped);
    }
    else
    {
        publish_pose_with_covariance_stamped = false;
    }

    if (publish_pose_with_covariance_stamped)
        ROS_INFO_STREAM("Remapping Vicon topic " << topic_name_subscriber << " to " << topic_name_publisher << " [geometry_msgs/PoseWithCovarianceStamped]");
    else
        ROS_INFO_STREAM("Remapping Vicon topic " << topic_name_subscriber << " to " << topic_name_publisher << " [geometry_msgs/PoseStamped]");

    ViconRemap remapper(nh, frequency, topic_name_subscriber, topic_name_publisher, publish_pose_with_covariance_stamped);

    remapper.setup();

    return 0;
}