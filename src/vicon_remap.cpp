
#include <vicon_bridge/vicon_remap.h>

using namespace std::chrono;

ViconRemap::ViconRemap(std::shared_ptr<ros::NodeHandle> nh, const int frequency, const std::string topic_name_subscriber, const std::string topic_name_publisher, bool publish_pose_with_covariance_stamped)
    : _nh(nh), _loop_rate(frequency), _publish_pose_with_covariance_stamped(publish_pose_with_covariance_stamped)
{
    if (publish_pose_with_covariance_stamped)
        _pub_remap = _nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name_publisher, 10);
    else
        _pub_remap = _nh->advertise<geometry_msgs::PoseStamped>(topic_name_publisher, 10);

    _sub_pose = _nh->subscribe(topic_name_subscriber, 100, &ViconRemap::callback_pose, this);
}

void ViconRemap::setup()
{
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
        if (_publish_pose_with_covariance_stamped)
            _pub_remap.publish(poseToPoseWithCovarianceStampedMsg());
        else
            _pub_remap.publish(poseToPoseStampedMsg());

        // msg_remap.header.seq = _msg.header.seq;
        // msg_remap.header.frame_id = "";
        // msg_remap.header.stamp = _msg.header.stamp;

        ros::spinOnce;

        _loop_rate.sleep();
    }
}

geometry_msgs::PoseStamped ViconRemap::poseToPoseStampedMsg()
{
    geometry_msgs::PoseStamped msg_remap;
    msg_remap.header = _msg.header;

    msg_remap.pose.orientation = _msg.transform.rotation;
    msg_remap.pose.position.x = _msg.transform.translation.x;
    msg_remap.pose.position.y = _msg.transform.translation.y;
    msg_remap.pose.position.z = _msg.transform.translation.z - _offset_z;

    return msg_remap;
}

geometry_msgs::PoseWithCovarianceStamped ViconRemap::poseToPoseWithCovarianceStampedMsg()
{
    geometry_msgs::PoseWithCovarianceStamped msg_remap;
    msg_remap.header = _msg.header;

    msg_remap.pose.pose.orientation = _msg.transform.rotation;
    msg_remap.pose.pose.position.x = _msg.transform.translation.x;
    msg_remap.pose.pose.position.y = _msg.transform.translation.y;
    msg_remap.pose.pose.position.z = _msg.transform.translation.z - _offset_z;

    // Identity covariance for now
    msg_remap.pose.covariance[0] = 1.;
    msg_remap.pose.covariance[7] = 1;
    msg_remap.pose.covariance[14] = 1;
    msg_remap.pose.covariance[21] = 1;
    msg_remap.pose.covariance[28] = 1;
    msg_remap.pose.covariance[35] = 1;

    return msg_remap;
}

void ViconRemap::callback_pose(const geometry_msgs::TransformStamped &msg)
{
    if (_first_frame)
    {
        _offset_z = msg.transform.translation.z;
        _first_frame = false;
        ROS_INFO_STREAM("data reset for z-axis offset :" << _offset_z);
    }
    _msg = msg;
}
