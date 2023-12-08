
#include <vicon_bridge/vicon_remap.h>

using namespace std::chrono;


ViconRemap::ViconRemap(std::shared_ptr<ros::NodeHandle> nh, const int frequency, const std::string topic_name_subscriber, const std::string topic_name_publisher) 
: _nh(nh), _loop_rate(frequency) {

    _pub_remap = _nh->advertise<geometry_msgs::PoseStamped>(topic_name_publisher, 10);
    _sub_pose = _nh->subscribe(topic_name_subscriber, 100, &ViconRemap::callback_pose, this);
}

void ViconRemap::setup()
{
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while(ros::ok())
    {
        geometry_msgs::PoseStamped msg_remap;
        msg_remap.header = _msg.header;
        //msg_remap.header.seq = _msg.header.seq;
        //msg_remap.header.frame_id = "";
        //msg_remap.header.stamp = _msg.header.stamp;
        msg_remap.pose.orientation = _msg.transform.rotation;
        msg_remap.pose.position.x = _msg.transform.translation.x;
        msg_remap.pose.position.y = _msg.transform.translation.y;
        msg_remap.pose.position.z = _msg.transform.translation.z - _offset_z;

        _pub_remap.publish(msg_remap);

        ros::spinOnce;

        _loop_rate.sleep();
    }
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