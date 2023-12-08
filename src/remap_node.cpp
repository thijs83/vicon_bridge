
#include <vicon_bridge/vicon_remap.h>
#include <ros/ros.h>

int main(int argc, char **argv) {

    
    //double frequency = 40;
    std::string arg;
    if (argc > 1) {arg = argv[1];};

    ros::init(argc, argv, "vicon_remap"+arg);

    auto nh = std::make_shared<ros::NodeHandle>("~");

    double frequency;
    if(nh->hasParam("remap/frequency"))
    {
        nh->getParam("remap/frequency", frequency);
        ROS_INFO_STREAM("[VICON] Frequency is set to " << frequency);
    } else {
        ROS_ERROR_STREAM("[VICON] No remap value specified in the config");
    }
    
    std::string topic_name_subscriber;
    std::string topic_name_publisher;

    if(nh->hasParam("remap/topic_name_subscriber"))
    {
        nh->getParam("remap/topic_name_subscriber", topic_name_subscriber);
        ROS_INFO_STREAM("[VICON] Topic name is set to " << topic_name_subscriber);
    } else {
        ROS_ERROR_STREAM("[VICON] No topic name specified in the config");
    }
    if(nh->hasParam("remap/topic_name_publisher"))
    {
        nh->getParam("remap/topic_name_publisher", topic_name_publisher);
        ROS_INFO_STREAM("[VICON] Topic name is set to " << topic_name_publisher);
    } else {
        ROS_ERROR_STREAM("[VICON] No topic name specified in the config");
    }


    ViconRemap remapper(nh,frequency,topic_name_subscriber, topic_name_publisher);

    remapper.setup();

    return 0;
}