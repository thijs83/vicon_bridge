#ifndef VICON_BRIDGE_SEGMENT_PUBLISHER_H_
#define VICON_BRIDGE_SEGMENT_PUBLISHER_H_

#include <string>

#include <vicon_bridge/node_handler_ros.h>
#include <vicon_bridge/ros_definitions.h>

class SegmentPublisher
{
public:

  std::string frame_id_ = "world";

  // Variables for publishing at a lower rate
  int counter = 0;
  const int publish_on_count = 0;

  // If we want to reset the z-axis
  double z_axis_offset_ = 0.0;


  SegmentPublisher(const std::string frame_id, const std::string publish_topic, const int frequency_divider, const double z_axis_offset):
    frame_id_(frame_id),
    publish_on_count(frequency_divider),
    z_axis_offset_(z_axis_offset)
  {};

  ~SegmentPublisher(){};

  bool discardDueToLowerRate()
  {
    counter++;

    if (counter < publish_on_count)
    {
      return true;
    }
    counter = 0;
    return false;
  }

  //virtual void setMsg(ros::NodeHandle nh, std::string publish_topic)=0;

  virtual void publishMsg(const ROS_TIME frame_time, const double position[3], const double rotation[4])=0;

};

#endif /* VICON_BRIDGE_SEGMENT_PUBLISHER_H_ */