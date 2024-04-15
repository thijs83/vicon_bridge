#ifndef VICON_BRIDGE_SEGMENT_PUBLISHER_TRANSFORMSTAMPED_H_
#define VICON_BRIDGE_SEGMENT_PUBLISHER_TRANSFORMSTAMPED_H_

#include <vicon_bridge/segment_publisher.h>

#ifdef ROS_VERSION1
  #include <geometry_msgs/TransformStamped.h>
  typedef geometry_msgs::TransformStamped GEOMETRY_MSG_TRANSFORMSTAMPED;
#endif
#ifdef ROS_VERSION2
  #include <geometry_msgs/msg/transform_stamped.hpp>
  typedef geometry_msgs::msg::TransformStamped GEOMETRY_MSG_TRANSFORMSTAMPED;
#endif



class SegmentPublisherTransformStamped: public SegmentPublisher
{
private:
  // Convention class to both allow for ROS1 and ROS2
  RosPublisher<GEOMETRY_MSG_TRANSFORMSTAMPED> pub_;

public:

  SegmentPublisherTransformStamped(NodeHandler& node_handler, std::string frame_id, std::string publish_topic, int frequency_divider, double z_axis_offset):
    SegmentPublisher(frame_id, publish_topic, frequency_divider, z_axis_offset),
    pub_(node_handler.createPublisher<GEOMETRY_MSG_TRANSFORMSTAMPED>(publish_topic, 1))
  {}

  void publishMsg(const ROS_TIME frame_time, const double position[3], const double rotation[4]) override
  {
     if (this->discardDueToLowerRate()){return;};

    GEOMETRY_MSG_TRANSFORMSTAMPED transform_stamped;

    transform_stamped.header.stamp = frame_time;
    transform_stamped.header.frame_id = frame_id_;
    transform_stamped.transform.translation.x = position[0];
    transform_stamped.transform.translation.y = position[1];
    transform_stamped.transform.translation.z = position[2]-z_axis_offset_;
    transform_stamped.transform.rotation.x = rotation[0];
    transform_stamped.transform.rotation.y = rotation[1];
    transform_stamped.transform.rotation.z = rotation[2];
    transform_stamped.transform.rotation.w = rotation[3];

    this->pub_.publishNow(transform_stamped);
  }

};

#endif /* VICON_BRIDGE_SEGMENT_PUBLISHER_TRANSFORMSTAMPED_H_ */