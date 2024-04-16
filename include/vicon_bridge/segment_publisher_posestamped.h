#ifndef VICON_BRIDGE_SEGMENT_PUBLISHER_POSESTAMPED_H_
#define VICON_BRIDGE_SEGMENT_PUBLISHER_POSESTAMPED_H_

#include <vicon_bridge/segment_publisher.h>

#ifdef ROS_VERSION1
  #include <geometry_msgs/PoseStamped.h>
  typedef geometry_msgs::PoseStamped GEOMETRY_MSG_POSESTAMPED;
#endif
#ifdef ROS_VERSION2
  #include "geometry_msgs/msg/pose_stamped.hpp"
  typedef geometry_msgs::msg::PoseStamped GEOMETRY_MSG_POSESTAMPED;
#endif


class SegmentPublisherPoseStamped: public SegmentPublisher
{
private:
  // Convention class to both allow for ROS1 and ROS2
  RosPublisher<GEOMETRY_MSG_POSESTAMPED> pub_;

public:

  SegmentPublisherPoseStamped(NodeHandler& node_handler, std::string frame_id, std::string publish_topic, int frequency_divider, double z_axis_offset):
    SegmentPublisher(frame_id, publish_topic, frequency_divider, z_axis_offset),
    pub_(node_handler.createPublisher<GEOMETRY_MSG_POSESTAMPED>(publish_topic, 1))
  {};

  void publishMsg(const ROS_TIME frame_time, const double position[3], const double rotation[4]) override
  {
    if (this->discardDueToLowerRate()){return;};
    

    GEOMETRY_MSG_POSESTAMPED pose_stamped;

    pose_stamped.header.stamp = frame_time;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.pose.position.x = position[0];
    pose_stamped.pose.position.y = position[1];
    pose_stamped.pose.position.z = position[2]-z_axis_offset_;
    pose_stamped.pose.orientation.x = rotation[0];
    pose_stamped.pose.orientation.y = rotation[1];
    pose_stamped.pose.orientation.z = rotation[2];
    pose_stamped.pose.orientation.w = rotation[3];

    this->pub_.publishNow(pose_stamped);
  }

};

#endif /* VICON_BRIDGE_SEGMENT_PUBLISHER_POSESTAMPED_H_ */