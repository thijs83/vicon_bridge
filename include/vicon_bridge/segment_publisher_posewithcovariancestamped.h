#ifndef VICON_BRIDGE_SEGMENT_PUBLISHER_POSEWITHCOVARIANCESTAMPED_H_
#define VICON_BRIDGE_SEGMENT_PUBLISHER_POSEWITHCOVARIANCESTAMPED_H_

#include <vicon_bridge/segment_publisher.h>

#ifdef ROS_VERSION1
  #include <geometry_msgs/PoseWithCovarianceStamped.h>
  typedef geometry_msgs::PoseWithCovarianceStamped GEOMETRY_MSG_POSEWITHCOVARIANCESTAMPED;
#endif
#ifdef ROS_VERSION2
  #include <geometry_msgs/msg/PoseWithCovarianceStamped.hpp>
  typedef geometry_msgs::msg::PoseWithCovarianceStamped GEOMETRY_MSG_POSEWITHCOVARIANCESTAMPED;
#endif

class SegmentPublisherPosewithcovarianceStamped: public SegmentPublisher
{
private:
  // Convention class to both allow for ROS1 and ROS2
  RosPublisher<GEOMETRY_MSG_POSEWITHCOVARIANCESTAMPED> pub_;

public:

  SegmentPublisherPosewithcovarianceStamped(NodeHandler& node_handler, std::string frame_id, std::string publish_topic, int frequency_divider, double z_axis_offset):
    SegmentPublisher(frame_id, publish_topic, frequency_divider, z_axis_offset),
    pub_(node_handler.createPublisher<GEOMETRY_MSG_POSEWITHCOVARIANCESTAMPED>(publish_topic, 1))
  {}

  void publishMsg(const ROS_TIME frame_time, const double position[3], const double rotation[4]) override
  {
     if (this->discardDueToLowerRate()){return;};

    GEOMETRY_MSG_POSEWITHCOVARIANCESTAMPED pose_with_covariance_stamped;

    pose_with_covariance_stamped.header.stamp = frame_time;
    pose_with_covariance_stamped.header.frame_id = frame_id_;
    pose_with_covariance_stamped.pose.pose.position.x = position[0];
    pose_with_covariance_stamped.pose.pose.position.y = position[1];
    pose_with_covariance_stamped.pose.pose.position.z = position[2]-z_axis_offset_;
    pose_with_covariance_stamped.pose.pose.orientation.x = rotation[0];
    pose_with_covariance_stamped.pose.pose.orientation.y = rotation[1];
    pose_with_covariance_stamped.pose.pose.orientation.z = rotation[2];
    pose_with_covariance_stamped.pose.pose.orientation.w = rotation[3];

    this->pub_.publishNow(pose_with_covariance_stamped);
    
  }

};

#endif /* VICON_BRIDGE_SEGMENT_PUBLISHER_POSEWITHCOVARIANCESTAMPED_H_ */
