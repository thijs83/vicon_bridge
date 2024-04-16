#ifndef VICON_BRIDGE_VICON_BRIDGE_H
#define VICON_BRIDGE_VICON_BRIDGE_H

#include <string>
#include <vector>
#include <array>


#include <vicon_bridge/segment_publisher.h>

// Vicon SDK
#include <ViconDataStreamSDK_CPP/DataStreamClient.h>

#include <vicon_bridge/ros_definitions.h>
#include <vicon_bridge/node_handler_ros.h>

using namespace ViconDataStreamSDK::CPP;

std::string Adapt(const Direction::Enum i_Direction)
{
  switch (i_Direction)
  {
    case Direction::Forward:
      return "Forward";
    case Direction::Backward:
      return "Backward";
    case Direction::Left:
      return "Left";
    case Direction::Right:
      return "Right";
    case Direction::Up:
      return "Up";
    case Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

std::string Adapt(const Result::Enum i_result)
{
  switch (i_result)
  {
    case Result::ClientAlreadyConnected:
      return "ClientAlreadyConnected";
    case Result::ClientConnectionFailed:
      return "";
    case Result::CoLinearAxes:
      return "CoLinearAxes";
    case Result::InvalidDeviceName:
      return "InvalidDeviceName";
    case Result::InvalidDeviceOutputName:
      return "InvalidDeviceOutputName";
    case Result::InvalidHostName:
      return "InvalidHostName";
    case Result::InvalidIndex:
      return "InvalidIndex";
    case Result::InvalidLatencySampleName:
      return "InvalidLatencySampleName";
    case Result::InvalidMarkerName:
      return "InvalidMarkerName";
    case Result::InvalidMulticastIP:
      return "InvalidMulticastIP";
    case Result::InvalidSegmentName:
      return "InvalidSegmentName";
    case Result::InvalidSubjectName:
      return "InvalidSubjectName";
    case Result::LeftHandedAxes:
      return "LeftHandedAxes";
    case Result::NoFrame:
      return "NoFrame";
    case Result::NotConnected:
      return "NotConnected";
    case Result::NotImplemented:
      return "NotImplemented";
    case Result::ServerAlreadyTransmittingMulticast:
      return "ServerAlreadyTransmittingMulticast";
    case Result::ServerNotTransmittingMulticast:
      return "ServerNotTransmittingMulticast";
    case Result::Success:
      return "Success";
    case Result::Unknown:
      return "Unknown";
    default:
      return "unknown";
  }
}

class ViconReceiver : public NodeHandler
{
private:
  // Parameters:
  std::string tracked_frame_suffix_;
  std::string frame_id_all_ = "map";
  std::string msg_type_all_ = "geometry_msgs/PoseStamped";
  int frequency_divider_all_ = 1;


  unsigned int lastFrameNumber_ = 0;
  unsigned int frameCount_ = 0;
  unsigned int droppedFrameCount_ = 0;
  unsigned int n_markers_ = 0;
  unsigned int n_unlabeled_markers_ = 0;
  bool marker_data_enabled = false;
  bool unlabeled_marker_data_enabled = false;

  bool publish_segments_ = true;
  bool broadcast_tf_ = false;
  bool publish_tf_ = false;
  bool publish_markers_ = false;
  bool object_specific_only_ = false;
  bool reset_z_axis_ = false;

  std::map<std::string, std::unique_ptr<SegmentPublisher>> segment_publishers_;

  std::map<std::string, std::array<std::string, 4>> object_specific_details_;

  std::vector<std::string> time_log_;

  Client vicon_client_;

public:

  ViconReceiver();
  ~ViconReceiver();

private:
  bool init_vicon(std::string host_name, std::string stream_mode);

  void process_frame();

  void process_subjects(const ROS_TIME& frame_time);

  void process_markers(const ROS_TIME& frame_time, unsigned int vicon_frame_num);
};

#endif // VICON_BRIDGE_VICON_BRIDGE_H