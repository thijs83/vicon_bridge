/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  Copyright (c) 2011, Markus Achtelik, ETH Zurich, Autonomous Systems Lab (modifications)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <vicon_bridge/vicon_bridge.h>


#include <iostream>
#include <map>
#include <unordered_map>

#include <vicon_bridge/segment_publisher_posestamped.h>
#include <vicon_bridge/segment_publisher_posewithcovariancestamped.h>
#include <vicon_bridge/segment_publisher_transformstamped.h>

#include <vicon_bridge/printer.h>

#include <vicon_bridge/ros_definitions.h>

ViconReceiver::ViconReceiver() :
    NodeHandler("vicon_bridge_node")
{
  // Setting up the vicon client connection
  std::string host_name = "";
  std::string stream_mode = "ClientPull";
  bool setup_grabpose = false;

  this->getParameter("stream_mode", stream_mode);
  this->getParameter("datastream_hostport", host_name);

  if (init_vicon(host_name, stream_mode) == false)
  {
    VICON_ERROR("Error while connecting to Vicon. Exiting now.");
    return;
  }

  this->getParameter("publish_transform", publish_tf_);
  this->getParameter("publish_markers", publish_markers_);
  this->getParameter("publish_segments", publish_segments_);
  this->getParameter("setup_grabpose", setup_grabpose);

  // // Parameters for the tracked objects
  this->getParameter("msg_type", msg_type_all_);
  this->getParameter("frame_id", frame_id_all_);
  this->getParameter("frequency_divider", frequency_divider_all_);
  this->getParameter("reset_z_axis", reset_z_axis_);

  vicon_client_.GetFrame();
  double client_framerate = vicon_client_.GetFrameRate().FrameRateHz;
  VICON_INFO("Vicon client framerate: %f", client_framerate);


  //check if msg type is correct
  if (!(msg_type_all_ == "geometry_msgs/PoseStamped" || msg_type_all_ == "geometry_msgs/PoseWithCovarianceStamped" || msg_type_all_ == "geometry_msgs/TransformStamped"))
  {
    VICON_ERROR("msg_type %s is not supported. Please use geometry_msgs/PoseStamped, geometry_msgs/PoseWithCovarianceStamped or geometry_msgs/TransformStamped", msg_type_all_.c_str());
    return;
  }

  // Parameters for tracking specific objects
  this->getParameter("only_use_object_specific", object_specific_only_);

  std::vector<std::string> object_names;
  std::vector<std::string> object_msg_types;
  std::vector<std::string> object_frame_ids;
  std::vector<std::string> object_publish_topics;
  std::vector<double> object_frequency_divider;
  this->getParameter("object_specific/object_names", object_names);
  this->getParameter("object_specific/object_msg_types", object_msg_types);
  this->getParameter("object_specific/object_frame_ids", object_frame_ids);
  this->getParameter("object_specific/object_publish_topics", object_publish_topics);
  this->getParameter("object_specific/object_publish_topics", object_publish_topics);
  this->getParameter("object_specific/object_frequency_divider", object_frequency_divider);


  // Check if the sizes of the vectors are equal
  if (!(object_names.size() == object_msg_types.size() && object_msg_types.size() == object_frame_ids.size() && object_frame_ids.size() == object_publish_topics.size() && object_publish_topics.size() == object_frequency_divider.size()))
  {
    VICON_ERROR("The sizes of the object_specific vectors are not equal. Please check the sizes of the vectors");
    return;
  }

  if (!object_names.size() == 0)
  {
    VICON_INFO("Found %d objects to determine specific settings", (int)object_names.size());

    for (int i = 0; i < object_names.size(); i++)
    {
      // Check if msg type is correct
      if (!(object_msg_types[i] == "geometry_msgs/PoseStamped" || object_msg_types[i] == "geometry_msgs/PoseWithCovarianceStamped" || object_msg_types[i] == "geometry_msgs/TransformStamped"))
      {
        VICON_ERROR("msg_type %s is not supported. Please use geometry_msgs/PoseStamped, geometry_msgs/PoseWithCovarianceStamped or geometry_msgs/TransformStamped", object_msg_types[i].c_str());
        return;
      }

      std::array<std::string, 4> object_details = {object_msg_types[i], object_frame_ids[i], object_publish_topics[i], std::to_string(object_frequency_divider[i])};

      object_specific_details_.insert(std::pair<std::string ,std::array<std::string, 4>>(object_names[i], object_details));

      int frequency = client_framerate / object_frequency_divider[i];
      int object_frequency_int = object_frequency_divider[i];

      VICON_INFO("Object %s: \n"
                "\t\t\t\t\t msg type: %s \n"
                "\t\t\t\t\t frame id: %s \n"
                "\t\t\t\t\t topic name: %s \n"
                "\t\t\t\t\t frequency divider: %i \n"
                "\t\t\t\t\t actual frequency: %i", object_names[i].c_str(), object_msg_types[i].c_str(), object_frame_ids[i].c_str(), object_publish_topics[i].c_str(), object_frequency_int, frequency);
    }
  }

  // Publishers
  if(publish_markers_)
  {
    createMarkerPublisher(tracked_frame_suffix_ + "/markers", 10);
  }
  
  ROS_RATE loop_rate(1);

  bool ros_ok = ROS_OK();
  while (ros_ok)
  {
    // Ask for a new frame or wait untill a new frame is available (BLOCKING)
    if (vicon_client_.GetFrame().Result == Result::Success)
    {
      process_frame();
    }
    else
    {
      // Vicon client needs to be disconnected since connection is lost
      vicon_client_.Disconnect();
      VICON_WARN("Vicon client connection lost. Waiting for connection to grab a new frame ...");
      while (!vicon_client_.IsConnected().Connected)
      {
        vicon_client_.Connect(host_name);
        VICON_INFO_STREAM(".");
        loop_rate.sleep();
      }
      VICON_INFO_STREAM("... connection re-established!");
    }

    ros_ok = ROS_OK();   
  }
}

ViconReceiver::~ViconReceiver()
{
  for (size_t i = 0; i < time_log_.size(); i++)
  {
    std::cout << time_log_[i] << std::endl;
  }

  VICON_INFO_STREAM("Disconnecting from Vicon DataStream SDK");

  if (vicon_client_.Disconnect().Result == Result::Success){
    VICON_INFO("Vicon connection shut down.");
  } else
  {
    VICON_ERROR("Error while shutting down Vicon.");
  }
  VICON_ASSERT(!vicon_client_.IsConnected().Connected, "Vicon connection still active! Hard shutdown.");
}


bool ViconReceiver::init_vicon(std::string host_name, std::string stream_mode)
{
  VICON_INFO_STREAM("Connecting to Vicon DataStream SDK at " << host_name << " ...");

  ROS_RATE d(1);
  Result::Enum result(Result::Unknown);

  while (!vicon_client_.IsConnected().Connected)
  {
    vicon_client_.Connect(host_name);
    VICON_INFO(".");
    d.sleep();
    bool ros_ok = ROS_OK();
    if (!ros_ok)
      return false;
  }
  VICON_ASSERT(vicon_client_.IsConnected().Connected, "Vicon connection failed!");
  VICON_INFO_STREAM("... connected!");

  // ClientPullPrefetch doesn't make much sense here, since we're only forwarding the data
  if (stream_mode == "ServerPush")
  {
    result = vicon_client_.SetStreamMode(StreamMode::ServerPush).Result;
  }
  else if (stream_mode == "ClientPull")
  {
    result = vicon_client_.SetStreamMode(StreamMode::ClientPull).Result;
  }
  else
  {
    VICON_ERROR("Unknown stream mode -- options are ServerPush, ClientPull");
    ROS_SHUTDOWN();
  }

  VICON_INFO_STREAM("Setting Stream Mode to " << stream_mode<< ": "<< Adapt(result));

  vicon_client_.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up); // 'Z-up'
  Output_GetAxisMapping _Output_GetAxisMapping = vicon_client_.GetAxisMapping();

  VICON_INFO_STREAM("Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis) << " Y-"
      << Adapt(_Output_GetAxisMapping.YAxis) << " Z-" << Adapt(_Output_GetAxisMapping.ZAxis));

  vicon_client_.EnableSegmentData();
  VICON_ASSERT(vicon_client_.IsSegmentDataEnabled().Enabled, "SegmentData not enabled");

  Output_GetVersion _Output_GetVersion = vicon_client_.GetVersion();
  VICON_INFO_STREAM("Version: " << _Output_GetVersion.Major << "." << _Output_GetVersion.Minor << "."
      << _Output_GetVersion.Point);
  return true;
}


void ViconReceiver::process_frame()
{
  Output_GetFrameNumber OutputFrameNum = vicon_client_.GetFrameNumber();

  int frameDiff = OutputFrameNum.FrameNumber - lastFrameNumber_;
  lastFrameNumber_ = OutputFrameNum.FrameNumber;
  if (frameDiff == 0)
  {
    VICON_WARN_STREAM("Frame number unchanged (" << OutputFrameNum.FrameNumber << "). Skipping frame.");
    return;
  }

  ROS_DURATION  vicon_latency(vicon_client_.GetLatencyTotal().Total);
  if(publish_segments_)
  {
    ROS_TIME time = GET_ROS_CLOCK();
    process_subjects( time - vicon_latency);
  }

  // TODO: function below is not yet checked
  if(publish_markers_)
  {
    ROS_TIME time = GET_ROS_CLOCK();
    process_markers( time - vicon_latency, lastFrameNumber_);
  }

  frameCount_ += frameDiff;
  if ((frameDiff) > 1)
  {
    droppedFrameCount_ += frameDiff - 1;
    double droppedFramePct = (double)droppedFrameCount_ / frameCount_ * 100;
    VICON_DEBUG_STREAM(frameDiff << " more (total " << droppedFrameCount_ << "/" << frameCount_ << ", "
        << droppedFramePct << "%) frame(s) dropped. Consider adjusting rates.");
  }
}


void ViconReceiver::process_subjects(const ROS_TIME& frame_time)
{
  std::vector<GEOMETRY_MSG_TRANSFORMSTAMPED> transforms;
  static unsigned int cnt = 0;

  unsigned int n_subjects = vicon_client_.GetSubjectCount().SubjectCount;

  for (unsigned int i_subjects = 0; i_subjects < n_subjects; i_subjects++)
  {

    std::string subject_name = vicon_client_.GetSubjectName(i_subjects).SubjectName;

    unsigned int n_segments = vicon_client_.GetSegmentCount(subject_name).SegmentCount;
    for (unsigned int i_segments = 0; i_segments < n_segments; i_segments++)
    {
      std::string segment_name = vicon_client_.GetSegmentName(subject_name, i_segments).SegmentName;

      std::string name;
      if (n_segments == 1)
      {
        name = subject_name;
      }
      else
      {
        name = subject_name + "/" + segment_name;
      }

      // Check if we have a publisher for this segment
      std::map<std::string, std::unique_ptr<SegmentPublisher>>::iterator pub_it = segment_publishers_.find(name);

      // If it is not in the map, create a new one
      // Only if we want to make a new one
      if (pub_it == segment_publishers_.end())
      {
        

        // Check if the name of the object exists in the specific details
        std::map<std::string, std::array<std::string, 4>>::iterator object_specific_details_it = object_specific_details_.find(name);

        if (!object_specific_only_ && object_specific_details_it == object_specific_details_.end())
        {
          double z_axis_offset = 0.0;
          if (reset_z_axis_)
          {
            Output_GetSegmentGlobalTranslation trans = vicon_client_.GetSegmentGlobalTranslation(subject_name, segment_name);
            if (trans.Occluded)
            {
              VICON_WARN_STREAM(name <<" occluded during initialisation, not resetting the z-axis! " );
            } else
            {
              z_axis_offset = trans.Translation[2] / 1000;
            }
          }

          if (msg_type_all_ == "geometry_msgs/PoseStamped")
          {
            segment_publishers_.insert(std::make_pair(name, new SegmentPublisherPoseStamped(*this, frame_id_all_, name, frequency_divider_all_, z_axis_offset)));
          } else if (msg_type_all_ == "geometry_msgs/PoseWithCovarianceStamped")
          {
            segment_publishers_.insert(std::make_pair(name, new SegmentPublisherPosewithcovarianceStamped(*this, frame_id_all_, name, frequency_divider_all_, z_axis_offset)));
          } else if (msg_type_all_ == "geometry_msgs/TransformStamped")
          {
            segment_publishers_.insert(std::make_pair(name, new SegmentPublisherTransformStamped(*this, frame_id_all_, name, frequency_divider_all_, z_axis_offset)));
          }

        } else if (!(object_specific_details_it == object_specific_details_.end()))
        {
          double z_axis_offset = 0.0;
          if (reset_z_axis_)
          {
            Output_GetSegmentGlobalTranslation trans = vicon_client_.GetSegmentGlobalTranslation(subject_name, segment_name);
            if (trans.Occluded)
            {
              VICON_WARN_STREAM(name <<" occluded during initialisation, not resetting the z-axis! " );
            } else
            {
              z_axis_offset = trans.Translation[2] / 1000;
            }
          }

          std::string msg_type = object_specific_details_it->second[0];
          std::string frame_id = object_specific_details_it->second[1];
          std::string publish_topic = object_specific_details_it->second[2];
          int frequency_divider = std::stoi(object_specific_details_it->second[3]);

          // Use specific option
          if (msg_type == "geometry_msgs/PoseStamped")
          {
            segment_publishers_.insert(std::make_pair(name, new SegmentPublisherPoseStamped(*this, frame_id, publish_topic, frequency_divider, z_axis_offset)));
          } else if (msg_type == "geometry_msgs/PoseWithCovarianceStamped")
          {
            segment_publishers_.insert(std::make_pair(name, new SegmentPublisherPosewithcovarianceStamped(*this, frame_id, publish_topic, frequency_divider, z_axis_offset)));
          } else if (msg_type == "geometry_msgs/TransformStamped")
          {
            segment_publishers_.insert(std::make_pair(name, new SegmentPublisherTransformStamped(*this, frame_id, publish_topic, frequency_divider, z_axis_offset)));
          }

          object_specific_details_.erase(object_specific_details_it);

        } else
        {
          continue;
        }
        

        VICON_INFO("creating new object %s/%s ...",subject_name.c_str(), segment_name.c_str() );
        VICON_INFO("... done, advertised as  %s", name.c_str());

        continue;
      }

      Output_GetSegmentGlobalTranslation trans = vicon_client_.GetSegmentGlobalTranslation(subject_name, segment_name);
      Output_GetSegmentGlobalRotationQuaternion quat = vicon_client_.GetSegmentGlobalRotationQuaternion(subject_name,
                                                                                                      segment_name);

      if (!trans.Result == Result::Success || !quat.Result == Result::Success)
      {
        VICON_WARN("GetSegmentGlobalTranslation/Rotation failed (result = %s, %s), not publishing ...",
            Adapt(trans.Result).c_str(), Adapt(quat.Result).c_str());
        continue;
      }

      if (trans.Occluded || quat.Occluded)
      {
        if (cnt % 100 == 0)
            VICON_WARN_STREAM("" << name <<" occluded, not publishing... " );
        continue;
      }

      if (publish_tf_)
      {
        GEOMETRY_MSG_TRANSFORMSTAMPED transform;
        transform.header.stamp = frame_time;
        transform.header.frame_id = frame_id_all_;
        transform.child_frame_id = name;

        transform.transform.translation.x = trans.Translation[0] / 1000;
        transform.transform.translation.y = trans.Translation[1] / 1000;
        transform.transform.translation.z = trans.Translation[2] / 1000;
        transform.transform.rotation.x = quat.Rotation[0];
        transform.transform.rotation.y = quat.Rotation[1];
        transform.transform.rotation.z = quat.Rotation[2];
        transform.transform.rotation.w = quat.Rotation[3];

        transforms.push_back(transform);
      }

      double translation[3] = {trans.Translation[0] / 1000, trans.Translation[1] / 1000,
                                              trans.Translation[2] / 1000};
      double rotation[4] = {quat.Rotation[0], quat.Rotation[1], quat.Rotation[2],
                                                        quat.Rotation[3]};
      
      pub_it->second->publishMsg(frame_time, translation, rotation);
    }
  }

  if(publish_tf_)
  {
    this->tf_broadcaster_.sendTransform(transforms);
  }
  cnt++;
}

void ViconReceiver::process_markers(const ROS_TIME& frame_time, unsigned int vicon_frame_num)
{
  if (this->subscribersCount() > 0)
  {
    if (!marker_data_enabled)
    {
      vicon_client_.EnableMarkerData();
      VICON_ASSERT(vicon_client_.IsMarkerDataEnabled().Enabled, "MarkerData not enabled");
      marker_data_enabled = true;
    }
    if (!unlabeled_marker_data_enabled)
    {
      vicon_client_.EnableUnlabeledMarkerData();
      VICON_ASSERT(vicon_client_.IsUnlabeledMarkerDataEnabled().Enabled, "UnlabeledMarkerData not enabled");
      unlabeled_marker_data_enabled = true;
    }
    n_markers_ = 0;
    VICON_BRIDGE_MARKERS markers_msg;
    markers_msg.header.stamp = frame_time;
    markers_msg.frame_number = vicon_frame_num;
    // Count the number of subjects
    unsigned int SubjectCount = vicon_client_.GetSubjectCount().SubjectCount;
    // Get labeled markers
    for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
    {
      std::string this_subject_name = vicon_client_.GetSubjectName(SubjectIndex).SubjectName;
      // Count the number of markers
      unsigned int num_subject_markers = vicon_client_.GetMarkerCount(this_subject_name).MarkerCount;
      n_markers_ += num_subject_markers;
      //std::cout << "    Markers (" << MarkerCount << "):" << std::endl;
      for (unsigned int MarkerIndex = 0; MarkerIndex < num_subject_markers; ++MarkerIndex)
      {
        VICON_BRIDGE_MARKER this_marker;
        this_marker.marker_name = vicon_client_.GetMarkerName(this_subject_name, MarkerIndex).MarkerName;
        this_marker.subject_name = this_subject_name;
        this_marker.segment_name
            = vicon_client_.GetMarkerParentName(this_subject_name, this_marker.marker_name).SegmentName;

        // Get the global marker translation
        Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
            vicon_client_.GetMarkerGlobalTranslation(this_subject_name, this_marker.marker_name);

        this_marker.translation.x = _Output_GetMarkerGlobalTranslation.Translation[0];
        this_marker.translation.y = _Output_GetMarkerGlobalTranslation.Translation[1];
        this_marker.translation.z = _Output_GetMarkerGlobalTranslation.Translation[2];
        this_marker.occluded = _Output_GetMarkerGlobalTranslation.Occluded;

        markers_msg.markers.push_back(this_marker);
      }
    }
    // get unlabeled markers
    unsigned int UnlabeledMarkerCount = vicon_client_.GetUnlabeledMarkerCount().MarkerCount;
    //VICON_INFO("# unlabeled markers: %d", UnlabeledMarkerCount);
    n_markers_ += UnlabeledMarkerCount;
    n_unlabeled_markers_ = UnlabeledMarkerCount;
    for (unsigned int UnlabeledMarkerIndex = 0; UnlabeledMarkerIndex < UnlabeledMarkerCount; ++UnlabeledMarkerIndex)
    {
      // Get the global marker translation
      Output_GetUnlabeledMarkerGlobalTranslation _Output_GetUnlabeledMarkerGlobalTranslation =
          vicon_client_.GetUnlabeledMarkerGlobalTranslation(UnlabeledMarkerIndex);

      if (_Output_GetUnlabeledMarkerGlobalTranslation.Result == Result::Success)
      {
        VICON_BRIDGE_MARKER this_marker;
        this_marker.translation.x = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[0];
        this_marker.translation.y = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[1];
        this_marker.translation.z = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[2];
        this_marker.occluded = false; // unlabeled markers can't be occluded
        markers_msg.markers.push_back(this_marker);
      }
      else
      {
        VICON_WARN("GetUnlabeledMarkerGlobalTranslation failed (result = %s)",
            Adapt(_Output_GetUnlabeledMarkerGlobalTranslation.Result).c_str());
      }
    }
    this->pub_marker_->publish(markers_msg);
  }
}



int main(int argc, char** argv)
{

#ifdef ROS_VERSION1
  ros::init(argc, argv, "vicon");
  ros::AsyncSpinner aspin(1);
  aspin.start();
  ViconReceiver vr;
  aspin.stop();
#endif

#ifdef ROS_VERSION2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViconReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
#endif

  return 0;
}
