/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "gazebo_ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h"

#include <ocs2_msgs/mpc_observation.h>
#include <gazebo_ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesInteractiveMarker::TargetTrajectoriesInteractiveMarker(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                                                         GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories,
                                                                         const std::string& targetFrame)
    : server_("simple_marker_" + targetFrame), gaolPoseToTargetTrajectories_(std::move(gaolPoseToTargetTrajectories)),
      targetFrame_(targetFrame){

  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

  // receive global frame for visualization
  nodeHandle.getParam("/global_frame", globalFrame_);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix, targetFrame_));

  // create an interactive marker for our server
  menuHandler_.insert("Send target pose", boost::bind(&TargetTrajectoriesInteractiveMarker::processFeedback, this, _1));

  // create an interactive marker for our server
  auto interactiveMarker = createInteractiveMarker();

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server_.insert(interactiveMarker);  //, boost::bind(&TargetTrajectoriesInteractiveMarker::processFeedback, this, _1));
  menuHandler_.apply(server_, interactiveMarker.name);

  // 'commit' changes and send to all clients
  server_.applyChanges();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
visualization_msgs::InteractiveMarker TargetTrajectoriesInteractiveMarker::createInteractiveMarker() const {
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = globalFrame_;
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "Goal";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.z = 5.0;

  //TODO: modify to enter the appropriate initial pose of the corresponding targeFrame_
  if (targetFrame_ != "") {
      interactiveMarker.pose.position.x = 0.488;
      interactiveMarker.pose.position.y = 0.170;
      interactiveMarker.pose.position.z = 1.033;
      interactiveMarker.pose.orientation.x = 0.081;
      interactiveMarker.pose.orientation.y = 0.942;
      interactiveMarker.pose.orientation.z = 0.239;
      interactiveMarker.pose.orientation.w = -0.221;
  }

  // create a grey box marker
  const auto boxMarker = []() {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  return interactiveMarker;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesInteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  // Desired state trajectory
  const Eigen::Vector3d position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  const Eigen::Quaterniond orientation(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                                       feedback->pose.orientation.z);

  // get the latest observation
  SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    observation = latestObservation_;
  }

  // get TargetTrajectories
  const auto targetTrajectories = gaolPoseToTargetTrajectories_(position, orientation, observation);

  // publish TargetTrajectories
  targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
}

}  // namespace ocs2
