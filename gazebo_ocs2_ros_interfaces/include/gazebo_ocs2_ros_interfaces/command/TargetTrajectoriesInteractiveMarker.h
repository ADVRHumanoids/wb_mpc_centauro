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

#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <ocs2_mpc/SystemObservation.h>
#include <gazebo_ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace ocs2 {

/**
 * This class lets the user to command robot form interactive marker.
 */
class TargetTrajectoriesInteractiveMarker final {
 public:
  using GaolPoseToTargetTrajectories = std::function<TargetTrajectories(
      const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const SystemObservation& observation)>;

  /**
   * Constructor
   *
   * @param [in] nodeHandle: ROS node handle.
   * @param [in] topicPrefix: The TargetTrajectories will be published on "topicPrefix_mpc_target" topic. Moreover, the latest
   * observation is be expected on "topicPrefix_mpc_observation" topic.
   * @param [in] gaolPoseToTargetTrajectories: A function which transforms the commanded pose to TargetTrajectories.
   * @param [in] targetFrame: the frame to which the targetTrajectories refer.

   */
  TargetTrajectoriesInteractiveMarker(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                      GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories,
                                      const std::string& targetFrame = "");
  /**
   * Spins ROS to update the interactive markers.
   */
  void publishInteractiveMarker() { ::ros::spin(); }

 private:
  visualization_msgs::InteractiveMarker createInteractiveMarker() const;
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  interactive_markers::MenuHandler menuHandler_;
  interactive_markers::InteractiveMarkerServer server_;

  GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;

  ::ros::Subscriber observationSubscriber_;
  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;

  // the frame to which the targetTrajectories refer, "" if they do not refe to a frame.
  const std::string targetFrame_;
  std::string globalFrame_;
};

}  // namespace ocs2
