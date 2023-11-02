/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <string>
#include <vector>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include "gazebo_ocs2_ros_interfaces/common/RosMsgConversions.h"

namespace ocs2 {

/**
 * This class provides a ROS publisher for the TargetTrajectories.
 */
class TargetTrajectoriesRosPublisher final {
 public:
  /**
   * Constructor.
   * @param [in] nodeHandle: ROS node handle.
   * @param [in] topicPrefix: The TargetTrajectories will be published on "topicPrefix_mpc_$(targetFrame)_target" topic.
   * @param [in] targetFrame: The frame to which TargetTrajectories refer.
   */
  TargetTrajectoriesRosPublisher(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix = "anonymousRobot",
                                 const std::string& targetFrame = "");

  /** Destructor. */
  ~TargetTrajectoriesRosPublisher();

  /** Publishes the target trajectories. */
  void publishTargetTrajectories(const TargetTrajectories& targetTrajectories);

 private:
  ::ros::Publisher targetTrajectoriesPublisher_;
};

}  // namespace ocs2
