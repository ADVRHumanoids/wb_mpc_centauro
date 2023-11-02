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

Copyright (c) 2023, Ioannis Dadiotis <ioannis.dadiotis@iit.it>. All rights reserved.
Additional modifications and contributions by Ioannis Dadiotis:
- framesTargetTrajectoriesSubscriber_
******************************************************************************/

#include "gazebo_ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h"

#include "gazebo_ocs2_ros_interfaces/common/RosMsgConversions.h"

#include <ros/transport_hints.h>

// MPC messages
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_target_trajectories.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RosReferenceManager::RosReferenceManager(std::string topicPrefix, std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
    : ReferenceManagerDecorator(std::move(referenceManagerPtr)), topicPrefix_(std::move(topicPrefix)),
      referenceGeneratorPtr_(nullptr) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RosReferenceManager::subscribe(ros::NodeHandle& nodeHandle, const std::vector<std::string>& targetFrameNames) {
  // TargetTrajectories
  auto targetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) {
    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
    referenceManagerPtr_->setTargetTrajectories(std::move(targetTrajectories));
  };
  targetTrajectoriesSubscriber_ =
      nodeHandle.subscribe<ocs2_msgs::mpc_target_trajectories>(topicPrefix_ + "_mpc_target", 1, targetTrajectoriesCallback);

  // Subscribers for TargetTrajectories of TargetFrames
  auto frameTargetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg, int i) {
    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
    referenceManagerPtr_->setFrameTargetTrajectories(std::move(targetTrajectories), i);
  };
  // initialize as many subscribers as the target frames
  if (targetFrameNames.size() > 0) {
      framesTargetTrajectoriesSubscriber_.reserve(targetFrameNames.size());
      for (int i = 0; i < targetFrameNames.size(); i++) {
          const std::string& frameName = targetFrameNames[i];
          framesTargetTrajectoriesSubscriber_.push_back(
              nodeHandle.subscribe<ocs2_msgs::mpc_target_trajectories>(topicPrefix_ + "_mpc_" + frameName + "target", 1,
                                                                       std::bind(frameTargetTrajectoriesCallback,
                                                                                 std::placeholders::_1, i)
                                                                       ));
      }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RosReferenceManager::setReferenceGenerator(const force_control::AdmittanceReferenceGenerator& generator) {
    referenceGeneratorPtr_.reset(generator.clone());
}

}  // namespace ocs2
