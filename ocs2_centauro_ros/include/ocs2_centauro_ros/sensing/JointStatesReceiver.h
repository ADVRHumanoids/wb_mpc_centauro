/******************************************************************************
Copyright (c) 2023, Ioannis Dadiotis <ioannis.dadiotis@iit.it>. All rights reserved.

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

#include <mutex>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <xbot_msgs/JointState.h>
#include <eigen_conversions/eigen_msg.h>
#include <ocs2_centauro/sensing/SensingDefinitions.h>
#include <ocs2_centauro/sensing/ForceTorqueSensing.h>

namespace ocs2 {
namespace legged_robot {

class JointStatesReceiver : public SolverSynchronizedModule {
 public:
  JointStatesReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<ForceTorqueSensing> forceTorqueSensingPtr);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override {};

 private:
  void jointStatesCallback(const xbot_msgs::JointStateConstPtr& msg);
  ros::Subscriber jointStatesSubscriber_;

  std::mutex receivedJointStatesMutex_;
  std::atomic_bool jointStatesUpdated_;
  JointStates receivedJointStates_;
  std::shared_ptr<ForceTorqueSensing> forceTorqueSensingPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline void readJointNameMsg(const xbot_msgs::JointState& msg, JointStates& jointStates) {
    jointStates.jointNumber = msg.name.size();
    jointStates.name = msg.name;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline void readJointPositionMsg(const xbot_msgs::JointState& msg, JointStates& jointStates) {
    jointStates.linkPosition = msg.link_position;
    jointStates.positionReference = msg.position_reference;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline void readJointVelocityMsg(const xbot_msgs::JointState& msg, JointStates& jointStates) {
    jointStates.linkVelocity = msg.link_velocity;
    jointStates.velocityReference = msg.velocity_reference;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline void readJointEffortMsg(const xbot_msgs::JointState& msg, JointStates& jointStates) {
    jointStates.effort = msg.effort;
    jointStates.effortReference = msg.effort_reference;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline void readJointStiffnessDampingMsg(const xbot_msgs::JointState& msg, JointStates& jointStates) {
    jointStates.stiffness = msg.stiffness;
    jointStates.damping = msg.damping;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline void readJointStatesMsg(const xbot_msgs::JointState& msg, JointStates& jointStates) {
    readJointNameMsg(msg, jointStates);
    readJointPositionMsg(msg, jointStates);
    readJointVelocityMsg(msg, jointStates);
    readJointEffortMsg(msg, jointStates);
    readJointStiffnessDampingMsg(msg, jointStates);
}
}  // namespace legged_robot
}  // namespace ocs2
