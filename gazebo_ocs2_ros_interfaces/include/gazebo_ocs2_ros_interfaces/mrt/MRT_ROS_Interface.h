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

Copyright (c) 2023, Ioannis Dadiotis <ioannis.dadiotis@iit.it>. All rights reserved.
Additional modifications and contributions by Ioannis Dadiotis:
- resetMpcNodeCustom
******************************************************************************/

#pragma once

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>

// MPC messages
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/reset.h>

#include <ocs2_mpc/MRT_BASE.h>

#include "gazebo_ocs2_ros_interfaces/common/RosMsgConversions.h"

#define PUBLISH_THREAD

namespace ocs2 {

/**
 * This class implements MRT (Model Reference Tracking) communication interface using ROS.
 */
class MRT_ROS_Interface : public MRT_BASE {
 public:
  /**
   * Constructor
   *
   * @param [in] topicPrefix: The prefix defines the names for: observation's publishing topic "topicPrefix_mpc_observation",
   * policy's receiving topic "topicPrefix_mpc_policy", and MPC reset service "topicPrefix_mpc_reset".
   * @param [in] mrtTransportHints: ROS transmission protocol.
   */
  explicit MRT_ROS_Interface(std::string topicPrefix = "anonymousRobot",
                             ::ros::TransportHints mrtTransportHints = ::ros::TransportHints().tcpNoDelay());

  /**
   * Destructor
   */
  ~MRT_ROS_Interface() override;

  void resetMpcNode(const TargetTrajectories& initTargetTrajectories) override;

  // custom reset mpc for ee target
  void resetMpcNodeCustom(const TargetTrajectories& initTargetTrajectories,
                          const std::vector<TargetTrajectories>& initFrameTargetTrajectories);
  /**
   * Shut down the ROS nodes.
   */
  void shutdownNodes();

  /**
   * Shut down publisher
   */
  void shutdownPublisher();

  /**
   * spin the MRT callback queue
   */
  void spinMRT();

  /**
   * Launches the ROS publishers and subscribers to communicate with the MPC node.
   * @param nodeHandle
   */
  void launchNodes(::ros::NodeHandle& nodeHandle);

  void setCurrentObservation(const SystemObservation& currentObservation) override;

 private:
  /**
   * Callback method to receive the MPC policy as well as the mode sequence.
   * It only updates the policy variables with suffix (*Buffer_) variables.
   *
   * @param [in] msg: A constant pointer to the message
   */
  void mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg);

  /**
   * Helper function to read a MPC policy message.
   *
   * @param [in] msg: A constant pointer to the message
   * @param [out] commandData: The MPC command data
   * @param [out] primalSolution: The MPC policy data
   * @param [out] performanceIndices: The MPC performance indices data
   */
  static void readPolicyMsg(const ocs2_msgs::mpc_flattened_controller& msg, CommandData& commandData, PrimalSolution& primalSolution,
                            PerformanceIndex& performanceIndices);

  /**
   * A thread function which sends the current state and checks for a new MPC update.
   */
  void publisherWorkerThread();

 private:
  std::string topicPrefix_;

  // Publishers and subscribers
  ::ros::Publisher mpcObservationPublisher_;
  ::ros::Subscriber mpcPolicySubscriber_;
  ::ros::ServiceClient mpcResetServiceClient_;

  // ROS messages
  ocs2_msgs::mpc_observation mpcObservationMsg_;
  ocs2_msgs::mpc_observation mpcObservationMsgBuffer_;

  ::ros::CallbackQueue mrtCallbackQueue_;
  ::ros::TransportHints mrtTransportHints_;

  // Multi-threading for publishers
  bool terminateThread_;
  bool readyToPublish_;
  std::thread publisherWorker_;
  std::mutex publisherMutex_;
  std::condition_variable msgReady_;
};

}  // namespace ocs2
