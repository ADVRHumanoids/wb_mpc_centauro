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
- resetMpcNodeCustom

******************************************************************************/

#pragma once

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>

#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_target_trajectories.h>
#include <ocs2_msgs/reset.h>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/CommandData.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

#define PUBLISH_THREAD

namespace ocs2 {

/**
 * This class implements MPC communication interface using ROS.
 */
class MPC_ROS_Interface {
 public:
  /**
   * Constructor.
   *
   * @param [in] mpc: The underlying MPC class to be used.
   * @param [in] topicPrefix: The robot's name.
   */
  explicit MPC_ROS_Interface(MPC_BASE& mpc, std::string topicPrefix = "anonymousRobot");

  /**
   * Destructor.
   */
  virtual ~MPC_ROS_Interface();

  /**
   * Resets the class to its instantiation state.
   *
   * @param [in] initTargetTrajectories: The initial desired cost trajectories.
   */
  void resetMpcNode(TargetTrajectories&& initTargetTrajectories);

  // custom to reset also using the EE target trajectories
  void resetMpcNodeCustom(TargetTrajectories&& initTargetTrajectories, std::vector<TargetTrajectories>&& initEndEffectorTargetTrajectories);

  /**
   * Shutdowns the ROS node.
   */
  void shutdownNode();

  /**
   * Spins ROS.
   */
  void spin();

  /**
   * This is the main routine which launches all the nodes required for MPC to run which includes:
   * (1) The MPC policy publisher (either feedback or feedforward policy).
   * (2) The observation subscriber which gets the current measured state to invoke the MPC run routine.
   */
  void launchNodes(ros::NodeHandle& nodeHandle);

 protected:
  /**
   * Callback to reset MPC.
   *
   * @param req: Service request.
   * @param res: Service response.
   */
  bool resetMpcCallback(ocs2_msgs::reset::Request& req, ocs2_msgs::reset::Response& res);

  /**
   * Creates MPC Policy message.
   *
   * @param [in] primalSolution: The policy data of the MPC.
   * @param [in] commandData: The command data of the MPC.
   * @param [in] performanceIndices: The performance indices data of the solver.
   * @return MPC policy message.
   */
  static ocs2_msgs::mpc_flattened_controller createMpcPolicyMsg(const PrimalSolution& primalSolution, const CommandData& commandData,
                                                                const PerformanceIndex& performanceIndices);

  /**
   * Handles ROS publishing thread.
   */
  void publisherWorker();

  /**
   * Updates the buffer variables from the MPC object. This method is automatically called by advanceMpc()
   *
   * @param [in] mpcInitObservation: The observation used to run the MPC.
   */
  void copyToBuffer(const SystemObservation& mpcInitObservation);

  /**
   * The callback method which receives the current observation, invokes the MPC algorithm,
   * and finally publishes the optimized policy.
   *
   * @param [in] msg: The observation message.
   */
  void mpcObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

 protected:
  /*
   * Variables
   */
  MPC_BASE& mpc_;

  std::string topicPrefix_;

  std::shared_ptr<ros::NodeHandle> nodeHandlerPtr_;

  // Publishers and subscribers
  ::ros::Subscriber mpcObservationSubscriber_;
  ::ros::Subscriber mpcTargetTrajectoriesSubscriber_;
  ::ros::Publisher mpcPolicyPublisher_;
  ::ros::ServiceServer mpcResetServiceServer_;

  std::unique_ptr<CommandData> bufferCommandPtr_;
  std::unique_ptr<CommandData> publisherCommandPtr_;
  std::unique_ptr<PrimalSolution> bufferPrimalSolutionPtr_;
  std::unique_ptr<PrimalSolution> publisherPrimalSolutionPtr_;
  std::unique_ptr<PerformanceIndex> bufferPerformanceIndicesPtr_;
  std::unique_ptr<PerformanceIndex> publisherPerformanceIndicesPtr_;

  mutable std::mutex bufferMutex_;  // for policy variables with prefix (buffer*)

  // multi-threading for publishers
  std::atomic_bool terminateThread_{false};
  std::atomic_bool readyToPublish_{false};
  std::thread publisherWorker_;
  std::mutex publisherMutex_;
  std::condition_variable msgReady_;

  benchmark::RepeatedTimer mpcTimer_;

  // MPC reset
  std::mutex resetMutex_;
  std::atomic_bool resetRequestedEver_{false};
};

}  // namespace ocs2
