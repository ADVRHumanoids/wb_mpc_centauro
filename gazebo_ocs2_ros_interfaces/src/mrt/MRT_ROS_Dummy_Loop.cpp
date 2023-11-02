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
#include "gazebo_ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h"

namespace ocs2 {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_ROS_Dummy_Loop::MRT_ROS_Dummy_Loop(MRT_ROS_Interface& mrt, scalar_t mrtDesiredFrequency,
                                       std::shared_ptr<legged_robot::LeggedRobotInterface> leggedRobotInterfacePtr,
                                       ros::NodeHandle& nodeHandle, xbot_interface::XbotInterface::Config xbotConfig,
                                       scalar_t mpcDesiredFrequency)
    : mrt_(mrt), mrtDesiredFrequency_(mrtDesiredFrequency), mpcDesiredFrequency_(mpcDesiredFrequency), nodeHandle_(nodeHandle),
      leggedRobotInterfacePtr_(leggedRobotInterfacePtr),
      xbotInterface_(nodeHandle, std::move(leggedRobotInterfacePtr_->getPinocchioInterface()),
                     std::move(leggedRobotInterfacePtr->getCentroidalModelInfo()),
                     std::move(leggedRobotInterfacePtr->modelSettings().jointNames), xbotConfig)
{
  if (mrtDesiredFrequency_ < 0) {
    throw std::runtime_error("MRT loop frequency should be a positive number.");
  }

  if (mpcDesiredFrequency_ > 0) {
    ROS_WARN_STREAM("MPC loop is not realtime! For realtime setting, set mpcDesiredFrequency to any negative number.");
  }

  if (xbotInterface_.getConfig().xbotCoreRunning) {
    xbotInterface_.connectToXbot();
  }

  std::cout << "Initialized mrt dummy loop successfully!" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::run(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories,
                             std::vector<TargetTrajectories> initFrameTargetTrajectories) {
  ROS_INFO_STREAM("Waiting for the initial policy ...");

  // Reset MPC node
  if (initFrameTargetTrajectories.size() > 0) {
      mrt_.resetMpcNodeCustom(initTargetTrajectories, initFrameTargetTrajectories);
      ROS_INFO_STREAM("resetMpcNodeCustom");
  }
  else {
      mrt_.resetMpcNode(initTargetTrajectories);
      ROS_INFO_STREAM("resetMpcNode");
  }

  // Wait for the initial policy
  while (!mrt_.initialPolicyReceived() && ros::ok() && ros::master::check()) {
    mrt_.spinMRT();
    mrt_.setCurrentObservation(initObservation);
    ros::Rate(mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  // Pick simulation loop mode
  if (mpcDesiredFrequency_ > 0.0) {
      if (xbotInterface_.getConfig().xbotCoreRunning)
          synchronizedXbotCoreLoop(initObservation, initTargetTrajectories);
      else
          synchronizedDummyLoop(initObservation, initTargetTrajectories);       // no xbotcore
  } else {
      if (xbotInterface_.getConfig().xbotCoreRunning)
          realtimeXbotCoreLoop(initObservation, initTargetTrajectories);
      else
          realtimeDummyLoop(initObservation, initTargetTrajectories);     // no xbotcore
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::modifyObservation(SystemObservation &observation) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// TODO: maintain this, currently synchronized is not supported
void MRT_ROS_Dummy_Loop::synchronizedDummyLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) {
  // Determine the ratio between MPC updates and simulation steps.
  const auto mpcUpdateRatio = std::max(static_cast<size_t>(mrtDesiredFrequency_ / mpcDesiredFrequency_), size_t(1));

  // Loop variables
  size_t loopCounter = 0;
  SystemObservation currentObservation = initObservation;

  // Helper function to check if policy is updated and starts at the given time.
  // Due to ROS message conversion delay and very fast MPC loop, we might get an old policy instead of the latest one.
  const auto policyUpdatedForTime = [this](scalar_t time) {
    constexpr scalar_t tol = 0.1;  // policy must start within this fraction of dt
    return mrt_.updatePolicy() && std::abs(mrt_.getPolicy().timeTrajectory_.front() - time) < (tol / mpcDesiredFrequency_);
  };

  ros::Rate simRate(mrtDesiredFrequency_);
  while (ros::ok() && ros::master::check()) {
    std::cout << "### Current time " << currentObservation.time << "\n";

    // Trigger MRT callbacks
    mrt_.spinMRT();

    // Update the MPC policy if it is time to do so
    if (loopCounter % mpcUpdateRatio == 0) {
      // Wait for the policy to be updated
      while (!policyUpdatedForTime(currentObservation.time) && ros::ok() && ros::master::check()) {
        mrt_.spinMRT();
      }
      std::cout << "<<< New MPC policy starting at " << mrt_.getPolicy().timeTrajectory_.front() << "\n";
    }

    // get observation from xbotCore
//    currentObservation = xbotInterface_.getObservationFromXbot(currentObservation, mrtDesiredFrequency_,
//                                                               std::move(leggedRobotInterfacePtr_), mrt_);
    // Forward simulation
    currentObservation = forwardSimulation(currentObservation);

    // User-defined modifications before publishing
    modifyObservation(currentObservation);

    // Publish observation if at the next step we want a new policy
    if ((loopCounter + 1) % mpcUpdateRatio == 0) {
      mrt_.setCurrentObservation(currentObservation);
      std::cout << ">>> Observation is published at " << currentObservation.time << "\n";
    }

    // Update observers
    for (auto& observer : observers_) {
      observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    ++loopCounter;
    ros::spinOnce();
    simRate.sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::synchronizedXbotCoreLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) {
  // Determine the ratio between MPC updates and simulation steps.
  const auto mpcUpdateRatio = std::max(static_cast<size_t>(mrtDesiredFrequency_ / mpcDesiredFrequency_), size_t(1));

  // Loop variables
  size_t loopCounter = 0;
  SystemObservation currentObservation = initObservation;

  // Helper function to check if policy is updated and starts at the given time.
  // Due to ROS message conversion delay and very fast MPC loop, we might get an old policy instead of the latest one.
  const auto policyUpdatedForTime = [this](scalar_t time) {
    constexpr scalar_t tol = 0.1;  // policy must start within this fraction of dt
    return mrt_.updatePolicy() && std::abs(mrt_.getPolicy().timeTrajectory_.front() - time) < (tol / mpcDesiredFrequency_);
  };

  ros::Rate simRate(mrtDesiredFrequency_);
  while (ros::ok() && ros::master::check()) {
    std::cout << "### Current time " << currentObservation.time << "\n";

    // Trigger MRT callbacks
    mrt_.spinMRT();

    // Update the MPC policy if it is time to do so
    if (loopCounter % mpcUpdateRatio == 0) {
      // Wait for the policy to be updated
      while (!policyUpdatedForTime(currentObservation.time) && ros::ok() && ros::master::check()) {
        mrt_.spinMRT();
      }
      std::cout << "<<< New MPC policy starting at " << mrt_.getPolicy().timeTrajectory_.front() << "\n";
    }

    // get observation from xbotCore
    currentObservation = xbotInterface_.getObservationFromXbot(currentObservation, mrtDesiredFrequency_,
                                                               leggedRobotInterfacePtr_, mrt_);

    // User-defined modifications before publishing
//    modifyObservation(currentObservation);

    // Publish observation if at the next step we want a new policy
    if ((loopCounter + 1) % mpcUpdateRatio == 0) {
      mrt_.setCurrentObservation(currentObservation);
      std::cout << ">>> Observation is published at " << currentObservation.time << "\n";
    }

    // evaluate policy at current time
    vector_t optimalState;
    vector_t optimalInput;
    size_t mode;
    mrt_.evaluatePolicy(currentObservation.time, currentObservation.state, optimalState, optimalInput, mode);
    currentObservation.input = optimalInput;        // set input equal to optimal plan (since otherwise has to be zero and is not visualized)

    // send command to xbotcore
    xbotInterface_.sendCommandToXbot(leggedRobotInterfacePtr_, optimalState, optimalInput);


    // Update observers
    for (auto& observer : observers_) {
      observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    ++loopCounter;
    ros::spinOnce();
    simRate.sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::realtimeDummyLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) {
  // Loop variables
  SystemObservation currentObservation = initObservation;

  ros::Rate simRate(mrtDesiredFrequency_);
  while (ros::ok() && ros::master::check()) {
    std::cout << "### Current time " << currentObservation.time << "\n";

    // Trigger MRT callbacks
    mrt_.spinMRT();

    // Update the policy if a new one was received
    if (mrt_.updatePolicy()) {
      std::cout << "<<< New MPC policy starting at " << mrt_.getPolicy().timeTrajectory_.front() << "\n";
    }

    // Forward simulation
    currentObservation = forwardSimulation(currentObservation);

    // User-defined modifications before publishing
    modifyObservation(currentObservation);

    // Publish observation
    mrt_.setCurrentObservation(currentObservation);

    // Update observers
    for (auto& observer : observers_) {
      observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    ros::spinOnce();
    simRate.sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::realtimeXbotCoreLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) {
  // Loop variables
  SystemObservation currentObservation = initObservation;

  if (xbotInterface_.getConfig().xbotCoreFeedback) {
      ros::Rate simRate(mrtDesiredFrequency_);
      while (ros::ok() && ros::master::check()) {
        std::cout << "### Current time " << currentObservation.time << "\n";

        // Trigger MRT callbacks
        mrt_.spinMRT();

        // Update the policy if a new one was received
        if (mrt_.updatePolicy()) {
          std::cout << "<<< New MPC policy starting at " << mrt_.getPolicy().timeTrajectory_.front() << "\n";

//          PerformanceIndex currentPerformanceIndex = mrt_.getPerformanceIndices();
//          if (currentPerformanceIndex.cost > 500 || currentPerformanceIndex.merit > 500) {      // safety
//            std::cout << "*** Safety Stop! High rollout cost or merit." << std::endl;
//            break;
//          }
        }
        // get observation from xbotCore
        currentObservation = xbotInterface_.getObservationFromXbot(currentObservation, mrtDesiredFrequency_,
                                                                   leggedRobotInterfacePtr_, mrt_);
        // Publish observation
        mrt_.setCurrentObservation(currentObservation);

        // evaluate policy at current time
        vector_t optimalState;
        vector_t optimalInput;
        size_t mode;
        mrt_.evaluatePolicy(currentObservation.time, currentObservation.state, optimalState, optimalInput, mode);
        currentObservation.input = optimalInput;        // set input equal to optimal plan (since otherwise has to be zero and is not visualized)

        // send command to xbotcore
        xbotInterface_.sendCommandToXbot(leggedRobotInterfacePtr_, optimalState, optimalInput);

        // Update observers
        for (auto& observer : observers_) {
          observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
        }

        ros::spinOnce();
        simRate.sleep();
      } // while loop

  } else {
      ros::Rate simRate(mrtDesiredFrequency_);
      while (ros::ok() && ros::master::check()) {
        std::cout << "### Current time " << currentObservation.time << "\n";

        // Trigger MRT callbacks
        mrt_.spinMRT();

        // Update the policy if a new one was received
        if (mrt_.updatePolicy()) {
          std::cout << "<<< New MPC policy starting at " << mrt_.getPolicy().timeTrajectory_.front() << "\n";

//          PerformanceIndex currentPerformanceIndex = mrt_.getPerformanceIndices();
//          if (currentPerformanceIndex.cost > 500 || currentPerformanceIndex.merit > 500) {      // safety
//            std::cout << "*** Safety Stop! High rollout cost or merit." << std::endl;
//            break;
//          }
        }
        // Forward simulation
        currentObservation = forwardSimulation(currentObservation);

        // User-defined modifications before publishing
        modifyObservation(currentObservation);

        // Publish observation
        mrt_.setCurrentObservation(currentObservation);

        // evaluate policy at current time
        vector_t optimalState;
        vector_t optimalInput;
        size_t mode;
        mrt_.evaluatePolicy(currentObservation.time, currentObservation.state, optimalState, optimalInput, mode);

        xbotInterface_.sendPositionCommandToXbot(leggedRobotInterfacePtr_, optimalState);

        // Update observers
        for (auto& observer : observers_) {
          observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
        }

        ros::spinOnce();
        simRate.sleep();
      } // while loop
  } // xbotCoreFeedback == false
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemObservation MRT_ROS_Dummy_Loop::forwardSimulation(const SystemObservation& currentObservation) {
  const scalar_t dt = 1.0 / mrtDesiredFrequency_;

  SystemObservation nextObservation;
  nextObservation.time = currentObservation.time + dt;
  if (mrt_.isRolloutSet()) {  // If available, use the provided rollout as to integrate the dynamics.
    mrt_.rolloutPolicy(currentObservation.time, currentObservation.state, dt, nextObservation.state, nextObservation.input,
                       nextObservation.mode);
  } else {  // Otherwise, we fake integration by interpolating the current MPC policy at t+dt
    mrt_.evaluatePolicy(currentObservation.time + dt, currentObservation.state, nextObservation.state, nextObservation.input,
                        nextObservation.mode);
  }

  return nextObservation;
}

}  // namespace ocs2
