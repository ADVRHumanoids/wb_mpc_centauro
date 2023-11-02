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

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "ocs2_centauro/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_centauro/foot_planner/ArmSwingTrajectoryPlanner.h"
#include "ocs2_centauro/gait/GaitSchedule.h"
#include "ocs2_centauro/gait/MotionPhaseDefinition.h"
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>  // [Yiannis]
#include <ocs2_centauro/sensing/ForceTorqueSensing.h>

namespace ocs2 {
namespace legged_robot {

// Facilitate defining object related with kinematics [Yiannis]
using KinematicsPtrArray = feet_array_t<std::shared_ptr<EndEffectorKinematics<scalar_t>>>;      // array of pointers to kinematics
using ArmKinematicsPtrArray = arms_array_t<std::shared_ptr<EndEffectorKinematics<scalar_t>>>;      // array of pointers to kinematics
using KinematicsPtr = std::shared_ptr<EndEffectorKinematics<scalar_t>>;

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                int targetFrameNumber = 0);

  // custom constructor for a second SwingTrajectoryPlanner for arms and ForceTorqueSensing module
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                std::shared_ptr<ArmSwingTrajectoryPlanner> armSwingTrajectoryPtr,
                                std::shared_ptr<ForceTorqueSensing> forceTorqueSensingPtr, int targetFrameNumber);

  // Sets the class member variable related with leg and arm eeKinematics
  void setEeKinematics(KinematicsPtrArray eeKinematicsPtrArray) {eeKinematicsPtrArray_ = eeKinematicsPtrArray;}
  void setArmEeKinematics(ArmKinematicsPtrArray eeKinematicsPtrArray) {armEeKinematicsPtrArray_ = eeKinematicsPtrArray;}

  // Gets the class member variable related with leg and arm eeKinematics
  KinematicsPtrArray getEeKinematics() const {return eeKinematicsPtrArray_;}
  ArmKinematicsPtrArray getArmEeKinematics() const {return armEeKinematicsPtrArray_;}

  ~SwitchedModelReferenceManager() override = default;

  locoma_contact_flag_t getContactFlags(scalar_t time) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }
  const std::shared_ptr<ForceTorqueSensing>& getForceTorqueSensingPtr() const {
      if (forceTorqueSensingPtr_ == nullptr) {
        throw std::runtime_error(
            "[SwitchedModelReferenceManager::getForceTorqueSensingPtr] The ForceTorqueSensingPtr is not set!");
      } else
            return forceTorqueSensingPtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }
  const std::shared_ptr<ArmSwingTrajectoryPlanner>& getArmSwingTrajectoryPlanner() { return armSwingTrajectoryPtr_; }

 private:
  /**
   * @param [in] initTime : Start time of the optimization horizon.
   * @param [in] finalTime : Final time of the optimization horizon.
   * @param [in] initState : State at the start of the optimization horizon.
   * @param [in] targetTrajectories : interface for target trajectories, consists of time, state, input
   * @param [in] modeSchedule : Sequence of modes separated by event times.
   * Called inside ReferenceManager::preSolverRun
   */
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<ForceTorqueSensing> forceTorqueSensingPtr_ = nullptr;     // nullptr since it is not used in all cases
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
  std::shared_ptr<ArmSwingTrajectoryPlanner> armSwingTrajectoryPtr_ = nullptr;

  // class member variable for eeKinematics [Yiannis]
  KinematicsPtrArray eeKinematicsPtrArray_;
  ArmKinematicsPtrArray armEeKinematicsPtrArray_;
};

}  // namespace legged_robot
}  // namespace ocs2
