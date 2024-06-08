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

// ocs2
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/SqpSettings.h>

#include "ocs2_centauro/common/ModelSettings.h"
#include "ocs2_centauro/initialization/LeggedRobotInitializer.h"
#include "ocs2_centauro/reference_manager/SwitchedModelReferenceManager.h"

#include "ocs2_centauro/constraint/FrictionConeConstraint.h"
#include "ocs2_centauro/constraint/CoordinateVelocityConstraintCppAd.h"

/**
 * LeggedRobotInterface class
 * General interface for mpc implementation on the legged robot model
 */
namespace ocs2 {
namespace legged_robot {
using penalty_type = augmented::SlacknessSquaredHingePenalty;

class LeggedRobotInterface : public RobotInterface {
 public:
  /**
   * Constructor
   *
   * @throw Invalid argument error if input task file or urdf file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] urdfFile: The absolute path to the URDF file for the robot.
   * @param [in] referenceFile: The absolute path to the reference configuration file.
   */
  LeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile);

  ~LeggedRobotInterface() override = default;

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }
  const ddp::Settings& ddpSettings() const { return ddpSettings_; }
  const mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  const sqp::Settings& sqpSettings() { return sqpSettings_; }

  const vector_t& getInitialState() const { return initialState_; }
  void setInitialState(const vector_t& initialState) { initialState_ = initialState; }
  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const LeggedRobotInitializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

 private:
  // test self-collision
  std::unique_ptr<StateCost> getSelfCollisionConstraint(const std::string& taskFile, const std::string& urdfFile, const std::string& prefix);

  void setupOptimalConrolProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);

  std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;

  // custom full input cost for centauro
  std::unique_ptr<StateInputCost> getFullInputCost(const std::string& taskFile, const CentroidalModelInfo& info, bool verbose);
  std::unique_ptr<StateInputCost> getTaskSpaceVelCost(const std::string& taskFile, const CentroidalModelInfo& info, bool verbose);
  matrix_t initializeFullInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);
  matrix_t initializeTaskSpaceInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);

  FrictionConeConstraint::Config loadFrictionConeSettings(const std::string& taskFile, const size_t& contactPointIndex, bool verbose) const;
  std::unique_ptr<StateInputConstraint> getFrictionConeConstraint(const std::string& taskFile, const size_t& contactPointIndex, bool verebose) const;     // AL-based
  std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadUnilateralConstraintSettings(const std::string& taskFile, bool verbose) const;
  std::unique_ptr<StateInputCost> getUnilateralConstraint(const size_t& contactPointIndex, scalar_t minimumNormalForce,
                                                          const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  std::unique_ptr<StateInputConstraint> getZeroForceConstraint(const size_t& contactPointIndex);
  std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  const size_t& contactPointIndex, bool useAnalyticalGradients);
  std::unique_ptr<StateInputConstraint> getCoordinateVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics, const size_t& contactPointIndex,
                                                                        bool useAnalyticalGradients, const size_t coordinateNumber,
                                                                        const ConstraintActiveWhen activationState = ConstraintActiveWhen::NOTINCONTACT);
  std::unique_ptr<StateInputConstraint> getCoordinateAngularVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics, const size_t& contactPointIndex,
                                                                               bool useAnalyticalGradients, const size_t coordinateNumber,
                                                                               const ConstraintActiveWhen activationState = ConstraintActiveWhen::NOTINCONTACT);
  // arm EE soft constraint
  std::unique_ptr<StateCost> getEndEffectorConstraint(const std::string& taskFile, const std::string& prefix);

  ModelSettings modelSettings_;
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<LeggedRobotInitializer> initializerPtr_;

  vector_t initialState_;
};

}  // namespace legged_robot
}  // namespace ocs2
