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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/Numerics.h>

#include <ocs2_centauro/LeggedRobotPreComputation.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation::LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                     const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      settings_(std::move(settings)) {
  const size_t constrainedEndEffectorsNum = swingTrajectoryPlanner.getFeetNumber();
  eeNormalVelConConfigs_.resize(constrainedEndEffectorsNum);
  eeLongitVelConConfigs_.resize(constrainedEndEffectorsNum);
  eeLateralVelConConfigs_.resize(constrainedEndEffectorsNum);
  eeRollXVelConConfigs_.resize(0);
  eePitchYVelConConfigs_.resize(0);
  eeYawZVelConConfigs_.resize(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation::LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                     const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                     const ArmSwingTrajectoryPlanner& armSwingTrajectoryPlanner, ModelSettings settings)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      armSwingTrajectoryPlannerPtr_(&armSwingTrajectoryPlanner),
      settings_(std::move(settings)) {
  size_t constrainedEndEffectorsNum = swingTrajectoryPlanner.getFeetNumber() + armSwingTrajectoryPlanner.getArmsNumber();
  eeNormalVelConConfigs_.resize(constrainedEndEffectorsNum);
  eeLongitVelConConfigs_.resize(constrainedEndEffectorsNum);
  eeLateralVelConConfigs_.resize(constrainedEndEffectorsNum);

  size_t totalArmEesToBeConstrained;
  if (armSwingTrajectoryPlanner.isOrientationPlanner()) {
      totalArmEesToBeConstrained = armSwingTrajectoryPlanner.getArmsNumber();
  } else
      totalArmEesToBeConstrained = 0;
  eeRollXVelConConfigs_.resize(totalArmEesToBeConstrained);
  eePitchYVelConConfigs_.resize(totalArmEesToBeConstrained);
  eeYawZVelConConfigs_.resize(totalArmEesToBeConstrained);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation* LeggedRobotPreComputation::clone() const {
  return new LeggedRobotPreComputation(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  // lambda to set config for normal velocity constraints
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    if (footIndex < settings_.legContactNames3DoF.size()) {    // leg
        config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
        config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
        if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
          config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
          config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
        }
    }
    else {      // arm
        auto armsOnlyIndex = footIndex - settings_.legContactNames3DoF.size();
        config.b = (vector_t(1) << -armSwingTrajectoryPlannerPtr_->getZvelocityConstraint(armsOnlyIndex, t)).finished();
        config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
        if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
          config.b(0) -= settings_.positionErrorGain * armSwingTrajectoryPlannerPtr_->getZpositionConstraint(armsOnlyIndex, t);
          config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
        }
    }
    return config;
  };

  // lambda to set config for longitudinal velocity constraints
  auto eeLongitVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    if (footIndex < settings_.legContactNames3DoF.size()) {    // leg
        config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getXvelocityConstraint(footIndex, t)).finished();
        config.Av = (matrix_t(1, 3) << 1.0, 0.0, 0.0).finished();
        if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
          config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getXpositionConstraint(footIndex, t);
          config.Ax = (matrix_t(1, 3) << settings_.positionErrorGain, 0.0, 0.0).finished();
        }
    }
    else {
        auto armsOnlyIndex = footIndex - settings_.legContactNames3DoF.size();
        config.b = (vector_t(1) << -armSwingTrajectoryPlannerPtr_->getXvelocityConstraint(armsOnlyIndex, t)).finished();
        config.Av = (matrix_t(1, 3) << 1.0, 0.0, 0.0).finished();
        if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
          config.b(0) -= settings_.positionErrorGain * armSwingTrajectoryPlannerPtr_->getXpositionConstraint(armsOnlyIndex, t);
          config.Ax = (matrix_t(1, 3) << settings_.positionErrorGain, 0.0, 0.0).finished();
        }
    }
    return config;
  };

  // lambda to set config for lateral velocity constraints
  auto eeLateralVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    if (footIndex < settings_.legContactNames3DoF.size()) {    // leg
        config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getYvelocityConstraint(footIndex, t)).finished();
        config.Av = (matrix_t(1, 3) << 0.0, 1.0, 0.0).finished();
        if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
          config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getYpositionConstraint(footIndex, t);
          config.Ax = (matrix_t(1, 3) << 0.0, settings_.positionErrorGain, 0.0).finished();
        }
    }
    else {
        auto armsOnlyIndex = footIndex - settings_.legContactNames3DoF.size();
        config.b = (vector_t(1) << -armSwingTrajectoryPlannerPtr_->getYvelocityConstraint(armsOnlyIndex, t)).finished();
        config.Av = (matrix_t(1, 3) << 0.0, 1.0, 0.0).finished();
        if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
          config.b(0) -= settings_.positionErrorGain * armSwingTrajectoryPlannerPtr_->getYpositionConstraint(armsOnlyIndex, t);
          config.Ax = (matrix_t(1, 3) << 0.0, settings_.positionErrorGain, 0.0).finished();
        }
    }
    return config;
  };

  // roll
  auto eeRollXVelConConfig = [&](size_t armsOnlyIndex) {
    EndEffectorLinearConstraintAngularMotion::Config config;
    config.b = (vector_t(1) << -armSwingTrajectoryPlannerPtr_->getVelocityConstraintSlerp(armsOnlyIndex, t, 0)).finished();
    config.b_qref = armSwingTrajectoryPlannerPtr_->getOrientationConstraintSlerp(armsOnlyIndex, t);
    config.Av = (matrix_t(1, 3) << 1.0, 0.0, 0.0).finished();
    if (!numerics::almost_eq(settings_.orientationErrorGain, 0.0)) {
      config.Ax = (matrix_t(1, 3) << -settings_.orientationErrorGain, 0.0, 0.0).finished();
    }
    return config;
  };

  // pitch
  auto eePitchYVelConConfig = [&](size_t armsOnlyIndex) {
    EndEffectorLinearConstraintAngularMotion::Config config;
    config.b = (vector_t(1) << -armSwingTrajectoryPlannerPtr_->getVelocityConstraintSlerp(armsOnlyIndex, t, 1)).finished();
    config.b_qref = armSwingTrajectoryPlannerPtr_->getOrientationConstraintSlerp(armsOnlyIndex, t);
    config.Av = (matrix_t(1, 3) << 0.0, 1.0, 0.0).finished();
    if (!numerics::almost_eq(settings_.orientationErrorGain, 0.0)) {
      config.Ax = (matrix_t(1, 3) << 0.0, -settings_.orientationErrorGain, 0.0).finished();
    }
    return config;
  };

  // yaw, lambda to set config for angular velocity constraints
  auto eeYawZVelConConfig = [&](size_t armsOnlyIndex) {
    EndEffectorLinearConstraintAngularMotion::Config config;
    config.b = (vector_t(1) << -armSwingTrajectoryPlannerPtr_->getVelocityConstraintSlerp(armsOnlyIndex, t, 2)).finished();
    config.b_qref = armSwingTrajectoryPlannerPtr_->getOrientationConstraintSlerp(armsOnlyIndex, t);
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
    if (!numerics::almost_eq(settings_.orientationErrorGain, 0.0)) {
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, -settings_.orientationErrorGain).finished();
    }
    return config;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < eeNormalVelConConfigs_.size(); i++) {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
      eeLongitVelConConfigs_[i] = eeLongitVelConConfig(i);
      eeLateralVelConConfigs_[i] = eeLateralVelConConfig(i);
    }

    for (size_t i = 0; i < eeRollXVelConConfigs_.size(); i++) {     // angular motion
        eeRollXVelConConfigs_[i] = eeRollXVelConConfig(i);
        eePitchYVelConConfigs_[i] = eePitchYVelConConfig(i);
        eeYawZVelConConfigs_[i] = eeYawZVelConConfig(i);
    }
  }
}

}  // namespace legged_robot
}  // namespace ocs2
