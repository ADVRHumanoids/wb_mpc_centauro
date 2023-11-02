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

#include "ocs2_centauro/constraint/CoordinateVelocityConstraintCppAd.h"
#include "ocs2_centauro/LeggedRobotPreComputation.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CoordinateVelocityConstraintCppAd::CoordinateVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                                     const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                     size_t contactPointIndex, const size_t coordinateNumber,
                                                                     const ConstraintActiveWhen activeWhen)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 1)),
      contactPointIndex_(contactPointIndex),
      coordinateNumber_(coordinateNumber), activeWhen_(activeWhen) {} //activeAtSwingOnly_(activeAtSwingOnly){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CoordinateVelocityConstraintCppAd::CoordinateVelocityConstraintCppAd(const CoordinateVelocityConstraintCppAd& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      coordinateNumber_(rhs.coordinateNumber_), activeWhen_(rhs.activeWhen_) {} //activeAtSwingOnly_(rhs.activeAtSwingOnly_){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool CoordinateVelocityConstraintCppAd::isActive(scalar_t time) const {
  switch (activeWhen_) {
    case 0:                             //  INCONTACT = 0
      return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
    case 1:                             //  NOTINCONTACT = 1
      return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
    case 2:                             //  ALWAYS = 2
      return true;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CoordinateVelocityConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                     const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);

  // pick up the right coordinate
  switch (coordinateNumber_) {
  case 0:
      eeLinearConstraintPtr_->configure(preCompLegged.getEeLongitVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  case 1:
      eeLinearConstraintPtr_->configure(preCompLegged.getEeLateralVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  case 2:
      eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  }
  return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation CoordinateVelocityConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                            const vector_t& input,
                                                                                            const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  // pick up the right coordinate
  switch (coordinateNumber_) {
  case 0:
      eeLinearConstraintPtr_->configure(preCompLegged.getEeLongitVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  case 1:
      eeLinearConstraintPtr_->configure(preCompLegged.getEeLateralVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  case 2:
      eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  }
  return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CoordinateAngularVelocityConstraintCppAd::CoordinateAngularVelocityConstraintCppAd(
        const SwitchedModelReferenceManager& referenceManager, const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
        size_t contactPointIndex, const size_t coordinateNumber, const ConstraintActiveWhen activeWhen)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      eeLinearConstraintAngularMotionPtr_(new EndEffectorLinearConstraintAngularMotion(endEffectorKinematics, 1)),
      contactPointIndex_(contactPointIndex),
      coordinateNumber_(coordinateNumber), activeWhen_(activeWhen) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CoordinateAngularVelocityConstraintCppAd::CoordinateAngularVelocityConstraintCppAd(const CoordinateAngularVelocityConstraintCppAd& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      eeLinearConstraintAngularMotionPtr_(rhs.eeLinearConstraintAngularMotionPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      coordinateNumber_(rhs.coordinateNumber_), activeWhen_(rhs.activeWhen_){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool CoordinateAngularVelocityConstraintCppAd::isActive(scalar_t time) const {
    switch (activeWhen_) {
      case 0:                             //  INCONTACT = 0
        return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
      case 1:                             //  NOTINCONTACT = 1
        return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
      case 2:                             //  ALWAYS = 2
        return true;
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CoordinateAngularVelocityConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                     const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);

  // pick up the right coordinate
  switch (coordinateNumber_) {
  case 0:
      eeLinearConstraintAngularMotionPtr_->configure(preCompLegged.getEeRollVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  case 1:
      eeLinearConstraintAngularMotionPtr_->configure(preCompLegged.getEePitchVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  case 2:
      eeLinearConstraintAngularMotionPtr_->configure(preCompLegged.getEeYawVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  }
  return eeLinearConstraintAngularMotionPtr_->getValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation CoordinateAngularVelocityConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                            const vector_t& input,
                                                                                            const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  // pick up the right coordinate
  switch (coordinateNumber_) {
  case 0:
      eeLinearConstraintAngularMotionPtr_->configure(preCompLegged.getEeRollVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  case 1:
      eeLinearConstraintAngularMotionPtr_->configure(preCompLegged.getEePitchVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  case 2:
      eeLinearConstraintAngularMotionPtr_->configure(preCompLegged.getEeYawVelocityConstraintConfigs()[contactPointIndex_]);
      break;
  }
  return eeLinearConstraintAngularMotionPtr_->getLinearApproximation(time, state, input, preComp);
}
}  // namespace legged_robot
}  // namespace ocs2
