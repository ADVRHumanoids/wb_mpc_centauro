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

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_centauro/constraint/EndEffectorLinearConstraint.h"
#include "ocs2_centauro/reference_manager/SwitchedModelReferenceManager.h"
#include "ocs2_centauro/common/utils.h"

namespace ocs2 {
namespace legged_robot {

// to define when the constrait will be active
enum ConstraintActiveWhen {
  INCONTACT = 0,
  NOTINCONTACT = 1,
  ALWAYS = 2
};

/**
 * Specializes the CppAd version of normal velocity constraint on an end-effector position and linear velocity.
 * Constructs the member EndEffectorLinearConstraint object with number of constraints of 1.
 *
 * See also EndEffectorLinearConstraint for the underlying computation.
 */
class CoordinateVelocityConstraintCppAd : public StateInputConstraint {
 public:
    /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager
   * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   * @param [in] coordinateNumber: coordinate to which the velocity constraint will apply, 0 for X, 1 for Y, 2 for Z
   */
  CoordinateVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                    const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex,
                                    const size_t coordinateNumber,
                                    const ConstraintActiveWhen activeWhen = ConstraintActiveWhen::NOTINCONTACT);

  ~CoordinateVelocityConstraintCppAd() override = default;
  CoordinateVelocityConstraintCppAd* clone() const override { return new CoordinateVelocityConstraintCppAd(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  CoordinateVelocityConstraintCppAd(const CoordinateVelocityConstraintCppAd& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
  const size_t contactPointIndex_;
  const size_t coordinateNumber_;
  const ConstraintActiveWhen activeWhen_;
};

/**
 * CppAd version of an end-effector orientation and angular velocity.
 * Constructs the member EndEffectorLinearConstraintAngularMotion object with number of constraints of 1.
 *
 * See also EndEffectorLinearConstraintAngularMotion for the underlying computation.
 */
class CoordinateAngularVelocityConstraintCppAd : public StateInputConstraint {
 public:
  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager
   * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   * @param [in] coordinateNumber: coordinate to which the velocity constraint will apply, 0 for X, 1 for Y, 2 for Z (roll, pitch & yaw)
   */
  CoordinateAngularVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                           const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                           size_t contactPointIndex, const size_t coordinateNumber,
                                           const ConstraintActiveWhen activeWhen = ConstraintActiveWhen::NOTINCONTACT);

  ~CoordinateAngularVelocityConstraintCppAd() override = default;
  CoordinateAngularVelocityConstraintCppAd* clone() const override { return new CoordinateAngularVelocityConstraintCppAd(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  CoordinateAngularVelocityConstraintCppAd(const CoordinateAngularVelocityConstraintCppAd& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorLinearConstraintAngularMotion> eeLinearConstraintAngularMotionPtr_;
  const size_t contactPointIndex_;
  const size_t coordinateNumber_;
  const ConstraintActiveWhen activeWhen_;
};

}  // namespace legged_robot
}  // namespace ocs2
