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

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_centauro/common/Types.h"
#include "ocs2_centauro/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/**
 * Implements the constraint h(t,x,u) >= 0
 *
 * Fz >= 0 or Fz >= Fmin
 *
 */
class UnilateralConstraint final : public StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * minimumNormalForce: The minimum normal force to be impsosed for stability reasons.
   */
  struct Config {
    explicit Config(scalar_t minimumNormalForceParam = 0.0)
        : minimumNormalForce(minimumNormalForceParam) {
      assert(minimumNormalForce >= 0.0);
    }

    scalar_t minimumNormalForce;
  };

  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager.
   * @param [in] config : Friction model settings.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   * @param [in] info : The centroidal model information.
   */
  UnilateralConstraint(const SwitchedModelReferenceManager& referenceManager, Config config, size_t contactPointIndex,
                         CentroidalModelInfo info);

  ~UnilateralConstraint() override = default;
  UnilateralConstraint* clone() const override { return new UnilateralConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; };
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;
//  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
//                                                                 const PreComputation& preComp) const override;

  /** Sets the estimated terrain normal expressed in the world frame. */
  void setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld);

 private:
//  struct LocalForceDerivatives {
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    matrix3_t dF_du;  // derivative local force w.r.t. forces in world frame
//  };

//  struct ConeLocalDerivatives {
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    vector3_t dCone_dF;    // derivative w.r.t local force
//    matrix3_t d2Cone_dF2;  // second derivative w.r.t local force
//  };

//  struct ConeDerivatives {
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    vector3_t dCone_du;
//    matrix3_t d2Cone_du2;
//  };

  UnilateralConstraint(const UnilateralConstraint& other) = default;
  vector_t normalForceConstraint(const vector3_t& localForces) const;
//  LocalForceDerivatives computeLocalForceDerivatives(const vector3_t& forcesInBodyFrame) const;
//  ConeLocalDerivatives computeConeLocalDerivatives(const vector3_t& localForces) const;
//  ConeDerivatives computeConeConstraintDerivatives(const ConeLocalDerivatives& coneLocalDerivatives,
//                                                   const LocalForceDerivatives& localForceDerivatives) const;

//  matrix_t frictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
//  matrix_t frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
//  matrix_t frictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives) const;

  const SwitchedModelReferenceManager* referenceManagerPtr_;

  const Config config_;
  const size_t contactPointIndex_;
  const CentroidalModelInfo info_;

  // rotation world to terrain
  matrix3_t t_R_w = matrix3_t::Identity();
};

}  // namespace legged_robot
}  // namespace ocs2
