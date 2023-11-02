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

#include "ocs2_centauro/constraint/UnilateralConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
UnilateralConstraint::UnilateralConstraint(const SwitchedModelReferenceManager& referenceManager, Config config,
                                               size_t contactPointIndex, CentroidalModelInfo info)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      config_(std::move(config)),
      contactPointIndex_(contactPointIndex),
      info_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void UnilateralConstraint::setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld) {
  t_R_w.setIdentity();
  throw std::runtime_error("[UnilateralConstraint] setSurfaceNormalInWorld() is not implemented!");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool UnilateralConstraint::isActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t UnilateralConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                          const PreComputation& preComp) const {
  const auto forcesInWorldFrame = centroidal_model::getContactForces(input, contactPointIndex_, info_);
  const vector3_t localForce = t_R_w * forcesInWorldFrame;
  return normalForceConstraint(localForce);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation UnilateralConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input,
                                                                                 const PreComputation& preComp) const {
    VectorFunctionLinearApproximation approx(1, state.size(), input.size());
    approx.f = getValue(time, state, input, preComp);
    approx.dfdx = matrix_t::Zero(1, state.size());
    approx.dfdu = matrix_t::Zero(1, input.size());
    approx.dfdu.middleCols<1>(3 * contactPointIndex_ + 2).diagonal() = vector_t::Ones(1);

    return approx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//VectorFunctionQuadraticApproximation UnilateralConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state,
//                                                                                       const vector_t& input,
//                                                                                       const PreComputation& preComp) const {
//  const vector3_t forcesInWorldFrame = centroidal_model::getContactForces(input, contactPointIndex_, info_);
//  const vector3_t localForce = t_R_w * forcesInWorldFrame;

//  const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame);
//  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
//  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

//  VectorFunctionQuadraticApproximation quadraticApproximation;
//  quadraticApproximation.f = coneConstraint(localForce);
//  quadraticApproximation.dfdx = matrix_t::Zero(1, state.size());
//  quadraticApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
//  quadraticApproximation.dfdxx.emplace_back(frictionConeSecondDerivativeState(state.size(), coneDerivatives));
//  quadraticApproximation.dfduu.emplace_back(frictionConeSecondDerivativeInput(input.size(), coneDerivatives));
//  quadraticApproximation.dfdux.emplace_back(matrix_t::Zero(input.size(), state.size()));
//  return quadraticApproximation;
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//UnilateralConstraint::LocalForceDerivatives UnilateralConstraint::computeLocalForceDerivatives(
//    const vector3_t& forcesInWorldFrame) const {
//  LocalForceDerivatives localForceDerivatives{};
//  localForceDerivatives.dF_du = t_R_w;
//  return localForceDerivatives;
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//UnilateralConstraint::ConeLocalDerivatives UnilateralConstraint::computeConeLocalDerivatives(const vector3_t& localForces) const {
//  const auto F_x_square = localForces.x() * localForces.x();
//  const auto F_y_square = localForces.y() * localForces.y();
//  const auto F_tangent_square = F_x_square + F_y_square + config_.regularization;
//  const auto F_tangent_norm = sqrt(F_tangent_square);
//  const auto F_tangent_square_pow32 = F_tangent_norm * F_tangent_square;  // = F_tangent_square ^ (3/2)

//  ConeLocalDerivatives coneDerivatives{};
//  coneDerivatives.dCone_dF(0) = -localForces.x() / F_tangent_norm;
//  coneDerivatives.dCone_dF(1) = -localForces.y() / F_tangent_norm;
//  coneDerivatives.dCone_dF(2) = config_.frictionCoefficient;

//  coneDerivatives.d2Cone_dF2(0, 0) = -(F_y_square + config_.regularization) / F_tangent_square_pow32;
//  coneDerivatives.d2Cone_dF2(0, 1) = localForces.x() * localForces.y() / F_tangent_square_pow32;
//  coneDerivatives.d2Cone_dF2(0, 2) = 0.0;
//  coneDerivatives.d2Cone_dF2(1, 0) = coneDerivatives.d2Cone_dF2(0, 1);
//  coneDerivatives.d2Cone_dF2(1, 1) = -(F_x_square + config_.regularization) / F_tangent_square_pow32;
//  coneDerivatives.d2Cone_dF2(1, 2) = 0.0;
//  coneDerivatives.d2Cone_dF2(2, 0) = 0.0;
//  coneDerivatives.d2Cone_dF2(2, 1) = 0.0;
//  coneDerivatives.d2Cone_dF2(2, 2) = 0.0;

//  return coneDerivatives;
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t UnilateralConstraint::normalForceConstraint(const vector3_t& localForces) const {
  const auto F_z = localForces.z();
  const scalar_t normalForceConstraint = F_z - config_.minimumNormalForce;
  return (vector_t(1) << normalForceConstraint).finished();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//UnilateralConstraint::ConeDerivatives UnilateralConstraint::computeConeConstraintDerivatives(
//    const ConeLocalDerivatives& coneLocalDerivatives, const LocalForceDerivatives& localForceDerivatives) const {
//  ConeDerivatives coneDerivatives;
//  // First order derivatives
//  coneDerivatives.dCone_du.noalias() = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_du;

//  // Second order derivatives
//  coneDerivatives.d2Cone_du2.noalias() =
//      localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;

//  return coneDerivatives;
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//matrix_t UnilateralConstraint::frictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const {
//  matrix_t dhdu = matrix_t::Zero(1, inputDim);
//  dhdu.block<1, 3>(0, 3 * contactPointIndex_) = coneDerivatives.dCone_du;
//  return dhdu;
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//matrix_t UnilateralConstraint::frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const {
//  matrix_t ddhdudu = matrix_t::Zero(inputDim, inputDim);
//  ddhdudu.block<3, 3>(3 * contactPointIndex_, 3 * contactPointIndex_) = coneDerivatives.d2Cone_du2;
//  ddhdudu.diagonal().array() -= config_.hessianDiagonalShift;
//  return ddhdudu;
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//matrix_t UnilateralConstraint::frictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives) const {
//  matrix_t ddhdxdx = matrix_t::Zero(stateDim, stateDim);
//  ddhdxdx.diagonal().array() -= config_.hessianDiagonalShift;
//  return ddhdxdx;
//}

}  // namespace legged_robot
}  // namespace ocs2

