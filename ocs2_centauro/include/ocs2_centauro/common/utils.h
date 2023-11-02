/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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
- robotObjectWeightCompensatingInput, locoma_contact_flag_t etc.
******************************************************************************/

#pragma once

#include <array>
#include <cppad/cg.hpp>
#include <iostream>
#include <memory>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "ocs2_centauro/common/Types.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Counts contact feet */
inline size_t numberOfClosedContacts(const locoma_contact_flag_t& contactFlags) {
  size_t numStanceLegs = 0;
  for (auto legInContact : contactFlags) {
    if (legInContact) {
      ++numStanceLegs;
    }
  }
  return numStanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Computes an input with zero joint velocity and forces which equally distribute the robot weight between contact feet. */
inline vector_t weightCompensatingInput(const CentroidalModelInfoTpl<scalar_t>& info, const locoma_contact_flag_t& contactFlags) {
  const auto numStanceLegs = numberOfClosedContacts(contactFlags);
  vector_t input = vector_t::Zero(info.inputDim);
  if (numStanceLegs > 0) {
    const scalar_t totalWeight = info.robotMass * 9.81;
    const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
    for (size_t i = 0; i < contactFlags.size() - 2; i++) {      // only to leg EE contacts, exclude arm EEs
      if (contactFlags[i]) {
        centroidal_model::getContactForces(input, i, info) = forceInInertialFrame;
      }
    }  // end of i loop
  }
  return input;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Computes an input with zero joint velocity and forces which equally distribute the robot weight and consider a weight of
 * a manipulated object at each arm EE. */
inline vector_t robotObjectWeightCompensatingInput(const CentroidalModelInfoTpl<scalar_t>& info, const locoma_contact_flag_t& contactFlags,
                                                   const arms_array_t<vector3_t>& armsForceReferenceVectors) {
  const auto numStanceLegs = numberOfClosedContacts(contactFlags);
  size_t numLeggedContacts = 4;
  vector_t input = vector_t::Zero(info.inputDim);
  if (numStanceLegs > 0) {
    const scalar_t totalWeight = info.robotMass * 9.81;
    const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
    for (size_t i = 0; i < contactFlags.size(); i++) {      // only to leg EE contacts, exclude arm EEs
      if (contactFlags[i]) {
        if (i < numLeggedContacts)                // leg
            centroidal_model::getContactForces(input, i, info) = forceInInertialFrame;
        else {                                            // arm
            centroidal_model::getContactForces(input, i, info) = armsForceReferenceVectors[i - numLeggedContacts];
        }
      }
    }  // end of i loop
  }
  return input;
}

}  // namespace legged_robot
}  // namespace ocs2
