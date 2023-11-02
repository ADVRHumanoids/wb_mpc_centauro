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

#include <memory>

#include <ocs2_centauro/LeggedRobotPreComputation.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>

namespace ocs2 {
namespace legged_robot {

class CentauroSelfCollisionConstraint final : public SelfCollisionConstraint {
 public:
  CentauroSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                  PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
      : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}
  ~CentauroSelfCollisionConstraint() override = default;
  CentauroSelfCollisionConstraint(const CentauroSelfCollisionConstraint& other) = default;
  CentauroSelfCollisionConstraint* clone() const { return new CentauroSelfCollisionConstraint(*this); }

  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
    return cast<LeggedRobotPreComputation>(preComputation).getPinocchioInterface();
  }
};

}  // namespace mobile_manipulator
}  // namespace ocs2
