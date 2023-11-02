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

#ifndef SENSINGDEFINITIONS_H
#define SENSINGDEFINITIONS_H
#include <ocs2_core/Types.h>

namespace ocs2 {
namespace legged_robot {
using wrenches_matrix = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;

struct EeEstimatedWrenches {
  EeEstimatedWrenches(wrenches_matrix eeEstimatedWrenchesParam = wrenches_matrix::Zero(6, 6),
                      vector_t forceNormsParam = vector_t::Zero(6))
      : eeEstimatedWrenches(std::move(eeEstimatedWrenchesParam)), forceNorms(std::move(forceNormsParam)) {
    assert(eeEstimatedWrenches.cols() == forceNorms.size());
  }

  wrenches_matrix eeEstimatedWrenches;
  vector_t forceNorms;
  // TODO: add angle, direction etc.
};

struct JointStates {
  JointStates(size_t jointNumberParam = 37)
      : jointNumber(jointNumberParam), name(std::vector<std::string>(jointNumberParam, "")),
        linkPosition(std::vector<float>(jointNumberParam, 0)),
        linkVelocity(std::vector<float>(jointNumberParam, 0)),
        effort(std::vector<float>(jointNumberParam, 0)),
        positionReference(std::vector<float>(jointNumberParam, 0)),
        velocityReference(std::vector<float>(jointNumberParam, 0)),
        effortReference(std::vector<float>(jointNumberParam, 0)),
        stiffness(std::vector<float>(jointNumberParam, 0)),
        damping(std::vector<float>(jointNumberParam, 0)) {
    assert(jointNumber > 0);
    assert(!name.empty());
  }

  JointStates(size_t jointNumberParam, std::vector<std::string> nameParam,
              std::vector<float> linkPositionParam, std::vector<float> linkVelocityParam,
              std::vector<float> effortParam, std::vector<float> positionReferenceParam,
              std::vector<float> velocityReferenceParam, std::vector<float> effortReferenceParam,
              std::vector<float> stiffnessParam, std::vector<float> dampingParam)
      : jointNumber(std::move(jointNumberParam)), name(std::move(nameParam)),
        linkPosition(std::move(linkPositionParam)),
        linkVelocity(std::move(linkVelocityParam)),
        effort(std::move(effortParam)),
        positionReference(std::move(positionReferenceParam)),
        velocityReference(std::move(velocityReferenceParam)),
        effortReference(std::move(effortReferenceParam)),
        stiffness(std::move(stiffnessParam)),
        damping(std::move(dampingParam)) {
    assert(jointNumber > 0);
    assert(!name.empty());
  }

  JointStates(size_t jointNumberParam, std::vector<std::string> nameParam)
      : jointNumber(std::move(jointNumberParam)), name(std::move(nameParam)) {
    assert(jointNumber > 0);
    assert(!name.empty());
  }

  size_t jointNumber;
  std::vector<std::string> name;
  std::vector<float> linkPosition;
  std::vector<float> linkVelocity;
  std::vector<float> effort;
  std::vector<float> positionReference;
  std::vector<float> velocityReference;
  std::vector<float> effortReference;
  std::vector<float> stiffness;
  std::vector<float> damping;
};

}
}
#endif // SENSINGDEFINITIONS_H
