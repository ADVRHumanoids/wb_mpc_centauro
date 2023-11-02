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
******************************************************************************/
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_core/automatic_differentiation/FiniteDifferenceMethods.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <gtest/gtest.h>

//centauro
#include "CentauroUrdf.h"
#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"
#include "ocs2_centroidal_model/AccessHelperFunctions.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"
#include "ocs2_centroidal_model/FactoryFunctions.h"

#include "ocs2_core/Types.h"
#include "ocs2_core/automatic_differentiation/Types.h"
#include "ocs2_centauro/common/ModelSettings.h"
#include "ocs2_centauro/test/CentauroFactoryFunctions.h"

using namespace ocs2;
using namespace legged_robot;

class testEndEffector : public ::testing::Test {
 public:
  testEndEffector() {

    // model settings
    const ModelSettings modelSettings;  // = legged_robot::loadModelSettings(taskFile, "model_settings", false);

    // pinocchio mappings
    pinocchioMappingPtr.reset(new CentroidalModelPinocchioMapping(centroidalModelInfo));
    pinocchioMappingAdPtr.reset(new CentroidalModelPinocchioMappingCppAd(centroidalModelInfo.toCppAd()));

    // ee kinematics
    eeKinematicsPtr.reset(
        new PinocchioEndEffectorKinematics(*pinocchioInterfacePtr, *pinocchioMappingPtr, {"arm1_7"}));

    auto velocityUpdateCallback = [&](ad_vector_t state, PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterfaceAd) {
      const ad_vector_t& q = state.tail(centroidalModelInfo.generalizedCoordinatesNum);
      updateCentroidalDynamics(pinocchioInterfaceAd, centroidalModelInfo.toCppAd(), q);
    };
    eeKinematicsAdPtr.reset(new PinocchioEndEffectorKinematicsCppAd(
        *pinocchioInterfacePtr, *pinocchioMappingAdPtr, {"arm1_7"}, centroidalModelInfo.stateDim,
        centroidalModelInfo.inputDim, velocityUpdateCallback, "arm1_7", "/tmp/ocs2", true, true));

    // state, control and robot joints
    x.resize(centroidalModelInfo.stateDim);
    x << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,          // initial state, homing configuration
         0.0, 0.0, 0.718, 0.0, 0.0, 0.0,
         -0.746874, -1.25409, -1.55576, -0.301666, 0.746874, 0.0,
         0.746874, 1.25409, 1.55576, 0.301666, -0.746874, 0.0,
         0.746874, 1.25409, 1.55576, 0.301666, -0.746874, 0.0,
         -0.746874, -1.25409, -1.55576, -0.301666, 0.746874, 0.0,
         0.0,
         0.520149, 0.320865, 0.274669, -2.23604, 0.0500815, -0.781461, -0.0567608,
         0.520149, -0.320865, -0.274669, -2.23604, -0.0500815, -0.781461, 0.0567608;

    u = vector_t::Random(centroidalModelInfo.inputDim);
    q = pinocchioMappingPtr->getPinocchioJointPosition(x);

    std::cout << "End of constructor!" << std::endl;
  }

  void compareApproximation(const VectorFunctionLinearApproximation& f1, const VectorFunctionLinearApproximation& f2,
                            bool functionOfInput = false) {
    if (!f1.f.isApprox(f2.f)) {
      std::cerr << "f1.f  " << f1.f.transpose() << '\n';
      std::cerr << "f2.f  " << f2.f.transpose() << '\n';
    }

    if (!f1.dfdx.isApprox(f2.dfdx)) {
      std::cerr << "f1.dfdx\n" << f1.dfdx << '\n';
      std::cerr << "f2.dfdx\n" << f2.dfdx << '\n';
    }

    if (functionOfInput && !f1.dfdu.isApprox(f2.dfdu)) {
      std::cerr << "f1.dfdu\n" << f1.dfdu << '\n';
      std::cerr << "f2.dfdu\n" << f2.dfdu << '\n';
    }

    EXPECT_TRUE(f1.f.isApprox(f2.f));
    EXPECT_TRUE(f1.dfdx.isApprox(f2.dfdx));
    if (functionOfInput) {
      EXPECT_TRUE(f1.dfdu.isApprox(f2.dfdu));
    }
  }

  const CentroidalModelType centroidalModelType = CentroidalModelType::SingleRigidBodyDynamics;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr = createCentauroPinocchioInterface();
  const CentroidalModelInfo centroidalModelInfo = createCentauroCentroidalModelInfo(*pinocchioInterfacePtr, centroidalModelType);
  PreComputation preComputation;

  std::unique_ptr<CentroidalModelPinocchioMapping> pinocchioMappingPtr;
  std::unique_ptr<CentroidalModelPinocchioMappingCppAd> pinocchioMappingAdPtr;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr;
  std::unique_ptr<PinocchioEndEffectorKinematicsCppAd> eeKinematicsAdPtr;
//  EndEffectorLinearConstraint::Config config;
  vector_t x, u, q;
};

TEST_F(testEndEffector, testKinematicsPosition) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  const auto id = model.getBodyId("arm1_7");
  const vector_t pos = data.oMf[id].translation();
  matrix_t J = matrix_t::Zero(6, model.nq);
  pinocchio::getFrameJacobian(model, data, id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);
  const auto eePos = eeKinematicsPtr->getPosition(x)[0];
  std::cerr << "eePos = " << eePos.transpose() << std::endl;
  std::cerr << "  pos = " << pos.transpose() << std::endl;
  const auto eePosLin = eeKinematicsPtr->getPositionLinearApproximation(x)[0];

  EXPECT_TRUE(pos.isApprox(eePos));
  EXPECT_TRUE(pos.isApprox(eePosLin.f));
//  EXPECT_TRUE(J.topRows<3>().isApprox(eePosLin.dfdx));
}

TEST_F(testEndEffector, testPosition) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto eePos = eeKinematicsPtr->getPosition(x)[0];
  const auto eePosAd = eeKinematicsAdPtr->getPosition(x)[0];
  std::cerr << "eePos = " << eePos.transpose() << std::endl;
  std::cerr << "eePosAd = " << eePosAd.transpose() << std::endl;

  EXPECT_TRUE(eePos.isApprox(eePosAd));
}

//TEST_F(testEndEffector, testPositionApproximation) {
//  const auto& model = pinocchioInterfacePtr->getModel();
//  auto& data = pinocchioInterfacePtr->getData();

//  pinocchio::forwardKinematics(model, data, q);
//  pinocchio::updateFramePlacements(model, data);
//  pinocchio::computeJointJacobians(model, data);

//  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

//  const auto eePosLin = eeKinematicsPtr->getPositionLinearApproximation(x)[0];
//  const auto eePosLinAd = eeKinematicsAdPtr->getPositionLinearApproximation(x)[0];
//  compareApproximation(eePosLin, eePosLinAd);
//}
