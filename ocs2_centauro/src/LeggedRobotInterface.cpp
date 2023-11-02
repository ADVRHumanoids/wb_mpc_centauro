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

#include <iostream>
#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_centauro/LeggedRobotInterface.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <ocs2_core/constraint/LinearStateInputConstraint.h>    // augmented lagrangian
#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>

#include "ocs2_centauro/LeggedRobotPreComputation.h"
#include "ocs2_centauro/constraint/FrictionConeConstraint.h"
#include "ocs2_centauro/constraint/UnilateralConstraint.h"
#include "ocs2_centauro/constraint/ZeroForceConstraint.h"
#include "ocs2_centauro/constraint/ZeroVelocityConstraintCppAd.h"
#include "ocs2_centauro/cost/LeggedRobotStateInputQuadraticCost.h"
#include "ocs2_centauro/dynamics/LeggedRobotDynamicsAD.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <ros/package.h>

// self-collision
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

// arm EE soft constraint
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include "ocs2_centauro/constraint/EndEffectorConstraint.h"

namespace ocs2 {
namespace legged_robot {

using penalty_type = augmented::SlacknessSquaredHingePenalty;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInterface::LeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] URDF file not found: " + urdfFilePath.string());
  }
  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  bool verbose;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // load setting from loading file
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
  sqpSettings_ = multiple_shooting::loadSettings(taskFile, "multiple_shooting", verbose);

  // OptimalConrolProblem
  setupOptimalConrolProblem(taskFile, urdfFile, referenceFile, verbose);

  // initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);       // initial version
//  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInterface::setupOptimalConrolProblem(const std::string& taskFile, const std::string& urdfFile,
                                                     const std::string& referenceFile, bool verbose) {
  // get target frame number and names
  std::vector<std::string> targetFramesNames;
  loadData::loadStdVector(taskFile, "targetFramesNames", targetFramesNames);

  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings_.contactNames3DoF,
      modelSettings_.contactNames6DoF);

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4));


  // arm EE planning (Swing trajectory planner or just soft constraint)
  bool armEeSoftConstraints, armEeHardConstraints;
  loadData::loadCppDataType(taskFile, "armEeSoftConstraints.activate",  armEeSoftConstraints);
  loadData::loadCppDataType(taskFile, "arm_swing_trajectory_config.addConstraints",  armEeHardConstraints);
  std::unique_ptr<ArmSwingTrajectoryPlanner> armSwingTrajectoryPlanner = nullptr;
  size_array2_t hConstraintPositionIndices, hConstraintOrientationIndices;
  if (armEeHardConstraints) {
      // Arm swing trajectory planner
       armSwingTrajectoryPlanner.reset(
                  new ArmSwingTrajectoryPlanner(loadArmSwingTrajectorySettings(taskFile, "arm_swing_trajectory_config", verbose), 2));
      hConstraintPositionIndices = armSwingTrajectoryPlanner->getConfig().positionIndices;  // dofs to be constrained
      hConstraintOrientationIndices = armSwingTrajectoryPlanner->getConfig().orientationIndices;
  }

  // Sensing from estimated ee wrenches or joint states passed to mpc
  bool eeWrenchSensing, jointSensing;
  loadData::loadCppDataType(taskFile, "forceTorqueSensingInMpc.estimatedWrenches", eeWrenchSensing);
  loadData::loadCppDataType(taskFile, "forceTorqueSensingInMpc.jointStates", jointSensing);
  std::unique_ptr<ForceTorqueSensing> forceTorqueSensingPtr = nullptr;
  if (eeWrenchSensing || jointSensing)
      forceTorqueSensingPtr.reset(new ForceTorqueSensing());

  // reference manager
  referenceManagerPtr_ =
      std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose),
                                                      std::move(swingTrajectoryPlanner), std::move(armSwingTrajectoryPlanner),
                                                      std::move(forceTorqueSensingPtr), targetFramesNames.size());

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  bool useAnalyticalGradientsDynamics = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsDynamics", useAnalyticalGradientsDynamics);
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  if (useAnalyticalGradientsDynamics) {
    throw std::runtime_error("[LeggedRobotInterface::setupOptimalConrolProblem] The analytical dynamics class is not yet implemented!");
  } else {
    const std::string modelName = "dynamics";
    dynamicsPtr.reset(new LeggedRobotDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelName, modelSettings_));
  }
  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  problemPtr_->costPtr->add("fullInputCost", getFullInputCost(taskFile, centroidalModelInfo_, false));
  problemPtr_->costPtr->add("taskSpaceEeVelCost", getTaskSpaceVelCost(taskFile, centroidalModelInfo_, false));

  // Define pointer to an array of feet eeKinematics objects
  feet_array_t<std::shared_ptr<EndEffectorKinematics<scalar_t>>> eeKinematicsPtrArray;
  arms_array_t<std::shared_ptr<EndEffectorKinematics<scalar_t>>> armEeKinematicsPtrArray;   // for arms

  // Constraints
  bool useAnalyticalGradientsConstraints = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints", useAnalyticalGradientsConstraints);

  // Settings for stability constraint
  bool activateStability;
  scalar_t minimumNormalForce, maximumNormalForce;
  loadData::loadCppDataType(taskFile, "stabilityConstraint.activate", activateStability);
  loadData::loadCppDataType(taskFile, "minimumNormalForce", minimumNormalForce);
  loadData::loadCppDataType(taskFile, "maximumNormalForce", maximumNormalForce);
  auto getPenalty = [&]() {
    // one can use either augmented::SlacknessSquaredHingePenalty or augmented::ModifiedRelaxedBarrierPenalty
    penalty_type::Config boundsConfig;
    loadData::loadPenaltyConfig(taskFile, "stabilityConstraint.config", boundsConfig, verbose);
    return penalty_type::create(boundsConfig);
  };
  // lambda for friction cone AL parameters
  auto getFrictionConePenalty = [&]() {
      penalty_type::Config augmentedLagrangianConfig;
      loadData::loadPenaltyConfig(taskFile, "frictionConeConstraint.augmented_lagrangian",
                                  augmentedLagrangianConfig, verbose);
      return penalty_type::create(augmentedLagrangianConfig);
  };

  // loop over contacts
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings_.contactNames3DoF[i];
    auto& legContacts = modelSettings_.legContactNames3DoF;
    bool isLegContact = std::find(legContacts.begin(), legContacts.end(), footName) != legContacts.end();

    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;
    if (useAnalyticalGradientsConstraints) {
      throw std::runtime_error(
          "[LeggedRobotInterface::setupOptimalConrolProblem] The analytical end-effector linear constraint is not implemented!");
    } else {
      const auto infoCppAd = centroidalModelInfo_.toCppAd();
      const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
      auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
        const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
        updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
      };

      auto eeMotionFrame = footName;        // motion ee frame to be tracked, may be different for arms
      if (!isLegContact)
          eeMotionFrame = targetFramesNames[i - legContacts.size()];
      eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {eeMotionFrame},
                                                                    centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                    velocityUpdateCallback, eeMotionFrame, modelSettings_.modelFolderCppAd,
                                                                    modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
    }

    // for leg contacts
    if (isLegContact) {
        // stability constraint through augmented lagrangian
        auto getConstraint = [&]() {
            constexpr size_t numIneqConstraint = 2;     // two-side bound
            const vector_t e = (vector_t(numIneqConstraint) << -minimumNormalForce, maximumNormalForce).finished();
            matrix_t D = matrix_t::Zero(numIneqConstraint, centroidalModelInfo_.inputDim);
            D(0, 3*i+2) = 1.0;       // lower bound
            D(1, 3*i+2) = -1.0;       // upper bound
            const matrix_t C = matrix_t::Zero(numIneqConstraint, centroidalModelInfo_.stateDim);
            return std::unique_ptr<StateInputConstraint>(new LinearStateInputConstraint(e, C, D));
        };
        if (activateStability)
            problemPtr_->inequalityLagrangianPtr->add(footName + "_stability", create(getConstraint(), getPenalty()));
        problemPtr_->inequalityLagrangianPtr->add(footName + "_frictionCone", create(getFrictionConeConstraint(taskFile, i, verbose), getFrictionConePenalty()));
        problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", getZeroForceConstraint(i));
        problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity",
                                                getZeroVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
        if (referenceManagerPtr_->getSwingTrajectoryPlanner()->getConfig().addPlanarConstraints) {
            problemPtr_->equalityConstraintPtr->add(footName + "_velocityX",
                                                    getCoordinateVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints, 0));
            problemPtr_->equalityConstraintPtr->add(footName + "_velocityY",
                                                    getCoordinateVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints, 1));
        }
        problemPtr_->equalityConstraintPtr->add(footName + "_velocityZ",
                                                getCoordinateVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints, 2));
        // assign the kinematic pointer objects for the feet EE
        eeKinematicsPtrArray.at(i) = std::move(eeKinematicsPtr);
    }
    // for arm contacts, zero force and velocity do not apply
    // TODO: keep only one name for the frame to be tracked either from contacts or from targetFrame
    else {
        problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", getZeroForceConstraint(i));
        problemPtr_->inequalityLagrangianPtr->add(footName + "_frictionCone", create(getFrictionConeConstraint(taskFile, i, verbose), getFrictionConePenalty()));

        // arm EE tracking equality or soft constraints
        if (armEeSoftConstraints) {
            problemPtr_->stateSoftConstraintPtr->add(targetFramesNames[i - legContacts.size()] + "_eeSoftConstraint",
                                                     getEndEffectorConstraint(taskFile, "armEeSoftConstraints." + targetFramesNames[i - legContacts.size()] + "EndEffector"));
            problemPtr_->finalSoftConstraintPtr->add(targetFramesNames[i - legContacts.size()] + "_finalEeSoftConstraint",
                                                     getEndEffectorConstraint(taskFile, "armEeSoftConstraints." + targetFramesNames[i - legContacts.size()] + "EndEffector"));
        }
        if (armEeHardConstraints) {
            // add position hard constraints
            if (referenceManagerPtr_->getArmSwingTrajectoryPlanner()->isPositionPlanner() && !hConstraintPositionIndices[i - legContacts.size()].empty()) {
                for (auto& linearDoF: hConstraintPositionIndices[i - legContacts.size()]) {     // loop over XYZ
                    problemPtr_->equalityConstraintPtr->add(targetFramesNames[i - legContacts.size()] + "_velocity" + std::to_string(linearDoF),
                                                            getCoordinateVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints,
                                                                                            linearDoF, ConstraintActiveWhen::ALWAYS));
                }
            }
            // add orientation hard constraints
            if (referenceManagerPtr_->getArmSwingTrajectoryPlanner()->isOrientationPlanner() && !hConstraintOrientationIndices[i - legContacts.size()].empty()) {
                for (auto& angularDoF: hConstraintOrientationIndices[i - legContacts.size()]) {
                    problemPtr_->equalityConstraintPtr->add(targetFramesNames[i - legContacts.size()] + "_angularVelocity" + std::to_string(angularDoF),
                                                            getCoordinateAngularVelocityConstraint(*eeKinematicsPtr, i - legContacts.size(), useAnalyticalGradientsConstraints,
                                                                                                   angularDoF, ConstraintActiveWhen::ALWAYS));
                }
            }
        }
        armEeKinematicsPtrArray.at(i-legContacts.size()) = std::move(eeKinematicsPtr);
    }
  }

  // Limits for joint position, velocity
  vector_t maxJointVelocity(centroidalModelInfo_.actuatedDofNum), maxJointPosition(centroidalModelInfo_.actuatedDofNum),
          minJointPosition(centroidalModelInfo_.actuatedDofNum);
  auto jointLimitsFile = ros::package::getPath("ocs2_centauro") + "/config/command/joint_limits.info";
  loadData::loadEigenMatrix(jointLimitsFile, "jointVelocityLimits", maxJointVelocity);
  loadData::loadEigenMatrix(jointLimitsFile, "jointPositionLimits.upperBound", maxJointPosition);   // joint position limits
  loadData::loadEigenMatrix(jointLimitsFile, "jointPositionLimits.lowerBound", minJointPosition);

  auto getLimitsPenalty = [&](const std::string& configName) {      // penalty for joint position, velocity limits
    // one can use either augmented::SlacknessSquaredHingePenalty or augmented::ModifiedRelaxedBarrierPenalty
    penalty_type::Config boundsConfig;
    loadData::loadPenaltyConfig(taskFile, configName, boundsConfig, verbose);
    return penalty_type::create(boundsConfig);
  };

  bool activateVelocityLimits = false, activatePositionLimits = false;
  loadData::loadCppDataType(taskFile, "JointPositionLimits.activatePositionLimits",  activatePositionLimits);
  loadData::loadCppDataType(taskFile, "JointVelocityLimits.activateVelocityLimits",  activateVelocityLimits);
  // Position limits
  if (activatePositionLimits) {
    Eigen::Matrix<bool, Eigen::Dynamic, 1> jointPositionActivationFlags(centroidalModelInfo_.actuatedDofNum);
    loadData::loadEigenMatrix(jointLimitsFile, "jointPositionActivationFlags", jointPositionActivationFlags);
    auto getJointPositionConstraint = [&](int index) {
      constexpr size_t numIneqConstraint = 2;     // two-side bound
      const vector_t e = (vector_t(numIneqConstraint) << -minJointPosition[index], maxJointPosition[index]).finished();
      matrix_t C = matrix_t::Zero(numIneqConstraint, centroidalModelInfo_.stateDim);
      C.col(C.cols() - centroidalModelInfo_.actuatedDofNum + index) << 1.0, -1.0;  // lower, upper bound
      const matrix_t D = matrix_t::Zero(numIneqConstraint, centroidalModelInfo_.inputDim);
      return std::unique_ptr<StateInputConstraint>(new LinearStateInputConstraint(e, C, D));
    };
    for (int i = 0; i < centroidalModelInfo_.actuatedDofNum; i++) {        // loop over all actuated DoFs
      if (jointPositionActivationFlags[i]) {                             // if joint is selected
          problemPtr_->inequalityLagrangianPtr->add(modelSettings_.jointNames[i] + "_position_limits", create(getJointPositionConstraint(i), getLimitsPenalty("JointPositionLimits.config")));
      }
    }
  }

  // velocity limits
  if (activateVelocityLimits) {
      Eigen::Matrix<bool, Eigen::Dynamic, 1> jointVelocityActivationFlags(centroidalModelInfo_.actuatedDofNum);
      loadData::loadEigenMatrix(jointLimitsFile, "jointVelocityActivationFlags", jointVelocityActivationFlags);
      auto getJointVelocityConstraint = [&](int index) {
          constexpr size_t numIneqConstraint = 2;     // two-side bound
          const vector_t e = (vector_t(numIneqConstraint) << maxJointVelocity[index], maxJointVelocity[index]).finished();
          matrix_t D = matrix_t::Zero(numIneqConstraint, centroidalModelInfo_.inputDim);
          D.col(3*centroidalModelInfo_.numThreeDofContacts + index) << 1.0, -1.0;  // lower, upper bound
          const matrix_t C = matrix_t::Zero(numIneqConstraint, centroidalModelInfo_.stateDim);
          return std::unique_ptr<StateInputConstraint>(new LinearStateInputConstraint(e, C, D));
      };
      for (int i = 0; i < centroidalModelInfo_.actuatedDofNum; i++) {        // loop over all actuated DoFs
          if (jointVelocityActivationFlags[i]) {                             // if joint is selected
            problemPtr_->inequalityLagrangianPtr->add(modelSettings_.jointNames[i] + "_velocity_limits", create(getJointVelocityConstraint(i), getLimitsPenalty("JointVelocityLimits.config")));
          }
      }
  }

  // Pass Kinematics object to class member variables of reference manager
  referenceManagerPtr_->setEeKinematics(eeKinematicsPtrArray);
  referenceManagerPtr_->setArmEeKinematics(armEeKinematicsPtrArray);

  // self-collision avoidance constraint
  bool activateSelfCollision = false;
  loadData::loadCppDataType(taskFile, "selfCollision.activate",  activateSelfCollision);
  if (activateSelfCollision) {
    problemPtr_->stateSoftConstraintPtr->add(
        "selfCollision", getSelfCollisionConstraint(taskFile, urdfFile, "selfCollision"));
  }
  // Pre-computation
  if (!armEeHardConstraints) {      // soft constraint or locomotion, e.g. no hard constraint for arms
    problemPtr_->preComputationPtr.reset(new LeggedRobotPreComputation(*pinocchioInterfacePtr_, centroidalModelInfo_,
                                                                       *referenceManagerPtr_->getSwingTrajectoryPlanner(),
                                                                       modelSettings_));

  } else {
      problemPtr_->preComputationPtr.reset(new LeggedRobotPreComputation(*pinocchioInterfacePtr_, centroidalModelInfo_,
                                                                         *referenceManagerPtr_->getSwingTrajectoryPlanner(),
                                                                         *referenceManagerPtr_->getArmSwingTrajectoryPlanner(),
                                                                         modelSettings_));
  }

  // Just some printouts
  if (activateSelfCollision)
      std::cout << "[setupOCP] >>>>>>>> Self-collision avoidance ON" << std::endl;
  if (activatePositionLimits)
      std::cout << "[setupOCP] >>>>>>>> Joint position limits ON" << std::endl;
  if (activateVelocityLimits)
      std::cout << "[setupOCP] >>>>>>>> Joint velocity limits ON" << std::endl;
  if (armEeSoftConstraints)
      std::cout << "[setupOCP] >>>>>>>> Arm EE soft constraints ON" << std::endl;
  if (armEeHardConstraints)
      std::cout << "[setupOCP] >>>>>>>> Arm EE hard constraints ON" << std::endl;
  if (activateStability)
      std::cout << "[setupOCP] >>>>>>>> Leg EE stability constraints ON" << std::endl;
  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_.reset(new LeggedRobotInitializer(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedRobotInterface::loadGaitSchedule(const std::string& file, bool verbose) const {
  const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

  const auto defaultGait = [&] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  // display
  if (verbose) {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
    std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings_.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LeggedRobotInterface::initializeTaskSpaceInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info) {
  const size_t legContactVelDim = 3 * modelSettings_.legContactNames3DoF.size();
  const size_t totalForcesDim = 3 * info.numThreeDofContacts;
  const size_t legJointDim = 6 * modelSettings_.legContactNames3DoF.size();

  vector_t initialState(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState);        // jacobians at initial state

  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto q = centroidal_model::getGeneralizedCoordinates(initialState, centroidalModelInfo_);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // base to feet jacobian is 12 x legJointDim
  matrix_t baseToFeetJacobians(legContactVelDim, legJointDim);
  for (size_t i = 0; i < modelSettings_.legContactNames3DoF.size(); i++) {
    matrix_t jacobianWorldToContactPointInWorldFrame = matrix_t::Zero(6, info.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.legContactNames3DoF[i]), pinocchio::LOCAL_WORLD_ALIGNED,
                                jacobianWorldToContactPointInWorldFrame);

    baseToFeetJacobians.block(3 * i, 0, 3, legJointDim) =
        jacobianWorldToContactPointInWorldFrame.block(0, 6, 3, legJointDim);
  }

  // R taskspace is legContactVelDim x legContactVelDim
  matrix_t R_taskspace(legContactVelDim, legContactVelDim);
  loadData::loadEigenMatrix(taskFile, "R_taskspace", R_taskspace);

 // R is inputDim x inputDim
  matrix_t R = matrix_t::Zero(info.inputDim, info.inputDim);

  // cost on all joint velocities, all zero except the leg joints
  R.block(totalForcesDim, totalForcesDim, legJointDim, legJointDim) =
      baseToFeetJacobians.transpose() * R_taskspace * baseToFeetJacobians;
  return R;
}

/******************************************************************************************************/
/************************************ full input cost initialization **********************************/
/******************************************************************************************************/
matrix_t LeggedRobotInterface::initializeFullInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info) {
  const size_t totalContactDim = 3 * info.numThreeDofContacts;
  matrix_t R_j(totalContactDim + info.actuatedDofNum, totalContactDim + info.actuatedDofNum);
  loadData::loadEigenMatrix(taskFile, "R_j", R_j);

  return R_j;
}


/******************************************************************************************************/
/**************************** Cost on control in joint space ******************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getFullInputCost(const std::string& taskFile, const CentroidalModelInfo& info,
                                                                        bool verbose) {
  matrix_t Q(info.stateDim, info.stateDim);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  matrix_t R = initializeFullInputCostWeight(taskFile, info);

  if (verbose) {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(new LeggedRobotStateInputQuadraticCost(std::move(Q), std::move(R), info, *referenceManagerPtr_, taskFile));
}


/******************************************************************************************************/
/**************************** Cost on ee velocities in task space ******************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getTaskSpaceVelCost(const std::string& taskFile, const CentroidalModelInfo& info,
                                                                        bool verbose) {
  matrix_t Q = matrix_t::Zero(info.stateDim, info.stateDim);    // zero cost on state
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  matrix_t R = initializeTaskSpaceInputCostWeight(taskFile, info);

  if (verbose) {
    std::cerr << "\n #### Task Space EE velocities Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(new LeggedRobotStateInputQuadraticCost(std::move(Q), std::move(R), info, *referenceManagerPtr_, taskFile));
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::Config LeggedRobotInterface::loadFrictionConeSettings(const std::string& taskFile, const size_t& contactPointIndex,
                                                                              bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "frictionConeConstraint.";

  FrictionConeConstraint::Config frictionConeConfig;
  matrix_t allSurfaceNormals = matrix_t::Zero(3, centroidalModelInfo_.numThreeDofContacts);
  loadData::loadEigenMatrix(taskFile, prefix + "surfaceNormals", allSurfaceNormals);
  frictionConeConfig.surfaceNormal = allSurfaceNormals.col(contactPointIndex);
  if (verbose) {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### surfaceNormal (transpose) .........................................." << frictionConeConfig.surfaceNormal.transpose() << "\n";
  }
  loadData::loadPtreeValue(pt, frictionConeConfig.frictionCoefficient, prefix + "frictionCoefficient", verbose);
  loadData::loadPtreeValue(pt, frictionConeConfig.regularization, prefix + "regularization", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return std::move(frictionConeConfig);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getFrictionConeConstraint(const std::string& taskFile, const size_t& contactPointIndex, bool verbose) const {
  // friction cone settings
  FrictionConeConstraint::Config frictionConeConfig(0.7, 25.0, 0.0, 1e-6, -vector3_t::UnitZ());
  frictionConeConfig = loadFrictionConeSettings(taskFile, contactPointIndex, true);
  std::unique_ptr<FrictionConeConstraint> frictionConeConstraintPtr(
                    new FrictionConeConstraint(*referenceManagerPtr_, frictionConeConfig,
                                               contactPointIndex, centroidalModelInfo_));
  frictionConeConstraintPtr->setSurfaceNormalInWorld(frictionConeConfig.surfaceNormal);
  return frictionConeConstraintPtr;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config> LeggedRobotInterface::loadUnilateralConstraintSettings(const std::string& taskFile,
                                                                                                          bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "unilateralConstraint.";

  scalar_t minimumNormalForce = 1.0;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Unilateral Costraint Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, minimumNormalForce, prefix + "minimumNormalForce", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return {minimumNormalForce, std::move(barrierPenaltyConfig)};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getUnilateralConstraint(const size_t& contactPointIndex, scalar_t minimumNormalForce,
                                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  UnilateralConstraint::Config unilateralConConfig(minimumNormalForce);
  std::unique_ptr<UnilateralConstraint> unilateralConstraintPtr(
      new UnilateralConstraint(*referenceManagerPtr_, std::move(unilateralConConfig), contactPointIndex, centroidalModelInfo_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(unilateralConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getZeroForceConstraint(const size_t& contactPointIndex) {
  return std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(*referenceManagerPtr_, contactPointIndex, centroidalModelInfo_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                      const size_t& contactPointIndex,
                                                                                      bool useAnalyticalGradients) {
  auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
    EndEffectorLinearConstraint::Config config;
    config.b.setZero(3);
    config.Av.setIdentity(3, 3);
    if (!numerics::almost_eq(positionErrorGain, 0.0)) {
      config.Ax.setZero(3, 3);
      config.Ax(2, 2) = positionErrorGain;
    }
    return config;
  };

  if (useAnalyticalGradients) {
    throw std::runtime_error(
        "[LeggedRobotInterface::getZeroVelocityConstraint] The analytical end-effector zero velocity constraint is not implemented!");
  } else {
    return std::unique_ptr<StateInputConstraint>(new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                                                                 eeZeroVelConConfig(modelSettings_.positionErrorGain)));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getCoordinateVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                            const size_t& contactPointIndex, bool useAnalyticalGradients,
                                                                                            const size_t coordinateNumber,
                                                                                            const ConstraintActiveWhen activationState) {
  if (useAnalyticalGradients) {
    throw std::runtime_error(
        "[LeggedRobotInterface::getCoordinateVelocityConstraint] The analytical end-effector normal velocity constraint is not implemented!");
  } else {
    return std::unique_ptr<StateInputConstraint>(new CoordinateVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex, coordinateNumber, activationState));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getCoordinateAngularVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                                   const size_t& contactPointIndex, bool useAnalyticalGradients,
                                                                                                   const size_t coordinateNumber,
                                                                                                   const ConstraintActiveWhen activationState) {
  if (useAnalyticalGradients) {
    throw std::runtime_error(
        "[LeggedRobotInterface::getCoordinateVelocityConstraint] The analytical end-effector normal velocity constraint is not implemented!");
  } else {
    return std::unique_ptr<StateInputConstraint>(new CoordinateAngularVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex, coordinateNumber, activationState));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> LeggedRobotInterface::getEndEffectorConstraint(const std::string& taskFile, const std::string& prefix) {
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  std::string name;                 // target frame name
  int endEffectorBufferIndex;       // index of target frame's buffer

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  loadData::loadPtreeValue(pt, name, prefix + ".frameName", true);
  loadData::loadPtreeValue(pt, endEffectorBufferIndex, prefix + ".bufferIndex", true);
  std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }

  // define EE kinematics object
  // TODO: receive eeKinematics from setupOptimalControlProblem
  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;
  const auto infoCppAd = centroidalModelInfo_.toCppAd();
  const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
  auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
    const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
    updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
  };
  eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {name},
                                                                centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                velocityUpdateCallback, name, modelSettings_.modelFolderCppAd,
                                                                modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

  std::unique_ptr<StateConstraint> constraint;
  constraint.reset(new EndEffectorConstraint(*eeKinematicsPtr, *referenceManagerPtr_, endEffectorBufferIndex));

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> LeggedRobotInterface::getSelfCollisionConstraint(const std::string& taskFile, const std::string& urdfFile,
                                                                            const std::string& prefix) {
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.0;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### SelfCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
  std::cerr << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(*pinocchioInterfacePtr_, collisionLinkPairs, collisionObjectPairs, urdfFile);

  const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();
  std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

  std::unique_ptr<StateConstraint> constraint;

  // no caching/no precomputations
  const CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo_);

  // CppAd update callback
//  const auto infoCppAd = centroidalModelInfo_.toCppAd();
//  auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
//    const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
//    updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
//  };

  // Non-CppAd update callback
  auto velocityUpdateCallback = [this](const vector_t& state, PinocchioInterface& pinocchioInterface) {
    const vector_t q = centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_);
    updateCentroidalDynamics(pinocchioInterface, centroidalModelInfo_, q);
  };
  // strangely SelfCollisionConstraintCppAd does not accept CentroidalModelPinocchioMappingCppAd
  constraint = std::unique_ptr<StateConstraint>(new SelfCollisionConstraintCppAd(
      *pinocchioInterfacePtr_, pinocchioMapping, std::move(geometryInterface), minimumDistance,
      velocityUpdateCallback, "self_collision", modelSettings_.modelFolderCppAd, modelSettings_.recompileLibrariesCppAd, true));
  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty({mu, delta}));
  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penalty)));
}

}  // namespace legged_robot
}  // namespace ocs2
