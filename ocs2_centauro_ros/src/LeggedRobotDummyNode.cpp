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

#include <ros/init.h>
#include <ros/package.h>
#include <ctime>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centauro/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <gazebo_ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <gazebo_ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "ocs2_centauro_ros/visualization/LeggedRobotVisualizer.h"
#include "ocs2_centauro_ros/torque_mapping/JointImpedanceMappedPolicyPublisher.h"
#include "gazebo_ocs2_ros_interfaces/xbot2/XbotInterface.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string taskFile, urdfFile, referenceFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/referenceFile", referenceFile);

  // copy taskfile file
  std::time_t t = std::time(0);   // get time now
  std::tm* now = std::localtime(&t);
  std::string dateTime = std::to_string(now->tm_year + 1900) +
          '-' + std::to_string(now->tm_mon + 1) + '-' + std::to_string(now->tm_mday) +
          "-" + std::to_string(now->tm_hour) + "-" + std::to_string(now->tm_min) +
          "-" + std::to_string(now->tm_sec);
  std::string com = std::string("cp ") + taskFile + " " + "/tmp/taskFile_" + dateTime + ".info";
  int systemRet = system(com.c_str());
  if (systemRet == -1)
    std::cout << "[Error] may not succeeded to save .info taskfile." << std::endl;

  // Robot interface
//  LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);
  std::shared_ptr<LeggedRobotInterface> interfacePtr(new LeggedRobotInterface(taskFile, urdfFile, referenceFile));

  // get target frame number and names
  std::vector<std::string> targetFramesNames;
  loadData::loadStdVector(taskFile, "targetFramesNames", targetFramesNames);
  int targetFramesNumber = targetFramesNames.size();

  // xbotcore config and initial state
  vector_t initialState = vector_t::Constant(interfacePtr->getCentroidalModelInfo().stateDim, 0);
  xbot_interface::XbotInterface::Config xbotConfig(false, false, "/xbotcore/link_state/pelvis", false, false, false);
  loadData::loadCppDataType(taskFile, "xbotcore.xbotCoreRunning", xbotConfig.xbotCoreRunning);
  if (!xbotConfig.xbotCoreRunning) {
    std::cout << "Dummy node will get initial state from .info file." << std::endl;
    loadData::loadEigenMatrix(taskFile, "initialState", initialState);
  }
  else {
    loadData::loadCppDataType(taskFile, "xbotcore.xbotCoreFeedback", xbotConfig.xbotCoreFeedback);
    std::cout << "Dummy node will launch a XbotInterface." << std::endl;
    if (xbotConfig.xbotCoreFeedback)
        std::cout << "The mpc will also receive feedback from xbotcore." << std::endl;
    loadData::loadCppDataType(taskFile, "xbotcore.baseLinkTopicPrefix", xbotConfig.baseLinkTopicPrefix);
    loadData::loadCppDataType(taskFile, "xbotcore.clampTorqueCmd", xbotConfig.clampTorqueCmd);
    loadData::loadCppDataType(taskFile, "xbotcore.clampPositionCmd", xbotConfig.clampPositionCmd);
    loadData::loadCppDataType(taskFile, "xbotcore.clampVelocityCmd", xbotConfig.clampVelocityCmd);

    xbot_interface::XbotInterface xbotInterface(nodeHandle, interfacePtr->getPinocchioInterface(),
                                                interfacePtr->getCentroidalModelInfo(),
                                                interfacePtr->modelSettings().jointNames, xbotConfig);
    xbotInterface.getInitialMsg();
    initialState = xbotInterface.getCentroidalStateFromXbotInfo();       // correct
  }
  interfacePtr->setInitialState(initialState);

  // publish joint impedance mapped policy
  bool publishJointImpedancePolicy;
  loadData::loadCppDataType(taskFile, "publishJointImpedancePolicy",  publishJointImpedancePolicy);

  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interfacePtr->getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization, leggedRobotVisualizer is a DummyObserver, i.e. an observer of the dummy loop
  CentroidalModelPinocchioMapping pinocchioMapping(interfacePtr->getCentroidalModelInfo());

  PinocchioEndEffectorKinematics endEffectorKinematics(interfacePtr->getPinocchioInterface(), pinocchioMapping,
                                                       interfacePtr->modelSettings().contactNames3DoF);

  // mrt observers (visualizer, impedance mapped policy publisher)
  std::vector<std::shared_ptr<DummyObserver>> mrtObservers;
  if (publishJointImpedancePolicy) {
      std::shared_ptr<JointImpedanceMappedPolicyPublisher> mappedImpedancePublisher(
          new JointImpedanceMappedPolicyPublisher(interfacePtr->getPinocchioInterface(), interfacePtr->getCentroidalModelInfo(), nodeHandle));
      mrtObservers.emplace_back(std::move(mappedImpedancePublisher));
  }
  std::shared_ptr<LeggedRobotVisualizer> leggedRobotVisualizer(
      new LeggedRobotVisualizer(interfacePtr->getPinocchioInterface(), interfacePtr->getCentroidalModelInfo(), endEffectorKinematics, nodeHandle));
  mrtObservers.emplace_back(leggedRobotVisualizer);

  // Dummy legged robot
  MRT_ROS_Dummy_Loop leggedRobotDummySimulator(mrt, interfacePtr->mpcSettings().mrtDesiredFrequency_, interfacePtr,
                                               nodeHandle, xbotConfig, interfacePtr->mpcSettings().mpcDesiredFrequency_);
  leggedRobotDummySimulator.subscribeObservers(mrtObservers);    // Subscribe leggedRobotVisualizer to observe leggedRobotDummySimulator

  // Initial state
  SystemObservation initObservation;
  initObservation.state = interfacePtr->getInitialState();

  initObservation.input = vector_t::Zero(interfacePtr->getCentroidalModelInfo().inputDim);
  initObservation.mode = ModeNumber::STANCE;

  // ee target, initial targets coincide with homing configuration
  auto getArmEeFramePose = [&](size_t armIndex) {
      auto armKinematics = interfacePtr->getSwitchedModelReferenceManagerPtr()->getArmEeKinematics()[armIndex];
      return (vector_t(7) << armKinematics->getPosition(initialState)[0],
                             armKinematics->getOrientation(initialState)[0]).finished();
  };

  // Initial command
  TargetTrajectories initTargetTrajectories({0.0}, {initialState}, {initObservation.input});
  TargetTrajectories initLeftEndEffectorTargetTrajectories({1.0}, {getArmEeFramePose(0)}, {initObservation.input});
  TargetTrajectories initRightEndEffectorTargetTrajectories({1.0}, {getArmEeFramePose(1)}, {initObservation.input});

  // run dummy
  leggedRobotDummySimulator.run(initObservation, initTargetTrajectories,
                                {initLeftEndEffectorTargetTrajectories, initRightEndEffectorTargetTrajectories});

  // Successful exit
  return 0;
}
