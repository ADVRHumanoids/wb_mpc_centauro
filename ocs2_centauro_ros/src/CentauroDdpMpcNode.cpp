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

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_centauro/LeggedRobotInterface.h>
#include <gazebo_ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <gazebo_ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include "ocs2_centauro_ros/gait/GaitReceiver.h"
#include "gazebo_ocs2_ros_interfaces/xbot2/XbotInterface.h"

#include "ocs2_centauro_ros/sensing/EstimatedWrenchReceiver.h"
#include "ocs2_centauro_ros/sensing/JointStatesReceiver.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string taskFile, urdfFile, referenceFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/urdfFile", urdfFile);

  // Robot interface
  LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

  // get target frame number and names
  std::vector<std::string> targetFramesNames;
  loadData::loadStdVector(taskFile, "targetFramesNames", targetFramesNames);

  // xbotcore configuration and initial state
  xbot_interface::XbotInterface::Config xbotConfig(false, false, "/xbotcore/link_state/pelvis", false, false, false);
  vector_t initialState(interface.getCentroidalModelInfo().stateDim);
  loadData::loadCppDataType(taskFile, "xbotcore.xbotCoreRunning", xbotConfig.xbotCoreRunning);
  if (!xbotConfig.xbotCoreRunning) {
    std::cout << "MPC node will get initial state from .info file." << std::endl;
    loadData::loadEigenMatrix(taskFile, "initialState", initialState);
  }
  else {
    std::cout << "MPC node will launch a XbotInterface." << std::endl;
    loadData::loadCppDataType(taskFile, "xbotcore.baseLinkTopicPrefix", xbotConfig.baseLinkTopicPrefix);
    loadData::loadCppDataType(taskFile, "xbotcore.xbotCoreFeedback", xbotConfig.xbotCoreFeedback);
    loadData::loadCppDataType(taskFile, "xbotcore.clampTorqueCmd", xbotConfig.clampTorqueCmd);
    loadData::loadCppDataType(taskFile, "xbotcore.clampPositionCmd", xbotConfig.clampPositionCmd);
    loadData::loadCppDataType(taskFile, "xbotcore.clampVelocityCmd", xbotConfig.clampVelocityCmd);
    xbot_interface::XbotInterface xbotInterface(nodeHandle, interface.getPinocchioInterface(),
                                                interface.getCentroidalModelInfo(),
                                                interface.modelSettings().jointNames, xbotConfig);
    xbotInterface.getInitialMsg();
    initialState = xbotInterface.getCentroidalStateFromXbotInfo();
  }
  interface.setInitialState(initialState);

  // wrench or joint state to be passed to mpc through synchronized modules
  bool eeWrenchSensing = false;
  bool jointSensing = false;
  if (xbotConfig.xbotCoreRunning) {     // first check that there is xbotcore
      loadData::loadCppDataType(taskFile, "forceTorqueSensingInMpc.estimatedWrenches", eeWrenchSensing);
      loadData::loadCppDataType(taskFile, "forceTorqueSensingInMpc.jointStates", jointSensing);
  }

  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle, targetFramesNames);

  // MPC
  GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(), interface.getOptimalControlProblem(),
                         interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  if (eeWrenchSensing) {      // Wrench receiver
      auto wrenchesReceiverPtr =
          std::make_shared<EstimatedWrenchReceiver>(nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getForceTorqueSensingPtr());
      mpc.getSolverPtr()->addSynchronizedModule(wrenchesReceiverPtr);
  }
  if (jointSensing) {         // joint states receiver
      auto jointStatesReceiverPtr =
          std::make_shared<JointStatesReceiver>(nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getForceTorqueSensingPtr());
      mpc.getSolverPtr()->addSynchronizedModule(jointStatesReceiverPtr);
  }

  // Launch MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);

  // Successful exit
  return 0;
}
