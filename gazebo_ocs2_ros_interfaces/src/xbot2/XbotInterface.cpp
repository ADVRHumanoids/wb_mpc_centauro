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

#include "gazebo_ocs2_ros_interfaces/xbot2/XbotInterface.h"
#include "gazebo_ocs2_ros_interfaces/common/Kinematics.h"
#include <eigen_conversions/eigen_msg.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <xbot_msgs/JointCommand.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ros/package.h>

namespace ocs2 {
namespace xbot_interface {

/*
 * Map from baselinkeTopicPrefix to integer
 */
std::map<std::string, int> mapBaseLinkTopicPrefix = {
    {"/xbotcore/link_state/pelvis", 0},
    {"/centauro_base_estimation/base_link", 1}
};

/*
 * Map from baselinkeTopicPrefix to message type
 */
std::map<std::string, std::string> mapBaseLinkTopicPrefixToMsgType = {
    {"/xbotcore/link_state/pelvis", "geometry_msgs::PoseStamped"},
    {"/centauro_base_estimation/base_link", "geometry_msgs::TransformedStamped"}
};

/*
 * Map from joint name to an index characterizing motor size
 * 0 -> small, 1 -> medium, 2 -> big
 */
std::map<std::string, int> mapJointNameToMotorSizeIndex = {
    {"hip_yaw_1", 2}, {"hip_pitch_1", 2}, {"knee_pitch_1", 2}, {"ankle_pitch_1", 1}, {"ankle_yaw_1", 1}, {"j_wheel_1", 1},
    {"hip_yaw_2", 2}, {"hip_pitch_2", 2}, {"knee_pitch_2", 2}, {"ankle_pitch_2", 1}, {"ankle_yaw_2", 1}, {"j_wheel_2", 1},
    {"hip_yaw_3", 2}, {"hip_pitch_3", 2}, {"knee_pitch_3", 2}, {"ankle_pitch_3", 1}, {"ankle_yaw_3", 1}, {"j_wheel_3", 1},
    {"hip_yaw_4", 2}, {"hip_pitch_4", 2}, {"knee_pitch_4", 2}, {"ankle_pitch_4", 1}, {"ankle_yaw_4", 1}, {"j_wheel_4", 1},

    {"torso_yaw", 2},

    {"j_arm1_1", 1}, {"j_arm1_2", 1}, {"j_arm1_3", 1}, {"j_arm1_4", 1}, {"j_arm1_5", 0}, {"j_arm1_6", 0}, {"j_arm1_7", 0},
    {"j_arm2_1", 1}, {"j_arm2_2", 1}, {"j_arm2_3", 1}, {"j_arm2_4", 1}, {"j_arm2_5", 0}, {"j_arm2_6", 0}, {"j_arm2_7", 0},

    {"neck_yaw", 0}, {"neck_pitch", 0}, {"neck_velodyne", 0}, {"d435_head_joint", 0}, {"velodyne_joint", 0}
};

/*
 * joints belonging to lower body
 */
std::vector<std::string> lowerBodyJoints = {
    "hip_yaw_1", "hip_pitch_1", "knee_pitch_1", "ankle_pitch_1", "ankle_yaw_1", "j_wheel_1",
    "hip_yaw_2", "hip_pitch_2", "knee_pitch_2", "ankle_pitch_2", "ankle_yaw_2", "j_wheel_2",
    "hip_yaw_3", "hip_pitch_3", "knee_pitch_3", "ankle_pitch_3", "ankle_yaw_3", "j_wheel_3",
    "hip_yaw_4", "hip_pitch_4", "knee_pitch_4", "ankle_pitch_4", "ankle_yaw_4", "j_wheel_4"
};

// TODO: check which shared_ptr can be unique
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
XbotInterface::XbotInterface(ros::NodeHandle& nodeHandle, PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                             std::vector<std::string> jointNames, Config config)
    : config_(config),
      jointPos_(vector_t(centroidalModelInfo.actuatedDofNum)),
      jointPosRef_(vector_t(centroidalModelInfo.actuatedDofNum)),
      jointVel_(vector_t(centroidalModelInfo.actuatedDofNum)),
      baseTwist_(vector_t(6)),
      basePoseVector_(vector_t(6)),
      baseOrientation_(Eigen::Quaterniond::Identity()),
      continuousBaseOrientation_(vector_t(3)),
      eeEstimatedWrenches_(matrix_t::Zero(6, centroidalModelInfo.numThreeDofContacts + centroidalModelInfo.numSixDofContacts)),
      nodeHandle_(nodeHandle),
      centroidalModelRbdConversions_(pinocchioInterface, centroidalModelInfo),
      jointEffortLimits_(vector_t(jointNames_.size())), jointPositionLowerLimits_(vector_t(jointNames_.size())),  // joint limits
      jointPositionUpperLimits_(vector_t(jointNames_.size())), jointVelocityLimits_(vector_t(jointNames_.size())),
      jointNames_(std::move(jointNames))
{
  // temporarily initialize xbotJointNames equal to centroidal model joint Names
  // as soon as a xbot message is received this will be updated
  xbotJointNames_ = jointNames_;

  // load PD gains for control
  std::vector<float> lowerBodyPGains(3), lowerBodyDGains(3), upperBodyPGains(3), upperBodyDGains(3);
  std::string jointImpedanceGainsPrefix("mrt_joint_impedance_gains");
  nodeHandle_.getParamCached(jointImpedanceGainsPrefix + "/lower_body/p_gains", lowerBodyPGains);
  nodeHandle_.getParamCached(jointImpedanceGainsPrefix + "/lower_body/d_gains", lowerBodyDGains);
  nodeHandle_.getParamCached(jointImpedanceGainsPrefix + "/upper_body/p_gains", upperBodyPGains);
  nodeHandle_.getParamCached(jointImpedanceGainsPrefix + "/upper_body/d_gains", upperBodyDGains);
  const auto lowerBodyJointImpedanceGains = std::make_pair(lowerBodyPGains, lowerBodyDGains);
  const auto upperBodyJointImpedanceGains = std::make_pair(upperBodyPGains, upperBodyDGains);
  orderedJointImpedanceGains_ = getJointImpedanceGains(lowerBodyJointImpedanceGains, upperBodyJointImpedanceGains);

  // urdf model for limits
  std::string urdfString;
  nodeHandle.getParam("/urdfFile", urdfString);
  std::shared_ptr<const ::urdf::ModelInterface> urdfPtr = urdf::parseURDFFile(urdfString);   //      urdfPtr(leggedRobotInterfacePtr_->getPinocchioInterface().getUrdfModelPtr()),
  for (int ith_joint = 0; ith_joint < jointNames_.size(); ith_joint++) {
      jointEffortLimits_[ith_joint] = urdfPtr->getJoint(jointNames_.at(ith_joint))->limits->effort - 3.0;
      jointPositionLowerLimits_[ith_joint] = urdfPtr->getJoint(jointNames_.at(ith_joint))->limits->lower;
      jointPositionUpperLimits_[ith_joint] = urdfPtr->getJoint(jointNames_.at(ith_joint))->limits->upper;
      jointVelocityLimits_[ith_joint] = urdfPtr->getJoint(jointNames_.at(ith_joint))->limits->velocity;
  }

  // print configuration
  ROS_INFO("Successfully constructed XbotInterface.");
  std::cerr << "\n #### XbotInterface Config: ";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << " #### 'xbotCoreRunning'............................................................." << config.xbotCoreRunning << "\n";
  std::cerr << " #### 'xbotCoreFeedback'............................................................" << config.xbotCoreFeedback << "\n";
  std::cerr << " #### 'baseLinkTopicPrefix'......................." << config.baseLinkTopicPrefix << "\n";
  std::cerr << " #### 'clampTorqueCmd'.............................................................." << config.clampTorqueCmd << "\n";
  std::cerr << " #### 'clampPositionCmd'............................................................" << config.clampPositionCmd << "\n";
  std::cerr << " #### 'clampVelocityCmd'............................................................" << config.clampVelocityCmd << "\n";
  std::cerr << " #### =============================================================================\n";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void XbotInterface::getInitialMsg() {
  ROS_INFO("Get initial ros message.");

  const std::string basePoseTopic = config_.baseLinkTopicPrefix + "/pose";
  const std::string baseTwistTopic = config_.baseLinkTopicPrefix + "/twist";

  // ros messages
  boost::shared_ptr<xbot_msgs::JointState const> msgJointStatesPtr;
  boost::shared_ptr<geometry_msgs::TwistStamped const> msgBaseTwistPtr;
//  boost::shared_ptr<base_estimation::ContactsWrench const> msgContactsWrenchPtr;      // uncomment for estimated wrench
//  msgContactsWrenchPtr = ros::topic::waitForMessage<base_estimation::ContactsWrench>(contactsWrenchTopicName_, nodeHandle_);
//  this->onEeWrenchReceived(msgContactsWrenchPtr);

  if (mapBaseLinkTopicPrefix[config_.baseLinkTopicPrefix] == 0) {
    boost::shared_ptr<geometry_msgs::PoseStamped const> msgBasePosePtr;
    msgBasePosePtr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(basePoseTopic, nodeHandle_);
    msgJointStatesPtr = ros::topic::waitForMessage<xbot_msgs::JointState>(jointStatesTopicName_, nodeHandle_);
    msgBaseTwistPtr = ros::topic::waitForMessage<geometry_msgs::TwistStamped>(baseTwistTopic, nodeHandle_);
    ROS_INFO("Received xbotcore messages.");

    // callbacks
    this->onJointStateReceived(msgJointStatesPtr);
    this->onBasePoseStampedReceived(msgBasePosePtr);
    this->onBaseTwistReceived(msgBaseTwistPtr);
    ROS_INFO("Passed messages to callbacks..");
  }
  else if (mapBaseLinkTopicPrefix[config_.baseLinkTopicPrefix] == 1) {
    boost::shared_ptr<geometry_msgs::TransformStamped const> msgBasePosePtr;
    msgBasePosePtr = ros::topic::waitForMessage<geometry_msgs::TransformStamped>(basePoseTopic, nodeHandle_);
    msgJointStatesPtr = ros::topic::waitForMessage<xbot_msgs::JointState>(jointStatesTopicName_, nodeHandle_);
    msgBaseTwistPtr = ros::topic::waitForMessage<geometry_msgs::TwistStamped>(baseTwistTopic, nodeHandle_);
    ROS_INFO("Received base estimation messages.");


    // callbacks
    this->onJointStateReceived(msgJointStatesPtr);
    this->onBasePoseTransformReceived(msgBasePosePtr);
    this->onBaseTwistReceived(msgBaseTwistPtr);
    ROS_INFO("Passed messages to callbacks..");
  }
  else ROS_ERROR("No match for baseLinkTopicPrefix");

  // set xbotJointNames_ from the received message from xbot
  xbotJointNames_ = msgJointStatesPtr->name;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t XbotInterface::getCentroidalStateFromXbotInfo() {
//    TODO: should I add std::lock_guard<std::mutex>(//mutex1); ?
    // RbdState = [base pose, joint positions, base twist, joint velocities]
    vector_t rbdState(baseTwist_.rows() + jointPos_.rows() + jointVel_.rows() + basePoseVector_.rows());    // get size from basePose

    // assign rbd State, angular/orientation precedes linear/position
    rbdState << basePoseVector_.bottomRows(3), basePoseVector_.topRows(3), jointPos_, baseTwist_.bottomRows(3), baseTwist_.topRows(3), jointVel_;

    vector_t centroidalState = centroidalModelRbdConversions_.computeCentroidalStateFromRbdModel(rbdState);

//    std::cout << "Centroidal state from XbotCore: " << centroidalState << std::endl;
    return centroidalState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/         
SystemObservation XbotInterface::getObservationFromXbot(const SystemObservation& currentObservation,
                                                        const scalar_t& interfaceFrequency,
                                                        const std::shared_ptr<legged_robot::LeggedRobotInterface> leggedRobotInterfacePtr,
                                                        const MRT_ROS_Interface& mrt) {
  const scalar_t dt = 1.0 / interfaceFrequency;

  SystemObservation nextObservation;
  nextObservation.time = currentObservation.time + dt;
  vector_t modifiedState = getCentroidalStateFromXbotInfo();
  nextObservation.state = modifiedState;      // this actually sets the state from xbot
  nextObservation.input.setZero(leggedRobotInterfacePtr->getCentroidalModelInfo().inputDim);      // set zero input as work-around
  nextObservation.mode = mrt.getActivePrimalSolution().modeSchedule_.modeAtTime(nextObservation.time);

  return nextObservation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void XbotInterface::connectToXbot() {

  // load pd gains of simulation that may be used or maybe not
  std::string taskFile, jointGainsFile;                       // TODO: add gains for real hardware
  nodeHandle_.getParam("/taskFile", taskFile);
  jointGainsFile = ros::package::getPath("ocs2_centauro") + "/config/control/impedance_gains_simulation.info";
  centroidalModelRbdConversions_.loadSettings(jointGainsFile, "simulation");

  // publisher and subscribers
  jointCmdPublisher_ = nodeHandle_.advertise<xbot_msgs::JointCommand>("/xbotcore/command", 1);   // publisher to /xbotcore/command
  jointStateSubscriber_ = nodeHandle_.subscribe(jointStatesTopicName_, 1,
                                               &XbotInterface::onJointStateReceived, this,
                                               ros::TransportHints().tcpNoDelay());

  baseTwistSubscriber_ = nodeHandle_.subscribe(config_.baseLinkTopicPrefix + "/twist", 1,
                                              &XbotInterface::onBaseTwistReceived, this,
                                              ros::TransportHints().tcpNoDelay());

//  eeWrenchSubscriber_ = nodeHandle_.subscribe(contactsWrenchTopicName_, 1,        // uncomment for estimated wrench
//                                               &XbotInterface::onEeWrenchReceived, this,
//                                               ros::TransportHints().tcpNoDelay());

  if (mapBaseLinkTopicPrefix[config_.baseLinkTopicPrefix] == 0) {
      basePoseSubscriber_ = nodeHandle_.subscribe(config_.baseLinkTopicPrefix + "/pose", 1,
                                                 &XbotInterface::onBasePoseStampedReceived, this,
                                                 ros::TransportHints().tcpNoDelay());
      ROS_INFO("Base pose subscriber from xbotcore topic");
  }
  else if (mapBaseLinkTopicPrefix[config_.baseLinkTopicPrefix] == 1) {
      basePoseSubscriber_ = nodeHandle_.subscribe(config_.baseLinkTopicPrefix + "/pose", 1,
                                                 &XbotInterface::onBasePoseTransformReceived, this,
                                                 ros::TransportHints().tcpNoDelay());
      ROS_INFO("Base pose subscriber from base estimation topic");
  }
  else ROS_ERROR("No match for baseLinkTopicPrefix.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void XbotInterface::sendPositionCommandToXbot(const std::shared_ptr<legged_robot::LeggedRobotInterface> leggedRobotInterfacePtr,
                                              const vector_t& optimalState) {
    xbot_msgs::JointCommand cmd;
    // fill and publish xbotcore msg after getting optimal state
    int numberJoints = leggedRobotInterfacePtr->getCentroidalModelInfo().actuatedDofNum;
    cmd.name = leggedRobotInterfacePtr->modelSettings().jointNames;
    // deploy position control
    const auto qJoints = centroidal_model::getJointAngles(optimalState, leggedRobotInterfacePtr->getCentroidalModelInfo());

    // open loop
    cmd.position.assign(qJoints.data(), qJoints.data() + qJoints.size());
    cmd.ctrl_mode.assign(numberJoints, 1);  // 1 position, 2 velocity, 4 effort, 8 stiffness, 16 damping

    jointCmdPublisher_.publish(cmd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void XbotInterface::clampReferences(vector_t& ffTorques, vector_t& jointPositions, vector_t& jointVelocities) {
    // clamp ff torques
    if (config_.clampTorqueCmd) {
        ffTorques = ffTorques.cwiseMin(jointEffortLimits_).cwiseMax(- jointEffortLimits_);
    }

    // clamp position
    if (config_.clampPositionCmd) {
        // dposref/dt < vel limit
//        jointPositions = jointPositions.cwiseMin(jointVelocityLimits_ * (1.0 / 500.0) + jointPosRef_).cwiseMax(- jointVelocityLimits_ * (1.0 / 500.0) + jointPosRef_);
        jointPositions = jointPositions.cwiseMin(jointPositionUpperLimits_).cwiseMax(jointPositionLowerLimits_);
    }

    // clamp velocities
    if (config_.clampVelocityCmd) {
        jointVelocities = jointVelocities.cwiseMin(jointVelocityLimits_).cwiseMax(- jointVelocityLimits_);
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void XbotInterface::sendCommandToXbot(const std::shared_ptr<legged_robot::LeggedRobotInterface> leggedRobotInterfacePtr,
                                      const vector_t& optimalState, const vector_t& optimalInput) {
    xbot_msgs::JointCommand cmd;

    // fill and publish xbotcore msg after getting optimal state
    int numberJoints = leggedRobotInterfacePtr->getCentroidalModelInfo().actuatedDofNum;
    cmd.name = leggedRobotInterfacePtr->modelSettings().jointNames;
    // position & velocity
    vector_t qJoints = centroidal_model::getJointAngles(optimalState, leggedRobotInterfacePtr->getCentroidalModelInfo());
    vector_t dqJoints = centroidal_model::getJointVelocities(optimalInput, leggedRobotInterfacePtr->getCentroidalModelInfo());

//        std::cout << "[MRT_ROS_Dummy_Loop] observated state = " << currentObservation.state.transpose() << std::endl;
//        std::cout << "[MRT_ROS_Dummy_Loop] optimal state = " << optimalState.transpose() << std::endl;
//        std::cout << "[MRT_ROS_Dummy_Loop] norm " << (optimalState - currentObservation.state).norm() << std::endl;

    // compute torques with inverse dynamics
    vector_t ffModelTorque = centroidalModelRbdConversions_.computeRbdTorqueFromCentroidalModel(optimalState, optimalInput, vector_t::Zero(numberJoints));
    vector_t ffJointTorque = ffModelTorque.tail(numberJoints);      // ignore base wrench

    // clamp joint commands if true from task.info file
    clampReferences(ffJointTorque, qJoints, dqJoints);

    // get the desired joint impedance gains from the user
    const auto stiffness = orderedJointImpedanceGains_.first;
    const auto damping = orderedJointImpedanceGains_.second;

    // fill message
    cmd.position.assign(qJoints.data(), qJoints.data() + qJoints.size());
    cmd.velocity.assign(dqJoints.data(), dqJoints.data() + dqJoints.size());
    cmd.effort.assign(ffJointTorque.data(), ffJointTorque.data() + ffJointTorque.size());
    cmd.stiffness.assign(stiffness.data(), stiffness.data() + stiffness.size());
    cmd.damping.assign(damping.data(), damping.data() + damping.size());

    cmd.ctrl_mode.assign(numberJoints, 1 +2 +4 +8 +16);  // 1 position, 2 velocity, 4 effort, 8 stiffness, 16 damping

    // keep default control for the wheels
    std::vector<int> wheelIndices(4);
    for (int wheel_i = 1; wheel_i < 5; wheel_i++)
        wheelIndices[wheel_i - 1] = std::distance(cmd.name.begin(), std::find(cmd.name.begin(), cmd.name.end(), "j_wheel_" + std::to_string(wheel_i)));
    for (auto index: wheelIndices) {
        cmd.position.at(index) = 0.0;
        cmd.effort.at(index) = 0.0;
        cmd.stiffness.at(index) = 0.0;
        cmd.damping.at(index) = 10.0;
    }

    // just a try for extreme high poses
//    std::vector<int> arm1Indices(6);
//    for (int arm_i = 1; arm_i < 7; arm_i++)
//        arm1Indices[arm_i - 1] = std::distance(cmd.name.begin(), std::find(cmd.name.begin(), cmd.name.end(), "j_arm1_" + std::to_string(arm_i)));
//    for (auto index: arm1Indices) {
//        cmd.damping.at(index) = 10.0;
//    }

    jointCmdPublisher_.publish(cmd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void XbotInterface::onBasePoseTransformReceived(const geometry_msgs::TransformStampedConstPtr& msg)
{
    // get continuous orientation
    tf::quaternionMsgToEigen(msg->transform.rotation, baseOrientation_);                          // convert to Eigen::quaterniond
    Eigen::Vector3d orientation = baseOrientation_.toRotationMatrix().eulerAngles(2, 1, 0);     // get euler angles as Eigen::Vector3d
    makeEulerAnglesUnique(orientation);                                                         // make euler angles continuous

    const auto yaw = findOrientationClosestToReference(orientation[0], continuousBaseOrientation_(0));

    continuousBaseOrientation_ << yaw, orientation[1], orientation[2];

    // position
    basePoseVector_ << msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z, yaw, orientation[1], orientation[2];
//    std::cout << "[Callback] Base Pose Transform: " << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void XbotInterface::onBasePoseStampedReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // get continuous orientation
    tf::quaternionMsgToEigen(msg->pose.orientation, baseOrientation_);                          // convert to Eigen::quaterniond
    Eigen::Vector3d orientation = baseOrientation_.toRotationMatrix().eulerAngles(2, 1, 0);     // get euler angles as Eigen::Vector3d
    makeEulerAnglesUnique(orientation);                                                         // make euler angles continuous

    const auto yaw = findOrientationClosestToReference(orientation[0], continuousBaseOrientation_(0));

    continuousBaseOrientation_ << yaw, orientation[1], orientation[2];

    // position
    basePoseVector_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, yaw, orientation[1], orientation[2];
//    std::cout << "[Callback] Base Pose Stamped: " << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void XbotInterface::onJointStateReceived(const xbot_msgs::JointStateConstPtr& msg)
{
    xbotJointNames_ = msg->name;
    // save to jointPosition_ and jointVelocity_ according to mpc's internal model ordering
    for (int i = 0; i < jointNames_.size(); i++) {
        int jointIndexInXbot = std::distance(xbotJointNames_.begin(), std::find(xbotJointNames_.begin(), xbotJointNames_.end(), jointNames_.at(i)));
        jointPos_(i) = msg->link_position[jointIndexInXbot];
        jointVel_(i) = msg->link_velocity[jointIndexInXbot];
//        jointPosRef_(i) = msg->position_reference[jointIndexInXbot];

    }
//    std::cout << "Joint State: " << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void XbotInterface::onBaseTwistReceived(const geometry_msgs::TwistStampedConstPtr& msg)
{
    baseTwist_(0) = msg->twist.linear.x;
    baseTwist_(1) = msg->twist.linear.y;
    baseTwist_(2) = msg->twist.linear.z;
    baseTwist_(3) = msg->twist.angular.x;
    baseTwist_(4) = msg->twist.angular.y;
    baseTwist_(5) = msg->twist.angular.z;
//    std::cout << "Base Twists: " << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void XbotInterface::onEeWrenchReceived(const base_estimation::ContactsWrenchConstPtr& msg) {
    for (int i = 0; i < msg->contacts_wrench.size(); i++) {
        Eigen::Matrix<scalar_t, 6, 1> eeWrench;
        tf::wrenchMsgToEigen(msg->contacts_wrench[i].wrench, eeWrench);
        eeEstimatedWrenches_.col(i) = eeWrench;
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, vector_t> XbotInterface::getJointImpedanceGains(const std::pair<std::vector<float>, std::vector<float>>& lowerBodyJointImpedanceGains,
                                                                    const std::pair<std::vector<float>, std::vector<float>>& upperBodyJointImpedanceGains) const {
    vector_t stiffness(xbotJointNames_.size()), damping(xbotJointNames_.size());

    for (int i = 0; i < xbotJointNames_.size(); i++) {
        // if joint is in lower body
        if (std::find(lowerBodyJoints.begin(), lowerBodyJoints.end(), xbotJointNames_[i]) != lowerBodyJoints.end()) {
            stiffness[i] = lowerBodyJointImpedanceGains.first[mapJointNameToMotorSizeIndex[xbotJointNames_[i]]];
            damping[i] = lowerBodyJointImpedanceGains.second[mapJointNameToMotorSizeIndex[xbotJointNames_[i]]];
        } else {
            stiffness[i] = upperBodyJointImpedanceGains.first[mapJointNameToMotorSizeIndex[xbotJointNames_[i]]];
            damping[i] = upperBodyJointImpedanceGains.second[mapJointNameToMotorSizeIndex[xbotJointNames_[i]]];
        }
    }

    auto jointGains = std::make_pair(stiffness, damping);
    return jointGains;
}

}  // namespace xbot_interface
}  // namespace ocs2
