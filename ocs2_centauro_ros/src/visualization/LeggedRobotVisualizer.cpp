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
- self-collision visualization
******************************************************************************/

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_centauro_ros/visualization/LeggedRobotVisualizer.h"

// OCS2
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <gazebo_ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include "ocs2_centauro/gait/MotionPhaseDefinition.h"

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// self-collision visualization
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotVisualizer::LeggedRobotVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                                             const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle,
                                             scalar_t maxUpdateFrequency)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      centroidalModelInfo_(std::move(centroidalModelInfo)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {
  endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
  launchVisualizerNode(nodeHandle);
  jointNames_ = {"hip_yaw_1", "hip_pitch_1", "knee_pitch_1", "ankle_pitch_1", "ankle_yaw_1", "j_wheel_1",
                 "hip_yaw_2", "hip_pitch_2", "knee_pitch_2", "ankle_pitch_2", "ankle_yaw_2", "j_wheel_2",
                 "hip_yaw_3", "hip_pitch_3", "knee_pitch_3", "ankle_pitch_3", "ankle_yaw_3", "j_wheel_3",
                 "hip_yaw_4", "hip_pitch_4", "knee_pitch_4", "ankle_pitch_4", "ankle_yaw_4", "j_wheel_4",
                 "torso_yaw",
                 "j_arm1_1", "j_arm1_2", "j_arm1_3", "j_arm1_4", "j_arm1_5", "j_arm1_6",
                 "j_arm2_1", "j_arm2_2", "j_arm2_3", "j_arm2_4", "j_arm2_5", "j_arm2_6"
                 };
};
// TODO: replace hardcoded joints with the modelSettings from ocs2_centauro

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  costDesiredBasePositionPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot/desiredBaseTrajectory", 1);
  costDesiredFeetPositionPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
  costDesiredFeetPositionPublishers_[0] = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot/desiredFeetTrajectory/LF", 1);
  costDesiredFeetPositionPublishers_[1] = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot/desiredFeetTrajectory/RF", 1);
  costDesiredFeetPositionPublishers_[2] = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot/desiredFeetTrajectory/LH", 1);
  costDesiredFeetPositionPublishers_[3] = nodeHandle.advertise<visualization_msgs::Marker>("/legged_robot/desiredFeetTrajectory/RH", 1);
  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/legged_robot/optimizedStateTrajectory", 1);
  currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/legged_robot/currentState", 1);
//  jointCmdPublisher_ = nodeHandle.advertise<xbot_msgs::JointCommand>("/xbotcore/command", 1);

  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParam("legged_robot_description")) {
    std::cerr << "[LeggedRobotVisualizer] Could not read URDF from: \"legged_robot_description\"" << std::endl;
  } else {
    KDL::Tree kdlTree;
    kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

    // read taskfile info
    std::string taskFile, urdfFile;
    nodeHandle.getParam("/taskFile", taskFile);
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    // check if xbotcore exists which publishes tf
    bool xbotcoreRunning= false;
    loadData::loadCppDataType(taskFile, "xbotcore.xbotCoreRunning", xbotcoreRunning);
    loadData::loadCppDataType(taskFile, "xbotcore.xbotCoreRunning", xbotcoreRunning);

    if (xbotcoreRunning)
        robotStatePublisherPtr_ = nullptr;
    else {
        robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
#if ROS_VERSION_MINOR <= 14
        robotStatePublisherPtr_->publishFixedTransforms("", true);
#else
        robotStatePublisherPtr_->publishFixedTransforms(true);
#endif
    }

    // read if self-collision checking active
    nodeHandle.getParam("/urdfFile", urdfFile);
    bool activateSelfCollision = true;
    loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
    // activate markers for self-collision visualization
    if (activateSelfCollision) {
      std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
      std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
      loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs", collisionLinkPairs, true);
      loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionObjectPairs", collisionObjectPairs, true);
      PinocchioGeometryInterface geomInterface(pinocchioInterface_, collisionLinkPairs, collisionObjectPairs, urdfFile);
      // set geometry visualization markers
      geometryVisualization_.reset(new GeometryInterfaceVisualization(pinocchioInterface_, geomInterface, nodeHandle, frameId_));
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();

    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto timeStamp = ros::Time::now();
    publishObservation(timeStamp, observation);
    publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
    publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                    primalSolution.modeSchedule_);
    lastTime_ = observation.time;

    if (geometryVisualization_ != nullptr) {
      geometryVisualization_->publishDistances(centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishObservation(ros::Time timeStamp, const SystemObservation& observation) {
  // Extract components from state
  const auto basePose = centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
  const auto qJoints = centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

  // Compute cartesian state and inputs
  const auto feetPositions = endEffectorKinematicsPtr_->getPosition(observation.state);
  std::vector<vector3_t> feetForces(centroidalModelInfo_.numThreeDofContacts);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    feetForces[i] = centroidal_model::getContactForces(observation.input, i, centroidalModelInfo_);
  }

//  for (int i = 0; i < feetPositions.size(); i++)
//      ROS_INFO_STREAM("feetPositions = " << feetPositions[i].transpose());

  // Publish
  publishJointTransforms(timeStamp, qJoints);
  publishBaseTransform(timeStamp, basePose);
  publishCartesianMarkers(timeStamp, modeNumber2ActiveContacts(observation.mode), feetPositions, feetForces);

  // send command to xbot2 - for open loop this works
//  xbot_msgs::JointCommand cmd;
//  cmd.name = jointNames_;
//  cmd.position.assign(qJoints.data(), qJoints.data() + qJoints.size());
//  cmd.ctrl_mode.assign(cmd.position.size(), 1);  // 1 = we're only sending position
//  jointCmdPublisher_.publish(cmd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles) const {
  if (robotStatePublisherPtr_ != nullptr) {

    std::map<std::string, scalar_t> jointPositions;

    for (int i = 0; i < jointNames_.size(); i++)
        jointPositions[jointNames_[i]] = jointAngles[i];


#if ROS_VERSION_MINOR <= 14
    robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp, "");
#else
    robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
#endif

  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishBaseTransform(ros::Time timeStamp, const vector_t& basePose) {
  if (robotStatePublisherPtr_ != nullptr) {
    geometry_msgs::TransformStamped baseToWorldTransform;
    baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    baseToWorldTransform.child_frame_id = "base_link";

    const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
    baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
    baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
    tfBroadcaster_.sendTransform(baseToWorldTransform);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishTrajectory(const std::vector<SystemObservation>& system_observation_array, scalar_t speed) {
  for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
    scalar_t frameDuration = speed * (system_observation_array[k + 1].time - system_observation_array[k].time);
    scalar_t publishDuration = timedExecutionInSeconds([&]() { publishObservation(ros::Time::now(), system_observation_array[k]); });
    if (frameDuration > publishDuration) {
      ros::WallDuration(frameDuration - publishDuration).sleep();
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishCartesianMarkers(ros::Time timeStamp, const locoma_contact_flag_t& contactFlags,
                                                    const std::vector<vector3_t>& feetPositions,
                                                    const std::vector<vector3_t>& feetForces) const {
  // Reserve message
  const size_t numberOfCartesianMarkers = 10;
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  // Feet positions and Forces
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i) {
    markerArray.markers.emplace_back(
        getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i], footMarkerDiameter_, footAlphaWhenLifted_));
    markerArray.markers.emplace_back(getForceMarker(feetForces[i], feetPositions[i], contactFlags[i], Color::green, forceScale_));
  }

  // Center of pressure
  markerArray.markers.emplace_back(getCenterOfPressureMarker(feetForces.begin(), feetForces.end(), feetPositions.begin(),
                                                             contactFlags.begin(), Color::green, copMarkerDiameter_));

  // Support polygon
  markerArray.markers.emplace_back(
      getSupportPolygonMarker(feetPositions.begin(), feetPositions.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  currentStatePublisher_.publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories) {
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;

  // Reserve com messages
  std::vector<geometry_msgs::Point> desiredBasePositionMsg;
  desiredBasePositionMsg.reserve(stateTrajectory.size());

  // Reserve feet messages
  feet_arms_array_t<std::vector<geometry_msgs::Point>> desiredFeetPositionMsgs;
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    desiredFeetPositionMsgs[i].reserve(stateTrajectory.size());
  }

  for (size_t j = 0; j < stateTrajectory.size(); j++) {
    const auto state = stateTrajectory.at(j);
    vector_t input(centroidalModelInfo_.inputDim);
    if (j < inputTrajectory.size()) {
      input = inputTrajectory.at(j);
    } else {
      input.setZero();
    }

    // Construct base pose msg
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
    geometry_msgs::Pose pose;
    pose.position = getPointMsg(basePose.head<3>());

    // Fill message containers
    desiredBasePositionMsg.push_back(pose.position);

    // Fill feet msgs
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
      geometry_msgs::Pose footPose;
      footPose.position = getPointMsg(feetPositions[i]);
      desiredFeetPositionMsgs[i].push_back(footPose.position);

//      ROS_INFO_STREAM("feetPositions = " << feetPositions[i].transpose());
    }
  }

  // Headers
  auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green, trajectoryLineWidth_);
  comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
  comLineMsg.id = 0;

  // Publish
  costDesiredBasePositionPublisher_.publish(comLineMsg);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]), feetColorMap_[i], trajectoryLineWidth_);
    footLineMsg.header = getHeaderMsg(frameId_, timeStamp);
    footLineMsg.id = 0;
    costDesiredFeetPositionPublishers_[i].publish(footLineMsg);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                                            const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule) {
  if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
    return;  // Nothing to publish
  }

  // Reserve Feet msg
  feet_arms_array_t<std::vector<geometry_msgs::Point>> feetMsgs;
  std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::Point>& v) { v.reserve(mpcStateTrajectory.size()); });

  // Reserve Com Msg
  std::vector<geometry_msgs::Point> mpcComPositionMsgs;
  mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

  // Extract Com and Feet from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);

    // Fill com position and pose msgs
    geometry_msgs::Pose pose;
    pose.position = getPointMsg(basePose.head<3>());
    mpcComPositionMsgs.push_back(pose.position);

    // Fill feet msgs
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
      const auto position = getPointMsg(feetPositions[i]);
      feetMsgs[i].push_back(position);
//      ROS_INFO_STREAM("feetPositions = " << feetPositions[i].transpose());

    }

  });

  // Convert feet msgs to Array message
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(centroidalModelInfo_.numThreeDofContacts +
                              2);  // 1 trajectory per foot + 1 for the future footholds + 1 for the com trajectory
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    markerArray.markers.emplace_back(getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_));
  markerArray.markers.back().ns = "CoM Trajectory";

  // Future footholds
  visualization_msgs::Marker sphereList;
  sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
  sphereList.scale.x = footMarkerDiameter_;
  sphereList.scale.y = footMarkerDiameter_;
  sphereList.scale.z = footMarkerDiameter_;
  sphereList.ns = "Future footholds";
  sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto& subsystemSequence = modeSchedule.modeSequence;
  const auto tStart = mpcTimeTrajectory.front();
  const auto tEnd = mpcTimeTrajectory.back();
  for (size_t event = 0; event < eventTimes.size(); ++event) {
    if (tStart < eventTimes[event] && eventTimes[event] < tEnd) {  // Only publish future footholds within the optimized horizon
      const auto preEventContactFlags = modeNumber2ActiveContacts(subsystemSequence[event]);
      const auto postEventContactFlags = modeNumber2ActiveContacts(subsystemSequence[event + 1]);
      const auto postEventState = LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);

      const auto& model = pinocchioInterface_.getModel();
      auto& data = pinocchioInterface_.getData();
      pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(postEventState, centroidalModelInfo_));
      pinocchio::updateFramePlacements(model, data);

      const auto feetPosition = endEffectorKinematicsPtr_->getPosition(postEventState);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        if (!preEventContactFlags[i] && postEventContactFlags[i]) {  // If a foot lands, a marker is added at that location.
          sphereList.points.emplace_back(getPointMsg(feetPosition[i]));
          sphereList.colors.push_back(getColor(feetColorMap_[i]));
        }
      }
    }
  }
  markerArray.markers.push_back(std::move(sphereList));

  // Add headers and Id
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  stateOptimizedPublisher_.publish(markerArray);
}

}  // namespace legged_robot
}  // namespace ocs2
