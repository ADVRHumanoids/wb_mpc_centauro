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

#include <ros/ros.h>

#include <xbot_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <ocs2_core/Types.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_centauro/LeggedRobotInterface.h>
#include <ocs2_mpc/SystemObservation.h>
#include "gazebo_ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"
#include <base_estimation/ContactsWrench.h>

namespace ocs2 {
namespace xbot_interface {



/*!
 * Main class for the node to handle the ROS interfacing.
 */
class XbotInterface
{
 public:

    /**
     * @brief The Config struct
     * xbotCoreRunning: true if there is xbotcore running (simulation or real robot)
     * xbotCoreFeedback: true if the mpc has to take feedback from xbotcore before each iteration
     * baseLinkTopicPrefix: the topic prefix for the ros topic at which base link information (pose & twist) can be accessed
     * clampTorqueCmd : true for clamping the torque ff commanded within the urdf limits
     * clampPositionCmd : true for clamping the position commanded within the urdf limits
     * clampVelocityCmd : true for clamping the velocity commanded within the urdf limits
     */
    struct Config {
        Config(bool xbotCoreRunningParam = false, bool xbotCoreFeedbackParam = false,
               std::string baseLinkTopicPrefixParam = "/xbotcore/link_state/pelvis",
               bool clampTorqueCmdParam = false, bool clampPositionCmdParam = false, bool clampVelocityCmdParam = false)
            : xbotCoreRunning(xbotCoreRunningParam), xbotCoreFeedback(xbotCoreFeedbackParam),
              baseLinkTopicPrefix(baseLinkTopicPrefixParam), clampTorqueCmd(clampTorqueCmdParam),
              clampPositionCmd(clampPositionCmdParam), clampVelocityCmd(clampVelocityCmdParam) {}
        bool xbotCoreRunning;
        bool xbotCoreFeedback;
        std::string baseLinkTopicPrefix;
        bool clampTorqueCmd, clampPositionCmd, clampVelocityCmd;
    };

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param baseLinkTopicPrefix the prefix of the ros topic for accessing base pose and twist. It can be something like
   * "/xbotcore/link_state/pelvis" or "/centauro_base_estimation/base_link".
   */
  XbotInterface(ros::NodeHandle& nodeHandle, PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                std::vector<std::string> jointNames, Config config);
  /*!
   * Destructor.
   */
  virtual ~XbotInterface() = default;


  /*
   * Gets one message for each topic regarding joint states, base pose and twist
   */
  void getInitialMsg();

  /**
   * @brief subscribeToXbot: constructs the subscribers to xbotcore
   */
  void connectToXbot();

  /**
   * @brief sendCommandToXbot: publish command throught /xbotcore/command topic
   */
  void sendCommandToXbot(const std::shared_ptr<legged_robot::LeggedRobotInterface> leggedRobotInterfacePtr, const vector_t& optimalState, const vector_t& optimalInput);

  /**
   * @brief sendPositionCommandToXbot: publish position command throught /xbotcore/command topic, for position control without observation from xbotcore
   */
  void sendPositionCommandToXbot(const std::shared_ptr<legged_robot::LeggedRobotInterface> leggedRobotInterfacePtr, const vector_t& optimalState);

   /**
   * @brief getCongif: gives access to the xbot configuration object out of the class
   * @return xbotcore configuration
   */
  Config getConfig() const {return config_;}

  /** Constructs the centroidal state from xbotcore information
   *
  **/
  vector_t getCentroidalStateFromXbotInfo();

  /** Get observation from xbotcore */
  SystemObservation getObservationFromXbot(const SystemObservation& currentObservation, const scalar_t& interfaceFrequency,
                                           const std::shared_ptr<legged_robot::LeggedRobotInterface> leggedRobotInterfacePtr,
                                           const MRT_ROS_Interface& mrt);

  // get xbot-based ordered (based on xbotJointNames_) PD gains from P,D motor gains (low, medium, high power)
  std::pair<vector_t, vector_t> getJointImpedanceGains(const std::pair<std::vector<float>, std::vector<float>>& lowerBodyJointImpedanceGains,
                                                       const std::pair<std::vector<float>, std::vector<float>>& upperBodyJointImpedanceGains) const;

 private:

  const Config config_;     // xbot config

  // clamp joint commands within urdf limits
  void clampReferences(vector_t& ffTorques, vector_t& jointPositions, vector_t& jointVelocities);

  // callbacks
  void onJointStateReceived(const xbot_msgs::JointStateConstPtr& msg);
  void onBasePoseTransformReceived(const geometry_msgs::TransformStampedConstPtr& msg);
  void onBasePoseStampedReceived(const geometry_msgs::PoseStampedConstPtr& msg);
  void onBaseTwistReceived(const geometry_msgs::TwistStampedConstPtr& msg);
  void onEeWrenchReceived(const base_estimation::ContactsWrenchConstPtr& msg);

  ros::Subscriber jointStateSubscriber_;
  ros::Subscriber basePoseSubscriber_;
  ros::Subscriber baseTwistSubscriber_;
//  ros::Subscriber eeWrenchSubscriber_;      // subscribe wrenches from force estimation, not used for mpc observation
  ros::Publisher jointCmdPublisher_;

  vector_t basePoseVector_;
  vector_t baseTwist_;
  vector_t jointPos_;
  vector_t jointPosRef_;
  vector_t jointVel_;
  Eigen::Quaterniond baseOrientation_;
  Eigen::Vector3d continuousBaseOrientation_;
  matrix_t eeEstimatedWrenches_;        // size 6 x Nee

  ros::NodeHandle nodeHandle_;
  CentroidalModelRbdConversions centroidalModelRbdConversions_;
  std::vector<std::string> jointNames_;
  std::vector<std::string> xbotJointNames_;
  std::pair<vector_t, vector_t> orderedJointImpedanceGains_;            // store the joint impedance gains from yaml/info file according to xbotJointNames_ order

  // limits for clamping commands
  vector_t jointEffortLimits_, jointPositionLowerLimits_, jointPositionUpperLimits_;
  vector_t jointVelocityLimits_;

  const std::string jointStatesTopicName_ = "/xbotcore/joint_states";
//  const std::string contactsWrenchTopicName_ = "/centauro_base_estimation/contacts/wrench";

};

}  // namespace xbot_interface
}  // namespace ocs2
