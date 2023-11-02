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

#ifndef MRTPLUGIN_H
#define MRTPLUGIN_H


// main XBot2 include
#include <xbot2/xbot2.h>
#include <xbot2/gazebo/dev_link_state_sensor.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centauro/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <gazebo_ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <gazebo_ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "ocs2_centauro_ros/visualization/LeggedRobotVisualizer.h"

using namespace XBot;
using namespace ocs2;
using namespace legged_robot;
/**
 * @brief The MrtPlugin class is a ControlPlugin
 * implementing a simple homing motion.
 */
class MrtPlugin : public ControlPlugin
{

public:

    MrtPlugin(const Args &args);

    // initialization method; the plugin won't be run
    // if this returns 'false'
    bool on_initialize() override;

    // called when plugin is started
    void on_start() override;

    // called at each loop, after start, until start_completed() is called
    void starting() override;

    // callback for 'Run' state
    void run() override;

private:

    // nh
    std::unique_ptr<ros::NodeHandle> nodeHandlePtr_;

    // fake sensors attached to a robot's body
    std::shared_ptr<Hal::LinkStateSensor> lss_;

    // mrt
    SystemObservation initObservation_;
    SystemObservation currentObservation_;
    TargetTrajectories initTargetTrajectories_;
    MRT_ROS_Interface mrt_;

    // observers for visualization
    std::vector<std::shared_ptr<DummyObserver>> observers_;
    std::shared_ptr<LeggedRobotVisualizer> leggedRobotVisualizerPtr_;
    std::shared_ptr<PinocchioEndEffectorKinematics> endEffectorKinematics_;
    std::shared_ptr<CentroidalModelPinocchioMapping> pinocchioMapping_;

    std::shared_ptr<LeggedRobotInterface> interfacePtr_;
    std::shared_ptr<CentroidalModelRbdConversions> centroidalModelRbdConversions_;

    std::vector<std::string> xbotJointNames_;
    std::vector<std::string> centroidalJointNames_;

    SystemObservation getObservationFromXbot(const bool isInitial = false);
    Eigen::Vector3d continuousBaseOrientation_;
};

#endif // MRTPLUGIN_H
