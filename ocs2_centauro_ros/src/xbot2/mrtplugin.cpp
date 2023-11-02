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

#include "mrtplugin.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <gazebo_ocs2_ros_interfaces/common/Kinematics.h>

MrtPlugin::MrtPlugin(const ControlPlugin::Args &args):
    ControlPlugin(args),
    mrt_("legged_robot")
{}

bool MrtPlugin::on_initialize()
{
    // construct nodehandle
    nodeHandlePtr_ = std::make_unique<ros::NodeHandle>();

    // initialize link state sensor
    auto lss_container = _robot->getDevices<Hal::LinkStateSensor>();
    if(lss_container.get_device_vector().empty())
    {
        jerror("no Hal::LinkStateSensor detected");
        return false;
    }
    else
    {
        // assume it's the only one
        lss_ = lss_container.get_device_vector()[0];
    }

    return true;
}

void MrtPlugin::on_start()
{
    // Get file paths
    std::string taskFile, urdfFile, referenceFile;
    nodeHandlePtr_->getParam("/taskFile", taskFile);
    nodeHandlePtr_->getParam("/urdfFile", urdfFile);
    nodeHandlePtr_->getParam("/referenceFile", referenceFile);

    interfacePtr_ = std::make_shared<LeggedRobotInterface>(taskFile, urdfFile, referenceFile);
    centroidalModelRbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(
                std::move(interfacePtr_->getPinocchioInterface()),
                std::move(interfacePtr_->getCentroidalModelInfo())
                          );

    // joint names in centroidal model and xbot
    xbotJointNames_ = _robot->getEnabledJointNames();                       // xbot names
    centroidalJointNames_ = interfacePtr_->modelSettings().jointNames;

    // get initial state
    vector_t initialState = getObservationFromXbot(true).state;
    interfacePtr_->setInitialState(initialState);
    std::cout << "Malaka";

    // initial mrt
    mrt_.initRollout(&interfacePtr_->getRollout());
    mrt_.launchNodes(*nodeHandlePtr_);

    // Visualization
    // TODO: endEffectorKinematics fails in the constructor
//    pinocchioMapping_ = std::make_shared<CentroidalModelPinocchioMapping>(interfacePtr_->getCentroidalModelInfo());
//    endEffectorKinematics_ = std::make_shared<PinocchioEndEffectorKinematics>(interfacePtr_->getPinocchioInterface(), *pinocchioMapping_,
//                                                                              interfacePtr_->modelSettings().contactNames3DoF);
//    leggedRobotVisualizerPtr_ = std::make_shared<LeggedRobotVisualizer>(interfacePtr_->getPinocchioInterface(),
//                                                                        interfacePtr_->getCentroidalModelInfo(),
//                                                                        *endEffectorKinematics_, *nodeHandlePtr_);
//    observers_ = {leggedRobotVisualizerPtr_};    // Subscribe leggedRobotVisualizer

    // initial observation
    initObservation_.state = initialState;
    initObservation_.input = vector_t::Zero(interfacePtr_->getCentroidalModelInfo().inputDim);
    initObservation_.mode = ModeNumber::STANCE;
    currentObservation_ = initObservation_;

    std::cout << "Formed initial observation" << initObservation_.state << std::endl;

    initTargetTrajectories_ = TargetTrajectories({0.0}, {initObservation_.state}, {initObservation_.input});

    // set control mode
    auto ctrl_mode = ControlMode::Velocity() + ControlMode::Effort() + ControlMode::Stiffness() + ControlMode::Damping(); //ControlMode::Position();
    _robot->setControlMode(ctrl_mode);

    // keep default control for the wheels
    _robot->setControlMode({
                               {"j_wheel_1", ControlMode::Idle()},
                               {"j_wheel_2", ControlMode::Idle()},
                               {"j_wheel_3", ControlMode::Idle()},
                               {"j_wheel_4", ControlMode::Idle()},
                               {"d435_head_joint", ControlMode::Idle()},
                               {"velodyne_joint", ControlMode::Idle()}
                           });
}

void MrtPlugin::starting()
{
    std::cout << "In starting" << std::endl;

    // Reset MPC node
    mrt_.resetMpcNode(initTargetTrajectories_);
    std::cout << "Reseted mpc" << std::endl;

    // Wait for the initial policy
    while (!mrt_.initialPolicyReceived() && ros::ok() && ros::master::check())
    {
        mrt_.spinMRT();
        mrt_.setCurrentObservation(initObservation_);
    }
    std::cout << "Initial policy received" << std::endl;

    // after this has been called, execution switches to run()
    start_completed();
}

void MrtPlugin::run()
{

    // Trigger MRT callbacks
    mrt_.spinMRT();

    // Update the policy if a new one was received
    if (mrt_.updatePolicy()) {
      std::cout << "<<< New MPC policy starting at " << mrt_.getPolicy().timeTrajectory_.front() << "\n";

//          PerformanceIndex currentPerformanceIndex = mrt_.getPerformanceIndices();
//          if (currentPerformanceIndex.cost > 500 || currentPerformanceIndex.merit > 500) {      // safety
//            std::cout << "*** Safety Stop! High rollout cost or merit." << std::endl;
//            break;
//          }
    }

    // get observation from xbotCore
    currentObservation_ = getObservationFromXbot();

    // set observation
    mrt_.setCurrentObservation(currentObservation_);

    // Update observers/visualization
//    for (auto& observer : observers_) {
//      observer->update(currentObservation_, mrt_.getPolicy(), mrt_.getCommand());
//    }

    // evaluate policy at current time
    vector_t optimalState;
    vector_t optimalInput;
    size_t mode;
    mrt_.evaluatePolicy(currentObservation_.time, currentObservation_.state, optimalState, optimalInput, mode);

    // joint position & velocity
    const auto qJoints = centroidal_model::getJointAngles(optimalState, interfacePtr_->getCentroidalModelInfo());
    const auto dqJoints = centroidal_model::getJointVelocities(optimalInput, interfacePtr_->getCentroidalModelInfo());

    // compute torques with inverse dynamics for gravity compensation
    vector_t ffModelTorque = centroidalModelRbdConversions_->computeRbdTorqueFromCentroidalModel(optimalState, optimalInput, vector_t::Zero(centroidalJointNames_.size()));
    vector_t ffJointTorque(_robot->getJointNum());
    ffJointTorque.head(centroidalJointNames_.size()) = ffModelTorque.tail(centroidalJointNames_.size());

    // reference vectors
    Eigen::VectorXd positionRef(_robot->getJointNum()), velocityRef(_robot->getJointNum()), tauRef(_robot->getJointNum());
    Eigen::VectorXd stiffnessRef = Eigen::VectorXd::Constant(_robot->getJointNum(), 50.0);
    Eigen::VectorXd dampingRef = Eigen::VectorXd::Constant(_robot->getJointNum(), 100.0);

    // map for centroidal state to xbot
    for (int i = 0; i < centroidalJointNames_.size(); i++) {
        int jointIndexInXbot = std::distance(xbotJointNames_.begin(),
                                             std::find(xbotJointNames_.begin(), xbotJointNames_.end(), centroidalJointNames_.at(i)));
        positionRef(jointIndexInXbot) = qJoints[i];
        velocityRef(jointIndexInXbot) = dqJoints[i];
        tauRef(jointIndexInXbot) = ffJointTorque[i];
    }

    // send command
//    _robot->setPositionReference(positionRef);
    _robot->setVelocityReference(velocityRef);
    _robot->setEffortReference(tauRef);
    _robot->setStiffness(stiffnessRef);
    _robot->setDamping(dampingRef);
    _robot->move();

}

SystemObservation MrtPlugin::getObservationFromXbot(const bool isInitial)
{
    // sense position velocity
    Eigen::VectorXd qMotor, vMotor;
    _robot->sense();
    _robot->getMotorPosition(qMotor);
    _robot->getMotorVelocity(vMotor);

    Eigen::Affine3d w_T_pelvis = lss_->getPose();
    Eigen::Vector6d w_v_pelvis = lss_->getTwist();

    // on joints state
    Eigen::VectorXd jointPos(centroidalJointNames_.size());
    Eigen::VectorXd jointVel(centroidalJointNames_.size());
    for (int i = 0; i < centroidalJointNames_.size(); i++) {
        int jointIndexInXbot = std::distance(xbotJointNames_.begin(),
                                             std::find(xbotJointNames_.begin(), xbotJointNames_.end(), centroidalJointNames_.at(i)));
        jointPos(i) = qMotor[jointIndexInXbot];
        jointVel(i) = vMotor[jointIndexInXbot];
    }

    // on base pose and twist
    Eigen::Quaterniond baseQuaternion(w_T_pelvis.rotation());
    Eigen::Vector3d orientation = baseQuaternion.toRotationMatrix().eulerAngles(2, 1, 0);     // get euler angles as Eigen::Vector3d
    makeEulerAnglesUnique(orientation);                                                         // make euler angles continuous
    const auto yaw = findOrientationClosestToReference(orientation[0], continuousBaseOrientation_(0));
    continuousBaseOrientation_ << yaw, orientation[1], orientation[2];

    // position
    vector_t basePoseVector(6);
    basePoseVector << w_T_pelvis.translation()[0], w_T_pelvis.translation()[1], w_T_pelvis.translation()[2], yaw, orientation[1], orientation[2];

    // RbdState = [base pose, joint positions, base twist, joint velocities]
    vector_t rbdState(w_v_pelvis.rows() + jointPos.rows() + jointVel.rows() + basePoseVector.rows());    // get size from basePose

    // assign rbd State, angular/orientation precedes linear/position
    rbdState << basePoseVector.bottomRows(3), basePoseVector.topRows(3), jointPos, w_v_pelvis.bottomRows(3), w_v_pelvis.topRows(3), jointVel;

    SystemObservation obs;
    obs.state = centroidalModelRbdConversions_->computeCentroidalStateFromRbdModel(rbdState);
    obs.input.setZero(interfacePtr_->getCentroidalModelInfo().inputDim);

    // separate if it is initial
    if (isInitial) {
        obs.time = 0.0;
    } else {
        obs.time = currentObservation_.time + getPeriodSec();
        obs.mode = mrt_.getActivePrimalSolution().modeSchedule_.modeAtTime(obs.time);
    }

    return obs;
}

XBOT2_REGISTER_PLUGIN(MrtPlugin, mrt_plugin)
