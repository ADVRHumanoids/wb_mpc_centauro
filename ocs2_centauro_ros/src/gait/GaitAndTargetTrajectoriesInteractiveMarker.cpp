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

#include <ocs2_centauro_ros/gait/GaitAndTargetTrajectoriesInteractiveMarker.h>

#include <ocs2_msgs/mpc_observation.h>
#include <gazebo_ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_centauro_ros/gait/ModeSequenceTemplateRos.h>
#include <ocs2_centauro/common/ModelSettings.h>
#include <ocs2_core/misc/CommandLine.h>
#include <typeinfo>

using namespace interactive_markers;

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitAndTargetTrajectoriesInteractiveMarker::GaitAndTargetTrajectoriesInteractiveMarker(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                                                         GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories,
                                                                         const std::string& targetFrame)
    : server_("simple_marker_" + targetFrame), gaolPoseToTargetTrajectories_(std::move(gaolPoseToTargetTrajectories)),
      targetFrame_(targetFrame) {

  // get model settings for contact EEs info
  std::string taskFile;
  nodeHandle.getParam("/taskFile", taskFile);
  contactNames3DoF_ = loadModelSettings(taskFile, "model_settings", false).contactNames3DoF;

  // receive global frame for visualization
  nodeHandle.getParam("/global_frame", globalFrame_);

  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);
  modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>("legged_robot_mpc_mode_schedule", 1, true);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix, targetFrame_));

  // create an interactive marker for our server
  auto targetOnly = menuHandler_.insert("Send target pose only");
  auto contactSwitch = menuHandler_.insert("Send target pose and contact switch");
  auto targetOnlyDefaultDuration = menuHandler_.insert(targetOnly, "Default duration", boost::bind(&GaitAndTargetTrajectoriesInteractiveMarker::processFeedback, this, _1, false));
  auto targetOnlyCustomDuration = menuHandler_.insert(targetOnly, "Custom duration", boost::bind(&GaitAndTargetTrajectoriesInteractiveMarker::processFeedback, this, _1, true));
  auto contactSwitchDefaultDuration = menuHandler_.insert(contactSwitch, "Default duration", boost::bind(&GaitAndTargetTrajectoriesInteractiveMarker::processFeedbackWithContactChange, this, _1, false));
  auto contactSwitchCustomDuration = menuHandler_.insert(contactSwitch, "Custom duration", boost::bind(&GaitAndTargetTrajectoriesInteractiveMarker::processFeedbackWithContactChange, this, _1, true));

  // continuous target sending for targets in cost function
  targetOnlyContinuousCost_ = menuHandler_.insert(targetOnly, "Continuous control", boost::bind(&GaitAndTargetTrajectoriesInteractiveMarker::switchToFromContinuousControl, this, _1));
  menuHandler_.setCheckState(targetOnlyContinuousCost_, MenuHandler::UNCHECKED);

  // create an interactive marker for our server
  interactiveMarker_ = createInteractiveMarker();

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server_.insert(interactiveMarker_);  //, boost::bind(&GaitAndTargetTrajectoriesInteractiveMarker::processFeedback, this, _1));
  menuHandler_.apply(server_, interactiveMarker_.name);

  // 'commit' changes and send to all clients
  server_.applyChanges();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
visualization_msgs::InteractiveMarker GaitAndTargetTrajectoriesInteractiveMarker::createInteractiveMarker() const {
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = globalFrame_;
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "Goal";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.z = 5.0;

  //TODO: modify to enter the appropriate initial pose of the corresponding targeFrame_
  if (targetFrame_ != "") {
      interactiveMarker.pose.position.x = 0.488;
      interactiveMarker.pose.position.y = 0.170;
      interactiveMarker.pose.position.z = 1.033;
      interactiveMarker.pose.orientation.x = 0.081;
      interactiveMarker.pose.orientation.y = 0.942;
      interactiveMarker.pose.orientation.z = 0.239;
      interactiveMarker.pose.orientation.w = -0.221;
  }

  // create a grey box marker
  const auto boxMarker = []() {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  return interactiveMarker;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitAndTargetTrajectoriesInteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback, const bool& customDuration) {
    // Desired state trajectory
    const Eigen::Vector3d position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    const Eigen::Quaterniond orientation(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                                         feedback->pose.orientation.z);
    scalar_t targetReachTime = -1.0;
    if (customDuration) {
        targetReachTime = getReachTimeFromTerminal();      // get user input
    }

    // get the latest observation
    ::ros::spinOnce();
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // get TargetTrajectories
    const auto targetTrajectories = gaolPoseToTargetTrajectories_(position, orientation, observation, targetReachTime);

    // publish TargetTrajectories and modeSequence
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitAndTargetTrajectoriesInteractiveMarker::switchToFromContinuousControl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    MenuHandler::CheckState currentCheckState;
    menuHandler_.getCheckState(targetOnlyContinuousCost_, currentCheckState);
    if (currentCheckState == MenuHandler::UNCHECKED) {
        menuHandler_.setCheckState(targetOnlyContinuousCost_, MenuHandler::CHECKED);
        server_.setCallback(interactiveMarker_.name, boost::bind(&GaitAndTargetTrajectoriesInteractiveMarker::publishFeedbackContinuous, this, _1));
    } else if (currentCheckState == MenuHandler::CHECKED) {
        menuHandler_.setCheckState(targetOnlyContinuousCost_, MenuHandler::UNCHECKED);
        server_.setCallback(interactiveMarker_.name, boost::bind(&GaitAndTargetTrajectoriesInteractiveMarker::emptyCallback, this, _1));
    }
    menuHandler_.reApply(server_);
    server_.applyChanges();
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitAndTargetTrajectoriesInteractiveMarker::publishFeedbackContinuous(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    // Desired state trajectory
    const Eigen::Vector3d position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    const Eigen::Quaterniond orientation(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                                         feedback->pose.orientation.z);

    // get the latest observation
    ::ros::spinOnce();
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // get TargetTrajectories
    const auto targetTrajectories = gaolPoseToTargetTrajectories_(position, orientation, observation, -1.0);

    // publish TargetTrajectories and modeSequence
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitAndTargetTrajectoriesInteractiveMarker::processFeedbackWithContactChange(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const bool& customDuration)
{
    // Desired state trajectory
    const Eigen::Vector3d position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    const Eigen::Quaterniond orientation(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                                         feedback->pose.orientation.z);
    scalar_array_t targetSwitchAndReachTime = {1.0, -1.0};   // initialize
    if (customDuration) {
        targetSwitchAndReachTime = getSwitchAndReachTimeFromTerminal();      // get user input
    }

    // get the latest observation
    ::ros::spinOnce();
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // get TargetTrajectories
    const auto targetTrajectories = gaolPoseToTargetTrajectories_(position, orientation, observation, targetSwitchAndReachTime[1]);

    // find new mode
    locoma_contact_flag_t contactFlags = modeNumber2ActiveContacts(latestObservation_.mode);
    size_t targetFrameIndex = std::distance(contactNames3DoF_.begin(), std::find(contactNames3DoF_.begin(), contactNames3DoF_.end(), targetFrame_));
    contactFlags[targetFrameIndex] = !contactFlags[targetFrameIndex];
    size_t newMode = stanceLeg2ModeNumber(contact_flag_t{contactFlags[0], contactFlags[1], contactFlags[2], contactFlags[3]},
                                          arm_contact_flag_t{contactFlags[4], contactFlags[5]});
    // -1.0 since by default the template will be added after the horizon end, the second value does not affect for single mode templates
    ModeSequenceTemplate modeSequenceTemplate({targetSwitchAndReachTime[0] - 1.0, targetSwitchAndReachTime[0] + 1.0}, {newMode});       // TODO: define differently the timings


    // publish TargetTrajectories and modeSequence
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
    modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaitAndTargetTrajectoriesInteractiveMarker::getReachTimeFromTerminal() const {
    // get user input from terminal
    std::cout << "Enter desired motion duration:" << std::endl;

    auto shouldTerminate = []() { return !ros::ok() || !ros::master::check(); };
    const auto commandLine = stringToWords(getCommandLineString(shouldTerminate));

    if (commandLine.empty()) {
      return -1.0;
    }

    if (commandLine.size() > 1) {
      std::cout << "WARNING: The command should be a single word." << std::endl;
      return -1.0;
    }

    // lower case transform
    auto gaitCommand = commandLine.front();
//    std::transform(gaitCommand.begin(), gaitCommand.end(), gaitCommand.begin(), ::tolower);

    return std::stod(gaitCommand);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_array_t GaitAndTargetTrajectoriesInteractiveMarker::getSwitchAndReachTimeFromTerminal() const {
    // get user input from terminal
    std::cout << "Enter switch time (default horizon of mpc) and motion duration, separated by spaces:" << std::endl;

    // get command line as one long string
    auto shouldTerminate = []() { return !ros::ok() || !ros::master::check(); };
    const std::string line = getCommandLineString(shouldTerminate);

    // a line to words
    const std::vector<std::string> words = stringToWords(line);

    scalar_array_t targetCommand = {0.0, 0.0};
    for (size_t i = 0; i < std::min(words.size(), targetCommand.size()); i++) {
      targetCommand[i] = static_cast<scalar_t>(stof(words[i]));
    }

    if (targetCommand[1] < targetCommand[0]) {
        scalar_array_t defaultTarget{1.0, -1.0};
        std::cout  << "Total motion duration is less than switch contact time. Returned default values " << defaultTarget[0] << ", " << defaultTarget[1] << std::endl;;
        return defaultTarget;
    }

    return targetCommand;
}



}
}   // namespace ocs2
