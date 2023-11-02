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
#include <gazebo_ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>

using namespace ocs2;
using namespace legged_robot;

/**
 * Converts the pose of the interactive marker to TargetTrajectories.
 * reachTime: time after the observation for switch contact and complete motion, e.g. 2.0 sec
 */
TargetTrajectories goalPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const SystemObservation& observation, const scalar_t& reachTime) {

  scalar_array_t timeTrajectory(1, observation.time + reachTime);

  // state trajectory: 3 + 4 for desired position vector and orientation quaternion
  const vector_t target = (vector_t(7) << position, orientation.coeffs()).finished();

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t stateTrajectory{target};   //(timeTrajectory.size(), target);  // just publish the same as many times as needed
  // input trajectory
  const vector_array_t inputTrajectory(1, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/**
 * Converts the pose of the interactive marker to TargetTrajectories.
 */
TargetTrajectories goalPoseToTargetTrajectoriesDefault(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const SystemObservation& observation) {
  // time trajectory
  const scalar_array_t timeTrajectory{observation.time};
  // state trajectory: 3 + 4 for desired position vector and orientation quaternion
  const vector_t target = (vector_t(7) << position, orientation.coeffs()).finished();
  const vector_array_t stateTrajectory{target};
  // input trajectory
  const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";
  std::string armTargetFrame;
  ::ros::init(argc, argv, robotName + "_" + armTargetFrame + "target", ros::init_options::AnonymousName);
  ::ros::NodeHandle nodeHandle;
  ros::param::get("~frame_name", armTargetFrame);

GaitAndTargetTrajectoriesInteractiveMarker targetPoseCommand(nodeHandle, robotName, &goalPoseToTargetTrajectories, armTargetFrame);
//  TargetTrajectoriesInteractiveMarker targetPoseCommand(nodeHandle, robotName, &goalPoseToTargetTrajectoriesDefault, armTargetFrame);
  targetPoseCommand.publishInteractiveMarker();

  // Successful exit
  return 0;
}
