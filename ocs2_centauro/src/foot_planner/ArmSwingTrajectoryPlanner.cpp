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

#include "ocs2_centauro/foot_planner/ArmSwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "ocs2_centauro/gait/MotionPhaseDefinition.h"
#include <numeric>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ArmSwingTrajectoryPlanner::ArmSwingTrajectoryPlanner(Config config, size_t numArms) : config_(std::move(config)), numArms_(numArms), switchEventTime_(arms_array_t<scalar_t>{0.0, 0.0}) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ArmSwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(armsHeightTrajectoriesEvents_[leg], time);
  // debug
//  std::cout << "[ArmSwingTrajectoryPlanner::getZvelocityConstraint] armsHeightTrajectories_[leg][index].velocity(time) = " << armsHeightTrajectories_[leg][index].velocity(time) << std::endl;
//  std::cout << "[ArmSwingTrajectoryPlanner::getZvelocityConstraint] armsHeightTrajectories_[leg][index].position(time) = " << armsHeightTrajectories_[leg][index].position(time) << std::endl;
  return armsHeightTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ArmSwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(armsHeightTrajectoriesEvents_[leg], time);
  return armsHeightTrajectories_[leg][index].position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ArmSwingTrajectoryPlanner::getXvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(armsHeightTrajectoriesEvents_[leg], time);
//  if (leg == 0)
//      std::cout << "[ArmSwingTrajectoryPlanner::getXvelocityConstraint] = " << armsLongitTrajectories_[leg][index].velocity(time) << std::endl;
//      std::cout << "[ArmSwingTrajectoryPlanner::getXvelocityConstraint] index = " << index << std::endl;

  return armsLongitTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ArmSwingTrajectoryPlanner::getXpositionConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(armsHeightTrajectoriesEvents_[leg], time);
  return armsLongitTrajectories_[leg][index].position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ArmSwingTrajectoryPlanner::getYvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(armsHeightTrajectoriesEvents_[leg], time);
  return armsLateralTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ArmSwingTrajectoryPlanner::getYpositionConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(armsHeightTrajectoriesEvents_[leg], time);
  return armsLateralTrajectories_[leg][index].position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// TODO: slerp derivative needs to be checked if it is correct
quaternion_t ArmSwingTrajectoryPlanner::slerp_derivative(quaternion_t q0, quaternion_t q1, scalar_t time, size_t leg) const {
    quaternion_t q_interpol = getOrientationConstraintSlerp(leg, time);
    quaternion_t q_interpol_inv = q0.inverse();
    quaternion_t q_prod = q_interpol_inv * q1;
//    std::cout << "[slerp_derivative] q_prod = " << q_prod.coeffs().transpose() << std::endl;
    // log
    quaternion_t log_quat;
    log_quat.w() = 0;
    scalar_t theta = acos(std::max(std::min(q_prod.w(), 1.0), -1.0));
//    std::cout << "[slerp_derivative] theta = " << theta << std::endl;
    if (std::abs(sin(theta)) > 0.000001)
        log_quat.vec() = q_prod.vec() / sin(theta) * theta;
    else
        log_quat.vec() = q_prod.vec();
//    Quaternion q;
//       q.s_ = 0;
//       Real theta = acos(s_),
//       sin_theta = sin(theta);
//       if ( fabs(sin_theta) > EPSILON)
//          q.v_ = v_/sin_theta*theta;
//       else
//          q.v_ = v_;
//       return q;
//    std::cout << "[slerp_derivative] log_quat = " << log_quat.coeffs().transpose() << std::endl;
    quaternion_t slerp_derivative = q_interpol * log_quat;
    return slerp_derivative.normalized();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
quaternion_t ArmSwingTrajectoryPlanner::getOrientationConstraintSlerp(size_t leg, scalar_t time) const {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    scalar_t finalTime = latestSwingOrientationStartTime_[leg] + latestSwingOrientationDuration_[leg];
    std::tie(index, alpha) = LinearInterpolation::timeSegment(time, scalar_array_t{latestSwingOrientationStartTime_[leg], finalTime});

    quaternion_t orientation = latestInitialSwingQuaternion_[leg].slerp((1.0 - alpha), latestTargetQuaternion_[leg]);
    return orientation.normalized();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ArmSwingTrajectoryPlanner::getVelocityConstraintSlerp(size_t leg, scalar_t time, size_t coordinate) const {
    quaternion_t quat_difference =  latestTargetQuaternion_[leg].normalized() * latestInitialSwingQuaternion_[leg].inverse().normalized();
    auto quat_axisangle = Eigen::AngleAxis<scalar_t>(quat_difference.normalized());
    scalar_t angle = quat_axisangle.angle();
    scalar_t omegaMagnitude;
    if (latestSwingOrientationDuration_[leg] > 0.0001)
        omegaMagnitude = angle / latestSwingOrientationDuration_[leg];
    else
        omegaMagnitude = 0;
    auto axis = quat_axisangle.axis();
    auto omega = omegaMagnitude * axis;
//    std::cout << "angle = " << angle << std::endl;
//    std::cout << "omega magn = " << omegaMagnitude << std::endl;
//    std::cout << "axis = " << axis.transpose() << std::endl;
//    std::cout << "omega = " << omega.transpose() << std::endl;
    return omega[coordinate];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ArmSwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, scalar_t initTime, arms_array_t<scalar_array_t> currentEePosition,
                                       arms_array_t<scalar_array_t> currentEeOrientation, arms_array_t<TargetTrajectories> targetTrajectories) {
//  std::cout << "============================ UPDATE =======================" << std::endl;
//  targetTrajectories_ = targetTrajectories;
  for (int i = 0; i < targetTrajectories.size(); i++) {     // loop over arms

      // targetTrajectoriesPosition is an array of (waypoints and one final) 3-element arrays of scalars (for XYZ position)
      // for the target of arm i
      scalar_array2_t targetTrajectoriesPosition(targetTrajectories[i].stateTrajectory.size(), scalar_array_t(3));
      scalar_array2_t targetTrajectoriesOrientation(targetTrajectories[i].stateTrajectory.size(), scalar_array_t(4));

      int counter = 0;
      for (auto& target: targetTrajectories[i].stateTrajectory) {           // map TargetTrajectories to scalar_array2_t
        Eigen::Map<vector_t>(targetTrajectoriesPosition[counter].data(), targetTrajectoriesPosition[counter].size()) = target.topRows(3);
        Eigen::Map<vector_t>(targetTrajectoriesOrientation[counter].data(), targetTrajectoriesOrientation[counter].size()) = target.bottomRows(4);
        counter++;
      }

      // similar for targetTrajectoriesVelocity
      scalar_array2_t targetTrajectoriesVelocity(targetTrajectories[i].inputTrajectory.size(), scalar_array_t(3));
      counter = 0;
      for (auto& target: targetTrajectories[i].inputTrajectory) {           // map TargetTrajectories to scalar_array2_t
        Eigen::Map<vector_t>(targetTrajectoriesVelocity[counter].data(), targetTrajectoriesVelocity[counter].size()) = target.topRows(3);
        counter++;
      }

      // POSITION - Check if the last target is new or not
      if (targetTrajectoriesPosition.back() != latestTargetPosition_[i]) {      // new target received
          // clear waypoint data
          size_t waypointSize = std::max(int(targetTrajectoriesPosition.size() - 2), 0);
          latestWaypointPosition_[i].clear();
          latestWaypointPosition_[i].resize(waypointSize);
          latestWaypointVelocity_[i].clear();
          latestWaypointVelocity_[i].resize(waypointSize);
          waypointTime_[i].clear();
          switchEventTime_[i] = 0.0;        // reset value

          // save new swing information at class member variables, based on which the sequences and splines are planned
          latestSwingStartTime_[i] = initTime;
          latestInitialSwingPosition_[i] = currentEePosition[i];
          latestTargetPosition_[i] = targetTrajectoriesPosition.back();

          for (int w_index = 0; w_index < waypointSize; w_index++) {       // if more than one points are published
              latestWaypointPosition_[i][w_index] = targetTrajectoriesPosition[w_index];
          }
          for (int w_index = 0; w_index < waypointSize; w_index++) {       // if more than one points are published
              latestWaypointVelocity_[i][w_index] = targetTrajectoriesVelocity[w_index];
          }

          // default swing duration from task file or custom
          if (targetTrajectories[i].timeTrajectory.back() - initTime < 0.2) {       // if reach time is negative or has passed then default meanEeVelocity
              computeWaypointDefaultTimes(i);       // compute time at waypoints and target for default velocity
//              std::cout << "used meanEeVelocity! latestSwingDuration_ = " << latestSwingDuration_[i] << std::endl;
          } else {
              latestSwingDuration_[i] = targetTrajectories[i].timeTrajectory.back() - initTime;
              waypointTime_[i] = scalar_array_t(targetTrajectories[i].timeTrajectory.begin(), targetTrajectories[i].timeTrajectory.end() - 1);
//              std::cout << "custom latestSwingDuration_ = " << latestSwingDuration_[i] << std::endl;
          }

      } else if (initTime >= latestSwingStartTime_[i] + latestSwingDuration_[i]) {        // if current time >= latestSwingFinalTime
          latestInitialSwingPosition_[i] = latestTargetPosition_[i];                // update the start of the swing
          latestSwingStartTime_[i] = initTime;
          latestSwingDuration_[i] = 0.0;
      }
      // TODO: consider case current time is greater than latestSwingFinalTime but error between current pose
      // and target is large, i.e. the EE has not reached the target
//      printLatestSwingData();     // debug

      if (isOrientationPlanner()) {
          quaternion_t currentTargetQuaternion(targetTrajectoriesOrientation.back()[3],
                                               targetTrajectoriesOrientation.back()[0],
                                               targetTrajectoriesOrientation.back()[1],
                                               targetTrajectoriesOrientation.back()[2]);
          if (!currentTargetQuaternion.coeffs().isApprox(latestTargetQuaternion_[i].coeffs(), numeric_traits::limitEpsilon<scalar_t>())) {
              latestWaypointOrientation_[i].clear();
              latestWaypointOrientation_[i].resize(targetTrajectoriesOrientation.size() - 1);
              orientationWaypointTime_[i].clear();

              // save new swing information at class member variables, based on which the sequences and splines are planned
              latestSwingOrientationStartTime_[i] = initTime;
              latestTargetQuaternion_[i] = quaternion_t(targetTrajectoriesOrientation.back()[3],
                                                        targetTrajectoriesOrientation.back()[0],
                                                        targetTrajectoriesOrientation.back()[1],
                                                        targetTrajectoriesOrientation.back()[2]).normalized();
              latestInitialSwingQuaternion_[i] = quaternion_t(currentEeOrientation[i][3],
                                                              currentEeOrientation[i][0],
                                                              currentEeOrientation[i][1],
                                                              currentEeOrientation[i][2]).normalized();
              auto initialRPYAngles = latestInitialSwingQuaternion_[i].toRotationMatrix().eulerAngles(0, 1, 2);
              auto targetRPYAngles = latestTargetQuaternion_[i].toRotationMatrix().eulerAngles(0, 1, 2);
              latestInitialSwingOrientation_[i] = scalar_array_t(initialRPYAngles.data(), initialRPYAngles.data() + initialRPYAngles.size());
              latestTargetOrientation_[i] = scalar_array_t(targetRPYAngles.data(), targetRPYAngles.data() + targetRPYAngles.size());
              for (int w_index = 0; w_index < targetTrajectoriesOrientation.size() - 1; w_index++) {       // if more than one points are published
                  latestWaypointOrientation_[i][w_index] = targetTrajectoriesOrientation[w_index];
              }

              // default swing duration from task file or custom
              if (targetTrajectories[i].timeTrajectory.back() - initTime < 0.2) {       // if reach time is negative or has passed then default meanEeVelocity
                  computeOrientationWaypointDefaultTimes(i);
    //              std::cout << "used meanEeVelocity!" << std::endl;
              } else {
                  latestSwingOrientationDuration_[i] = targetTrajectories[i].timeTrajectory.back() - initTime;
                  orientationWaypointTime_[i] = scalar_array_t(targetTrajectories[i].timeTrajectory.begin(), targetTrajectories[i].timeTrajectory.end() - 1);
              }
//              std::cout << "latestTargetQuaternion_[i] = " <<
//                           latestTargetQuaternion_[i].coeffs().transpose() << std::endl;
//              std::cout << "latestInitialSwingQuaternion_[i] = " <<
//                           latestInitialSwingQuaternion_[i].coeffs().transpose() << std::endl;
//              std::cout << "latestSwingOrientationDuration_[i] = " << latestSwingOrientationDuration_[i] << std::endl;

          } else if (initTime >= latestSwingOrientationStartTime_[i] + latestSwingOrientationDuration_[i]) {        // if current time >= latestSwingFinalTime
    //          std::cout << "no new target" << std::endl;
              latestInitialSwingOrientation_[i] = latestTargetOrientation_[i];
              latestSwingOrientationStartTime_[i] = initTime;
              latestSwingOrientationDuration_[i] = 0.0;
              latestInitialSwingQuaternion_[i] = latestTargetQuaternion_[i];
          }
      }
  } // loop over arms

    /** // debug mode
    std::cout << "[ArmSwingTrajectoryPlanner::update] event times: ";
    for (int i = 0; i < modeSchedule.eventTimes.size(); i++) {
        std::cout << " " << modeSchedule.eventTimes[i];
    }
    std::cout << std::endl;

    std::cout << "[ArmSwingTrajectoryPlanner::update] mode sequence: ";
    for (int i = 0; i < modeSchedule.modeSequence.size(); i++) {
        std::cout << " " << modeSchedule.modeSequence[i];
    }
    std::cout << std::endl;
    **/

//  updateWithoutModes();             // spline generation without considering mode changes
  updateAlongModes(initTime, modeSchedule);             // spline by considering when a new contact is added for the leg
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ArmSwingTrajectoryPlanner::updateAlongModes(scalar_t initTime, const ModeSchedule& modeSchedule) {
  for (size_t j = 0; j < numArms_; j++) {
    // clear and re-reserve the space for the Trajectories because the modesequence has been updated.
    armsHeightTrajectories_[j].clear();
    armsHeightTrajectories_[j].reserve(3);
    armsLongitTrajectories_[j].clear();
    armsLongitTrajectories_[j].reserve(3);
    armsLateralTrajectories_[j].clear();
    armsLateralTrajectories_[j].reserve(3);

    // get contact switch time, returns existing value (zero) if no contact is detected, only get once
    if (switchEventTime_[j] == 0.0)
        switchEventTime_[j] = getContactSwitchTime(initTime, modeSchedule, j);
//    if (j == 0)
//        std::cout << "switchEventTime_[j] = " << switchEventTime_[j] << std::endl;

    // decide splineCpg junction time (end of 1st SplineCpg)
    scalar_t junctionTime;
    if (switchEventTime_[j] != 0.0) {                                     // if there is contact switch, then place junction there
        junctionTime = switchEventTime_[j];
    } else if (!latestWaypointPosition_[j].empty()) {                    // else if there is no contact switch but waypoint
        junctionTime = waypointTime_[j].front();
    } else {                                                            // else if there is neither waypoint nor contact switch just divide equally
        junctionTime = latestSwingStartTime_[j] + 0.5 * latestSwingDuration_[j];
    }

    // swing final time
    scalar_t swingFinalTime = latestSwingStartTime_[j] + latestSwingDuration_[j];       // end of swing
    const scalar_t scaling = swingTrajectoryScaling(latestSwingStartTime_[j], swingFinalTime, config_.swingTimeScale);

    // construct splines
    const vector_t initialSplineVelocities = (vector_t(3) << config_.liftOffLongVelocity, config_.liftOffLateralVelocity, config_.liftOffVelocity).finished();  // parameters from taskfile
    const vector_t finalSplineVelocities = (vector_t(3) << config_.touchDownLongVelocity, config_.touchDownLateralVelocity, config_.touchDownVelocity).finished();
    std::vector<CubicSpline::Node> initialSplineCpgNodes(3), midpointSplineCpgNodes(3), finalSplineCpgNodes(3);  // 1st spline
    std::vector<CubicSpline::Node> midpointSplineCpg2Nodes(3), finalSplineCpg2Nodes(3);   // 2nd spline
    std::vector<CubicSpline::Node> midpointSplineCpg3Nodes(3), finalSplineCpg3Nodes(3);   // 3rd spline
    scalar_t lastSplineDuration = 3.0;
    const scalar_t midpointTime = 0.5 * (latestSwingStartTime_[j] + junctionTime);   // midpoint time spline 1
    const scalar_t midpoint2Time = 0.5 * (junctionTime + swingFinalTime);   // midpoint time spline 2
    const scalar_t midpoint3Time = 0.5 * (swingFinalTime + lastSplineDuration);   // midpoint time spline 3

    // loop over XYZ coordinates
    for (int i = 0; i < 3; i++) {
        // junction position and velocity
        scalar_t junctionPosition;
        if (!latestWaypointPosition_[j].empty()) {
            junctionPosition = latestWaypointPosition_[j][0][i];
        } else {        // if no waypoint define the middle point based on the junctionTime
            scalar_t fraction;
            try {
              if (latestSwingDuration_[j] < 0.001)
                  throw std::invalid_argument("Cannot determine fraction of zero swing duration.");
              fraction = (junctionTime - latestSwingStartTime_[j]) / latestSwingDuration_[j];
            } catch (std::invalid_argument& e) {        // case denominator = 0
              fraction = 0.5;
            }
            junctionPosition = latestInitialSwingPosition_[j][i] + fraction * (latestTargetPosition_[j][i] - latestInitialSwingPosition_[j][i]);
        }

        scalar_t junctionVelocity;
        if (!latestWaypointVelocity_[j].empty()) {
            junctionVelocity = latestWaypointVelocity_[j][0][i];
        } else {        // if no waypoint define the middle point based on the junctionTime
            try {
              if (latestSwingDuration_[j] < 0.001)
                  throw std::invalid_argument("Cannot determine fraction of zero swing duration.");
              junctionVelocity = (latestTargetPosition_[j][i] - latestInitialSwingPosition_[j][i]) / latestSwingDuration_[j];       // mean velocity of the whole motion
            } catch (std::invalid_argument& e) {        // case denominator = 0
              junctionVelocity = 0.0;
            }
        }

        // splinecpg 1
        initialSplineCpgNodes[i] = CubicSpline::Node{latestSwingStartTime_[j], latestInitialSwingPosition_[j][i], scaling * initialSplineVelocities[i]};
        finalSplineCpgNodes[i] = CubicSpline::Node{junctionTime, junctionPosition, junctionVelocity};
        CubicSpline firstSpline(initialSplineCpgNodes[i], finalSplineCpgNodes[i]);      // set midpoint specs as if the splineCpg was a cubic polynomial
        const scalar_t midpointPosition = firstSpline.position(midpointTime);
        const scalar_t midpointVelocity = firstSpline.velocity(midpointTime);
        midpointSplineCpgNodes[i] = CubicSpline::Node{midpointTime, midpointPosition, midpointVelocity};

        // splinecpg 2
        finalSplineCpg2Nodes[i] = CubicSpline::Node{swingFinalTime, latestTargetPosition_[j][i], scaling * finalSplineVelocities[i]};
        CubicSpline secondSpline(finalSplineCpgNodes[i], finalSplineCpg2Nodes[i]);
        const scalar_t midpoint2Position = secondSpline.position(midpoint2Time);
        const scalar_t midpoint2Velocity = secondSpline.velocity(midpoint2Time);
        midpointSplineCpg2Nodes[i] = CubicSpline::Node{midpoint2Time, midpoint2Position, midpoint2Velocity};

        // splinecpg 3
        midpointSplineCpg3Nodes[i] = CubicSpline::Node{midpoint3Time, latestTargetPosition_[j][i], 0.0};
        finalSplineCpg3Nodes[i] = CubicSpline::Node{swingFinalTime + lastSplineDuration, latestTargetPosition_[j][i], scaling * finalSplineVelocities[i]};
    }

    // set to class member variables
    armsLongitTrajectories_[j].emplace_back(initialSplineCpgNodes[0], midpointSplineCpgNodes[0], finalSplineCpgNodes[0]);     // longitudinal
    armsLateralTrajectories_[j].emplace_back(initialSplineCpgNodes[1], midpointSplineCpgNodes[1], finalSplineCpgNodes[1]);     // lateral
    armsHeightTrajectories_[j].emplace_back(initialSplineCpgNodes[2], midpointSplineCpgNodes[2], finalSplineCpgNodes[2]);

    armsLongitTrajectories_[j].emplace_back(finalSplineCpgNodes[0], midpointSplineCpg2Nodes[0], finalSplineCpg2Nodes[0]);     // longitudinal
    armsLateralTrajectories_[j].emplace_back(finalSplineCpgNodes[1], midpointSplineCpg2Nodes[1], finalSplineCpg2Nodes[1]);     // lateral
    armsHeightTrajectories_[j].emplace_back(finalSplineCpgNodes[2], midpointSplineCpg2Nodes[2], finalSplineCpg2Nodes[2]);

    armsLongitTrajectories_[j].emplace_back(finalSplineCpg2Nodes[0], midpointSplineCpg3Nodes[0], finalSplineCpg3Nodes[0]);     // longitudinal
    armsLateralTrajectories_[j].emplace_back(finalSplineCpg2Nodes[1], midpointSplineCpg3Nodes[1], finalSplineCpg3Nodes[1]);     // lateral
    armsHeightTrajectories_[j].emplace_back(finalSplineCpg2Nodes[2], midpointSplineCpg3Nodes[2], finalSplineCpg3Nodes[2]);

    armsHeightTrajectoriesEvents_[j] = {junctionTime, swingFinalTime, swingFinalTime + lastSplineDuration};  //eventTimes;

//    if (j == 0) {
//        std::cout << " ======================================================================================" << std::endl;
//        std::cout << " ======================================================================================" << std::endl;
//        std::cout << " ======================================================================================" << std::endl;
//        std::cout << " ======================================================================================" << std::endl;
//        std::cout << " ======================================================================================" << std::endl;
//        std::cout << "1. pos x = " << initialSplineCpgNodes[0].position << ", " << midpointSplineCpgNodes[0].position << ", " << finalSplineCpgNodes[0].position << std::endl;
//        std::cout << "1. vel x = " << initialSplineCpgNodes[0].velocity << ", " << midpointSplineCpgNodes[0].velocity << ", " << finalSplineCpgNodes[0].velocity << std::endl;
//        std::cout << "2. pos x = " << finalSplineCpgNodes[0].position << ", " << midpointSplineCpg2Nodes[0].position << ", " << finalSplineCpg2Nodes[0].position << std::endl;
//        std::cout << "2. vel x = " << finalSplineCpgNodes[0].velocity << ", " << midpointSplineCpg2Nodes[0].velocity << ", " << finalSplineCpg2Nodes[0].velocity << std::endl;
//        std::cout << "3. pos x = " << finalSplineCpg2Nodes[0].position << ", " << midpointSplineCpg3Nodes[0].position << ", " << finalSplineCpg3Nodes[0].position << std::endl;
//        std::cout << "3. vel x = " << finalSplineCpg2Nodes[0].velocity << ", " << midpointSplineCpg3Nodes[0].velocity << ", " << finalSplineCpg3Nodes[0].velocity << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ArmSwingTrajectoryPlanner::computeWaypointDefaultTimes(const size_t& armIndex) {
    scalar_t dx, dy, dz;
    scalar_t dsSum = 0;
    scalar_array_t ds;
    auto previousWaypoint = latestInitialSwingPosition_[armIndex];
    for (auto& waypoint: latestWaypointPosition_[armIndex]) {
        dx = waypoint[0] - previousWaypoint[0];
        dy = waypoint[1] - previousWaypoint[1];
        dz = waypoint[2] - previousWaypoint[2];
        ds.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
        dsSum += ds.back();
        previousWaypoint = waypoint;
    }
    dx = latestTargetPosition_[armIndex][0] - previousWaypoint[0];
    dy = latestTargetPosition_[armIndex][1] - previousWaypoint[1];
    dz = latestTargetPosition_[armIndex][2] - previousWaypoint[2];
    ds.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
    dsSum += ds.back();

    latestSwingDuration_[armIndex] = dsSum / config_.meanEeVelocity;
    for (int index = 0; index < latestWaypointPosition_[armIndex].size(); index++) {
        waypointTime_[armIndex].push_back(latestSwingStartTime_[armIndex] + ds[index] * latestSwingDuration_[armIndex] / dsSum);
//        std::cout << "waypointTime_[armIndex].back() - initTime = " << waypointTime_[armIndex].back() - latestSwingStartTime_[armIndex] << std::endl;
//        std::cout << "time fraction = " << (waypointTime_[armIndex].back() - latestSwingStartTime_[armIndex])/latestSwingDuration_[armIndex] << std::endl;
//        std::cout << "waypointTime_[armIndex].back() = " << waypointTime_[armIndex].back() << std::endl;
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ArmSwingTrajectoryPlanner::computeOrientationWaypointDefaultTimes(const size_t& armIndex) {
    scalar_t dr, dp, dy;
    scalar_t dPhiSum = 0;
    scalar_array_t dPhi;
    auto previousWaypoint = latestInitialSwingOrientation_[armIndex];
    for (auto& waypoint: latestWaypointOrientation_[armIndex]) {
        dr = waypoint[0] - previousWaypoint[0];
        dp = waypoint[1] - previousWaypoint[1];
        dy = waypoint[2] - previousWaypoint[2];
        dPhi.push_back(std::sqrt(dr * dr + dp * dp + dy * dy));
        dPhiSum += dPhi.back();
        previousWaypoint = waypoint;
    }
    dr = latestTargetOrientation_[armIndex][0] - previousWaypoint[0];
    dp = latestTargetOrientation_[armIndex][1] - previousWaypoint[1];
    dy = latestTargetOrientation_[armIndex][2] - previousWaypoint[2];
    dPhi.push_back(std::sqrt(dr * dr + dp * dp + dy * dy));
    dPhiSum += dPhi.back();

    latestSwingOrientationDuration_[armIndex] = dPhiSum / 0.8; //config_.meanEeVelocity; hardcoded for orientation
    for (int index = 0; index < latestWaypointOrientation_[armIndex].size(); index++) {
        orientationWaypointTime_[armIndex].push_back(latestSwingOrientationStartTime_[armIndex] + dPhi[index] * latestSwingOrientationDuration_[armIndex] / dPhiSum);
//        std::cout << "orientationWaypointTime_[armIndex].back() - initTime = " << orientationWaypointTime_[armIndex].back() - latestSwingOrientationStartTime_[armIndex] << std::endl;
//        std::cout << "time fraction = " << (orientationWaypointTime_[armIndex].back() - latestSwingOrientationStartTime_[armIndex])/latestSwingOrientationDuration_[armIndex] << std::endl;
//        std::cout << "orientationWaypointTime_[armIndex].back() = " << orientationWaypointTime_[armIndex].back() << std::endl;
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ArmSwingTrajectoryPlanner::getContactSwitchTime(const scalar_t& initTime, const ModeSchedule& modeSchedule, size_t armIndex) {
    scalar_t contactSwitchTime = switchEventTime_[armIndex];     // initialize
    const auto currentArmContactFlags = extractContactFlags(modeSchedule.modeSequence)[armIndex];  // contact flags for the arm
    const auto currentModeIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);  // index of the current mode
    auto currentContactFlag = currentArmContactFlags[currentModeIndex];   // current contact flag for the arm

    /** search index of the new mode excluding modes of the past (.begin() + currentModeIndex)
      * and last mode (.end() - 1) that is always stance **/
    size_t contactFlagChangeIndex = std::distance(currentArmContactFlags.begin(),
                                                  std::find(currentArmContactFlags.begin() + currentModeIndex,
                                                            currentArmContactFlags.end() - 1,
                                                            !currentContactFlag));
    // if a contact change was found
    if (contactFlagChangeIndex != currentArmContactFlags.size() - 1) {
        contactSwitchTime = modeSchedule.eventTimes[contactFlagChangeIndex - 1];
        // debug
//        std::cout << "[updateAlongModes] currentArmContactFlags = ";
//        for (int iter = 0; iter < currentArmContactFlags.size(); iter++)
//            std::cout << currentArmContactFlags[iter] << " ";
//        std::cout << " " << std::endl;
//        std::cout << "[updateAlongModes] currentContactFlag = " << currentContactFlag << std::endl;
//        std::cout << "[updateAlongModes] currentModeIndex = " << currentModeIndex << std::endl;
//        std::cout << "[updateAlongModes] contactFlagChangeIndex = " << contactFlagChangeIndex << std::endl;
//        std::cout << "[updateAlongModes] eventTime selected!" << std::endl;
//        std::cout << "[ArmSwingTrajectoryPlanner::getContactSwitchTime] contactSwitchTime = " << contactSwitchTime << std::endl;
    }
//    else
//        std::cout << "[ArmSwingTrajectoryPlanner::getContactSwitchTime] No contact switch was found " << std::endl;
    return contactSwitchTime;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ArmSwingTrajectoryPlanner::printLatestSwingData(const size_t& armIndex) {
        std::cout << "" << std::endl;
        std::cout << "*****************************************************" << std::endl;
        std::cout << "latestSwingStartTime_[armIndex] = " << latestSwingStartTime_[armIndex] << std::endl;
        std::cout << "latestSwingDuration_[armIndex] = " << latestSwingDuration_[armIndex] << std::endl;
        std::cout << "latestInitialSwingPosition_[armIndex] = " << latestInitialSwingPosition_[armIndex][0] << ", " << latestInitialSwingPosition_[armIndex][1]
                  << ", " << latestInitialSwingPosition_[armIndex][2] << std::endl;
        std::cout << "latestTargetPosition_[armIndex] = " << latestTargetPosition_[armIndex][0] << ", "
                  << latestTargetPosition_[armIndex][1] << ", " << latestTargetPosition_[armIndex][2] << std::endl;
        if (latestWaypointPosition_[armIndex].size() > 0) {
            std::cout << "waypointTime_[armIndex][0] = " << waypointTime_[armIndex][0] << std::endl;
            std::cout << "latestWaypointPosition_[armIndex][0] = " << latestWaypointPosition_[armIndex][0][0] << ", " << latestWaypointPosition_[armIndex][0][1] << ", " << latestWaypointPosition_[armIndex][0][2] << std::endl;
        }
        std::cout << "*****************************************************" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ArmSwingTrajectoryPlanner::updateWithoutModes() {

  for (size_t j = 0; j < numArms_; j++) {
    // clear and re-reserve the space for the Trajectories because the modesequence has been updated.
    armsHeightTrajectories_[j].clear();
    armsHeightTrajectories_[j].reserve(2);
    armsLongitTrajectories_[j].clear();
    armsLongitTrajectories_[j].reserve(2);
    armsLateralTrajectories_[j].clear();
    armsLateralTrajectories_[j].reserve(2);

    // swing final time
    scalar_t swingFinalTime = latestSwingStartTime_[j] + latestSwingDuration_[j];       // end of swing
    const scalar_t scaling = swingTrajectoryScaling(latestSwingStartTime_[j], swingFinalTime, config_.swingTimeScale);
    //        std::cout << "[ArmSwingTrajectoryPlanner::update] latestSwingStartTime_[j] = " << latestSwingStartTime_[j] << std::endl;
    //        std::cout << "[ArmSwingTrajectoryPlanner::update] swingFinalTime = " << swingFinalTime << std::endl;

    // construct splines
    const vector_t initialSplineVelocities = (vector_t(3) << config_.liftOffLongVelocity, config_.liftOffLateralVelocity, config_.liftOffVelocity).finished();  // parameters from taskfile
    const vector_t finalSplineVelocities = (vector_t(3) << config_.touchDownLongVelocity, config_.touchDownLateralVelocity, config_.touchDownVelocity).finished();
    std::vector<CubicSpline::Node> initialSplineCpgNodes(3), midpointSplineCpgNodes(3), finalSplineCpgNodes(3);  // 1st spline
    std::vector<CubicSpline::Node> midpointSplineCpg2Nodes(3), finalSplineCpg2Nodes(3);   // 2nd spline
    scalar_t secondSplineDuration = 3.0;
    const scalar_t midpointTime = 0.5 * (latestSwingStartTime_[j] + swingFinalTime);        // midpoint time of first spline
    const scalar_t midpoint2Time = 0.5 * (swingFinalTime + secondSplineDuration);           // midpoint time of 2nd spline
    // loop over XYZ coordinates
    for (int i = 0; i < 3; i++) {
        // first spline
        const scalar_t midpointPosition = 0.5 * (latestInitialSwingPosition_[j][i] + latestTargetPosition_[j][i]);
        const scalar_t midpointVelocity = 3 * (latestTargetPosition_[j][i] - latestInitialSwingPosition_[j][i]) / (swingFinalTime - latestSwingStartTime_[j]);
        initialSplineCpgNodes[i] = CubicSpline::Node{latestSwingStartTime_[j], latestInitialSwingPosition_[j][i], scaling * initialSplineVelocities[i]};
        midpointSplineCpgNodes[i] = CubicSpline::Node{midpointTime, midpointPosition, midpointVelocity};
        finalSplineCpgNodes[i] = CubicSpline::Node{swingFinalTime, latestTargetPosition_[j][i], scaling * finalSplineVelocities[i]};

        // second spline
        midpointSplineCpg2Nodes[i] = CubicSpline::Node{midpoint2Time, latestTargetPosition_[j][i], 0.0};
        finalSplineCpg2Nodes[i] = CubicSpline::Node{swingFinalTime + secondSplineDuration, latestTargetPosition_[j][i], scaling * finalSplineVelocities[i]};
    }

    // set to class member variables
    armsLongitTrajectories_[j].emplace_back(initialSplineCpgNodes[0], midpointSplineCpgNodes[0], finalSplineCpgNodes[0]);     // longitudinal
    armsLateralTrajectories_[j].emplace_back(initialSplineCpgNodes[1], midpointSplineCpgNodes[1], finalSplineCpgNodes[1]);     // lateral
    armsHeightTrajectories_[j].emplace_back(initialSplineCpgNodes[2], midpointSplineCpgNodes[2], finalSplineCpgNodes[2]);

    // set to class member variables
    armsLongitTrajectories_[j].emplace_back(finalSplineCpgNodes[0], midpointSplineCpg2Nodes[0], finalSplineCpg2Nodes[0]);     // longitudinal
    armsLateralTrajectories_[j].emplace_back(finalSplineCpgNodes[1], midpointSplineCpg2Nodes[1], finalSplineCpg2Nodes[1]);     // lateral
    armsHeightTrajectories_[j].emplace_back(finalSplineCpgNodes[2], midpointSplineCpg2Nodes[2], finalSplineCpg2Nodes[2]);

    armsHeightTrajectoriesEvents_[j] = {swingFinalTime, swingFinalTime + secondSplineDuration};    //eventTimes;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::array<arms_array_t<scalar_array_t>, 6> ArmSwingTrajectoryPlanner::getSequenceFromMode(
        arms_array_t<scalar_array_t> initialEePosition, arms_array_t<scalar_array_t> targetEePosition,
        const ModeSchedule& modeSchedule) const
{
    // create lifOffSequence, touchdownSequence from initialEePosition, targetEePosition
//    std::array<arms_array_t<scalar_array_t>, 6> liftOffTouchDownSeqs = {liftOffLongitudinalSequence, touchDownLongitudinalSequence,
//                                                                        liftOffLateralSequence, touchDownLateralSequence,
//                                                                        liftOffVerticalSequence, touchDownVerticalSequence};
    std::array<arms_array_t<scalar_array_t>, 6> liftOffTouchDownSeqs;
    for (int j = 0; j < 3; j++) {                                               // loop over coordinates
        for (int i = 0; i < initialEePosition.size(); i++) {                    // loop over arms
            liftOffTouchDownSeqs[2*j][i] = scalar_array_t(modeSchedule.modeSequence.size(), initialEePosition[i].at(j));        // lift off
            liftOffTouchDownSeqs[2*j+1][i] = scalar_array_t(modeSchedule.modeSequence.size(), targetEePosition[i].at(j));       // touch down   
        }
    }

    // debug arm 1
    for (int i = 0; i < modeSchedule.modeSequence.size(); i++) {
        std::cout << "Mode " << i << std::endl;
        std::cout << "lifoff = " << liftOffTouchDownSeqs[0][0][i] << ", " << liftOffTouchDownSeqs[2][0][i] << ", " << liftOffTouchDownSeqs[4][0][i] << std::endl;;
        std::cout << "touchDown = " << liftOffTouchDownSeqs[1][0][i] << ", " << liftOffTouchDownSeqs[3][0][i] << ", " << liftOffTouchDownSeqs[5][0][i] << std::endl;;
        std::cout << " " << std::endl;
    }

    return liftOffTouchDownSeqs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<std::vector<int>, std::vector<int>> ArmSwingTrajectoryPlanner::updateFootSchedule(const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();

  std::vector<int> startTimeIndexStock(numPhases, 0);
  std::vector<int> finalTimeIndexStock(numPhases, 0);

  // find the startTime and finalTime indices for swing arms
  for (size_t i = 0; i < numPhases; i++) {
    if (!contactFlagStock[i]) {
      std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
    }
  }
  return {startTimeIndexStock, finalTimeIndexStock};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
arms_array_t<std::vector<bool>> ArmSwingTrajectoryPlanner::extractContactFlags(const std::vector<size_t>& phaseIDsStock) const {
  const size_t numPhases = phaseIDsStock.size();

  arms_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
//    std::cout << "[ArmSwingTrajectoryPlanner::extractContactFlags] phaseIDsStock[i] = " << phaseIDsStock[i] << std::endl;
    const auto contactFlag = modeNumber2StanceArm(phaseIDsStock[i]);
    for (size_t j = 0; j < numArms_; j++) {
      contactFlagStock[j][i] = contactFlag[j];
//      std::cout << "[ArmSwingTrajectoryPlanner::extractContactFlags] contactFlag[j] = " << contactFlag[j] << std::endl;
    }
  }
  return contactFlagStock;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<int, int> ArmSwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();

  // skip if it is a stance leg
//  if (contactFlagStock[index]) {
//    return {0, 0};
//  }

  std::cout << "[ArmSwingTrajectoryPlanner::findIndex] numPhases = " << numPhases << std::endl;
  // find the starting time
  int startTimesIndex = -1;
  std::cout << "[ArmSwingTrajectoryPlanner::findIndex] index = " << index << std::endl;
  for (int ip = index - 1; ip >= 0; ip--) {
    if (contactFlagStock[ip]) {
      startTimesIndex = ip;
      std::cout << "[ArmSwingTrajectoryPlanner::findIndex] startTimesIndex = " << startTimesIndex << std::endl;
      break;
    }
  }
  std::cout << "[ArmSwingTrajectoryPlanner::findIndex] startTimesIndex = " << startTimesIndex << std::endl;

  // find the final time
  int finalTimesIndex = numPhases - 1;
  std::cout << "[ArmSwingTrajectoryPlanner::findIndex] finalTimesIndex = " << finalTimesIndex << std::endl;
  for (size_t ip = index + 1; ip < numPhases; ip++) {
    if (contactFlagStock[ip]) {
      finalTimesIndex = ip - 1;
      std::cout << "[ArmSwingTrajectoryPlanner::findIndex] break finalTimesIndex = " << finalTimesIndex << std::endl;
      break;
    }
  }

  return {startTimesIndex, finalTimesIndex};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ArmSwingTrajectoryPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                      const std::vector<size_t>& phaseIDsStock) {
  const size_t numSubsystems = phaseIDsStock.size();
  if (startIndex < 0) {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
  if (finalIndex >= numSubsystems - 1) {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ArmSwingTrajectoryPlanner::swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale) {
  return std::min(1.0, (finalTime - startTime) / swingTimeScale);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ArmSwingTrajectoryPlanner::Config loadArmSwingTrajectorySettings(const std::string& fileName, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  size_array_t positionIndicesLeftArm, positionIndicesRightArm, orientationIndicesLeftArm, orientationIndicesRightArm;
  if (verbose) {
    std::cerr << "\n #### Arm Swing Trajectory Config:";
    std::cerr << "\n #### =============================================================================\n";
  }

  ArmSwingTrajectoryPlanner::Config config;
  const std::string prefix = fieldName + ".";

  loadData::loadPtreeValue(pt, config.meanEeVelocity, prefix + "meanEeVelocity", verbose);
  loadData::loadPtreeValue(pt, config.positionPlanner, prefix + "positionPlanner", verbose);
  loadData::loadPtreeValue(pt, config.orientationPlanner, prefix + "orientationPlanner", verbose);
  loadData::loadPtreeValue(pt, config.addConstraints, prefix + "addConstraints", verbose);
  loadData::loadStdVector(fileName, prefix + "positionIndicesLeftArm", positionIndicesLeftArm);
  loadData::loadStdVector(fileName, prefix + "positionIndicesRightArm", positionIndicesRightArm);
  loadData::loadStdVector(fileName, prefix + "orientationIndicesLeftArm", orientationIndicesLeftArm);
  loadData::loadStdVector(fileName, prefix + "orientationIndicesRightArm", orientationIndicesRightArm);
  config.positionIndices = {positionIndicesLeftArm, positionIndicesRightArm};
  config.orientationIndices = {orientationIndicesLeftArm, orientationIndicesRightArm};

  loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);
  loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);
  loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);

  loadData::loadPtreeValue(pt, config.liftOffLongVelocity, prefix + "liftOffLongVelocity", verbose);
  loadData::loadPtreeValue(pt, config.touchDownLongVelocity, prefix + "touchDownLongVelocity", verbose);

  loadData::loadPtreeValue(pt, config.liftOffLateralVelocity, prefix + "liftOffLateralVelocity", verbose);
  loadData::loadPtreeValue(pt, config.touchDownLateralVelocity, prefix + "touchDownLateralVelocity", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return config;
}

}  // namespace legged_robot
}  // namespace ocs2
