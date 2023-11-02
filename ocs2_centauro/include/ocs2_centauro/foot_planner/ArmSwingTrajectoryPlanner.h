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

#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_centauro/common/Types.h"
#include "ocs2_centauro/foot_planner/SplineCpg.h"
#include <ocs2_core/reference/TargetTrajectories.h>      // for using TargetTrajectories

namespace ocs2 {
namespace legged_robot {

class ArmSwingTrajectoryPlanner {
 public:
  struct Config {
    bool positionPlanner = true;
    bool orientationPlanner = false;
    bool addConstraints = true;     // add constraints for equality arm EEs position
    size_array2_t positionIndices = {{0, 1, 2}, {0, 1, 2}};     // left, right ee
    size_array2_t orientationIndices = {{0, 1, 2}, {0, 1, 2}};

    scalar_t meanEeVelocity = 1.0;
    scalar_t liftOffVelocity = 0.0;
    scalar_t touchDownVelocity = 0.0;
    scalar_t swingTimeScale = 0.15;  // swing phases shorter than this time will be scaled down in height and velocity

    scalar_t liftOffLongVelocity = 0.05;
    scalar_t touchDownLongVelocity = -0.05;

    scalar_t liftOffLateralVelocity = 0.0;
    scalar_t touchDownLateralVelocity = 0.0;
  };

  ArmSwingTrajectoryPlanner(Config config, size_t numArms);

  /**
   * @brief update sets the necessary class member variables for planning arm EE trajectories, e.g. latestTargetPosition etc.
   * @param modeSchedule the mode Schedule
   * @param initTime the current time
   * @param initialEePosition the current positions of the arm EEs
   * @param targetTrajectories the targets for the arm EE poses.
   *
   * This function checks and updates the member variables in case a new target is received. If the targetTrajectories have more than one pose, then
   * it as well considers waypoints (e.g. targetTrajectories[i].stateTrajectory[0] is a waypoint and targetTrajectories[i].stateTrajectory[1]
   * is the final target). Similarly, it considers the times for reaching this points defined in targetTrajectories[i].timeTrajectory.
   * If targetTrajectories[i].timeTrajectory.back() has already passed then targetTrajectories[i].timeTrajectory is not considered and the
   * trajectory planning is done using a default mean EE velocity from taskfile.
   */
  void update(const ModeSchedule& modeSchedule, scalar_t initTime, arms_array_t<scalar_array_t> initialEePosition,
              arms_array_t<scalar_array_t> initialEeOrientation, arms_array_t<TargetTrajectories> targetTrajectories);

  scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;
  scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;
  scalar_t getXvelocityConstraint(size_t leg, scalar_t time) const;
  scalar_t getXpositionConstraint(size_t leg, scalar_t time) const;
  scalar_t getYvelocityConstraint(size_t leg, scalar_t time) const;
  scalar_t getYpositionConstraint(size_t leg, scalar_t time) const;

  scalar_t getVelocityConstraintSlerp(size_t leg, scalar_t time, size_t coordinate) const;  // orientation control
  quaternion_t getOrientationConstraintSlerp(size_t leg, scalar_t time) const;


  ArmSwingTrajectoryPlanner::Config getConfig() const { return config_;}

  // TBD change initial and final Ee Position to arms_array_t<std::vector3_t>
  std::array<arms_array_t<scalar_array_t>, 6> getSequenceFromMode(arms_array_t<scalar_array_t> initialEePosition,
                                                                  arms_array_t<scalar_array_t> targetEePosition,
                                                                  const ModeSchedule& modeSchedule) const;
  size_t getArmsNumber() const {return numArms_;}
  bool isPositionPlanner() const {return config_.positionPlanner;}
  bool isOrientationPlanner() const {return config_.orientationPlanner;}

 private:
  quaternion_t slerp_derivative(quaternion_t q0, quaternion_t q1, scalar_t time, size_t leg) const;     // not used, not checked

  // Update by creating a single spline for the motion and not multiple ones for each mode
  void updateWithoutModes();

  /**
   * @brief updateAlongModes plans considering a possible change that may happen in modes
   * @param initTime current time
   * @param modeSchedule the mode schedule
   * Plans three splineCpg. The 1st splineCpg plans the trajectory until either the contact switch time (if this detected at the modeSchedule)
   * or the waypoint (if defined) or just half of the motion (if no waypoint and no contact switch are defined). The 2nd splineCpg plans
   * the rest of the trajectory until the final target and the 3rd splineCpg is just for the case there is no new target. The velocity at the
   * waypoint/contact switch time is received from targetTrajectories.timeTrajectory.front() if targetTrajectories.timeTrajectory.size() >= 2
   * (otherwise a default value is computed).
   */
  void updateAlongModes(scalar_t initTime, const ModeSchedule& modeSchedule);

  /**
   * @brief getContactChangeTime gives the time of an upcoming change in contact for a single arm
   * @param initTime the current time
   * @param modeSchedule the mode schedule
   * @param armIndex 0 or 1 for left and right arm
   */
  scalar_t getContactSwitchTime(const scalar_t& initTime, const ModeSchedule& modeSchedule, size_t armIndex);

  /**
   * @brief computeWaypointDefaultTimes compute waypointTime_ and latestSwingDuration_
   * @param armIndex
   */
  void computeWaypointDefaultTimes(const size_t& armIndex);

  /**
   * @brief computeOrientationWaypointDefaultTimes for orientation compute waypointTime_ and latestSwingDuration_
   * @param armIndex
   */
  void computeOrientationWaypointDefaultTimes(const size_t& armIndex);

  void printLatestSwingData(const size_t& armIndex);
  /**
   * Extracts for each leg the contact sequence over the motion phase sequence.
   * @param phaseIDsStock
   * @return contactFlagStock
   */
  arms_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

  /**
   * Finds the take-off and touch-down times indices for a specific leg.
   *
   * @param index
   * @param contactFlagStock
   * @return {The take-off time index for swing legs, touch-down time index for swing legs}
   */
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

  /**
   * based on the input phaseIDsStock finds the start subsystem and final subsystem of the swing
   * phases of the a foot in each subsystem.
   *
   * startTimeIndexStock: eventTimes[startTimesIndex] will be the take-off time for the requested leg.
   * finalTimeIndexStock: eventTimes[finalTimesIndex] will be the touch-down time for the requested leg.
   *
   * @param [in] footIndex: Foot index
   * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
   * @param [in] contactFlagStock: The sequence of the contact status for the requested leg.
   * @return { startTimeIndexStock, finalTimeIndexStock}
   */
  static std::pair<std::vector<int>, std::vector<int>> updateFootSchedule(const std::vector<bool>& contactFlagStock);

  /**
   * Check if event time indices are valid
   * @param leg
   * @param index : phase index
   * @param startIndex : liftoff event time index
   * @param finalIndex : touchdown event time index
   * @param phaseIDsStock : mode sequence
   */
  static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex, const std::vector<size_t>& phaseIDsStock);

  static scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale);

  const Config config_;
  const size_t numArms_;

  arms_array_t<std::vector<SplineCpg>> armsHeightTrajectories_;
  arms_array_t<std::vector<SplineCpg>> armsLongitTrajectories_;     // longitudinal
  arms_array_t<std::vector<SplineCpg>> armsLateralTrajectories_;     // lateral
  arms_array_t<std::vector<scalar_t>> armsHeightTrajectoriesEvents_;

  // related with updating swing targets
  arms_array_t<scalar_array_t> latestInitialSwingPosition_;
  arms_array_t<scalar_array2_t> latestWaypointPosition_;
  arms_array_t<scalar_array2_t> latestWaypointVelocity_;
  arms_array_t<scalar_array_t> latestTargetPosition_;
  arms_array_t<scalar_t> latestSwingStartTime_;
  arms_array_t<scalar_t> latestSwingDuration_;
  // orientation
  arms_array_t<scalar_t> latestSwingOrientationStartTime_;      // timings
  arms_array_t<scalar_t> latestSwingOrientationDuration_;
  arms_array_t<scalar_array_t> latestInitialSwingOrientation_;      // euler values
  arms_array_t<scalar_array_t> latestTargetOrientation_;
  arms_array_t<scalar_array2_t> latestWaypointOrientation_;
  arms_array_t<quaternion_t> latestInitialSwingQuaternion_;         // quaternion values
  arms_array_t<quaternion_t> latestTargetQuaternion_;
//  arms_array_t<std::vector<scalar_t>> armsHeightTrajectoriesOrientationEvents_;

  arms_array_t<scalar_t> switchEventTime_;    // related with updateAlongModes
  arms_array_t<scalar_array_t> waypointTime_;
  arms_array_t<scalar_array_t> orientationWaypointTime_;
};

ArmSwingTrajectoryPlanner::Config loadArmSwingTrajectorySettings(
        const std::string& fileName,
        const std::string& fieldName = "arm_swing_trajectory_config", bool verbose = true);

}  // namespace legged_robot
}  // namespace ocs2
