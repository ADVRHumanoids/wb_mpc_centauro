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

#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace legged_robot {

struct ModelSettings {
  scalar_t positionErrorGain = 0.0;
  scalar_t orientationErrorGain = 0.0;

  scalar_t phaseTransitionStanceTime = 0.4;

  bool verboseCppAd = true;
  bool recompileLibrariesCppAd = true;
  std::string modelFolderCppAd = "/tmp/ocs2";

  std::vector<std::string> jointNames{"hip_yaw_1", "hip_pitch_1", "knee_pitch_1", "ankle_pitch_1", "ankle_yaw_1", "j_wheel_1",
                                      "hip_yaw_2", "hip_pitch_2", "knee_pitch_2", "ankle_pitch_2", "ankle_yaw_2", "j_wheel_2",
                                      "hip_yaw_3", "hip_pitch_3", "knee_pitch_3", "ankle_pitch_3", "ankle_yaw_3", "j_wheel_3",
                                      "hip_yaw_4", "hip_pitch_4", "knee_pitch_4", "ankle_pitch_4", "ankle_yaw_4", "j_wheel_4",
                                      "torso_yaw",
                                      "j_arm1_1", "j_arm1_2", "j_arm1_3", "j_arm1_4", "j_arm1_5", "j_arm1_6",
                                      "j_arm2_1", "j_arm2_2", "j_arm2_3", "j_arm2_4", "j_arm2_5", "j_arm2_6"
                                     };

  std::vector<std::string> contactNames6DoF{};
  std::vector<std::string> contactNames3DoF{"contact_1", "contact_2", "contact_3", "contact_4", "arm1_8", "arm2_8"};

  std::vector<std::string> armContactNames3DoF{"arm1_8", "arm2_8"};
  std::vector<std::string> legContactNames3DoF{"contact_1", "contact_2", "contact_3", "contact_4"};


};

ModelSettings loadModelSettings(const std::string& filename, const std::string& fieldName = "model_settings", bool verbose = "true");

}  // namespace legged_robot
}  // namespace ocs2
