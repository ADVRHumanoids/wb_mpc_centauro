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
#include <map>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_centauro/common/Types.h"

namespace ocs2 {
namespace legged_robot {

enum ModeNumber {  // {LF, RF, LH, RH}
  FLY = 0,
  RH = 1,
  LH = 2,
  LH_RH = 3,
  RF = 4,
  RF_RH = 5,
  RF_LH = 6,
  RF_LH_RH = 7,
  LF = 8,
  LF_RH = 9,
  LF_LH = 10,
  LF_LH_RH = 11,
  LF_RF = 12,
  LF_RF_RH = 13,
  LF_RF_LH = 14,
  STANCE = 15,

  RA = 16,
  RA_RH = 17,
  RA_LH = 18,
  RA_LH_RH = 19,
  RA_RF = 20,
  RA_RF_RH = 21,
  RA_RF_LH = 22,
  RA_RF_LH_RH = 23,
  RA_LF = 24,
  RA_LF_RH = 25,
  RA_LF_LH = 26,
  RA_LF_LH_RH = 27,
  RA_LF_RF = 28,
  RA_LF_RF_RH = 29,
  RA_LF_RF_LH = 30,
  RA_STANCE = 31,

  LA = 32,
  LA_RH = 33,
  LA_LH = 34,
  LA_LH_RH = 35,
  LA_RF = 36,
  LA_RF_RH = 37,
  LA_RF_LH = 38,
  LA_RF_LH_RH = 39,
  LA_LF = 40,
  LA_LF_RH = 41,
  LA_LF_LH = 42,
  LA_LF_LH_RH = 43,
  LA_LF_RF = 44,
  LA_LF_RF_RH = 45,
  LA_LF_RF_LH = 46,
  LA_STANCE = 47,
  LA_RA = 48,
  LA_RA_RH = 49,
  LA_RA_LH = 50,
  LA_RA_LH_RH = 51,
  LA_RA_RF = 52,
  LA_RA_RF_RH = 53,
  LA_RA_RF_LH = 54,
  LA_RA_RF_LH_RH = 55,
  LA_RA_LF = 56,    //
  LA_RA_LF_RH = 57,
  LA_RA_LF_LH = 58,
  LA_RA_LF_LH_RH = 59,
  LA_RA_LF_RF = 60,
  LA_RA_LF_RF_RH = 61,
  LA_RA_LF_RF_LH = 62,
  LA_RA_STANCE = 63
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline contact_flag_t modeNumber2StanceLeg(const size_t& modeNumber) {
  contact_flag_t stanceLegs;  // {LF, RF, LH, RH}

  const int legModeOffset = 16;

  switch (modeNumber) {
    case 0: case (0 + legModeOffset): case (0 + 2 * legModeOffset): case (0 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, false, false, false};
      break;  // 0:  0-leg-stance
    case 1: case (1 + legModeOffset): case (1 + 2 * legModeOffset): case (1 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, false, false, true};
      break;  // 1:  RH
    case 2: case (2 + legModeOffset): case (2 + 2 * legModeOffset): case (2 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, false, true, false};
      break;  // 2:  LH
    case 3: case (3 + legModeOffset): case (3 + 2 * legModeOffset): case (3 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, false, true, true};
      break;  // 3:  RH, LH
    case 4: case (4 + legModeOffset): case (4 + 2 * legModeOffset): case (4 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, true, false, false};
      break;  // 4:  RF
    case 5: case (5 + legModeOffset): case (5 + 2 * legModeOffset): case (5 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, true, false, true};
      break;  // 5:  RF, RH
    case 6: case (6 + legModeOffset): case (6 + 2 * legModeOffset): case (6 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, true, true, false};
      break;  // 6:  RF, LH
    case 7: case (7 + legModeOffset): case (7 + 2 * legModeOffset): case (7 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{false, true, true, true};
      break;  // 7:  RF, LH, RH
    case 8: case (8 + legModeOffset): case (8 + 2 * legModeOffset): case (8 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, false, false, false};
      break;  // 8:  LF,
    case 9: case (9 + legModeOffset): case (9 + 2 * legModeOffset): case (9 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, false, false, true};
      break;  // 9:  LF, RH
    case 10: case (10 + legModeOffset): case (10 + 2 * legModeOffset): case (10 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, false, true, false};
      break;  // 10: LF, LH
    case 11: case (11 + legModeOffset): case (11 + 2 * legModeOffset): case (11 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, false, true, true};
      break;  // 11: LF, LH, RH
    case 12: case (12 + legModeOffset): case (12 + 2 * legModeOffset): case (12 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, true, false, false};
      break;  // 12: LF, RF
    case 13: case (13 + legModeOffset): case (13 + 2 * legModeOffset): case (13 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, true, false, true};
      break;  // 13: LF, RF, RH
    case 14: case (14 + legModeOffset): case (14 + 2 * legModeOffset): case (14 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, true, true, false};
      break;  // 14: LF, RF, LH
    case 15: case (15 + legModeOffset): case (15 + 2 * legModeOffset): case (15 + 3 * legModeOffset):
      stanceLegs = contact_flag_t{true, true, true, true};
      break;  // 15: 4-leg-stance
  }

  return stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline arm_contact_flag_t modeNumber2StanceArm(const size_t& modeNumber) {
  arm_contact_flag_t stanceArms;  // {LF, RF, LH, RH}
  const int legModeOffset = 16;

  if (modeNumber < legModeOffset) {
      stanceArms = arm_contact_flag_t{false, false};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  else if (legModeOffset - 1 <  modeNumber && modeNumber < 2 * legModeOffset) {
      stanceArms = arm_contact_flag_t{false, true};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  else if (2 * legModeOffset - 1 < modeNumber && modeNumber < 3 * legModeOffset) {
      stanceArms = arm_contact_flag_t{true, false};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  else if (3 * legModeOffset - 1 < modeNumber) {
      stanceArms = arm_contact_flag_t{true, true};
//      std::cout << "[modeNumber2StanceArm] modeNumber = " << modeNumber << ", stanceArms = " << stanceArms[0] << " " << stanceArms[1] << std::endl;
  }
  return stanceArms;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline locoma_contact_flag_t modeNumber2ActiveContacts(const size_t& modeNumber) {
  locoma_contact_flag_t stanceLegs;  // {LF, RF, LH, RH}

  arm_contact_flag_t contactArmFlags = modeNumber2StanceArm(modeNumber);
  contact_flag_t contactLegFlags = modeNumber2StanceLeg(modeNumber);
  stanceLegs = locoma_contact_flag_t{contactLegFlags[0], contactLegFlags[1], contactLegFlags[2], contactLegFlags[3], contactArmFlags[0], contactArmFlags[1]};
  return stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t stanceLeg2ModeNumber(const contact_flag_t& stanceLegs, const arm_contact_flag_t& stanceArms) {
  return static_cast<size_t>(stanceLegs[3]) + 2 * static_cast<size_t>(stanceLegs[2]) + 4 * static_cast<size_t>(stanceLegs[1]) +
         8 * static_cast<size_t>(stanceLegs[0]) + 16 * static_cast<size_t>(stanceArms[1]) + 32 * static_cast<size_t>(stanceArms[0]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::string modeNumber2String(const size_t& modeNumber) {

  // build the map from mode number to name
  std::map<size_t, std::string> modeToName;
  modeToName[FLY] = "FLY";
  modeToName[RH] = "RH";
  modeToName[LH] = "LH";
  modeToName[LH_RH] = "LH_RH";
  modeToName[RF] = "RF";
  modeToName[RF_RH] = "RF_RH";
  modeToName[RF_LH] = "RF_LH";
  modeToName[RF_LH_RH] = "RF_LH_RH";
  modeToName[LF] = "LF";
  modeToName[LF_RH] = "LF_RH";
  modeToName[LF_LH] = "LF_LH";
  modeToName[LF_LH_RH] = "LF_LH_RH";
  modeToName[LF_RF] = "LF_RF";
  modeToName[LF_RF_RH] = "LF_RF_RH";
  modeToName[LF_RF_LH] = "LF_RF_LH";
  modeToName[STANCE] = "STANCE";

  modeToName[RA] = "RA";
  modeToName[RA_RH] = "RA_RH";
  modeToName[RA_LH] = "RA_LH";
  modeToName[RA_LH_RH] = "RA_LH_RH";
  modeToName[RA_RF] = "RA_RF";
  modeToName[RA_RF_RH] = "RA_RF_RH";
  modeToName[RA_RF_LH] = "RA_RF_LH";
  modeToName[RA_RF_LH_RH] = "RA_RF_LH_RH";
  modeToName[RA_LF] = "RA_LF";
  modeToName[RA_LF_RH] = "RA_LF_RH";
  modeToName[RA_LF_LH] = "RA_LF_LH";
  modeToName[RA_LF_LH_RH] = "RA_LF_LH_RH";
  modeToName[RA_LF_RF] = "RA_LF_RF";
  modeToName[RA_LF_RF_RH] = "RA_LF_RF_RH";
  modeToName[RA_LF_RF_LH] = "RA_LF_RF_LH";
  modeToName[RA_STANCE] = "RA_STANCE";

  modeToName[LA] = "LA";
  modeToName[LA_RH] = "LA_RH";
  modeToName[LA_LH] = "LA_LH";
  modeToName[LA_LH_RH] = "LA_LH_RH";
  modeToName[LA_RF] = "LA_RF";
  modeToName[LA_RF_RH] = "LA_RF_RH";
  modeToName[LA_RF_LH] = "LA_RF_LH";
  modeToName[LA_RF_LH_RH] = "LA_RF_LH_RH";
  modeToName[LA_LF] = "LA_LF";
  modeToName[LA_LF_RH] = "LA_LF_RH";
  modeToName[LA_LF_LH] = "LA_LF_LH";
  modeToName[LA_LF_LH_RH] = "LA_LF_LH_RH";
  modeToName[LA_LF_RF] = "LA_LF_RF";
  modeToName[LA_LF_RF_RH] = "LA_LF_RF_RH";
  modeToName[LA_LF_RF_LH] = "LA_LF_RF_LH";
  modeToName[LA_STANCE] = "LA_STANCE";
  modeToName[LA_RA] = "LA_RA";
  modeToName[LA_RA_RH] = "LA_RA_RH";
  modeToName[LA_RA_LH] = "LA_RA_LH";
  modeToName[LA_RA_LH_RH] = "LA_RA_LH_RH";
  modeToName[LA_RA_RF] = "LA_RA_RF";
  modeToName[LA_RA_RF_RH] = "LA_RA_RF_RH";
  modeToName[LA_RA_RF_LH] = "LA_RA_RF_LH";
  modeToName[LA_RA_RF_LH_RH] = "LA_RA_RF_LH_RH";
  modeToName[LA_RA_LF] = "LA_RA_LF";
  modeToName[LA_RA_LF_RH] = "LA_RA_LF_RH";
  modeToName[LA_RA_LF_LH] = "LA_RA_LF_LH";
  modeToName[LA_RA_LF_LH_RH] = "LA_RA_LF_LH_RH";
  modeToName[LA_RA_LF_RF] = "LA_RA_LF_RF";
  modeToName[LA_RA_LF_RF_RH] = "LA_RA_LF_RF_RH";
  modeToName[LA_RA_LF_RF_LH] = "LA_RA_LF_RF_LH";
  modeToName[LA_RA_STANCE] = "LA_RA_STANCE";

  return modeToName[modeNumber];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t string2ModeNumber(const std::string& modeString) {

  // build the map from name to mode number
  std::map<std::string, size_t> nameToMode;
  nameToMode["FLY"] = FLY;
  nameToMode["RH"] = RH;
  nameToMode["LH"] = LH;
  nameToMode["LH_RH"] = LH_RH;
  nameToMode["RF"] = RF;
  nameToMode["RF_RH"] = RF_RH;
  nameToMode["RF_LH"] = RF_LH;
  nameToMode["RF_LH_RH"] = RF_LH_RH;
  nameToMode["LF"] = LF;
  nameToMode["LF_RH"] = LF_RH;
  nameToMode["LF_LH"] = LF_LH;
  nameToMode["LF_LH_RH"] = LF_LH_RH;
  nameToMode["LF_RF"] = LF_RF;
  nameToMode["LF_RF_RH"] = LF_RF_RH;
  nameToMode["LF_RF_LH"] = LF_RF_LH;
  nameToMode["STANCE"] = STANCE;

  nameToMode["RA"] = RA;
  nameToMode["RA_RH"] = RA_RH;
  nameToMode["RA_LH"] = RA_LH;
  nameToMode["RA_LH_RH"] = RA_LH_RH;
  nameToMode["RA_RF"] = RA_RF;
  nameToMode["RA_RF_RH"] = RA_RF_RH;
  nameToMode["RA_RF_LH"] = RA_RF_LH;
  nameToMode["RA_RF_LH_RH"] = RA_RF_LH_RH;
  nameToMode["RA_LF"] = RA_LF;
  nameToMode["RA_LF_RH"] = RA_LF_RH;
  nameToMode["RA_LF_LH"] = RA_LF_LH;
  nameToMode["RA_LF_LH_RH"] = RA_LF_LH_RH;
  nameToMode["RA_LF_RF"] = RA_LF_RF;
  nameToMode["RA_LF_RF_RH"] = RA_LF_RF_RH;
  nameToMode["RA_LF_RF_LH"] = RA_LF_RF_LH;
  nameToMode["RA_STANCE"] = RA_STANCE;

  nameToMode["LA"] = LA;
  nameToMode["LA_RH"] = LA_RH;
  nameToMode["LA_LH"] = LA_LH;
  nameToMode["LA_LH_RH"] = LA_LH_RH;
  nameToMode["LA_RF"] = LA_RF;
  nameToMode["LA_RF_RH"] = LA_RF_RH;
  nameToMode["LA_RF_LH"] = LA_RF_LH;
  nameToMode["LA_RF_LH_RH"] = LA_RF_LH_RH;
  nameToMode["LA_LF"] = LA_LF;
  nameToMode["LA_LF_RH"] = LA_LF_RH;
  nameToMode["LA_LF_LH"] = LA_LF_LH;
  nameToMode["LA_LF_LH_RH"] = LA_LF_LH_RH;
  nameToMode["LA_LF_RF"] = LA_LF_RF;
  nameToMode["LA_LF_RF_RH"] = LA_LF_RF_RH;
  nameToMode["LA_LF_RF_LH"] = LA_LF_RF_LH;
  nameToMode["LA_STANCE"] = LA_STANCE;
  nameToMode["LA_RA"] = LA_RA;
  nameToMode["LA_RA_RH"] = LA_RA_RH;
  nameToMode["LA_RA_LH"] = LA_RA_LH;
  nameToMode["LA_RA_LH_RH"] = LA_RA_LH_RH;
  nameToMode["LA_RA_RF"] = LA_RA_RF;
  nameToMode["LA_RA_RF_RH"] = LA_RA_RF_RH;
  nameToMode["LA_RA_RF_LH"] = LA_RA_RF_LH;
  nameToMode["LA_RA_RF_LH_RH"] = LA_RA_RF_LH_RH;
  nameToMode["LA_RA_LF"] = LA_RA_LF;
  nameToMode["LA_RA_LF_RH"] = LA_RA_LF_RH;
  nameToMode["LA_RA_LF_LH"] = LA_RA_LF_LH;
  nameToMode["LA_RA_LF_LH_RH"] = LA_RA_LF_LH_RH;
  nameToMode["LA_RA_LF_RF"] = LA_RA_LF_RF;
  nameToMode["LA_RA_LF_RF_RH"] = LA_RA_LF_RF_RH;
  nameToMode["LA_RA_LF_RF_LH"] = LA_RA_LF_RF_LH;
  nameToMode["LA_RA_STANCE"] = LA_RA_STANCE;

  return nameToMode[modeString];
}

}  // namespace legged_robot
}  // end of namespace ocs2
