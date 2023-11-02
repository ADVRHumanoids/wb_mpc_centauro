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

#ifndef FORCECONTROL_H
#define FORCECONTROL_H

#include <ros/ros.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <base_estimation/ContactsWrench.h>

namespace ocs2 {
namespace force_control {

class ForceControl
{
public:
    struct Config {
        Config(size_array_t eeIndicesParam = {4, 5}, size_array2_t eeControlledComponentsParam = {{1, 2, 3}, {1, 2, 3}},
               bool controlForceNormFlag = true, std::vector<std::string> eeFrameNamesParam = {"arm1_8", "arm2_8"},
               vector_t eeRefWrenchesParam = vector_t::Zero(12), scalar_t controlFrequencyParam = 500.0)
            : eeIndices(eeIndicesParam), controlForceNorm(controlForceNormFlag),
              eeControlledComponents(eeControlledComponentsParam), eeFrameNames(eeFrameNamesParam),
              eeRefWrenches(eeRefWrenchesParam), controlFrequency(controlFrequencyParam) {}
        size_array_t eeIndices;     // end effector indices wrt all contacts at ModelSettings in ocs2_centauro
        size_array2_t eeControlledComponents;       // components to be controlled
        bool controlForceNorm;                      // control norm of forces
        std::vector<std::string> eeFrameNames;
        vector_t eeRefWrenches;
        scalar_t controlFrequency;
    };

    explicit ForceControl(const ros::NodeHandle& nodeHandle, const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& centroidalModelInfo,
                          Config config = Config());
    virtual ~ForceControl() = default;
    virtual ForceControl* clone() const = 0;

    Config getConfig() const {return config_;}
    vector_t getWrenchError() const;
    vector_t getEstimatedWrenches() const {return eeEstimatedWrenches_;}
    vector_t mapWrenchErrorToArmJointTorques(const vector_t& state) const;

protected:
    ForceControl(const ForceControl& rhs) = default;
    const ros::NodeHandle getRosNodeHandle() const {return nodeHandle_;}
    const PinocchioInterface getPinocchioInterface() const {return *pinocchioInterfacePtr_;}
    const CentroidalModelInfo getCentroidalModelInfo() const {return centroidalModelInfo_;}

private:
    void onEeWrenchReceived(const base_estimation::ContactsWrenchConstPtr& msg);

    Config config_;
    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
    CentroidalModelInfo centroidalModelInfo_;

    vector_t eeEstimatedWrenches_;  // concatenated wrenches, size nee * 6 x 1

    ros::Subscriber eeWrenchSubscriber_;
    ros::NodeHandle nodeHandle_;
};
}
}
#endif // FORCECONTROL_H
