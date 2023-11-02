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
#include <gazebo_ocs2_ros_interfaces/xbot2/AdmittanceReferenceGenerator.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace force_control {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AdmittanceReferenceGenerator::AdmittanceReferenceGenerator(ros::NodeHandle& nodeHandle, PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                             Config config)
    : ForceControl(nodeHandle, pinocchioInterface, centroidalModelInfo, config),
      wrenchIntegralError_(vector_t::Zero(6)), pGains_(vector_t::Zero(6)), iGains_(vector_t::Zero(6)),
      wrenchIntegralMax_(vector_t::Zero(6))
{
    std::string taskFile;
    nodeHandle.getParam("/taskFile", taskFile);
    loadData::loadEigenMatrix(taskFile, "force_control.admittance.pGains", pGains_);
    loadData::loadEigenMatrix(taskFile, "force_control.admittance.iGains", iGains_);
    loadData::loadEigenMatrix(taskFile, "force_control.admittance.wrenchIntegralMax", wrenchIntegralMax_);

    // print configuration
    ROS_INFO("Successfully constructed Admittance Reference Generator.");
    std::cerr << "\n #### Admittance Reference Generator: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### 'pGains'............................................................." << pGains_.transpose() << "\n";
    std::cerr << " #### 'iGains'............................................................" << iGains_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AdmittanceReferenceGenerator::AdmittanceReferenceGenerator(const AdmittanceReferenceGenerator& rhs)
    : ForceControl(rhs.getRosNodeHandle(), rhs.getPinocchioInterface(), rhs.getCentroidalModelInfo(), rhs.getConfig()),
      wrenchIntegralError_(rhs.wrenchIntegralError_), pGains_(rhs.pGains_), iGains_(rhs.iGains_),
      wrenchIntegralMax_(rhs.wrenchIntegralMax_) {
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AdmittanceReferenceGenerator::updateWrenchIntegralError() {
    wrenchIntegralError_ += getWrenchError() * (1.0 / getConfig().controlFrequency);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AdmittanceReferenceGenerator::antiWindupIntegralError() {
    wrenchIntegralError_ = wrenchIntegralError_.cwiseMin(wrenchIntegralMax_).cwiseMax(-wrenchIntegralMax_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t AdmittanceReferenceGenerator::getPoseCorrection() {
    updateWrenchIntegralError();
    antiWindupIntegralError();

    vector_t poseCorrection = pGains_.cwiseProduct(getWrenchError()) + iGains_.cwiseProduct(wrenchIntegralError_);

    // TODO: handle quaternion or orientation
//    Eigen::Quaterniond deltaRot;
//    double r = poseCorrection.bottomRows(3).norm();
//    if (r > 0) {
//      deltaRot.w() = std::cos(r);
//      Eigen::Vector3d quatImg = std::sin(r) / r * poseCorrection.bottomRows(3);
//      deltaRot.x() = quatImg[0];
//      deltaRot.y() = quatImg[1];
//      deltaRot.z() = quatImg[2];
//    }

    return poseCorrection;
}

}
}
