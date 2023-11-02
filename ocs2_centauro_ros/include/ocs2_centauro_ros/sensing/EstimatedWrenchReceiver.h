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

#include <mutex>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <base_estimation/ContactsWrench.h>
#include <eigen_conversions/eigen_msg.h>
#include <ocs2_centauro/sensing/SensingDefinitions.h>
#include <ocs2_centauro/sensing/ForceTorqueSensing.h>

namespace ocs2 {
namespace legged_robot {

class EstimatedWrenchReceiver : public SolverSynchronizedModule {
 public:
  EstimatedWrenchReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<ForceTorqueSensing> forceTorqueSensingPtr);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override {};

 private:
  void eeEstimatedWrenchCallback(const base_estimation::ContactsWrenchConstPtr& msg);
  ros::Subscriber eeEstimatedWrenchSubscriber_;

  std::mutex receivedWrenchesMutex_;
  std::atomic_bool wrenchesUpdated_;
  EeEstimatedWrenches receivedWrenches_;
  std::shared_ptr<ForceTorqueSensing> forceTorqueSensingPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline EeEstimatedWrenches readEstimatedWrenchMsg(const base_estimation::ContactsWrench& msg) {
    wrenches_matrix eeEstimatedWrenches(6, msg.contacts_wrench.size());
    vector_t forceNorms(msg.contacts_wrench.size());
    for (int i = 0; i < msg.contacts_wrench.size(); i++) {
        Eigen::Matrix<scalar_t, 6, 1> eeWrench;
        tf::wrenchMsgToEigen(msg.contacts_wrench[i].wrench, eeWrench);
        eeEstimatedWrenches.col(i) = std::move(eeWrench);
        forceNorms[i] = msg.force_norm[i];
    }
    return {eeEstimatedWrenches, forceNorms};
}

}  // namespace legged_robot
}  // namespace ocs2
