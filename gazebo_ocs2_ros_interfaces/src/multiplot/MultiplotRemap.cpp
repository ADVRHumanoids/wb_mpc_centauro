/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <string>

#include <ros/init.h>
#include <ros/ros.h>

#include <ocs2_core/Types.h>

#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mpc_performance_indices.h>

namespace {

class MultiplotRemap {
 public:
  /** Constructor */
  explicit MultiplotRemap(const std::string& mpcPolicyTopicName, ::ros::NodeHandle& nodeHandle) {
    mpcPolicySubscriber_ =
        nodeHandle.subscribe(mpcPolicyTopicName, 1, &MultiplotRemap::mpcPoicyCallback, this, ::ros::TransportHints().tcpNoDelay());
    mpcPerformanceIndicesPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_performance_indices>("mpc_performance_indices", 1);
  }

  /** Default deconstructor */
  ~MultiplotRemap() = default;

 private:
  void mpcPoicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& policyMsg) {
    mpcPerformanceIndicesPublisher_.publish(policyMsg->performanceIndices);
  }

  // publishers and subscribers
  ::ros::Subscriber mpcPolicySubscriber_;
  ::ros::Publisher mpcPerformanceIndicesPublisher_;
};

}  // unnamed namespace

int main(int argc, char** argv) {
  // mpc policy topic name
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("MPC policy topic name is not specified!");
  }
  const std::string mpcPolicyTopicName = std::string(programArgs[1]);

  ::ros::init(argc, argv, "multiplot_remap");
  ::ros::NodeHandle nodeHandle;
  MultiplotRemap multiplotRemap(mpcPolicyTopicName, nodeHandle);

  ::ros::spin();

  return 0;
}
