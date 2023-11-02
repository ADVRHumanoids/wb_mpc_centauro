###############################################################################
# Copyright (c) 2023, Ioannis Dadiotis <ioannis.dadiotis@iit.it>. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
#  * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###############################################################################

from BagFileInterface import BagFileInterface
import matplotlib.pyplot as plt


if __name__ == "__main__":

    # trot comparison wrt cost in simulation
    # pathNoCost = '/home/idadiotis/Documents/wb_mpc_paper/data_only/rviz_2023-07-12-11-49-12.bag'
    # pathCost = '/home/idadiotis/Documents/wb_mpc_paper/data_only/rviz_2023-07-12-11-51-11.bag'
    # pathNoCost = '/home/idadiotis/Documents/wb_mpc_paper/data_only/xbotcore_2023-07-12-11-49-12.bag'
    # pathCost = '/home/idadiotis/Documents/wb_mpc_paper/data_only/xbotcore_2023-07-12-11-51-11.bag'
    # pathNoCost = '/home/idadiotis/Documents/wb_mpc_paper/data_only/base_est_2023-07-12-11-49-12.bag'
    # pathCost = '/home/idadiotis/Documents/wb_mpc_paper/data_only/base_est_2023-07-12-11-51-11.bag'

    # hw experiments 27 March & 18 June
    # trotExperimentPath = '/home/idadiotis/Documents/wb_mpc_paper/videos/hw_experiments/27MarchTrot/_2023-03-27-19-01-05_cutted.bag'      # trot
    # trotLowerBodyExperimentPath = '/home/idadiotis/Documents/wb_mpc_paper/videos/hw_experiments/27MarchTrot/_2023-03-27-19-11-20_cutted.bag'      # trot
    # freeMotionExperimentPath = '/home/idadiotis/Desktop/experiments/18June23/all_2023-06-18-17-05-24.bag'      # 18 June free motion

    # hw experiments 23 June
    # trotExperimentPath = '/home/idadiotis/Documents/wb_mpc_paper/videos/hw_experiments/23June/all_2023-06-23-19-00-54_cutted.bag'      # trot
    # trotUsefullSamples = [[0, -500]]
    # crawlExperimentPath = '/home/idadiotis/Documents/wb_mpc_paper/videos/hw_experiments/23June/all_2023-06-23-20-25-54_cutted.bag'     # crawl
    # crawlUsefullSamples = [[0, -300]]

    # freeMotionExperiment = '/home/idadiotis/Documents/wb_mpc_paper/videos/hw_experiments/23June/all_2023-06-23-18-19-17.bag'      # free motion 1
    # freeMotionUsefullSamples = [[0, -1]]
    # freeMotionExperiment = '/home/idadiotis/Documents/wb_mpc_paper/videos/hw_experiments/23June/all_2023-06-23-18-24-27_cutted.bag'     # free motion 2
    # freeMotionUsefullSamples = [[0, -1]]

    # object retrieval 10 july - equality
    # folder = "/home/idadiotis/Desktop/ocs2_experiments/10July2023/"
    # file = folder + "mpc_2023-07-10-14-51-50.bag"
    # file = folder + "rviz_2023-07-10-14-51-50.bag"
    # file = folder + "base_est_2023-07-10-14-51-50.bag"
    # file = folder + "xbotcore_2023-07-10-14-51-50.bag"

    # object retrieval 15 july
    # folder = "/home/idadiotis/Desktop/ocs2_experiments/15July2023/"
    # file15July_mpc = folder + "mpc_2023-07-15-12-09-24.bag"
    # file15July_rviz = folder + "rviz_2023-07-15-12-09-24_cutted.bag"
    # # file = folder + "base_est_2023-07-15-12-09-24.bag"
    # file15July_xbot = folder + "xbotcore_2023-07-15-12-09-24.bag"

    # simulation locomanipulation circle trajectory
    locoma_sim_bag = '/home/idadiotis/Documents/wb_mpc_paper/data_only/locoma_circle/cd_locomaCircle.bag'

    bagFileInterface = BagFileInterface([locoma_sim_bag],
                                        ['locoma'],
                                        jointStatesTopic='/xbotcore/joint_states',
                                        jointCommandTopic='/xbotcore/command',
                                        torqueTermsTopic='/xbotcore/command/torque_terms',
                                        mpcObservationTopic="/legged_robot_mpc_observation",
                                        eesTrajectoryTopic=["/legged_robot/currentState"],
                                        eesReferenceTopic=["/legged_robot_mpc_dagana_2_tcptarget"],
                                        printLegs=True, printArms=False, usefullSamples=[[0, -1], [0, -1]],
                                        mpcFile=[locoma_sim_bag])

    #bagFileInterface.printTorqueTracking(arms=True, usefullSamples=crawlUsefullSamples)
    # bagFileInterface.printTorques(usefullSamples=None, arms=False)
    # bagFileInterface.printTorqueTerms(usefullSamples=None, arms=True)
    # bagFileInterface.printVelocities(arms=False, usefullSamples=crawlUsefullSamples)
    # bagFileInterface.printPositions(arms=False)
    # bagFileInterface.printForcesFromBaseEstimation(usefullSamples=[[0, -1], [0, -1]])
    # bagFileInterface.printMpcState(usefullSamples=[[0, -1], [0, -1]])
    bagFileInterface.CompareTfAndCurrentStateTopics(eeIndex=5, usefullSamples=[[0, -1]], showReference=True)
    bagFileInterface.printEeTrajectory(eeIndex=5, usefullSamples=[[0, -1]], showReference=True)
    # bagFileInterface.printEeVelocity(eeIndex=3, usefullSamples=[[95, -1], [120, 250]])

    # bag.close()




