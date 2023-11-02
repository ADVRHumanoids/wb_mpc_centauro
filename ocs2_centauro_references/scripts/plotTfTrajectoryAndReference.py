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

from BagFileInterface import getMessagesFromTopic
import matplotlib.pyplot as plt
from bisect import bisect
import matplotlib
import rosbag


def fromRosTimeToMpcTime(rosTime, mpcObservationMessages, mpcTopicRosTime):
    index = max(bisect(mpcTopicRosTime, rosTime) - 1, 0)
    return mpcObservationMessages[index].time


##############################################################################################
##############################################################################################
##############################################################################################
def printEeTrajectory(eeIndex=4, usefullSamples=None, showReference=False):
    if usefullSamples == None:
        usefullSamples = [0, -1]
    mpcBagFile = rosbag.Bag(mpcBagPath)

    # get all mpc observations
    mpcRosTime, mpcObsMessages = getMessagesFromTopic("/legged_robot_mpc_observation", mpcBagPath)

    # check start and end of recording mpc time
    startMpcRecordingTime = mpcObsMessages[0].time
    endMpcRecordingTime = mpcObsMessages[-1].time
    recordingDuration = endMpcRecordingTime - startMpcRecordingTime
    print("Recording started at mpc time: ", startMpcRecordingTime,
          " and finished at mpc time: ", endMpcRecordingTime)
    print("Recording duration: ", recordingDuration)

    # get EE trj from /currentState topic
    eePoseTrj = []
    time = []
    tfTime, tfList = getMessagesFromTopic("tf_odometry/world_dagana_2_tcp", tfFilePath)
    tfPosition = [k.transform.translation for k in tfList]
    # index of mpc time in tfTime
    mpcTimeOfTfs = []
    for i, iTfTime in enumerate(tfTime):
        mpcTimeOfTfs.append(fromRosTimeToMpcTime(iTfTime, mpcObsMessages, mpcRosTime))

    tfTime2, tfList2 = getMessagesFromTopic("tf_odometry/world_dagana_2_tcp", tfFilePath2)
    tfPosition2 = [k.transform.translation for k in tfList2]
    # index of mpc time in tfTime
    mpcTimeOfTfs2 = []
    for i, iTfTime in enumerate(tfTime2):
        mpcTimeOfTfs2.append(fromRosTimeToMpcTime(iTfTime, mpcObsMessages, mpcRosTime))
    # for topic, msg, t in tfBagFile.read_messages(topics=[self._eesTrajectoryTopic[ee]]):
    #     eePoseTrj.append(msg.markers[2 * eeIndex].pose)
    #
    #     if len(time) == 0:  # time
    #         initialTime = t.to_sec()
    #     # time.append(t.to_sec() - initialTime)
    #     time.append(t.to_sec())
    # eePoseTrjData.update({experimentName: eePoseTrj[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]})
    # timeData.update({experimentName: time[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]})

    # get tf fromself._bagFile tf topic
    # eeTfPosition = self.getTfFromBagFile(self._bagFile[fl_num], 'odometry/world', "dagana_2_tcp")

    if showReference:
        # construct reference as a sequence of step commands
        eeRefTrajectoryMsgList = []
        for topic, msg, t in mpcBagFile.read_messages(topics=[eesReferenceTopic]):
            eeRefTrajectoryMsgList.append(msg)
        refTimeTrajectory = [mpcObsMessages[0].time]  # append initial not published point
        refPoseTrajectory = [eeRefTrajectoryMsgList[0].stateTrajectory[0].value[:3]]
        for t in range(len(eeRefTrajectoryMsgList)):
            # # index of mpc time in rostime
            # index = max(
            #     bisect([k.time for k in mpcObsMessages], eeRefTrajectoryMsgList[t].timeTrajectory[0]) - 1, 0)
            # refTimeTrajectory.append(mpcObsMessages[t].time)
            refTimeTrajectory.append(eeRefTrajectoryMsgList[t].timeTrajectory[0])
            refPoseTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[0].value[:3])

            refTimeTrajectory.append(eeRefTrajectoryMsgList[t].timeTrajectory[-1])
            # index = max(
            #     bisect([k.time for k in mpcObsMessages], eeRefTrajectoryMsgList[t].timeTrajectory[-1]) - 1, 0)
            # refTimeTrajectory.append(mpcRosTime[index])
            refPoseTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[-1].value[:3])
        refTimeTrajectory.append(mpcObsMessages[-1].time)  # append final not published point
        refPoseTrajectory.append(eeRefTrajectoryMsgList[-1].stateTrajectory[-1].value[:3])
        print('refTimeTrajectory = ', refTimeTrajectory)

        # row and column labels
        # cols = [col for col in jointsToPlot]
        # rows = ['{}'.format(row) for row in ['position (rad)$']]
        fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 8))
        matplotlib.rc('xtick', labelsize=14)
        matplotlib.rc('ytick', labelsize=14)
        coordinates = ['x', 'y', 'z']

        # loop over coordinates
        for coord, coord_str in enumerate(coordinates):
            axes[coord].plot(mpcTimeOfTfs,
                             [getattr(kk, coordinates[coord]) for kk in tfPosition],
                             linewidth=3, label='tf ')  # ee trj
            axes[coord].plot(mpcTimeOfTfs2,
                             [getattr(kk, coordinates[coord]) for kk in tfPosition2],
                             linewidth=3, label='tf 2 ')  # ee trj
            if showReference:
                axes[coord].plot(refTimeTrajectory,
                                 [kk[coord] for kk in refPoseTrajectory],
                                 linewidth=2, label='reference ')  # ee ref
            axes[coord].grid()
            axes[coord].legend()
            axes[coord].set_ylabel(coord_str, rotation=90, fontsize=20)
            # labels
            axes[-1].set(xlabel='Time (sec)')
            # axes[coord].set_title('MPC State Observation', size='large', y=1)
            plt.suptitle('EE trajectory tracking ', y=0.99, fontsize=20)  # title
            # fig.tight_layout()
            fig.subplots_adjust(top=0.9, left=0.1, right=0.98)

        fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 8))
        matplotlib.rc('xtick', labelsize=14)
        matplotlib.rc('ytick', labelsize=14)
        coordinates = ['x', 'y', 'z']

        # loop over coordinates
        for coord, coord_str in enumerate(coordinates):
            axes[coord].plot(tfTime,
                             [getattr(kk, coordinates[coord]) for kk in tfPosition],
                             linewidth=3, label='tf ')  # ee trj
            axes[coord].plot(tfTime2,
                             [getattr(kk, coordinates[coord]) for kk in tfPosition2],
                             linewidth=3, label='tf 2 ')  # ee trj
            # if showReference:
            #     axes[coord].plot(refTimeTrajectory,
            #                      [kk[coord] for kk in refPoseTrajectory],
            #                      linewidth=2, label='reference ')  # ee ref
            axes[coord].grid()
            axes[coord].legend()
            axes[coord].set_ylabel(coord_str, rotation=90, fontsize=20)
            # labels
            axes[-1].set(xlabel='Time (sec)')
            # axes[coord].set_title('MPC State Observation', size='large', y=1)
            plt.suptitle('EE trajectory tracking ', y=0.99, fontsize=20)  # title
            # fig.tight_layout()
            fig.subplots_adjust(top=0.9, left=0.1, right=0.98)

    plt.show()


if __name__ == "__main__":
    tfFilePath = "/home/idadiotis/Desktop/ocs2_experiments/15July2023/tf_2023-07-17-16-31-16.bag"
    tfFilePath2 = "/home/idadiotis/Desktop/ocs2_experiments/15July2023/tf_2023-07-17-17-24-46.bag"
    # time, tfList = getMessagesFromTopic("tf_odometry/world_dagana_2_tcp", tfFilePath)
    # time2, tfList2 = getMessagesFromTopic("tf_odometry/world_dagana_2_tcp", tfFilePath2)
    eesReferenceTopic = "/legged_robot_mpc_dagana_2_tcptarget"
    showReference = True
    mpcBagPath = "/home/idadiotis/Desktop/ocs2_experiments/15July2023/mpc_2023-07-15-12-09-24.bag"

    printEeTrajectory(5, showReference=showReference)