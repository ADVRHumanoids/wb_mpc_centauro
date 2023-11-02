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

# Better execute the commands below for using the openrobots version of pinocchio
'''
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
'''

import pinocchio
import rospkg
from BagFileInterface import getMessagesFromTopic, getBaseEstimationPoseFromTopic, getJointStateAttributesFromTopic, getBaseEstimationTwistFromTopic
import rosbag
from bisect import bisect
import numpy as np
import rospy

import matplotlib.pyplot as plt
import matplotlib

# ee names
eeNames = ['contact_1', 'contact_2', 'contact_3', 'contact_4', 'arm1_8', 'dagana_2_tcp']
legNumber = 4
armNumber = 2
referenceFrame = "world"


class PinocchioModelInterface(object):
    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def __init__(self, urdfPath, isFreeFlyer, dataFilesFolder, fileName, multipleBagFiles=True):
        # create pinocchio models
        rospack = rospkg.RosPack()
        rospack.list()
        if isFreeFlyer:
            self._model = pinocchio.buildModelFromUrdf(urdfPath, pinocchio.JointModelFreeFlyer())
        else:
            self._model = pinocchio.buildModelFromUrdf(urdfPath)

        # access data from rosbag
        if multipleBagFiles:
            self._filePath = {'xbotcore': dataFilesFolder + "xbotcore" + fileName,
                              'rviz': dataFilesFolder + "rviz" + fileName,
                              'mpc': dataFilesFolder + "mpc" + fileName,
                              'base_est': dataFilesFolder + "base_est" + fileName}
            self._jointStatesTime = getJointStateAttributesFromTopic(
                "/xbotcore/joint_states", dataFilesFolder + "xbotcore" + fileName)['time']
            self._jointStatesTimeRaw = getJointStateAttributesFromTopic(
                "/xbotcore/joint_states", dataFilesFolder + "xbotcore" + fileName)['time_raw']
            # self._imuTime, self._imuMessages = getMessagesFromTopic(
            #     "/xbotcore/imu/imu_link", dataFilesFolder + "xbotcore" + fileName)
            self._baseEstimationPoseTime = getBaseEstimationPoseFromTopic(
                "/centauro_base_estimation/base_link/pose", dataFilesFolder + "base_est" + fileName)['time']
            self._baseEstimationPoseTimeRaw = getBaseEstimationPoseFromTopic(
                "/centauro_base_estimation/base_link/pose", dataFilesFolder + "base_est" + fileName)['time_raw']
            # self._baseEstimationTwist = getBaseEstimationTwistFromTopic(
            #     "/centauro_base_estimation/base_link/twist", dataFilesFolder + "base_est" + fileName)
            # self._contactsStatusMessages = getMessagesFromTopic(
            #     "/centauro_base_estimation/contacts/status", dataFilesFolder + "base_est" + fileName)
        else:
            self._filePath = dataFilesFolder + fileName
            self._jointStatesTime = getJointStateAttributesFromTopic(
                "/xbotcore/joint_states", self._filePath)['time']
            self._jointStatesTimeRaw = getJointStateAttributesFromTopic(
                "/xbotcore/joint_states", self._filePath)['time_raw']
            # self._imuTime, self._imuMessages = getMessagesFromTopic(
            #     "/xbotcore/imu/imu_link", self._filePath)
            self._baseEstimationPoseTime = getBaseEstimationPoseFromTopic(
                "/centauro_base_estimation/base_link/pose", self._filePath)['time']
            self._baseEstimationPoseTimeRaw = getBaseEstimationPoseFromTopic(
                "/centauro_base_estimation/base_link/pose", dataFilesFolder + "base_est" + fileName)['time_raw']
            # self._baseEstimationTwist = getBaseEstimationTwistFromTopic(
            #     "/centauro_base_estimation/base_link/twist", self._filePath)
            # self._contactsStatusTime, self._contactsStatusMessages = getMessagesFromTopic(
            #     "/centauro_base_estimation/contacts/status", self._filePath)

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def getJointStatesTime(self):
        return self._jointStatesTime['time']

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def getBasePoseTime(self):
        return self._baseEstimationPoseTime['time']

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def getUpdatedModelData(self, iBaseEstPose, generalizedVelocity=True, effort=False, frameName=None):
        # TODO: effort update
        # timeInstant = self._jointStatesTime[jointStateIndex]
        # timeInstantRaw = self._jointStatesTimeRaw[jointStateIndex]
        # # since base estimation is published at different freq check timestamps to determine the message at that time intant
        # iBaseEstPose = max(bisect(self._baseEstimationPoseTime, timeInstant) - 1, 0)

        timeInstant = self._baseEstimationPoseTime[iBaseEstPose]
        timeInstantRaw = self._baseEstimationPoseTimeRaw[iBaseEstPose]
        # since base estimation is published at different freq check timestamps to determine the message at that time intant
        jointStatesIndex = max(bisect(self._jointStatesTimeRaw, timeInstantRaw) - 1, 0)

        # configuration and velocity vectors
        jointStates = getJointStateAttributesFromTopic(
            "/xbotcore/joint_states", self._filePath['xbotcore'], requestedTime=self._jointStatesTimeRaw[jointStatesIndex])
        baseEstimationPose = getBaseEstimationPoseFromTopic(
            "/centauro_base_estimation/base_link/pose", self._filePath['base_est'],
            requestedTime=timeInstantRaw)
        baseEstimationTwist = getBaseEstimationTwistFromTopic(
            "/centauro_base_estimation/base_link/twist", self._filePath['base_est'],
            requestedTime=timeInstantRaw)

        data = self._model.createData()
        qInstant = baseEstimationPose['position'][0] + baseEstimationPose['quaternion'][0] + list(
            jointStates['motor_position'][0][:-3])
        if generalizedVelocity:
            vInstant = baseEstimationTwist['linear'][0] + baseEstimationTwist['angular'][0] + list(
                jointStates['motor_velocity'][0][:-3])
            pinocchio.forwardKinematics(self._model, data, np.array(qInstant), np.array(vInstant))
        else:
            pinocchio.forwardKinematics(self._model, data, np.array(qInstant))
        if frameName == None:
            pinocchio.updateFramePlacements(self._model, data)
        else:
            pinocchio.updateFramePlacement(self._model, data, self._model.getFrameId(frameName))
        return data

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def getFramePose(self, frameName):
        framePoseTrajectory = []
        for iTime in range(len(self._baseEstimationPoseTime)):
            print(iTime)
            iData = self.getUpdatedModelData(iTime, generalizedVelocity=True, effort=False, frameName=frameName)
            framePoseTrajectory.append(iData.oMf[self._model.getFrameId(frameName)].translation)
        print('got pose trj')
        return framePoseTrajectory

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def getFrameVelocity(self, frameName):
        frameVelocityTrajectory = []
        for iTime in range(len(self._jointStates['time'])):
            iData = self.getUpdatedModelData(iTime, frameName)
            frameVelocityTrajectory.append(pinocchio.getFrameVelocity(
                self._model, iData, self._model.getFrameId(frameName), pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED))
        print('got vel trj')
        return frameVelocityTrajectory


def printFramePoseTrajectory(framePoseTrajectory, timeTrajectory, frameName):

    # PRINT
    matplotlib.rc('xtick', labelsize=16)
    matplotlib.rc('ytick', labelsize=16)
    colors = ['b', 'r']
    coordinates = ['X', 'Y', 'Z']

    # translation
    fig, axes = plt.subplots(nrows=len(coordinates), ncols=1, figsize=(8, 8))
    for c, coord in enumerate(coordinates):
        coordinateValue = framePoseTrajectory
        axes[c].plot(timeTrajectory, coordinateValue, linewidth=2)
        axes[c].grid()
        axes[c].legend()
        axes[c].set_ylabel(coord + ' $[m]$', fontsize=25)
    axes[-1].set_xlabel('Time $[s]$', fontsize=25)
    plt.suptitle(frameName + ' Position', y=0.99, fontsize=20)
    plt.show()


def printMultipleFramePoseTrajectories(framePoseTrajectories):
    trajectoriesNum = len(framePoseTrajectories)
    if trajectoriesNum < 2:
        raise Exception("[printFramePose] Error. Size of framePoseTrajectory < 2.")


if __name__ == "__main__":
    # define paths for bag files
    # withCost = "_2023-07-12-11-51-11.bag"
    # noCost = "_2023-07-12-11-49-12.bag"
    # folder = "/home/idadiotis/Documents/wb_mpc_paper/data_only/feet_ee_cart_vel_cost/"
    # filenames = [noCost, withCost]
    # scenarioNames = ["no cost", 'with cost']
    # scenarioLabelNames = [r'$\mathbf{R}_{ts} = \mathbf{0}$', r'$\mathbf{R}_{ts} \neq \mathbf{0}$']

    # july15 = "_2023-07-15-12-09-24_cutted.bag"
    july15 = "_2023-07-15-12-09-24_small.bag"
    folder = "/home/idadiotis/Desktop/ocs2_experiments/15July2023/"
    filenames = [july15]
    scenarioNames = ["15 July"]
    scenarioLabelNames = [r'$\mathbf{R}_{ts} = \mathbf{0}$', r'$\mathbf{R}_{ts} \neq \mathbf{0}$']


    # create pinocchio model interface
    rospack = rospkg.RosPack()
    rospack.list()
    #urdf_path = rospack.get_path('centauro_urdf') + '/urdf/centauro_dagana.urdf'
    urdf_path = rospack.get_path('ocs2_robotic_assets') + '/resources/centauro/urdf/centauro_dagana.urdf'
    modelInterface = PinocchioModelInterface(urdf_path, isFreeFlyer=True, dataFilesFolder=folder, fileName=july15, multipleBagFiles=True)
    dagana_2_tcpPoseTrj = modelInterface.getFramePose("dagana_2_tcp")
    print("got pose trj")
    # for iTime in range(modelInterface._jointStatesTime):
    #     iData = modelInterface.getUpdatedModelData(iTime)
    #
    # printFramePoseTrajectory(daganaPose, modelInterface._jointStatesTime, "dagana_2_tcp")
    # dagana_2_tcpPoseTrj = modelInterface.getFramePose("dagana_2_tcp")
    printFramePoseTrajectory(dagana_2_tcpPoseTrj, modelInterface.getBasePoseTime(), "dagana_2_tcp")
