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

import numpy as np
import rosbag
import matplotlib.pyplot as plt
import matplotlib
from tf_bag import BagTfTransformer
from tf.transformations import euler_from_quaternion
from bisect import bisect
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R
import pickle
import math


def getMessagesFromTopic(topic, filePath, requestedTime=None):
    messageList = []
    time = []
    if requestedTime is None:
        for topic, msg, t in rosbag.Bag(filePath).read_messages(topics=[topic]):
            messageList.append(msg)
            time.append(t.to_sec())
    else:
        for topic, msg, t in rosbag.Bag(filePath).read_messages(topics=[topic], start_time=requestedTime, end_time=requestedTime):
            messageList.append(msg)
            time.append(t.to_sec())
    return time, messageList


def getJointStateAttributesFromTopic(topic, filePath, requestedTime=None):
    time, time_raw, q_ref, dq_ref, effort_ref, q_motor, dq_motor, q_link, dq_link, effort, name = ([] for i in range(11))
    if requestedTime is None:
        for topic, msg, t in rosbag.Bag(filePath).read_messages(topics=[topic]):
            q_ref.append(msg.position_reference)
            dq_ref.append(msg.velocity_reference)
            effort_ref.append(msg.effort_reference)
            q_motor.append(msg.motor_position)
            dq_motor.append(msg.motor_velocity)
            q_link.append(msg.link_position)
            dq_link.append(msg.link_velocity)
            effort.append(msg.effort)
            time.append(t.to_sec())
            time_raw.append(t)
            name.append(msg.name)
    else:
        for topic, msg, t in rosbag.Bag(filePath).read_messages(topics=[topic], start_time=requestedTime, end_time=requestedTime):
            q_ref.append(msg.position_reference)
            dq_ref.append(msg.velocity_reference)
            effort_ref.append(msg.effort_reference)
            q_motor.append(msg.motor_position)
            dq_motor.append(msg.motor_velocity)
            q_link.append(msg.link_position)
            dq_link.append(msg.link_velocity)
            effort.append(msg.effort)
            time.append(t.to_sec())
            time_raw.append(t)
            name.append(msg.name)

    jointStates = {
        'position_reference': q_ref,
        'velocity_reference': dq_ref,
        'effort_reference': effort_ref,
        'motor_position': q_motor,
        'motor_velocity': dq_motor,
        'link_position': q_link,
        'link_velocity': dq_link,
        'effort': effort,
        'time': time,
        'time_raw': time_raw,
        'name': name
    }
    return jointStates


def getBaseEstimationPoseFromTopic(topic, filePath, requestedTime=None):
    time, time_raw, baseEstimPos, baseEstimRot, baseEstimEuler = ([] for i in range(5))
    if requestedTime is None:
        for topic, msg, t in rosbag.Bag(filePath).read_messages(topics=[topic]):
            baseEstimPos.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            baseEstimRot.append([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
            baseEstimEuler.append(list(euler_from_quaternion([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w], 'sxyz')))
            time.append(t.to_sec())
            time_raw.append(t)
    else:
        for topic, msg, t in rosbag.Bag(filePath).read_messages(topics=[topic], start_time=requestedTime, end_time=requestedTime):
            baseEstimPos.append([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            baseEstimRot.append([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,
                                 msg.transform.rotation.w])
            baseEstimEuler.append(list(euler_from_quaternion(
                [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,
                 msg.transform.rotation.w], 'sxyz')))
            time.append(t.to_sec())
            time_raw.append(t)

    baseEstimationPose = {
        'position': baseEstimPos,
        'quaternion': baseEstimRot,
        'euler': baseEstimEuler,
        'time': time,
        'time_raw': time_raw
    }
    return baseEstimationPose


def getBaseEstimationTwistFromTopic(topic, filePath, requestedTime=None):
    time = []
    baseEstimLinearTwist, baseEstimAngularTwist = ([] for i in range(2))
    if requestedTime is None:
        for topic, msg, t in rosbag.Bag(filePath).read_messages(topics=[topic]):
            baseEstimLinearTwist.append([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            baseEstimAngularTwist.append([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
            time.append(t.to_sec())
    else:
        for topic, msg, t in rosbag.Bag(filePath).read_messages(topics=[topic]):
            baseEstimLinearTwist.append([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            baseEstimAngularTwist.append([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
            time.append(t.to_sec())

    baseEstimationTwist = {
        'linear': baseEstimLinearTwist,
        'angular': baseEstimAngularTwist,
        'time': time
    }
    return baseEstimationTwist


def smooth_roll(previous_roll, current_roll, threshold=math.pi):
    # Calculate the absolute difference between the previous and current roll angles
    roll_difference = abs(current_roll - previous_roll)

    # Check if the roll difference exceeds the threshold
    if roll_difference >= threshold:
        # Adjust the current roll angle to ensure a smooth transition
        if current_roll > previous_roll:
            current_roll -= 2 * math.pi
        else:
            current_roll += 2 * math.pi

    return current_roll


class BagFileInterface(object):
    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def __init__(self, pathFile, experimentName, jointStatesTopic, jointCommandTopic, torqueTermsTopic,
                 mpcObservationTopic, eesTrajectoryTopic, eesReferenceTopic, printLegs, printArms, usefullSamples,
                 mpcFile=None):

        self._pathFile = pathFile
        self._bagFile = [rosbag.Bag(i) for i in pathFile]
        self._experimentName = experimentName
        self._startTime = [i.get_start_time() for i in self._bagFile]
        self._endTime = [i.get_end_time() for i in self._bagFile]
        self._messageNum = [i.get_message_count() for i in self._bagFile]
        self._usefullSamples = [i for i in usefullSamples]
        self._eesTrajectoryTopic = [i for i in eesTrajectoryTopic]
        self._eesReferenceTopic = [i for i in eesReferenceTopic]

        self._jointStatesTopic = jointStatesTopic
        self._jointCommandTopic = jointCommandTopic
        self._torqueTermsTopic = torqueTermsTopic
        self._jointStatesTopicFreq = 1000
        self._baseEstimationTopicFreq = 500
        print('ATTENTION" default frequency of joint states and base estimation topics is considered --> 1000 and 500 Hz')

        if mpcFile == None:
            print('No specific mpc file added! It is used in printEeTrajectory.')
        else:
            self._mpcBagFile = [rosbag.Bag(i) for i in mpcFile]
            self._mpcBagPath = mpcFile

        self._mpcObservationTopic = mpcObservationTopic
        
        self._printArms = printArms
        self._printLegs = printLegs
        self._legJointsPrefix = ['hip_yaw_', 'hip_pitch_', 'knee_pitch_', 'ankle_pitch_']
        self._legJointsStrings = ['hip yaw', 'hip pitch', 'knee pitch', 'ankle pitch']
        self._armJointsPostfix = ['_1', '_2', '_3', '_4', '_5', '_6']
        self._legInitials = ['FL', 'FR', 'HL', 'HR']
        self._armInitials = ['L', 'R']

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def printTorques(self, usefullSamples=None, legs=True, arms=True):
        if usefullSamples == None:
            usefullSamples = self._usefullSamples

        data = {}
        timeData = {}
        for file_num, fl in enumerate(self._bagFile):     # loop over all bag files
            effortReference = []
            effort = []
            time = []
            for topic, msg, t in fl.read_messages(topics=[self._jointStatesTopic]):
                effortReference.append(msg.effort_reference)
                effort.append(msg.effort)
                if len(effort) == 1:
                    names = msg.name

                if len(time) == 0:
                    initialTime = t.to_sec()
                time.append(t.to_sec() - initialTime)

            # wrap data in the order to be plotted
            timeData.update({self._experimentName[file_num]: time})
            data.update({self._experimentName[file_num]: [effortReference, effort]})
        dataNames = ['$τ_{ref, ff}$', '$τ$']                                            # labels for different quantities
        # torqueLabels = ['ref', 'real']
        xlabel = 'Time ($sec$)'

        # linestyle and colors
        linestyles = ['-', '--', '-.']
        colors = ['tab:red', 'tab:blue', 'tab:gray', 'tab:green']
        linewidths = [2.5, 1.5]
        alphas = [0.5, 1.0]

        # plots
        if legs:
            for leg_number in range(1, 5):                                                  # loop over 4 legs
                jointsToPlot = [joint_name + str(leg_number) for joint_name in
                                self._legJointsPrefix]  # joints to plot
                jointsIndicesToPlot = [names.index(i) for i in jointsToPlot]                # indices of joints

                # row and column labels
                cols = [self._legInitials[leg_number - 1] + " " + col for col in self._legJointsStrings]

                rows = ['{}'.format(row) for row in dataNames]

                # create subplot objects
                fig, axes = plt.subplots(nrows=len(data), ncols=len(jointsToPlot), figsize=(14, 8))
                matplotlib.rc('xtick', labelsize=25)
                matplotlib.rc('ytick', labelsize=25)

                for fl_num in range(len(data)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for joint_num, joint in enumerate(jointsToPlot):              # loop over joints
                        for num, tau in enumerate(data[experimentName]):          # loop over quantities
                            if len(self._experimentName) > 1:
                                lineLabel = dataNames[num] + " " + experimentName
                            else:
                                lineLabel = dataNames[num]
                            axes[joint_num].plot(
                                timeData[experimentName][usefullSamples[fl_num][0]:usefullSamples[fl_num][1]],
                                [sample[jointsIndicesToPlot[joint_num]] for sample in tau[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                linewidth=linewidths[num], label=lineLabel, linestyle=linestyles[fl_num], color=colors[num], alpha=alphas[num])
                        axes[joint_num].grid()
                        axes[-1].legend()

                # axes[3, 0].plot([sample[jointsIndicesToPlot[0]] for sample in data[2]], linewidth=2, linestyle="--")
                # axes[3, 0].plot([sample[jointsIndicesToPlot[0]] for sample in data[1]], linewidth=2, linestyle="-.")
                # axes[0, 0].text(2200, -10, 'lift off', 'o')
                # axes[0, 0].annotate('LO', (1200, -10), color='r')
                # axes[0, 0].annotate('TD', (1500, 125), color='r')

                # labels
                for ax in axes.flat:
                    ax.set_xlabel(xlabel, fontsize=25)
                for ax, col in zip(axes, cols):
                    ax.set_title(col, size='large', y=1, fontsize=25)
                for ax, row in zip(axes, rows):
                    ax.set_ylabel(row, rotation=90, fontsize=25)
                # title
                plt.suptitle('Torque ff ref and torque measurements (all $N*m$)', y=0.99, fontsize=20)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)

        if arms:
            for arm_number in range(1, 3):
                jointsToPlot = ['j_arm' + str(arm_number) + joint_name for joint_name in
                                self._armJointsPostfix]
                jointsIndicesToPlot = [names.index(i) for i in jointsToPlot]

                # row and column labels
                cols = [col for col in jointsToPlot]
                rows = ['{}'.format(row) for row in dataNames]

                fig, axes = plt.subplots(nrows=len(data[self._experimentName[0]]), ncols=len(jointsToPlot), figsize=(14, 8))
                matplotlib.rc('xtick', labelsize=25)
                matplotlib.rc('ytick', labelsize=25)

                for fl_num in range(len(data)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for num, tau in enumerate(data[experimentName]):          # loop over quantities
                        for joint_num, joint in enumerate(jointsToPlot):                    # loop over joints
                            axes[num, joint_num].plot(
                                [sample[jointsIndicesToPlot[joint_num]] for sample in tau[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                linewidth=2, label=experimentName)
                            axes[num, joint_num].grid()

                # axes[3, 0].plot([sample[jointsIndicesToPlot[0]] for sample in data[2]], linewidth=2, linestyle="--")
                # axes[3, 0].plot([sample[jointsIndicesToPlot[0]] for sample in data[1]], linewidth=2, linestyle="-.")
                # axes[0, 0].text(2200, -10, 'lift off', 'o')
                # axes[0, 0].annotate('LO', (1200, -10), color='r')
                # axes[0, 0].annotate('TD', (1500, 125), color='r')

                # labels
                for ax in axes[-1].flat:
                    ax.set(xlabel='Samples')
                for ax, col in zip(axes[0], cols):
                    ax.set_title(col, size='large', y=1, fontsize=25)

                for ax, row in zip(axes[:, 0], rows):
                    ax.set_ylabel(row, rotation=90, fontsize=20)
                # title
                plt.suptitle(self._experimentName[0] + ' (all $N*m$)', y=0.99, fontsize=20)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        plt.show()

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def printTorqueTerms(self, usefullSamples=None, legs=True, arms=True):
        if usefullSamples == None:
            usefullSamples = self._usefullSamples

        # get data
        data = {}
        percentageData = {}
        for file_num, fl in enumerate(self._bagFile):  # loop over all bag files
            effortReference, positionRelatedTorque, velocityRelatedTorque, feedforwardTorque = ([] for i in range(4))
            positionRelatedAbs, velocityRelatedAbs, feedforwardTorqueAbs = ([] for i in range(3))
            positionRelatedPercentage, velocityRelatedPercentage, feedforwardTorquePercentage = ([] for i in range(3))
            time = []
            for topic, msg, t in fl.read_messages(topics=[self._jointStatesTopic]):
                # torques
                positionRelatedTorque.append(np.multiply(msg.stiffness, np.subtract(msg.position_reference, msg.motor_position)))
                velocityRelatedTorque.append(np.multiply(msg.damping, np.subtract(msg.velocity_reference, msg.motor_velocity)))
                feedforwardTorque.append(msg.effort_reference)
                effortReference.append(np.add(np.add(positionRelatedTorque[-1], velocityRelatedTorque[-1]), feedforwardTorque[-1]))
                if len(feedforwardTorque) == 1:
                    names = msg.name

                # absolute values
                positionRelatedAbs.append(np.abs(np.multiply(msg.stiffness, np.subtract(msg.position_reference, msg.motor_position))))
                velocityRelatedAbs.append(np.abs(np.multiply(msg.damping, np.subtract(msg.velocity_reference, msg.motor_velocity))))
                feedforwardTorqueAbs.append(np.abs(msg.effort_reference))
                sum = positionRelatedAbs[-1] + velocityRelatedAbs[-1] + feedforwardTorqueAbs[-1]


                # percentages
                positionRelatedPercentage.append(100 * positionRelatedAbs[-1] / sum)
                velocityRelatedPercentage.append(
                    100 * velocityRelatedAbs[-1] / sum)
                feedforwardTorquePercentage.append(
                    100 * feedforwardTorqueAbs[-1] / sum)

                # time
                if len(time) == 0:
                    initialTime = t.to_sec()
                time.append(t.to_sec() - initialTime)

            # get contact status from base estimation topic
            contactStatus = []
            baseEstTime = []
            for topic, msg, t in fl.read_messages(topics=['/centauro_base_estimation/contacts/status']):
                contactStatus.append([k.status for k in msg.contacts_status])
                if len(baseEstTime) == 0:  # time
                    initialTime = t.to_sec()
                baseEstTime.append(t.to_sec() - initialTime)

            # wrap data in the order to be plotted
            data.update({self._experimentName[file_num]: [effortReference, positionRelatedTorque, velocityRelatedTorque, feedforwardTorque]})
            percentageData.update({self._experimentName[file_num]: [positionRelatedPercentage, velocityRelatedPercentage, feedforwardTorquePercentage]})
        dataNames = ['$τ_{ref}$', '$K_p * e_q$', '$K_d * e_{\dot{q}}$', '$τ_{ff}$']
        percentageDataNames = ['position-based', 'velocity-based', 'feedforward']

        # linestyle and colors
        linestyles = ['-', '--', '-']
        markers = ['', '', '']
        colors = ['r', 'b', 'g', 'tab:gray', 'tab:green']
        linewidths = [4.5, 3.5]
        alphas = [0.75, 0.75, 0.75]

        matplotlib.rc('xtick', labelsize=20)
        matplotlib.rc('ytick', labelsize=20)
        xlabel = 'Time ($sec$)'

        # plots
        if legs:
            figPercentage, axesPercentage = plt.subplots(nrows=len(self._legJointsPrefix), ncols=len(self._legJointsPrefix), figsize=(25, 4))
            for leg_number in range(1, 5):
                jointsToPlot = [joint_name + str(leg_number) for joint_name in self._legJointsPrefix]
                jointsIndicesToPlot = [names.index(i) for i in jointsToPlot]

                # row and column labels
                cols = [self._legInitials[leg_number - 1] + " " + col for col in self._legJointsStrings]
                rows = ['{}'.format(row) for row in dataNames]
                fig, axes = plt.subplots(nrows=len(data[self._experimentName[0]]), ncols=len(jointsToPlot), figsize=(20, 8))

                # percentages
                for fl_num in range(len(data)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for num, tau in enumerate(percentageData[experimentName]):              # loop over quantities
                        for joint_num, joint in enumerate(jointsToPlot):                    # loop over joints
                            axesPercentage[leg_number-1, joint_num].plot(
                                time[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]],
                                [k[jointsIndicesToPlot[joint_num]] for k in tau[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                linewidth=linewidths[fl_num], label=percentageDataNames[num], linestyle=linestyles[fl_num], color=colors[num], alpha=alphas[num], marker=markers[num])
                            # shade graphs when i contact
                            lastContactIndex = 0
                            while True:
                                leg_num = leg_number - 1
                                try:
                                    # try to detect start of contact
                                    legListToSearchOne = contactStatus[lastContactIndex:]
                                    firstContactIndex = lastContactIndex + [sample[leg_num] for sample in
                                                                            legListToSearchOne].index(1)
                                except:
                                    # if there is no other start of contact exit while loo with break
                                    break
                                try:
                                    # after detecting a start of contact now try to detect loss of contact
                                    legListToSearchLossOfContact = contactStatus[firstContactIndex:]
                                    lastContactIndex = firstContactIndex + [sample[leg_num] for sample in
                                                                            legListToSearchLossOfContact].index(0)
                                    axesPercentage[leg_number-1, joint_num].axvspan(
                                        baseEstTime[firstContactIndex],
                                        baseEstTime[lastContactIndex],
                                        facecolor='tab:gray', alpha=0.1)  # plot shaded region
                                except:
                                    # of there is no loss of contact detected then leg was in contact until the last sample
                                    axesPercentage[leg_number-1, joint_num].axvspan(
                                        baseEstTime[firstContactIndex],
                                        baseEstTime[-1],
                                        facecolor='tab:gray', alpha=0.1)
                                    break
                            axesPercentage[leg_number-1, joint_num].grid()
                            # legends = axesPercentage[0, -1].legend(fontsize=25)
                            # for line in legends.get_lines():
                            #     line.set_linewidth(10.0)
                # labels
                for ax in axesPercentage[-1].flat:
                    ax.set_xlabel(xlabel, fontsize=25)
                for ax, col in zip(axesPercentage[0], self._legJointsStrings):
                    ax.set_title(col, size='large', y=1, fontsize=20)
                for ax, row in zip(axesPercentage[:, 0], rows):
                    ax.set_ylabel(r'$ \alpha_i $ (%)', rotation=90, fontsize=25)
                # plt.suptitle(self._experimentName[0] + ' (all $N*m$)', y=0.99, fontsize=25)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)

                # TORQUE TERMS IN SEPARATED GRAPHS
                for num, tau in enumerate(data[experimentName]):          # loop over quantities
                    for joint_num, joint in enumerate(jointsToPlot):                    # loop over joints
                        axes[num, joint_num].plot(
                            time[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]],
                            [sample[jointsIndicesToPlot[joint_num]] for sample in tau[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                            linewidth=2+fl_num, label=experimentName)
                        axes[num, joint_num].grid()
                        axes[num, joint_num].legend()

                # labels
                for ax in axes[-1].flat:
                    ax.set_xlabel(xlabel, fontsize=25)
                for ax, jointstr in zip(axes[0], cols):
                    ax.set_title(jointstr, size='large', y=1, fontsize=25)
                for ax, row in zip(axes[:, 0], rows):
                    ax.set_ylabel(row, rotation=90, fontsize=25)
                # title
                # plt.suptitle(self._experimentName[0] + ' (all $N*m$)', y=0.99, fontsize=25)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        if arms:
            for arm_number in range(1, 3):
                jointsToPlot = ['j_arm' + str(arm_number) + joint_name for joint_name in
                                self._armJointsPostfix]
                jointsIndicesToPlot = [names.index(i) for i in jointsToPlot]

                # row and column labels
                cols = [col for col in jointsToPlot]
                rows = ['{}'.format(row) for row in dataNames]

                fig, axes = plt.subplots(nrows=len(data[self._experimentName[0]]), ncols=len(jointsToPlot), figsize=(20, 8))
                figPercentage, axesPercentage = plt.subplots(nrows=len(data), ncols=len(jointsToPlot), figsize=(25, 4))
                xlabel = 'Time $sec$'

                # percentages
                for fl_num in range(len(data)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for num, tau in enumerate(percentageData[experimentName]):              # loop over quantities
                        for joint_num, joint in enumerate(jointsToPlot):                    # loop over joints
                            axesPercentage[joint_num].plot(
                                time[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]],
                                [k[jointsIndicesToPlot[joint_num]] for k in tau[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                linewidth=2.0, label=percentageDataNames[num], linestyle=linestyles[fl_num], color=colors[num], alpha=alphas[num])
                            axesPercentage[joint_num].grid()
                        axesPercentage[-1].legend()
                    # labels
                    for ax in axesPercentage.flat:
                        ax.set_xlabel(xlabel, fontsize=25)
                    for ax, col in zip(axesPercentage, cols):
                        ax.set_title(col, size='large', y=1, fontsize=25)
                    axesPercentage[0].set_ylabel(r'$ \alpha_i $ (%)', rotation=90, fontsize=25)

                    # plt.suptitle(self._experimentName[0] + ' (all $N*m$)', y=0.99, fontsize=25)
                    # fig.tight_layout()
                    fig.subplots_adjust(top=0.9, left=0.1, right=0.98)

                # TORQUE TERMS IN SEPARATED GRAPHS
                for fl_num in range(len(data)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for num, tau in enumerate(data[experimentName]):          # loop over quantities
                        for joint_num, joint in enumerate(jointsToPlot):                    # loop over joints
                            axes[num, joint_num].plot([sample[jointsIndicesToPlot[joint_num]] for sample in tau[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                                      linewidth=2, label=experimentName)
                            axes[num, joint_num].grid()
                            axes[num, joint_num].legend()

                # labels
                for ax in axes[-1].flat:
                    ax.set(xlabel='Samples')
                for ax, col in zip(axes[0], cols):
                    ax.set_title(col, size='large', y=1, fontsize=25)

                for ax, row in zip(axes[:, 0], rows):
                    ax.set_ylabel(row, rotation=90, fontsize=20)
                # title
                # plt.suptitle(self._experimentName[0] + ' (all $N*m$)', y=0.99, fontsize=20)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        plt.show()

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def printTorqueTracking(self, usefullSamples=None, legs=True, arms=True):
        if usefullSamples == None:
            usefullSamples = self._usefullSamples

        data = {}
        timeData = {}
        for file_num, fl in enumerate(self._bagFile):  # loop over all bag files
            effortReference = []
            effort = []
            time = []
            timeJointStates = []
            for topic, msg, t in fl.read_messages(topics=[self._torqueTermsTopic]):
                effortReference.append(msg.reference)

                if len(effortReference) == 1:
                    names = msg.name

                if len(time) == 0:                      # time
                    initialTime = t.to_sec()
                time.append(t.to_sec() - initialTime)

            for topic, msg, t in fl.read_messages(topics=[self._jointStatesTopic]):
                effort.append(msg.effort)
                if len(timeJointStates) == 0:                      # time
                    initialTime = t.to_sec()
                timeJointStates.append(t.to_sec() - initialTime)

            # wrap data in the order to be plotted
            data.update({self._experimentName[file_num]: [effortReference, effort]})
            timeData.update({self._experimentName[file_num]: [time, timeJointStates]})
        torqueLabels = ['reference', 'measured']
        xlabel = 'Time ($sec$)'

        # linestyle and colors
        linestyles = ['-', '--', '-.']
        colors = ['r', 'b', 'tab:gray', 'tab:green']
        linewidths = [4.5, 5.0]
        alphas = [0.9, 0.7]

        # plots
        matplotlib.rc('xtick', labelsize=50)
        matplotlib.rc('ytick', labelsize=50)
        if legs:
            for leg_number in range(1, 5):
                jointsToPlot = [joint_name + str(leg_number) for joint_name in
                                self._legJointsPrefix]
                jointsIndicesToPlot = [names.index(i) for i in jointsToPlot]
                # jointsToPlotStrings = [names[i] for i in jointsIndicesToPlot]

                # row and column labels
                cols = [self._legInitials[leg_number - 1] + " " + col for col in self._legJointsStrings]
                rows = ['{}'.format(row) for row in ['Torque $(N \cdot m)$']]

                fig, axes = plt.subplots(nrows=1, ncols=len(jointsToPlot), figsize=(20, 6))

                for fl_num in range(len(data)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for joint_num, joint in enumerate(jointsToPlot):  # loop over joints
                        for num, tau in enumerate(data[experimentName]):          # loop over quantities
                            if len(self._experimentName) > 1:
                                lineLabel = torqueLabels[num] + " " + experimentName
                            else:
                                lineLabel = torqueLabels[num]
                            axes[joint_num].plot(
                                timeData[experimentName][num][usefullSamples[fl_num][0]:usefullSamples[fl_num][1]],
                                [sample[jointsIndicesToPlot[joint_num]] for sample in tau[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                linewidth=linewidths[num], label=lineLabel, linestyle=linestyles[fl_num], color=colors[num], alpha=alphas[num])
                        axes[joint_num].grid()
                #leg = axes[-1].legend(fontsize=40)
                # for line in leg.get_lines():
                #     line.set_linewidth(10.0)


                # axes[3, 0].plot([sample[jointsIndicesToPlot[0]] for sample in data[2]], linewidth=2, linestyle="--")
                # axes[3, 0].plot([sample[jointsIndicesToPlot[0]] for sample in data[1]], linewidth=2, linestyle="-.")
                # axes[0, 0].text(2200, -10, 'lift off', 'o')
                # axes[0, 0].annotate('LO', (1200, -10), color='r')
                # axes[0, 0].annotate('TD', (1500, 125), color='r')

                # labels
                for ax in axes.flat:
                    ax.set_xlabel(xlabel, fontsize=50)
                for ax, col in zip(axes, cols):
                    ax.set_title(col, size='large', y=1, fontsize=50)
                for ax, row in zip(axes, rows):
                    ax.set_ylabel(row, rotation=90, fontsize=50)
                # title
                # plt.suptitle(self._experimentName[0] + ' (all $N*m$)', y=0.99, fontsize=20)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        if arms:
            for arm_number in range(1, 3):
                jointsToPlot = ['j_arm' + str(arm_number) + joint_name for joint_name in
                                self._armJointsPostfix]
                jointsIndicesToPlot = [names.index(i) for i in jointsToPlot]
                # jointsToPlot = [names[i] for i in jointsIndicesToPlot]

                # row and column labels
                cols = [col for col in jointsToPlot]
                rows = ['{}'.format(row) for row in ['Torque $(N \cdot m)$']]

                fig, axes = plt.subplots(nrows=len(data), ncols=len(jointsToPlot), figsize=(14, 8))
                matplotlib.rc('xtick', labelsize=35)
                matplotlib.rc('ytick', labelsize=35)

                for fl_num in range(len(data)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for joint_num, joint in enumerate(jointsToPlot):  # loop over joints
                        for num, tau in enumerate(data[experimentName]):          # loop over quantities
                            if len(self._experimentName) > 1:
                                lineLabel = torqueLabels[num] + " " + experimentName
                            else:
                                lineLabel = torqueLabels[num]
                            axes[joint_num].plot(
                                timeData[experimentName][num][usefullSamples[fl_num][0]:usefullSamples[fl_num][1]],
                                [sample[jointsIndicesToPlot[joint_num]] for sample in tau[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                linewidth=linewidths[num], label=lineLabel, linestyle=linestyles[0], color=colors[num], alpha=alphas[num])
                        axes[joint_num].grid()
                    axes[-1].legend()

                # labels
                for ax in axes.flat:
                    ax.set_xlabel(xlabel, fontsize=50)
                for ax, col in zip(axes, cols):
                    ax.set_title(col, size='large', y=1, fontsize=50)

                for ax, row in zip(axes, rows):
                    ax.set_ylabel(row, rotation=90, fontsize=50)
                # title
                # plt.suptitle(self._experimentName[0] + ' (all $N*m$)', y=0.99, fontsize=20)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        plt.show()

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def printVelocities(self, usefullSamples=None, legs=True, arms=True):
        if usefullSamples == None:
            usefullSamples = self._usefullSamples

        velocityData = {}
        timeData = {}
        for file_num, fl in enumerate(self._bagFile):  # loop over all bag files
            # create list of msgs
            velocityReference = []
            motorVelocity = []
            time = []
            for topic, msg, t in fl.read_messages(topics=[self._jointStatesTopic]):
                velocityReference.append(msg.velocity_reference)
                motorVelocity.append(msg.motor_velocity)

                if len(time) == 0:                      # time
                    initialTime = t.to_sec()
                time.append(t.to_sec() - initialTime)

                if len(velocityReference) == 1:
                    names_joint_state = msg.name

            velocityData.update({self._experimentName[file_num]: [velocityReference, motorVelocity]})
            timeData.update({self._experimentName[file_num]: time})
        jointStateLabels = ['reference', 'measured']
        xlabel = 'Time ($sec$)'

        # linestyle and colors
        linestyles = ['-', '--', '-.']
        colors = ['r', 'b', 'tab:gray', 'tab:green']
        linewidths = [4.5, 6.0]
        alphas = [0.9, 0.7]

        # plots
        matplotlib.rc('xtick', labelsize=50)
        matplotlib.rc('ytick', labelsize=50)
        if legs:
            for leg_number in range(1, 5):
                jointsToPlot = [joint_name + str(leg_number) for joint_name in self._legJointsPrefix]
                jointsIndicesJointState = [names_joint_state.index(i) for i in jointsToPlot]

                # row and column labels
                cols = [self._legInitials[leg_number - 1] + " " + col for col in self._legJointsStrings]

                rows = ['{}'.format(row) for row in ['Velocity  $(rad/sec)$']]

                fig, axes = plt.subplots(nrows=1, ncols=len(jointsToPlot), figsize=(14, 8))

                for fl_num in range(len(velocityData)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for joint_num, joint in enumerate(jointsToPlot):  # loop over joints
                        for num, vel in enumerate(velocityData[experimentName]):          # loop over quantities
                            if len(self._experimentName) > 1:
                                lineLabel = jointStateLabels[num] + " " + experimentName
                            else:
                                lineLabel = jointStateLabels[num]
                            axes[joint_num].plot(timeData[experimentName][usefullSamples[fl_num][0]:usefullSamples[fl_num][1]],
                                [sample[jointsIndicesJointState[joint_num]] for sample in vel[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                linewidth=linewidths[num], label=lineLabel, linestyle=linestyles[0], color=colors[num], alpha=alphas[num])
                        axes[joint_num].grid()
                        # axes[-1].legend(fontsize=25)
                # labels
                for ax in axes:
                    ax.set_xlabel(xlabel, fontsize=30)
                for ax, col in zip(axes, cols):
                    ax.set_title(col, size='large', y=1, fontsize=50)

                for ax, row in zip(axes, rows):
                    ax.set_ylabel(row, rotation=90, fontsize=50)
                # title
                # plt.suptitle(self._experimentName[0], y=0.99, fontsize=20)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        if arms:
            for arm_number in range(1, 3):
                jointsToPlot = ['j_arm' + str(arm_number) + joint_name for joint_name in
                                self._armJointsPostfix]
                jointsIndicesJointState = [names_joint_state.index(i) for i in jointsToPlot]

                # row and column labels
                cols = [self._armInitials[arm_number - 1] + " " + col for col in self._armJointsPostfix]

                rows = ['{}'.format(row) for row in ['$velocity (rad/sec)$']]

                fig, axes = plt.subplots(nrows=1, ncols=len(jointsToPlot), figsize=(14, 8))

                for fl_num in range(len(velocityData)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for joint_num, joint in enumerate(jointsToPlot):
                        for num, vel in enumerate(velocityData[experimentName]):
                            if len(self._experimentName) > 1:
                                lineLabel = jointStateLabels[num] + " " + experimentName
                            else:
                                lineLabel = jointStateLabels[num]
                            axes[joint_num].plot(
                                timeData[experimentName][usefullSamples[fl_num][0]:usefullSamples[fl_num][1]],
                                [sample[jointsIndicesJointState[joint_num]] for sample in vel[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                linewidth=linewidths[num], label=lineLabel, linestyle=linestyles[0], color=colors[num], alpha=alphas[num])
                        axes[joint_num].grid()
                        axes[-1].legend()
                # labels
                for ax in axes:
                    ax.set_xlabel(xlabel, fontsize=25)
                for ax, col in zip(axes, cols):
                    ax.set_title(col, size='large', y=1, fontsize=25)

                for ax, row in zip(axes, rows):
                    ax.set_ylabel(row, rotation=90, fontsize=25)
                # title
                # plt.suptitle(self._experimentName, y=0.99, fontsize=20)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        plt.show()

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def printPositions(self, usefullSamples=None, legs=True, arms=True):
        if usefullSamples == None:
            usefullSamples = self._usefullSamples

        positionData = {}
        for file_num, fl in enumerate(self._bagFile):  # loop over all bag files
            # create list of msgs
            positionReference = []
            motorPosition = []
            for topic, msg, t in fl.read_messages(topics=[self._jointStatesTopic]):
                positionReference.append(msg.position_reference)
                motorPosition.append(msg.motor_position)

                if len(positionReference) == 1:
                    names_joint_state = msg.name

            positionData.update({self._experimentName[file_num]: [positionReference, motorPosition]})
        jointStateLabels = ['ref', 'motor']

        # plots
        if legs:
            for leg_number in range(1, 5):
                jointsToPlot = [joint_name + str(leg_number) for joint_name in self._legJointsPrefix]
                jointsIndicesJointState = [names_joint_state.index(i) for i in jointsToPlot]

                # row and column labels
                cols = [self._legInitials[leg_number - 1] + " " + col for col in self._legJointsStrings]

                rows = ['{}'.format(row) for row in ['position (rad)$']]

                fig, axes = plt.subplots(nrows=1, ncols=len(jointsToPlot), figsize=(14, 8))
                matplotlib.rc('xtick', labelsize=25)
                matplotlib.rc('ytick', labelsize=25)

                for fl_num in range(len(positionData)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for joint_num, joint in enumerate(jointsToPlot):
                        for num, vel in enumerate(positionData[experimentName]):
                            axes[joint_num].plot([sample[jointsIndicesJointState[joint_num]] for sample in vel[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                                 linewidth=2, label=jointStateLabels[num] + " " + experimentName)
                        axes[joint_num].grid()
                        axes[joint_num].legend()
                # labels
                for ax in axes:
                    ax.set(xlabel='Samples')
                for ax, col in zip(axes, cols):
                    ax.set_title(col, size='large', y=1, fontsize=25)

                for ax, row in zip(axes, rows):
                    ax.set_ylabel(row, rotation=90, fontsize=20)
                # title
                plt.suptitle(self._experimentName, y=0.99, fontsize=20)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        if arms:
            for arm_number in range(1, 3):
                jointsToPlot = ['j_arm' + str(arm_number) + joint_name for joint_name in
                                self._armJointsPostfix]
                jointsIndicesJointState = [names_joint_state.index(i) for i in jointsToPlot]

                # row and column labels
                cols = [self._armInitials[arm_number - 1] + " " + col for col in self._armJointsPostfix]

                rows = ['{}'.format(row) for row in ['position (rad)$']]

                fig, axes = plt.subplots(nrows=1, ncols=len(jointsToPlot), figsize=(14, 8))
                matplotlib.rc('xtick', labelsize=25)
                matplotlib.rc('ytick', labelsize=25)

                for fl_num in range(len(positionData)):                                             # loop over bag files
                    experimentName = self._experimentName[fl_num]
                    for joint_num, joint in enumerate(jointsToPlot):
                        for num, vel in enumerate(positionData[experimentName]):
                            axes[joint_num].plot([sample[jointsIndicesJointState[joint_num]] for sample in vel[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                                                 linewidth=2,
                                                 label=jointStateLabels[num] + " " + experimentName)
                        axes[joint_num].grid()
                        axes[joint_num].legend()
                # labels
                for ax in axes:
                    ax.set(xlabel='Samples')
                for ax, col in zip(axes, cols):
                    ax.set_title(col, size='large', y=1, fontsize=25)

                for ax, row in zip(axes, rows):
                    ax.set_ylabel(row, rotation=90, fontsize=20)
                # title
                plt.suptitle(self._experimentName, y=0.99, fontsize=20)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        plt.show()

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def printForcesFromBaseEstimation(self, usefullSamples=None, legs=True, arms=True):
        if usefullSamples == None:
            usefullSamples = self._usefullSamples

        ContactsForcesData = {}
        for file_num, fl in enumerate(self._bagFile):  # loop over all bag files
            # create list of msgs
            contactStatus = []
            for topic, msg, t in fl.read_messages(topics=['/centauro_base_estimation/contacts/status']):
                contactStatus.append([k.status for k in msg.contacts_status])

            contactWrench = []
            for topic, msg, t in fl.read_messages(topics=['/centauro_base_estimation/contacts/wrench']):
                contactWrench.append(msg.force_norm)
            ContactsForcesData.update({self._experimentName[file_num]: [contactStatus, contactWrench]})
        ContactsDataLabels = ['contact status', 'contact force norm']

        if legs:
            # plots
            leg_list = ['FL', 'FR', 'HL', 'HR']

            # row and column labels
            cols = [col for col in leg_list[:2]]
            rows = [row for row in leg_list[2:]]

            fig, axes = plt.subplots(nrows=len(rows), ncols=len(cols), figsize=(14, 8))
            matplotlib.rc('xtick', labelsize=25)
            matplotlib.rc('ytick', labelsize=25)

            for fl_num in range(len(ContactsForcesData)):                                    # loop over bag files
                experimentName = self._experimentName[fl_num]
                for leg_num, leg in enumerate(leg_list):
                    for num, data_type in enumerate(ContactsForcesData[experimentName]):
                        # plot contact status and wrench force norm
                        axes[int(leg_num/2), leg_num % 2].plot(
                            [sample[leg_num] for sample in data_type[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                            linewidth=2, label=ContactsDataLabels[num])
                    # shade graphs when i contact
                    lastContactIndex = 0
                    while True:
                        try:
                            # try to detect start of contact
                            legListToSearchOne = ContactsForcesData[experimentName][0][lastContactIndex:]
                            firstContactIndex = lastContactIndex + [sample[leg_num] for sample in legListToSearchOne].index(1)
                        except:
                            # if there is no other start of contact exit while loo with break
                            break
                        try:
                            # after detecting a start of contact now try to detect loss of contact
                            legListToSearchLossOfContact = ContactsForcesData[experimentName][0][firstContactIndex:]
                            lastContactIndex = firstContactIndex + [sample[leg_num] for sample in legListToSearchLossOfContact].index(0)
                            axes[int(leg_num/2), leg_num % 2].axvspan(firstContactIndex, lastContactIndex, facecolor='b', alpha=0.4)    # plot shaded region
                        except:
                            # of there is no loss of contact detected then leg was in contact until the last sample
                            axes[int(leg_num / 2), leg_num % 2].axvspan(firstContactIndex, len(contactStatus), facecolor='b', alpha=0.4)
                            break
                    # labels
                    axes[int(leg_num/2), leg_num % 2].grid()
                    axes[int(leg_num/2), leg_num % 2].legend()
                    axes[int(leg_num/2), leg_num % 2].set_ylabel(leg + '($N$)', rotation=90, fontsize=20)
            for ax in axes[-1].flat:
                ax.set(xlabel='Samples')

            # title
            plt.suptitle('Contact Wrench estimation and contact status', y=0.99, fontsize=20)
            # fig.tight_layout()
            fig.subplots_adjust(top=0.9, left=0.1, right=0.98)

        if arms:
            # plots
            arm_list = ['LA', 'RA']

            # row and column labels
            cols = [col for col in arm_list]
            # rows = ['']
            fig, axes = plt.subplots(nrows=1, ncols=len(cols), figsize=(14, 8))
            matplotlib.rc('xtick', labelsize=25)
            matplotlib.rc('ytick', labelsize=25)

            # keep only arm data
            armContactForcesData = [[i[4:6] for i in k] for k in ContactsForcesData]
            for fl_num in range(len(ContactsForcesData)):                                    # loop over bag files
                experimentName = self._experimentName[fl_num]
                for arm_num, arm in enumerate(arm_list):
                    for num, data_type in enumerate(ContactsForcesData[experimentName]):
                        # plot contact status and wrench force norm
                        # axes[arm_num].plot([sample[arm_num] for sample in data_type[usefullSamples[0]:usefullSamples[1]]], linewidth=2,
                        #                                          label=ContactsDataLabels[num])
                        axes[arm_num].plot(
                            [sample[len(leg_list) + arm_num] for sample in
                             data_type[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]],
                            linewidth=2, label=ContactsDataLabels[num])
                    # shade graphs when i contact
                    # shade graphs when i contact
                    lastContactIndex = 0
                    while True:
                        try:
                            # try to detect start of contact
                            armListToSearchOne = ContactsForcesData[experimentName][0][lastContactIndex:]
                            firstContactIndex = lastContactIndex + [sample[len(leg_list) + arm_num] for sample in
                                                                    armListToSearchOne].index(1)
                        except:
                            # if there is no other start of contact exit while loo with break
                            break
                        try:
                            # after detecting a start of contact now try to detect loss of contact
                            armListToSearchLossOfContact = ContactsForcesData[experimentName][0][firstContactIndex:]
                            lastContactIndex = firstContactIndex + [sample[len(leg_list) + arm_num] for sample in
                                                                    armListToSearchLossOfContact].index(0)
                            axes[arm_num].axvspan(firstContactIndex, lastContactIndex,
                                                                        facecolor='b',
                                                                        alpha=0.4)  # plot shaded region
                        except:
                            # of there is no loss of contact detected then leg was in contact until the last sample
                            axes[arm_num].axvspan(firstContactIndex, len(contactStatus),
                                                                        facecolor='b', alpha=0.4)
                            break
                    # labels
                    axes[arm_num].grid()
                    axes[arm_num].legend()
                    axes[arm_num].set_ylabel(arm + '($N$)', rotation=90, fontsize=20)
                    axes[arm_num].set(xlabel='Samples')
            # title
            plt.suptitle('Contact Wrench estimation and contact status', y=0.99, fontsize=20)
            # fig.tight_layout()
            fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        plt.show()

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def printMpcState(self, usefullSamples=None):
        if usefullSamples == None:
            usefullSamples = self._usefullSamples

        # create list of msgs
        mpcStateData = {}
        for file_num, fl in enumerate(self._bagFile):  # loop over all bag files
            mpcCoMMomentum = []
            mpcBasePose = []
            mpcJointPosition = []
            for topic, msg, t in fl.read_messages(topics=[self._mpcObservationTopic]):
                mpcCoMMomentum.append(msg.state.value[:6])
                mpcBasePose.append(msg.state.value[6:12])
                mpcJointPosition.append(msg.state.value[12:])
            mpcStateData.update({self._experimentName[file_num]: [mpcCoMMomentum, mpcBasePose, mpcJointPosition]})
        # list of samples, where sample is a list of 4 force norms i.e. [330, 350, 220, 200]
        mpcStateDataLabels = ['CoM Momentum', 'Base Pose', 'Joint Positions']


        # row and column labels
        # cols = [col for col in jointsToPlot]
        # rows = ['{}'.format(row) for row in ['position (rad)$']]

        for fl_num, fl in enumerate(self._bagFile):                                 # loop over all bag files
            experimentName = self._experimentName[fl_num]
            for data_num, data in enumerate(mpcStateData[experimentName]):
                fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(14, 8))
                matplotlib.rc('xtick', labelsize=25)
                matplotlib.rc('ytick', labelsize=25)

                for state_index in range(len(data[0])):
                    axes.plot([kk[state_index] for kk in data[usefullSamples[fl_num][0]: usefullSamples[fl_num][1]]], linewidth=2, label=str(state_index))
                axes.grid()
                axes.legend()
                # labels
                axes.set(xlabel='Samples')
                axes.set_title('MPC State Observation', size='large', y=1)
                axes.set_ylabel(mpcStateDataLabels[data_num], rotation=90, fontsize=20)
                # title
                plt.suptitle(self._experimentName, y=0.99, fontsize=20)
                # fig.tight_layout()
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)

        plt.show()

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def getTfFromBagFile(self, bagFile, source_tf, target_tf):
        # get ee trj from tf
        print('tf msg num = ', bagFile.get_message_count('/tf'))
        bagTransformer = BagTfTransformer(bagFile)
        print('constructed')
        eeTfPosition = []
        eeTfOrientation = []
        time = []
        for topic, msg, t in bagFile.read_messages(topics=["/tf"]):
            currentTf = bagTransformer.lookupTransform(source_tf, target_tf, t)
            eeTfPosition.append(currentTf[0])
            eeTfOrientation.append(currentTf[1])
            time.append(t.to_sec())
        # bagTransformer.plotTranslation('odometry/world', 'arm1_6',axis='z', trigger_orig_frame='base_link', trigger_dest_frame='arm1_5')
        return time, eeTfPosition, eeTfOrientation

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def printEeTrajectory(self, eeIndex=4, usefullSamples=None, showReference=False):
        if usefullSamples == None:
            usefullSamples = self._usefullSamples

        startMpcRecordingTime = {}
        endMpcRecordingTime = {}
        recordingDuration = {}
        for fl_num, fl in enumerate(self._mpcBagFile):  # loop over all bag files
            experimentName = self._experimentName[fl_num]
            startMpcRecordingTime.update({experimentName: 0})
            endMpcRecordingTime.update({experimentName: 0})
            recordingDuration.update({experimentName: 0})

            # get all mpc observations
            mpcRosTime, mpcObsMessages = getMessagesFromTopic(self._mpcObservationTopic, self._mpcBagPath[fl_num])

            # check start and end of recording mpc time
            startMpcRecordingTime[experimentName] = mpcObsMessages[0].time
            endMpcRecordingTime[experimentName] = mpcObsMessages[-1].time
            recordingDuration[experimentName] = endMpcRecordingTime[experimentName] - startMpcRecordingTime[experimentName]
            print("Recording started at mpc time: ", startMpcRecordingTime[experimentName],
                  " and finished at mpc time: ", endMpcRecordingTime[experimentName])
            print("Recording duration: ", recordingDuration[experimentName])

        # loop over EEs
        eeNumber = len(self._eesReferenceTopic)
        for ee in range(eeNumber):
            # get EE trj from /currentState topic
            eePoseTrjData = {}
            refTimeTrajectoryData = {}
            refPoseTrajectoryData = {}
            timeData = {}
            for fl_num, fl in enumerate(self._bagFile):  # loop over all bag files
                experimentName = self._experimentName[fl_num]
                eePoseTrj = []
                time = []
                for topic, msg, t in fl.read_messages(topics=[self._eesTrajectoryTopic[ee]]):
                    eePoseTrj.append(msg.markers[2 * eeIndex].pose)

                    if len(time) == 0:  # time
                        initialTime = t.to_sec()
                    # time.append(t.to_sec() - initialTime)
                    time.append(t.to_sec())
                eePoseTrjData.update({experimentName: eePoseTrj[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]})
                timeData.update({experimentName: time[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]})

                # get tf fromself._bagFile tf topic
                # eeTfPosition = self.getTfFromBagFile(self._bagFile[fl_num], 'odometry/world', "dagana_2_tcp")

            if showReference:
                for fl_num, fl in enumerate(self._mpcBagFile):  # loop over all bag files
                    experimentName = self._experimentName[fl_num]
                    # construct reference as a sequence of step commands
                    eeRefTrajectoryMsgList = []
                    for topic, msg, t in fl.read_messages(topics=[self._eesReferenceTopic[ee]]):
                        eeRefTrajectoryMsgList.append(msg)
                    refTimeTrajectory = [timeData[experimentName][0]]                                                   # append initial not published point
                    refPoseTrajectory = [eeRefTrajectoryMsgList[0].stateTrajectory[0].value[:3]]
                    for t in range(len(eeRefTrajectoryMsgList)):
                        # index of mpc time in rostime
                        index = max(bisect([k.time for k in mpcObsMessages], eeRefTrajectoryMsgList[t].timeTrajectory[0]) - 1, 0)
                        refTimeTrajectory.append(mpcRosTime[index])
                        # refTimeTrajectory.append(eeRefTrajectoryMsgList[t].timeTrajectory[0] - startMpcRecordingTime[experimentName])
                        refPoseTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[0].value[:3])

                        #refTimeTrajectory.append(eeRefTrajectoryMsgList[t].timeTrajectory[-1] - startMpcRecordingTime[experimentName])
                        index = max(
                            bisect([k.time for k in mpcObsMessages], eeRefTrajectoryMsgList[t].timeTrajectory[-1]) - 1, 0)
                        refTimeTrajectory.append(mpcRosTime[index])
                        refPoseTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[-1].value[:3])
                    refTimeTrajectory.append(timeData[experimentName][-1])                                 # append final not published point
                    refPoseTrajectory.append(eeRefTrajectoryMsgList[-1].stateTrajectory[-1].value[:3])
                    print('refTimeTrajectory = ', refTimeTrajectory)
                    refTimeTrajectoryData.update({experimentName: refTimeTrajectory})
                    refPoseTrajectoryData.update({experimentName: refPoseTrajectory})

            # row and column labels
            # cols = [col for col in jointsToPlot]
            # rows = ['{}'.format(row) for row in ['position (rad)$']]
            fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 8))
            matplotlib.rc('xtick', labelsize=25)
            matplotlib.rc('ytick', labelsize=25)
            coordinates = ['x', 'y', 'z']

            for fl_num, fl in enumerate(self._bagFile):                             # loop over all bag files
                experimentName = self._experimentName[fl_num]

                # loop over coordinates
                for coord, coord_str in enumerate(coordinates):
                    axes[coord].plot(timeData[experimentName],
                                     [getattr(kk.position, coordinates[coord]) for kk in eePoseTrjData[experimentName]],
                                     linewidth=3, label='ee_mpc_observation ' + experimentName)  # ee trj

                    if showReference:
                        axes[coord].plot(refTimeTrajectoryData[experimentName], [kk[coord] for kk in refPoseTrajectoryData[experimentName]],
                                         linewidth=2, label='reference ' + experimentName)              # ee ref
                    axes[coord].grid()
                    axes[coord].legend()
                    axes[coord].set_ylabel(coord_str, rotation=90, fontsize=20)
                    # labels
                    axes[-1].set(xlabel='Time (sec)')
                    # axes[coord].set_title('MPC State Observation', size='large', y=1)
                    plt.suptitle('EE trajectory tracking ' + experimentName, y=0.99, fontsize=20)                 # title
                    # fig.tight_layout()
                    fig.subplots_adjust(top=0.9, left=0.1, right=0.98)

        plt.show()

    def plot3DPlot(self, achievedTrj, referenceTrj, timeTrj):
        from scipy.interpolate import interp1d
        from mpl_toolkits.mplot3d import Axes3D
        from matplotlib.colors import Normalize
        # Convert the list of 3D points to a 2D array (3 rows, N columns)
        achieved_trajectory = np.array(achievedTrj).T
        reference_trajectory = np.array(referenceTrj).T

        # Create timestamps for the trajectories
        achieved_timestamps = np.linspace(timeTrj[0], timeTrj[-1], len(achievedTrj))
        reference_timestamps = np.linspace(timeTrj[0], timeTrj[-1], len(referenceTrj))

        # Create interpolation functions for achieved and reference trajectories
        interp_achieved = interp1d(achieved_timestamps, achieved_trajectory, kind='cubic', axis=1)
        interp_reference = interp1d(reference_timestamps, reference_trajectory, kind='cubic', axis=1)

        # Resample the trajectories at the reference timestamps
        achieved_resampled = interp_achieved(reference_timestamps)
        reference_resampled = interp_reference(reference_timestamps)

        # Normalize timestamps to [0, 1] for color mapping
        norm = Normalize(vmin=reference_timestamps.min(), vmax=reference_timestamps.max())
        colors = plt.cm.viridis(norm(reference_timestamps))

        # Create a 3D figure
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the achieved trajectory with color mapping based on time
        for i in range(len(achieved_resampled.T) - 1):
            ax.plot(achieved_resampled[0, i:i + 2], achieved_resampled[1, i:i + 2], achieved_resampled[2, i:i + 2],
                    c=colors[i], linewidth=6)

        # Plot the reference trajectory with color mapping based on time
        for i in range(len(reference_resampled.T) - 1):
            ax.plot(reference_resampled[0, i:i + 2], reference_resampled[1, i:i + 2], reference_resampled[2, i:i + 2],
                    c=colors[i], linewidth=6)

        # Add labels and legend
        matplotlib.rc('xtick', labelsize=25)
        matplotlib.rc('ytick', labelsize=25)
        ax.set_xlabel('X [m]', fontsize=20)
        ax.set_ylabel('Y [m]', fontsize=20)
        ax.set_zlabel('Z [m]', fontsize=20)
        ax.set_title('Arm End-Effector Position Tracking', fontsize=30)
        # ax.legend([])

        # Add a color bar to show the time progression
        sm = plt.cm.ScalarMappable(cmap=plt.cm.viridis, norm=norm)
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax)
        cbar.set_label('Time [s]', fontsize=30)

        # Show the plot
        plt.show()


    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def CompareTfAndCurrentStateTopics(self, eeIndex=4, usefullSamples=None, showReference=False):
        if usefullSamples == None:
            usefullSamples = self._usefullSamples

        # debug: a republished poses from tf listener, it exhibits time offset
        # tfFilePath = "/home/idadiotis/Desktop/ocs2_experiments/15July2023/tf_2023-07-17-18-29-13.bag"
        # tfTime, tfList = getMessagesFromTopic("tf_odometry/world_dagana_2_tcp", tfFilePath)
        # tfPosition = [k.transform.translation for k in tfList]

        startMpcRecordingTime = {}
        endMpcRecordingTime = {}
        recordingDuration = {}
        for fl_num, fl in enumerate(self._mpcBagFile):  # loop over all bag files
            experimentName = self._experimentName[fl_num]
            startMpcRecordingTime.update({experimentName: 0})
            endMpcRecordingTime.update({experimentName: 0})
            recordingDuration.update({experimentName: 0})

            # get all mpc observations
            mpcRosTime, mpcObsMessages = getMessagesFromTopic(self._mpcObservationTopic, self._mpcBagPath[fl_num])

            # check start and end of recording mpc time
            startMpcRecordingTime[experimentName] = mpcObsMessages[0].time
            endMpcRecordingTime[experimentName] = mpcObsMessages[-1].time
            recordingDuration[experimentName] = endMpcRecordingTime[experimentName] - startMpcRecordingTime[
                experimentName]
            print("Recording started at mpc time: ", startMpcRecordingTime[experimentName],
                  " and finished at mpc time: ", endMpcRecordingTime[experimentName])
            print("Recording duration: ", recordingDuration[experimentName])

        # loop over EEs, get datan for /legged_robot/currentState topic
        eeNumber = len(self._eesReferenceTopic)
        for ee in range(eeNumber):
            # get EE trj from /currentState topic
            eePoseTrjData = {}
            refTimeTrajectoryData = {}
            refPoseTrajectoryData = {}
            refOrientationTrajectoryData = {}
            timeData = {}
            for fl_num, fl in enumerate(self._bagFile):  # loop over all bag files
                experimentName = self._experimentName[fl_num]
                eePoseTrj = []
                time = []
                for topic, msg, t in fl.read_messages(topics=[self._eesTrajectoryTopic[ee]]):
                    eePoseTrj.append(msg.markers[2 * eeIndex].pose)

                    if len(time) == 0:  # time
                        initialTime = t.to_sec()
                    # time.append(t.to_sec() - initialTime)
                    time.append(t.to_sec())
                eePoseTrjData.update(
                    {experimentName: eePoseTrj[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]})
                timeData.update({experimentName: time[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]})

            # get data from tf topic, very slow
            eeTfTime, eeTfPosition, eeTfOrientation = self.getTfFromBagFile(self._bagFile[fl_num], 'odometry/world', "dagana_2_tcp")
            # with open("15July2023DaganaTfPosition", "wb") as fp:  # Pickling
            #     pickle.dump(eeTfPosition, fp)
            # with open("15July2023DaganaTfOrientation", "wb") as fp:  # Pickling
            #     pickle.dump(eeTfOrientation, fp)
            # with open("15July2023DaganaTfTime", "wb") as fp1:  # Pickling
            #     pickle.dump(eeTfTime, fp1)

            # load files
            # with open("15July2023DaganaTfPosition", "rb") as fp:  # Unpickling
            #     eeTfPosition = pickle.load(fp)
            # with open("15July2023DaganaTfOrientation", "rb") as fp:  # Unpickling
            #     eeTfOrientation = pickle.load(fp)
            # with open("15July2023DaganaTfTime", "rb") as fp:  # Unpickling
            #     eeTfTime = pickle.load(fp)

            # convert quaternion from tf to euler
            eeTfEuler = []
            previous_roll = 0
            for quat in eeTfOrientation:
                eulerAngles = list(euler_from_quaternion(quat, 'sxyz'))
                eulerAngles[0] = smooth_roll(previous_roll, eulerAngles[0])
                previous_roll = eulerAngles[0]
                # eulerAngles[0] = smoothed_roll
                eeTfEuler.append(eulerAngles)

            if showReference:
                refTimeTrajectoryData, refPoseTrajectoryData, refOrientationTrajectoryData = self.getReferenceFromTargetTopic(eeTfTime[0], eeTfTime[-1], ee, singleElementMessage=True)
                # for fl_num, fl in enumerate(self._mpcBagFile):  # loop over all bag files
                #     experimentName = self._experimentName[fl_num]
                #     # construct reference as a sequence of step commands
                #     eeRefTrajectoryMsgList = []
                #     for topic, msg, t in fl.read_messages(topics=[self._eesReferenceTopic[ee]]):
                #         eeRefTrajectoryMsgList.append(msg)
                #     # refTimeTrajectory = [timeData[experimentName][0]]  # append initial not published point
                #     refTimeTrajectory = [eeTfTime[0]]  # append initial not published point
                #     # refTimeTrajectory = [1689415775]  # append initial not published point
                #     refPoseTrajectory = [eeRefTrajectoryMsgList[0].stateTrajectory[0].value[:3],        # first target, not published
                #                          eeRefTrajectoryMsgList[0].stateTrajectory[0].value[:3]]
                #     refOrientationTrajectory = [eeRefTrajectoryMsgList[0].stateTrajectory[0].value[3:],        # first target, not published
                #                                 eeRefTrajectoryMsgList[0].stateTrajectory[0].value[3:]]
                #     for t in range(len(eeRefTrajectoryMsgList)):
                #         # index of mpc time in rostime
                #         index = max(bisect([k.time for k in mpcObsMessages],
                #                            eeRefTrajectoryMsgList[t].timeTrajectory[0]) - 1, 0)
                #         refTimeTrajectory.append(mpcRosTime[index])
                #         # refTimeTrajectory.append(eeRefTrajectoryMsgList[t].timeTrajectory[0] - startMpcRecordingTime[experimentName])
                #         # refTimeTrajectory.append(eeRefTrajectoryMsgList[t].timeTrajectory[-1] - startMpcRecordingTime[experimentName])
                #         index = max(
                #             bisect([k.time for k in mpcObsMessages],
                #                    eeRefTrajectoryMsgList[t].timeTrajectory[-1]) - 1, 0)
                #         refTimeTrajectory.append(mpcRosTime[index])
                #         refPoseTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[-1].value[:3])   # next target, append twice until new target arrives
                #         refPoseTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[-1].value[:3])
                #         refOrientationTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[-1].value[3:])  # next target, append twice until new target arrives
                #         refOrientationTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[-1].value[3:])
                #     # refTimeTrajectory.append(timeData[experimentName][-1])  # append final not published point
                #     refTimeTrajectory.append(eeTfTime[-1])  # append initial not published point
                #     print('refTimeTrajectory = ', refTimeTrajectory)
                #     refTimeTrajectoryData.update({experimentName: refTimeTrajectory})
                #     refPoseTrajectoryData.update({experimentName: refPoseTrajectory})
                #     refOrientationTrajectoryData.update({experimentName: [euler_from_quaternion(k, 'sxyz') for k in refOrientationTrajectory]})
                #     slerpReference = Slerp(refTimeTrajectory, R.from_quat(refOrientationTrajectory))

            # for itt, tt in enumerate(eeTfTime):
            #     if tt < refTimeTrajectory[0] or tt> refTimeTrajectory[-1]:
            #         print(itt)

            refTimeFromZero = [k - refTimeTrajectoryData[self._experimentName[0]][0] for k in refTimeTrajectoryData[self._experimentName[0]]]
            tfTimeFromZero = [k - refTimeTrajectoryData[self._experimentName[0]][0] for k in eeTfTime]
            # index of ref time in tf time
            initialIndex = max(bisect(tfTimeFromZero, refTimeFromZero[0]) - 1, 0)
            finalIndex = max(bisect(tfTimeFromZero, refTimeFromZero[-1]) - 1, 0)
            self.plot3DPlot(eeTfPosition[initialIndex:finalIndex + 1], refPoseTrajectoryData[self._experimentName[0]], refTimeFromZero)
            # row and column labels
            matplotlib.rc('xtick', labelsize=25)
            matplotlib.rc('ytick', labelsize=25)
            fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 8))
            coordinates = ['X', 'Y', 'Z']
            lwidth = 7

            for fl_num, fl in enumerate(self._bagFile):  # loop over all bag files
                experimentName = self._experimentName[fl_num]

                # loop over coordinates
                for coord, coord_str in enumerate(coordinates):
                    # axes[coord].plot(timeData[experimentName],        # currentState topic
                    #                  [getattr(kk.position, coordinates[coord]) for kk in
                    #                   eePoseTrjData[experimentName]],
                    #                  linewidth=3, label='ee_mpc_observation ' + experimentName)  # ee trj
                    if showReference:
                        axes[coord].plot([k - eeTfTime[0] for k in refTimeTrajectoryData[experimentName]],
                                         [kk[coord] for kk in refPoseTrajectoryData[experimentName]],
                                         linewidth=lwidth, label='reference', color='r', alpha=0.7)  # ee ref
                    # from tf
                    axes[coord].plot([k - eeTfTime[0] for k in eeTfTime],
                                     [kk[coord] for kk in eeTfPosition],
                                     linewidth=lwidth, label='end-effector', color='b', alpha=0.7)  # tf trj

                    axes[coord].grid()
                    axes[coord].set_ylabel(coord_str + ' [$m$]', rotation=90, fontsize=30)
                axes[0].legend(fontsize=30)
                # labels
                axes[-1].set_xlabel('Time (sec)', fontsize=30)
                # axes[coord].set_title('MPC State Observation', size='large', y=1)
                plt.suptitle('EE trajectory tracking ' + experimentName, y=0.99, fontsize=20)  # title
                fig.align_ylabels(axes)
                fig.subplots_adjust(top=0.9, left=0.1, right=0.98)

        # orientation
        fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 8))
        euler_coordinates = ['Roll', 'Pitch', 'Yaw']

        for fl_num, fl in enumerate(self._bagFile):  # loop over all bag files
            experimentName = self._experimentName[fl_num]

            # loop over coordinates
            for coord, coord_str in enumerate(euler_coordinates):
                # if showReference:
                    # axes[coord].plot([k - eeTfTime[0] for k in eeTfTime[:-7]],
                    #                  slerpReference(eeTfTime[:-7]).as_euler('xyz')[:, coord],
                    #                  linewidth=lwidth, label='reference', color='r', alpha=0.7)  # ee ref
                # from tf
                axes[coord].plot([k - eeTfTime[0] for k in eeTfTime],
                                 [kk[coord] for kk in eeTfEuler],
                                 linewidth=lwidth, label='end-effector', color='b', alpha=0.7)  # tf trj
                axes[coord].grid()
                axes[coord].set_ylabel(coord_str + ' [$rad$]', rotation=90, fontsize=30)
            #axes[-1].legend(fontsize=25)
            # labels
            axes[-1].set_xlabel('Time (sec)', fontsize=30)
            # axes[coord].set_title('MPC State Observation', size='large', y=1)
            plt.suptitle('EE trajectory tracking ' + experimentName, y=0.99, fontsize=20)  # title
            # fig.tight_layout()
            fig.align_ylabels(axes)
            fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
        plt.show()

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def getReferenceFromTargetTopic(self, initialTime, finalTime, referenceTopicIndex, singleElementMessage=False):
        # mpc time
        startMpcRecordingTime = {}
        endMpcRecordingTime = {}
        recordingDuration = {}
        for fl_num, fl in enumerate(self._mpcBagFile):  # loop over all bag files
            experimentName = self._experimentName[fl_num]
            startMpcRecordingTime.update({experimentName: 0})
            endMpcRecordingTime.update({experimentName: 0})
            recordingDuration.update({experimentName: 0})

            # get all mpc observations
            mpcRosTime, mpcObsMessages = getMessagesFromTopic(self._mpcObservationTopic, self._mpcBagPath[fl_num])

            # check start and end of recording mpc time
            startMpcRecordingTime[experimentName] = mpcObsMessages[0].time
            endMpcRecordingTime[experimentName] = mpcObsMessages[-1].time
            recordingDuration[experimentName] = endMpcRecordingTime[experimentName] - startMpcRecordingTime[
                experimentName]
            print("Recording started at mpc time: ", startMpcRecordingTime[experimentName],
                  " and finished at mpc time: ", endMpcRecordingTime[experimentName])
            print("Recording duration: ", recordingDuration[experimentName])

        # get data
        eePoseTrjData = {}
        refTimeTrajectoryData = {}
        refPoseTrajectoryData = {}
        refOrientationTrajectoryData = {}
        for fl_num, fl in enumerate(self._mpcBagFile):  # loop over all bag files
            experimentName = self._experimentName[fl_num]
            # construct reference as a sequence of step commands
            eeRefTrajectoryMsgList = []
            for topic, msg, t in fl.read_messages(topics=[self._eesReferenceTopic[referenceTopicIndex]]):
                eeRefTrajectoryMsgList.append(msg)

            if singleElementMessage:
                # refTimeTrajectory = [eeTfTime[0]]  # append initial not published point
                # refPoseTrajectory = [eeRefTrajectoryMsgList[0].stateTrajectory[0].value[:3],
                #                      eeRefTrajectoryMsgList[0].stateTrajectory[0].value[:3]]
                # refOrientationTrajectory = [eeRefTrajectoryMsgList[0].stateTrajectory[0].value[3:],
                #                             eeRefTrajectoryMsgList[0].stateTrajectory[0].value[3:]]
                refTimeTrajectory = []  #[initialTime]
                refPoseTrajectory, refOrientationTrajectory = ([] for k in range(2))
                for t in range(len(eeRefTrajectoryMsgList)):
                    # index of mpc time in rostime
                    index = max(bisect([k.time for k in mpcObsMessages],
                                       eeRefTrajectoryMsgList[t].timeTrajectory[0]) - 1, 0)
                    refTimeTrajectory.append(mpcRosTime[index])
                    refPoseTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[0].value[:3])
                    refOrientationTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[0].value[3:])
                #refTimeTrajectory.append(finalTime)  # append initial not published point
                refTimeTrajectoryData.update({experimentName: refTimeTrajectory})
                refPoseTrajectoryData.update({experimentName: refPoseTrajectory})
                refOrientationTrajectoryData.update(
                    {experimentName: [euler_from_quaternion(k, 'sxyz') for k in refOrientationTrajectory]})
            else:
                refTimeTrajectory = [initialTime]  # append initial not published point
                refPoseTrajectory = [eeRefTrajectoryMsgList[0].stateTrajectory[0].value[:3],  # first target, not published
                                     eeRefTrajectoryMsgList[0].stateTrajectory[0].value[:3]]
                refOrientationTrajectory = [eeRefTrajectoryMsgList[0].stateTrajectory[0].value[3:],
                                            # first target, not published
                                            eeRefTrajectoryMsgList[0].stateTrajectory[0].value[3:]]
                for t in range(len(eeRefTrajectoryMsgList)):
                    # index of mpc time in rostime
                    index = max(bisect([k.time for k in mpcObsMessages],
                                       eeRefTrajectoryMsgList[t].timeTrajectory[0]) - 1, 0)
                    refTimeTrajectory.append(mpcRosTime[index])
                    index = max(
                        bisect([k.time for k in mpcObsMessages],
                               eeRefTrajectoryMsgList[t].timeTrajectory[-1]) - 1, 0)
                    refTimeTrajectory.append(mpcRosTime[index])
                    refPoseTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[-1].value[
                                             :3])  # next target, append twice until new target arrives
                    refPoseTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[-1].value[:3])
                    refOrientationTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[-1].value[
                                                    3:])  # next target, append twice until new target arrives
                    refOrientationTrajectory.append(eeRefTrajectoryMsgList[t].stateTrajectory[-1].value[3:])
                refTimeTrajectory.append(finalTime)  # append initial not published point
                print('refTimeTrajectory = ', refTimeTrajectory)
                refTimeTrajectoryData.update({experimentName: refTimeTrajectory})
                refPoseTrajectoryData.update({experimentName: refPoseTrajectory})
                refOrientationTrajectoryData.update(
                    {experimentName: [euler_from_quaternion(k, 'sxyz') for k in refOrientationTrajectory]})
                slerpReference = Slerp(refTimeTrajectory, R.from_quat(refOrientationTrajectory))
                return refTimeTrajectoryData, refPoseTrajectoryData, refOrientationTrajectoryData, slerpReference
        return refTimeTrajectoryData, refPoseTrajectoryData, refOrientationTrajectoryData


    ##############################################################################################
    ##############################################################################################
    ##############################################################################################
    def printEeVelocity(self, eeIndex=4, usefullSamples=None, showReference=False):
        if usefullSamples == None:
            usefullSamples = self._usefullSamples

        startMpcRecordingTime = {}
        endMpcRecordingTime = {}
        recordingDuration = {}
        for fl_num, fl in enumerate(self._bagFile):  # loop over all bag files
            experimentName = self._experimentName[fl_num]
            startMpcRecordingTime.update({experimentName: 0})
            endMpcRecordingTime.update({experimentName: 0})
            recordingDuration.update({experimentName: 0})
            # check start and end of recording mpc time
            for topic, msg, t in fl.read_messages(topics=[self._mpcObservationTopic]):
                startMpcRecordingTime[experimentName] = msg.time
                break
            for topic, msg, t in fl.read_messages(topics=[self._mpcObservationTopic]):
                endMpcRecordingTime[experimentName] = msg.time
            recordingDuration[experimentName] = endMpcRecordingTime[experimentName] - startMpcRecordingTime[
                experimentName]
            print("Recording started at mpc time: ", startMpcRecordingTime[experimentName],
                  " and finished at mpc time: ", endMpcRecordingTime[experimentName])
            print("Recording duration: ", recordingDuration[experimentName])

        coordinates = ['x', 'y', 'z']
        # loop over EEs
        eeNumber = len(self._eesReferenceTopic)
        eeFrameName = ['arm1_8']
        for ee in range(eeNumber):
            # get EE trj from /currentState topic
            eePoseTrjData = {}
            timeData = {}
            eeVelocityData = {}
            for fl_num, fl in enumerate(self._bagFile):  # loop over all bag files
                experimentName = self._experimentName[fl_num]
                eePoseTrj = []
                time = []
                eeVelocity = []
                for topic, msg, t in fl.read_messages(topics=[self._eesTrajectoryTopic[ee]]):
                    eePoseTrj.append(msg.markers[2 * eeIndex].pose)

                    if len(time) == 0:  # time
                        initialTime = t.to_sec()
                    time.append(t.to_sec() - initialTime)

                    if len(eePoseTrj) > 1:
                        dt = t.to_sec() - t_prev.to_sec()
                        dt = max(dt, 0.1)
                        eeVelocity.append([(msg.markers[2 * eeIndex].pose.position.x - prev_position[0]) / dt,
                                           (msg.markers[2 * eeIndex].pose.position.y - prev_position[1]) / dt,
                                           (msg.markers[2 * eeIndex].pose.position.z - prev_position[2]) / dt])

                    prev_position = [msg.markers[2 * eeIndex].pose.position.x,
                                     msg.markers[2 * eeIndex].pose.position.y,
                                     msg.markers[2 * eeIndex].pose.position.z]
                    t_prev = t

                eePoseTrjData.update(
                    {experimentName: eePoseTrj[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]})
                timeData.update({experimentName: time[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]})
                eeVelocityData.update({experimentName: eeVelocity[usefullSamples[fl_num][0]:usefullSamples[fl_num][1]]})

                # get tf fromself._bagFile tf topic
                # eeTfPosition = self.getTfFromBagFile('odometry/world', eeFrameName[ee])

            # row and column labels

            fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(14, 8))
            matplotlib.rc('xtick', labelsize=25)
            matplotlib.rc('ytick', labelsize=25)

            for fl_num, fl in enumerate(self._bagFile):  # loop over all bag files
                experimentName = self._experimentName[fl_num]

                # loop over coordinates
                for coord, coord_str in enumerate(coordinates):
                    axes[coord].plot(  # timeData[experimentName],
                        [kk[coord] for kk in eeVelocityData[experimentName]],
                        linewidth=2, label='ee_mpc_observation ' + experimentName)  # ee trj
                    axes[coord].grid()
                    axes[coord].legend()
                    axes[coord].set_ylabel(coord_str, rotation=90, fontsize=20)
                    # labels
                    axes[-1].set(xlabel='Time (sec)')
                    # axes[coord].set_title('MPC State Observation', size='large', y=1)
                    plt.suptitle('EE trajectory tracking ' + experimentName, y=0.99, fontsize=20)  # title
                    # fig.tight_layout()
                    fig.subplots_adjust(top=0.9, left=0.1, right=0.98)

        plt.show()


if __name__ == "__main__":

   # bagFileInterface = BagFileInterface('/home/idadiotis/Desktop/experiments/27Mar23/_2023-03-27-18-55-49.bag',
   #                                     'Random experiment',
   #                                     '/xbotcore/joint_states', '/xbotcore/command', '/xbotcore/command/torque_terms',
   #                                     "/legged_robot/currentState", True, True, [0, -1])
   # bagFileInterface.printTorques()
   # bagFileInterface.printTorqueTerms()
   # bagFileInterface.printVelocities()
   # bagFileInterface.printPositions()
   filepath = "/home/idadiotis/Desktop/ocs2_experiments/15July2023/tf_2023-07-17-15-47-04.bag"
   time, tfList = getMessagesFromTopic("tf_odometry/world_dagana_2_tcp", filepath)
   filepath = "/home/idadiotis/Desktop/ocs2_experiments/15July2023/xbotcore_2023-07-15-12-09-24.bag"
   time_js, js = getMessagesFromTopic("/xbotcore/joint_states", filepath)
   for i in range(10):
       print(time[i], time_js[i])