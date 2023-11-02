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

#!/usr/bin/env python3
import numpy as np
import sys
import rospy
import tf
import yaml
import rospkg

from visualization_msgs.msg import Marker
from ocs2_msgs.msg import mpc_state
from ocs2_msgs.msg import mpc_input
from ocs2_msgs.msg import mpc_target_trajectories
from ocs2_msgs.msg import mpc_observation
from ocs2_msgs.msg import mode_schedule
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_matrix


'''
    Publish a simple reference or a sequence of references of an arm EE in ocs2 for a desired scenario
'''

observationTime = 0.0
observationMode = 15
daganaPosition = 0
daganaEffort = 0


def observationCallback(data):
    '''
    Observes current mode (contact state) and time from the ocs2-based MPC.
    '''
    if not rospy.is_shutdown():
        global observationTime, observationMode
        observationTime = data.time
        observationMode = data.mode


def daganaStateCallback(data):
    '''
    Observes dagana's state, i.e. position and effort
    '''
    if not rospy.is_shutdown():
        global daganaPosition, daganaEffort
        daganaPosition = data.position[0]
        daganaEffort = data.effort[0]


def fromTfToOcs2TargetMsg(listener, referenceFrame, frameName, timeAhead=0.0):
    '''
    Receives the tf of a frame wrt to a reference frame and converts to ocs2-related messages
    :param listener: object to listen for tf
    :param referenceFrame: reference frame of the requested transformation
    :param frameName: frame of which the tf is requested
    :param timeAhead: time at which this tf should be reached (for already achieved tf timeAhead = 0)
    :return: a directory with elements ocs2-based messages: mpc_state, mpc_input, time
    '''
    listenRate = rospy.Rate(1000.0)
    # get current ee frame pose wrt world and include it in the message, then inside the mpc cost
    # the current and target will be interpolated
    while True:
        try:
            (eeCurrentTrans, eeCurrentRot) = listener.lookupTransform(referenceFrame, frameName, rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    p_WC = tf.transformations.translation_matrix(eeCurrentTrans)
    R_WC = tf.transformations.quaternion_matrix(eeCurrentRot)
    T_WC = np.matmul(p_WC, R_WC)
    # quaternion
    quat_WC = quaternion_from_matrix(T_WC)

    # create and set msg
    state = mpc_state()
    input = mpc_input()
    state.value = T_WC[:-1, -1].tolist() + quat_WC.tolist()
    input.value = [0.0, 0.0, 0.0]
    targetDirectory = {
        'state': state,
        'input': input,
        'time': observationTime + timeAhead     # although later is overwritten
    }
    return targetDirectory


def fromTargetTrajectoriesToTargetMessage(currentTrajectory, targetTrajectory, motionDuration, includeCurrent=True):
    '''
    Creates an target trajectory message for ocs2-based mpc from currentTrajectory (if selected) and targetTrajectory
    :param currentTrajectory: a directory from fromTfToOcs2TargetMsg and refers to current EE Pose (already achieved)
    from which the mpc motion of the EE will start
    :param targetTrajectory: a directory from fromTfToOcs2TargetMsg and refers to target EE Pose.
    :param motionDuration: the duration of the motion to be sent
    :param includeCurrent: if True the currentTrajectory will be included in the final message
    :return: an mpc_target_trajectories() message for a motion of an arm EE.
    '''
    message = mpc_target_trajectories()
    if includeCurrent:
        message.stateTrajectory = [currentTrajectory['state'], targetTrajectory['state']]
        message.inputTrajectory = [currentTrajectory['input'], targetTrajectory['input']]
        message.timeTrajectory = [observationTime, observationTime + motionDuration]
    else:
        message.stateTrajectory = [targetTrajectory['state']]
        message.inputTrajectory = [targetTrajectory['input']]
        message.timeTrajectory = [observationTime + motionDuration]

    return message


def closeDagana(gripperDetails, publisher):
    '''
    Takes care of closing the dagana gripper by publishing reference position and effort
    :param gripperDetails: contains information regarding the reference position and effort to be achieved on the motor
    :param publisher: the ros publisher object for sending commands to the gripper
    '''
    daganaRefRate = rospy.Rate(1000.0)
    posTrajectory = np.linspace(daganaPosition, gripperDetails['refGraspingPosition'], 1000).tolist()
    tauTrajectory = np.linspace(daganaEffort, gripperDetails['refGraspingEffort'], 1000).tolist()
    for posPointNum in range(len(posTrajectory)):
        daganaMsg = JointState()
        daganaMsg.position.append(posTrajectory[posPointNum])
        publisher.publish(daganaMsg)
        daganaRefRate.sleep()
    for tauPointNum in range(len(tauTrajectory)):
        daganaMsg = JointState()
        daganaMsg.effort.append(tauTrajectory[tauPointNum])
        publisher.publish(daganaMsg)
        daganaRefRate.sleep()
    print("If gripper is closed, press a key!")
    something = input()
    print("You pressed " + something)


def openDagana(gripperDetails, publisher):
    '''
    Takes care of opening the dagana gripper by publishing position (only for now)
    :param gripperDetails: contains info regarding the reference position for the open dagana
    :param publisher: the ros publisher object for sending commands to the gripper
    '''
    daganaRefRate = rospy.Rate(1000.0)
    posTrajectory = np.linspace(daganaPosition, gripperDetails['refReleasingPosition'], 1000).tolist()
    tauTrajectory = np.linspace(daganaEffort, gripperDetails['refReleasingEffort'], 1000).tolist()
    for posPointNum in range(len(posTrajectory)):
        daganaMsg = JointState()
        daganaMsg.position.append(posTrajectory[posPointNum])
        publisher.publish(daganaMsg)
        daganaRefRate.sleep()
#    for tauPointNum in range(len(tauTrajectory)):
#        daganaMsg = JointState()
#        daganaMsg.effort.append(tauTrajectory[tauPointNum])
#        publisher.publish(daganaMsg)
#        daganaRefRate.sleep()
    print("Gripper should be open! Continuing..")


'''
    STANCE          15
    LA_STANCE       47
    RA_STANCE       31
    LA_RA_STANCE    63
'''
def switchContactAtArm(armIndex, modePublisher):
    '''
    Takes care of sending a message to ocs2-based mpc for switching (make or break) an arm contact.
    :param armIndex: the index of the arm to make/break contact, is 0 or 1 since centauro robot has two arms
    :param modePublisher: the ros publisher object for sending contact (mode) changing commands to the mpc
    Contact states of all contact points of the robot are represented with a mode (denoted with an integer number)
    just like above:
    STANCE          15      (all leg EEs in contact)
    LA_STANCE       47      (all leg EEs and left arm EE in contact)
    RA_STANCE       31      (all leg EEs and right arm in contact)
    LA_RA_STANCE    63      (all leg and arm EEs in contact)
    '''
    newModeMsg = mode_schedule()
    if observationMode == 15:                           # currently STANCE
        if armIndex == 0:
            newModeMsg.modeSequence = [47]                  # make LA_STANCE
        elif armIndex == 1:
            newModeMsg.modeSequence = [31]                  # make RA_STANCE
    elif observationMode == 47:                        # currently LA_STANCE
        if armIndex == 0:
            newModeMsg.modeSequence = [15]                  # make STANCE
        elif armIndex == 1:
            newModeMsg.modeSequence = [63]                  # make LA_RA_STANCE
    elif observationMode == 31:                        # currently RA_STANCE
        if armIndex == 0:
            newModeMsg.modeSequence = [63]                  # make STANCE
        elif armIndex == 1:
            newModeMsg.modeSequence = [15]                  # make LA_RA_STANCE
    else:
        print("Did not define properly a new gait! Check the commanded modeSequence.")
    newModeMsg.eventTimes = [0.0, 0.5]
    modePublisher.publish(newModeMsg)


if __name__ == "__main__":

    rospy.init_node('aruco_to_mpc_target_trajectories', anonymous=False)

    # get params of references
    arucoTargetTopic = rospy.get_param("~aruco_target_topic")
    leftContactSwitch = rospy.get_param("~contact_switch1")
    rightContactSwitch = rospy.get_param("~contact_switch2")
    leftDurations = rospy.get_param("~durations1")
    rightDurations = rospy.get_param("~durations2")
    scenario = rospy.get_param("/aruco_based_targets/scenario")     # case of publish to a single arm
    include_waypoint = False

    # convert strings to lists
    leftDurations = leftDurations.rstrip(']').lstrip('[').split(',')
    leftDurations = [float(i) for i in leftDurations]
    rightDurations = rightDurations.rstrip(']').lstrip('[').split(',')
    rightDurations = [float(i) for i in rightDurations]

    # load params from yaml file
    r = rospkg.RosPack()
    if scenario == "dagana_grasp":
        paramPath = r.get_path('ocs2_centauro_references') + '/yaml/' + scenario + '.yaml'
    elif scenario == "bimanual_box_grasp":
        paramPath = r.get_path('ocs2_centauro_references') + '/yaml/' + scenario + '.yaml'
    else:
        sys.exit("[sys.exit] Did not received an existed scenario")
    with open(paramPath) as info:
        scenarioParams = yaml.safe_load(info)
    locals().update(scenarioParams)

    # subscribers & publishers
    observationSub = rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation, observationCallback)
    leftTargetPub = rospy.Publisher(leftTargetTopic, mpc_target_trajectories, queue_size=1)
    rightTargetPub = rospy.Publisher(rightTargetTopic, mpc_target_trajectories, queue_size=1)
    targetPubList = [leftTargetPub, rightTargetPub]
    modePub = rospy.Publisher("/legged_robot_mpc_mode_schedule", mode_schedule, queue_size=1)

    listener = tf.TransformListener()
    rospy.sleep(1)

    # first get current ee poses
    initialTrajectoryDirs = [fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[0], timeAhead=0.0),
                             fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[1], timeAhead=0.0)]
    currentTrajectoryDirs = initialTrajectoryDirs.copy()

    #################################################
    ################## DAGANA GRASP #################
    #################################################
    if scenario == "dagana_grasp":
        daganaSub = rospy.Subscriber("/xbotcore/gripper/dagana_2/state", JointState, daganaStateCallback)
        daganaPub = rospy.Publisher("/xbotcore/gripper/dagana_2/command", JointState, queue_size=1)

        # get all possible targets
        reachTargetDir = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[armIndex] + "_target", timeAhead=reachingPhase['reachTime'])
        if approachingPhase['active']:
            approachTargetDir = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[armIndex] + "_approach", timeAhead=approachingPhase['approachTime'])
        if supportSurface['active']:
            supportTargetDir = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[supportSurface['armIndex']] + "_support", timeAhead=supportSurface['reachTime'])

        ############ support phase ###############
        if supportSurface['active']:
            if supportSurface['interpolateReference']:
                # get current ee frame pose wrt world and include it in the message
                currentTrajectoryDirs[supportSurface['armIndex']] = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[supportSurface['armIndex']], timeAhead=0.0)
                supportTargetMsg = fromTargetTrajectoriesToTargetMessage(currentTrajectoryDirs[supportSurface['armIndex']], supportTargetDir, supportSurface['reachTime'], True)
            else:
                supportTargetMsg = fromTargetTrajectoriesToTargetMessage(None, supportTargetDir, supportSurface['reachTime'], False)
            targetPubList[supportSurface['armIndex']].publish(supportTargetMsg)
            print("Published support pose: ", supportTargetMsg, "Press sth if you want reach the target.")
            something = input()
            print("You pressed " + something)

            # if robot was moved get again current tfs
            currentTrajectoryDirs[armIndex] = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[armIndex], timeAhead=0.0)

        ############ open gripper ###############
        if gripperControl['active']:
            openDagana(gripperDetails=gripperControl, publisher=daganaPub)

        ############ approaching phase ###############
        if approachingPhase['active']:
            if approachingPhase['interpolateReference']:
                currentTrajectoryDirs[armIndex] = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[armIndex], timeAhead=0.0)
                approachTargetMsg = fromTargetTrajectoriesToTargetMessage(currentTrajectoryDirs[armIndex], approachTargetDir, approachingPhase['approachTime'], True)
            else:
                approachTargetMsg = fromTargetTrajectoriesToTargetMessage(None, approachTargetDir, approachingPhase['approachTime'], False)
            # publish msg
            targetPubList[armIndex].publish(approachTargetMsg)
            print("Published approach pose: ", approachTargetMsg, "Press sth if you want reach the target.")
            something = input()
            print("You pressed " + something)

        ############ reaching phase ###############
        if reachingPhase['active']:
            if reachingPhase['interpolateReference']:
                currentTrajectoryDirs[armIndex] = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[armIndex], timeAhead=0.0)
                reachTargetMsg = fromTargetTrajectoriesToTargetMessage(currentTrajectoryDirs[armIndex], reachTargetDir, reachingPhase['reachTime'], True)
            else:
                reachTargetMsg = fromTargetTrajectoriesToTargetMessage(None, reachTargetDir, reachingPhase['reachTime'], False)
            targetPubList[armIndex].publish(reachTargetMsg)
            print("Published pose: ", reachTargetMsg, "\n If desired target has been reached, press a key to continue!")
            something = input()
            print("You pressed " + something)

        ############ close gripper ###############
        if gripperControl['active']:
            closeDagana(gripperDetails=gripperControl, publisher=daganaPub)

        ############ retun to initial pose phase ###############
        if returnToInitialPhase['active']:
            # contact switch since object is grasped
            if contactSwitch:
                switchContactAtArm(armIndex=armIndex, modePublisher=modePub)
                rospy.sleep(1.5)  # just wait an OCP horizon to coincide with contact switch

            if returnToInitialPhase['interpolateReference']:
                currentTrajectoryDirs[armIndex] = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[armIndex], timeAhead=0.0)
                returnTargetMsg = fromTargetTrajectoriesToTargetMessage(currentTrajectoryDirs[armIndex], initialTrajectoryDirs[armIndex], returnToInitialPhase['reachTime'], True)
            else:
                returnTargetMsg = fromTargetTrajectoriesToTargetMessage(None, initialTrajectoryDirs[armIndex], returnToInitialPhase['reachTime'], False)
            targetPubList[armIndex].publish(returnTargetMsg)
            print("Published return pose: ", returnTargetMsg, ". Press a key to continue with release if selected")
            something = input()
            print("You pressed " + something)

        ############ release phase ###############
        if releasingPhase['active']:
            # get now the release target because this is defined wrt to current pelvis pose
            releasingTargetDir = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[armIndex] + "_release", timeAhead=releasingPhase['reachTime'])
            if releasingPhase['interpolateReference']:
                currentTrajectoryDirs[armIndex] = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[armIndex], timeAhead=0.0)
                releaseTargetMsg = fromTargetTrajectoriesToTargetMessage(currentTrajectoryDirs[armIndex], releasingTargetDir, releasingPhase['reachTime'], True)
            else:
                releaseTargetMsg = fromTargetTrajectoriesToTargetMessage(None, releasingTargetDir, releasingPhase['reachTime'], False)
            targetPubList[armIndex].publish(releaseTargetMsg)
            print("Published release pose: ", releaseTargetMsg, "If release pose is achieved, press a key to release the object!")
            something = input()
            print("You pressed " + something)

            # contact switch since object is released
            if contactSwitch:
                switchContactAtArm(armIndex=armIndex, modePublisher=modePub)
                rospy.sleep(0.9)  # just wait an OCP horizon to coincide with contact switch

            ############ open gripper ###############
            if gripperControl['active']:
                openDagana(gripperDetails=gripperControl, publisher=daganaPub)
            print("If object released, press a key to continue!")
            something = input()
            print("You pressed " + something)

            ############ return to initial pose phase (after releasing the object) ###############
            if returnToInitialPhase['active']:
                if returnToInitialPhase['interpolateReference']:
                    currentTrajectoryDirs[armIndex] = fromTfToOcs2TargetMsg(listener, globalFrame, eeNames[armIndex], timeAhead=0.0)
                    returnTargetMsg = fromTargetTrajectoriesToTargetMessage(currentTrajectoryDirs[armIndex], initialTrajectoryDirs[armIndex],
                                                                            0.5 * returnToInitialPhase['reachTime'], True)
                else:
                    returnTargetMsg = fromTargetTrajectoriesToTargetMessage(None, initialTrajectoryDirs[armIndex],
                                                                            0.5 * returnToInitialPhase['reachTime'], False)
                # returnMsg.timeTrajectory.append(timeAchievedTarget + 0.5 * returnToInitialPhase['reachTime'])
                targetPubList[armIndex].publish(returnTargetMsg)
                print("Published return pose: ", returnTargetMsg, ". Finished.")

    #################################################
    ############## BIMANUAL BOX GRASP ###############
    #################################################
    elif scenario == "bimanual_box_grasp":

        # approach phase
        for armIndex, arm_i in enumerate(eeNames):
            ############ approaching phase ###############
            if approachingPhase['active']:
                approachTargetDir = fromTfToOcs2TargetMsg(listener, globalFrame, arm_i + "_approach",
                                                          timeAhead=approachingPhase['approachTime'])
                if approachingPhase['interpolateReference']:
                    currentTrajectoryDirs[armIndex] = fromTfToOcs2TargetMsg(listener, globalFrame, arm_i,
                                                                            timeAhead=0.0)
                    approachTargetMsg = fromTargetTrajectoriesToTargetMessage(currentTrajectoryDirs[armIndex],
                                                                              approachTargetDir,
                                                                              approachingPhase['approachTime'], True)
                else:
                    approachTargetMsg = fromTargetTrajectoriesToTargetMessage(None, approachTargetDir,
                                                                              approachingPhase['approachTime'], False)
                # publish msg
                targetPubList[armIndex].publish(approachTargetMsg)
                print("Published approach pose: ", approachTargetMsg)
        print("Press sth if you want to grasp the box.")
        something = input()
        print("You pressed " + something)

        # grasping phase
        for armIndex, arm_i in enumerate(eeNames):
            graspingTargetDir = fromTfToOcs2TargetMsg(listener, globalFrame, arm_i + "_grasp",
                                                      timeAhead=graspingPhase['time'])

            if graspingPhase['interpolateReference']:
                currentTrajectoryDirs[armIndex] = fromTfToOcs2TargetMsg(listener, globalFrame, arm_i,
                                                                        timeAhead=0.0)
                graspingTargetMsg = fromTargetTrajectoriesToTargetMessage(currentTrajectoryDirs[armIndex],
                                                                          graspingTargetDir,
                                                                          graspingPhase['time'], True)
            else:
                graspingTargetMsg = fromTargetTrajectoriesToTargetMessage(None, graspingTargetDir,
                                                                          graspingPhase['time'], False)

            # publish msg
            targetPubList[armIndex].publish(graspingTargetMsg)
            print("Published msg: ", graspingTargetMsg)

        if liftingPhase['active']:
            print("Press sth if you want to continue to lifting.")
            something = input()
            print("You pressed " + something)

            # lifting phase
            for armIndex, arm_i in enumerate(eeNames):
                liftingTargetDir = fromTfToOcs2TargetMsg(listener, globalFrame, arm_i + "_lift",
                                                          timeAhead=liftingPhase['time'])
                if liftingPhase['interpolateReference']:
                    currentTrajectoryDirs[armIndex] = fromTfToOcs2TargetMsg(listener, globalFrame, arm_i,
                                                                            timeAhead=0.0)
                    liftingTargetMsg = fromTargetTrajectoriesToTargetMessage(currentTrajectoryDirs[armIndex],
                                                                              liftingTargetDir,
                                                                              liftingPhase['time'], True)
                else:
                    liftingTargetMsg = fromTargetTrajectoriesToTargetMessage(None, liftingTargetDir,
                                                                              liftingPhase['time'], False)

                # publish msg
                targetPubList[armIndex].publish(liftingTargetMsg)
                print("Published msg:", liftingTargetMsg)

        # new mode message
        # if leftContactSwitch:
        #     modeMsg = mode_schedule()
        #     if observationMode == 15:
        #         modeMsg.modeSequence = [47]
        #     elif observationMode == 47:
        #         modeMsg.modeSequence = [15]
        #     modeMsg.eventTimes = [leftDurations[0] - 1.0, leftDurations[0]]
        #     modePub.publish(modeMsg)

