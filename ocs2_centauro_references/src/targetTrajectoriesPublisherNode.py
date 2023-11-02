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

#!/usr/bin/env python
import numpy as np
import rospy
import tf

from visualization_msgs.msg import InteractiveMarkerUpdate
from ocs2_msgs.msg import mpc_state
from ocs2_msgs.msg import mpc_input
from ocs2_msgs.msg import mpc_target_trajectories
from ocs2_msgs.msg import mpc_observation
from ocs2_msgs.msg import mode_schedule

'''
    Publish a simple reference for arm ee in ocs2
'''

observationTime = 0.0
observationMode = 15


def observationCallback(data):
    if not rospy.is_shutdown():
        global observationTime, observationMode
        observationTime = data.time
        observationMode = data.mode


if __name__ == "__main__":

    rospy.init_node('target_trajectories_publisher', anonymous=False)

    # get params of references
    eeFrame = rospy.get_param("~ee_frame")
    globalFrame = rospy.get_param("~global_frame")
    targetTopic = rospy.get_param("~target_topic")
    relative_target = rospy.get_param("~relative_target")
    waypoint = rospy.get_param("~waypoint")
    contact_switch = rospy.get_param("~contact_switch")
    include_waypoint = rospy.get_param("~include_waypoint")
    durations = rospy.get_param("~durations")
    waypoint_velocity = rospy.get_param("~waypoint_velocity")

    # convert to list
    relative_target = relative_target.rstrip(']').lstrip('[').split(',')
    relative_target = [float(i) for i in relative_target]
    waypoint = waypoint.rstrip(']').lstrip('[').split(',')
    waypoint = [float(i) for i in waypoint]
    durations = durations.rstrip(']').lstrip('[').split(',')
    durations = [float(i) for i in durations]
    waypoint_velocity = waypoint_velocity.rstrip(']').lstrip('[').split(',')
    waypoint_velocity = [float(i) for i in waypoint_velocity]

    # eeFrame = "arm1_8"
    # globalFrame = "odom"
    # targetTopic = "/legged_robot_mpc_arm1_8target"
    # relative_target = [0.1, 0.0, 0.2]
    # waypoint = [0.1, 0.0, 0.0]
    # contact_switch = False

    observationSub = rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation, observationCallback)
    targetPub = rospy.Publisher(targetTopic, mpc_target_trajectories, queue_size=1)
    modePub = rospy.Publisher("/legged_robot_mpc_mode_schedule", mode_schedule, queue_size=1)
    listener = tf.TransformListener()
    rospy.sleep(1)

    # get robot's state
    rate = rospy.Rate(4000.0)
    while not rospy.is_shutdown():
        try:
            # get EE frame
            (eeTrans, eeRot) = listener.lookupTransform(globalFrame, eeFrame, rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()

    # calculate absolute final target
    targetAbsolutePose = (eeTrans + np.array(relative_target)).tolist() + eeRot

    # create msg
    msg = mpc_target_trajectories()
    state = mpc_state()
    state2 = mpc_state()
    input = mpc_input()

    # fill msg
    # waypoint
    print("include_waypoint = ", include_waypoint)
    if include_waypoint:
        waypointAbsolutePose = (eeTrans + np.array(waypoint)).tolist() + eeRot
        state.value = waypointAbsolutePose
        input.value = waypoint_velocity
        msg.stateTrajectory.append(state)
        msg.inputTrajectory.append(input)
        msg.timeTrajectory.append(observationTime + durations[0])

    # final target
    state2.value = targetAbsolutePose
    msg.stateTrajectory.append(state2)
    msg.inputTrajectory.append(input)
    msg.timeTrajectory.append(observationTime + durations[1])

    # new mode message
    if contact_switch:
        modeMsg = mode_schedule()
        if observationMode == 15:
            modeMsg.modeSequence = [47]
        elif observationMode == 47:
            modeMsg.modeSequence = [15]
        modeMsg.eventTimes = [durations[0] - 1.0, durations[0]]
        modePub.publish(modeMsg)

    targetPub.publish(msg)

