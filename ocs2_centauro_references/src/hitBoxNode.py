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

    rospy.init_node('hit_box_publisher', anonymous=False)

    # get params of references
    eeFrame = rospy.get_param("~ee_frame")
    globalFrame = rospy.get_param("~global_frame")
    targetTopic = rospy.get_param("~target_topic")
    relative_target = rospy.get_param("~relative_target")
    box_point = rospy.get_param("~box_point")
    durations = rospy.get_param("~durations")
    contact_events = rospy.get_param("~contact_events")
    contact_velocity = rospy.get_param("~contact_velocity")
    keep_contact = rospy.get_param("~keep_contact")

    # convert to list
    relative_target = relative_target.rstrip(']').lstrip('[').split(',')
    relative_target = [float(i) for i in relative_target]
    box_point = box_point.rstrip(']').lstrip('[').split(',')
    box_point = [float(i) for i in box_point]
    durations = durations.rstrip(']').lstrip('[').split(',')
    durations = [float(i) for i in durations]
    contact_events = contact_events.rstrip(']').lstrip('[').split(',')
    contact_events = [float(i) for i in contact_events]
    contact_velocity = contact_velocity.rstrip(']').lstrip('[').split(',')
    contact_velocity = [float(i) for i in contact_velocity]

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
    ballEeRadius = 0.07
    targetSurfacePosition = [box_point[0] - ballEeRadius, eeTrans[1], eeTrans[2]]
    targetSurfacePose = targetSurfacePosition + eeRot

    # calculate absolute final target
    targetAbsolutePose = (eeTrans + np.array(relative_target)).tolist() + eeRot

    # create msg
    msg = mpc_target_trajectories()
    state = mpc_state()
    state2 = mpc_state()
    input = mpc_input()

    # fill msg
    # contact switch
    state.value = targetSurfacePose
    input.value = [1.0, 0.0, 0.0]
    msg.stateTrajectory.append(state)
    msg.inputTrajectory.append(input)
    msg.timeTrajectory.append(observationTime + durations[0])

    # final target
    state2.value = targetAbsolutePose
    msg.stateTrajectory.append(state2)
    msg.inputTrajectory.append(input)
    msg.timeTrajectory.append(observationTime + durations[1])

    # new mode message
    modeMsg = mode_schedule()
    if not keep_contact:
        modeMsg.modeSequence = [47, 15]
        modeMsg.eventTimes = [contact_events[0] - 1.0, contact_events[1] - 1.0, contact_events[1] + 500]
    else:
        modeMsg.modeSequence = [47]
        modeMsg.eventTimes = [contact_events[0] - 1.0, contact_events[1] - 1.0]
    modePub.publish(modeMsg)

    targetPub.publish(msg)

    print("Published ", eeFrame, " to hit surface at position ", box_point, " in ", durations[0], " sec with contact velocity ", contact_velocity)
    print("Final target of ", eeFrame, " is ", targetAbsolutePose[:3], ' in ', durations[1], ' sec')
    print("New contact mode published ", modeMsg.modeSequence, " \n", modeMsg.eventTimes)
