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
import rospy
import tf
import yaml
import rospkg

from ocs2_msgs.msg import mpc_state
from ocs2_msgs.msg import mpc_input
from ocs2_msgs.msg import mpc_target_trajectories
from ocs2_msgs.msg import mpc_observation

'''
    Publish a simple reference for arm ee, for now only position
'''

observationTime = 0.0
observationMode = 15
'''
    Get mpc observation message.
'''
def observationCallback(data):
    if not rospy.is_shutdown():
        global observationTime, observationMode
        observationTime = data.time
        observationMode = data.mode


if __name__ == "__main__":

    rospy.init_node('eeFreeMotionReferencePublisher', anonymous=False)

    # load params from yaml file
    r = rospkg.RosPack()
    print(r)
    paramPath = r.get_path('ocs2_centauro_references') + '/yaml/ee_free_motion.yaml'
    with open(paramPath) as info:
        scenarioParams = yaml.safe_load(info)
    locals().update(scenarioParams)

    observationSub = rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation, observationCallback)
    targetPub = rospy.Publisher(eeTargetTopics[armIndex], mpc_target_trajectories, queue_size=1)

    listener = tf.TransformListener()
    rospy.sleep(1)

    motionsToExecute = len(targetsWrtRefFrame)
    motionsRate = rospy.Rate(0.15)
    # loop over the number of motions to execute
    for motionNum in range(motionsToExecute):

        # get robot's state
        rate = rospy.Rate(4000.0)
        while not rospy.is_shutdown():
            try:
                # get EE frame
                (eeTrans, eeRot) = listener.lookupTransform(globalFrame, eeNames[armIndex], rospy.Time(0))

                # target wrt polygon center
                if targetRefFrame == "polygon_center":
                    # get feet contact position
                    contactsTrans = []
                    contactsRot = []
                    for i in range(1, 5):
                        (contactTrans, contactRot) = listener.lookupTransform(globalFrame, 'contact_' + str(i),
                                                                              rospy.Time(0))
                        contactsTrans.append(np.array(contactTrans))
                        contactsRot.append(np.array(contactRot))

                    # mean feet contacts
                    refFrameTrans = sum(contactsTrans) / 4

                    # consider base link orientation as ref frame orientation
                    (BaseTrans, BaseRot) = listener.lookupTransform(globalFrame, 'base_link',
                                                                      rospy.Time(0))
                    refFrameRot = BaseRot
                    break
                # target wrt to reference frame
                else:
                    (refFrameTrans, refFrameRot) = listener.lookupTransform(globalFrame, targetRefFrame,
                                                                          rospy.Time(0))
                    # consider base link orientation as ref frame orientation
                    (BaseTrans, BaseRot) = listener.lookupTransform(globalFrame, 'base_link',
                                                                    rospy.Time(0))
                    refFrameRot = BaseRot
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            rate.sleep()
        # rotate target wrt globalFrame
        p_WRf = tf.transformations.translation_matrix(refFrameTrans)  # tf camera wrt world
        R_WRf = tf.transformations.quaternion_matrix(refFrameRot)

        # calculate absolute final target
        finalTarget = refFrameTrans + np.matmul(R_WRf[:3, :3], np.array(targetsWrtRefFrame[motionNum]))
        displacement = finalTarget - eeTrans

        if cutInSegments:
            segmentNum = 20
            segmentDisplacement = displacement/segmentNum
        else:
            segmentNum = 1
            segmentDisplacement = displacement / segmentNum

        # wait for connection
        waitConnectionRate = rospy.Rate(4000)

        # first check that there is at least a connection
        ctrl_c = False
        while not ctrl_c:
            connections = targetPub.get_num_connections()
            # print('connections = ', connections)
            if connections > 0:
                # publish rate
                slow_mean_velocity = 0.1
                publishRate = rospy.Rate(0.8 * slow_mean_velocity/np.linalg.norm(segmentDisplacement))  # half of time to wait

                for i in range(segmentNum):
                    # create msg
                    msg = mpc_target_trajectories()

                    if interpolateReference:      # include intiial for interpolation
                        initialState = mpc_state()
                        initialInput = mpc_input()

                        # fill msg
                        initialState.value = eeTrans + eeRot
                        initialInput.value = [0.0]  # uselesss
                        msg.stateTrajectory.append(initialState)
                        msg.inputTrajectory.append(initialInput)
                        msg.timeTrajectory.append(observationTime)

                    # create msg
                    state = mpc_state()
                    input = mpc_input()
                    # markerMsg = InteractiveMarkerUpdate()

                    # fill msg
                    eeTrans += np.array(segmentDisplacement)
                    state.value = eeTrans.tolist() + eeRot
                    input.value = [0.0]     # uselesss
                    msg.stateTrajectory.append(state)
                    msg.inputTrajectory.append(input)
                    msg.timeTrajectory.append(observationTime + np.linalg.norm(segmentDisplacement)/slow_mean_velocity)

                    targetPub.publish(msg)
                    print("published msg", msg)
                    ctrl_c = True
                    publishRate.sleep()
            else:
                waitConnectionRate.sleep()

        print('Completed motion ', motionNum+1)
        motionsRate.sleep()

    exit()