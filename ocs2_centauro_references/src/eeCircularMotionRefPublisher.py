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
import math

import matplotlib.pyplot as plt
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
    Publish a circular reference for arm ee, for now only position
'''


class CircularMotion(object):
    def __init__(self, circleCenter, radius, singleRotationPeriod, rotationAxis=0):
        self._radius = radius
        self._center = circleCenter
        self._period = singleRotationPeriod
        self._omega = 2 * math.pi / singleRotationPeriod
        self._rotationAxis = rotationAxis     # 0, 1, 2 for x, y, z

    def getCircularCoordinates(self, time):
        if self._rotationAxis == 0:
            x = self._center[0]
            y = self._center[1] + self._radius * math.cos(self._omega * time)
            z = self._center[2] + self._radius * math.sin(self._omega * time)
        elif self._rotationAxis == 1:
            x = self._center[0] + self._radius * math.cos(self._omega * time)
            y = self._center[1]
            z = self._center[2] + self._radius * math.sin(self._omega * time)
        elif self._rotationAxis == 2:
            x = self._center[0] + self._radius * math.cos(self._omega * time)
            y = self._center[1] + self._radius * math.sin(self._omega * time)
            z = self._center[2]
        return [x, y, z]

    def printCircularMotion(self):
        pointsXList, pointsYList, pointsZList = ([] for i in range(3))
        numSegments = 1000
        dt = self._period / numSegments
        for i in range(1000):
            point = self.getCircularCoordinates(i * dt)
            pointsXList.append(point[0])
            pointsYList.append(point[1])
            pointsZList.append(point[1])

        plt.figure()
        plt.plot(pointsXList, pointsYList)
        plt.show()


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
    rospy.init_node('eeCircularMotionReferencePublisher', anonymous=False)

    # load params from yaml file
    r = rospkg.RosPack()
    print(r)
    paramPath = r.get_path('ocs2_centauro_references') + '/yaml/ee_circular_motion.yaml'
    with open(paramPath) as info:
        scenarioParams = yaml.safe_load(info)
    locals().update(scenarioParams)

    observationSub = rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation, observationCallback)
    targetPub = rospy.Publisher(eeTargetTopics[armIndex], mpc_target_trajectories, queue_size=1)

    listener = tf.TransformListener()
    rospy.sleep(1)

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

    # define circular motion
    print("dagana tcp = ", p_WRf[:3, -1])
    circleCenterWrtWorld = np.array(refFrameTrans) + np.array(circleCenter)
    circleMotion = CircularMotion(circleCenterWrtWorld.tolist(), circleRadius, rotationPeriod)

    # wait for connection
    waitConnectionRate = rospy.Rate(4000)

    # first check that there is at least a connection
    ctrl_c = False
    while not ctrl_c:
        connections = targetPub.get_num_connections()
        # print('connections = ', connections)
        if connections > 0:
            # publish rate
            freq = 1000
            publishRate = rospy.Rate(freq)  # half of time to wait
            pointNum = freq * circleMotion._period
            trj = []
            initialTime = observationTime
            for i in range(pointNum):
                timeInstant = i * (circleMotion._period/pointNum)
                print(timeInstant)
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

                # fill msg
                pointXYZ = circleMotion.getCircularCoordinates(timeInstant)
                trj.append(pointXYZ)
                state.value = pointXYZ + eeRot
                input.value = [0.0]     # uselesss
                msg.stateTrajectory.append(state)
                msg.inputTrajectory.append(input)
                msg.timeTrajectory.append(initialTime + timeInstant)

                targetPub.publish(msg)
                print("published msg", msg)
                ctrl_c = True
                publishRate.sleep()
        else:
            waitConnectionRate.sleep()

    # plt.figure()
    # plt.plot([k[0] for k in trj], [k[1] for k in trj])
    # plt.title('xy')
    #
    # plt.figure()
    # plt.plot([k[1] for k in trj], [k[2] for k in trj])
    # plt.title('yz')
    #
    # plt.show()
    exit()