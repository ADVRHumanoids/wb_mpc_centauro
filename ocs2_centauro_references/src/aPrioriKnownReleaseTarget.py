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
import yaml
import rospkg
import tf
from tf.transformations import euler_from_quaternion

if __name__ == "__main__":

    rospy.init_node('apriori_known_targets', anonymous=False)

    # get params of references
    scenario = rospy.get_param("~scenario")
    # scenario = "dagana_grasp"

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

    # receive tf of camera frame wrt world
    listener = tf.TransformListener()
    rospy.sleep(1)

    # tf broadcaster from target frame in case needed
    targetBroadcaster = tf.TransformBroadcaster()

    # if the dagana has to release the object at the end
    # ASSUMPTION: table laterally, not related with aruco
    if releasingPhase['active']:
        tfRate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            try:
                # get ref frame of release wrt world
                (eeTrans, eeRot) = listener.lookupTransform(globalFrame, releasingPhase['refFrame'], rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            T_WRRef = np.matmul(tf.transformations.translation_matrix(eeTrans),  # tf of ref frame of release wrt world
                                tf.transformations.quaternion_matrix(eeRot))
            T_RRefR = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(
               releasingPhase['orientationYPR'][0],
               releasingPhase['orientationYPR'][1],
               releasingPhase['orientationYPR'][2], 'szyx')
            )
            T_RRefR[:-1, -1] = releasingPhase['relWrtRefFrame'] + [releasingPhase['table_height'] - eeTrans[2]]
            # (yaw, pitch, roll) = tf.transformations.euler_from_matrix(T_RRefR, 'szyx')
            # print(yaw, pitch, roll)

            # tf of release frame wrt world
            T_WR = np.matmul(T_WRRef, T_RRefR)
            releasePoseTfName = eeNames[armIndex] + "_release"
            targetBroadcaster.sendTransform(T_WR[:-1, -1],
                                            tf.transformations.quaternion_from_matrix(T_WR),
                                            rospy.Time.now(),
                                            releasePoseTfName,
                                            globalFrame)
            tfRate.sleep()

    exit()
