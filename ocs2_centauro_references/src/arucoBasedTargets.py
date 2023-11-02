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
from fiducial_msgs.msg import FiducialTransformArray

# visualization
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Transform
from std_msgs.msg import Header, ColorRGBA

from tf.transformations import euler_from_quaternion, euler_from_matrix

'''
    Find aruco, define targets wrt to aruco and publish 
'''

# aruco detection related
aruco_parent_frame = "camera_color_optical_frame"
aruco_id = []
T_CM = []
markerPublisher = rospy.Publisher('aruco_based/lateral_targets', Marker, queue_size=1)
MarkerTobeUpdated = []

'''
    Subscribes the mpc time and the mpc observation
'''
def observationCallback(data):
    if not rospy.is_shutdown():
        global observationTime, observationMode
        observationTime = data.time
        observationMode = data.mode


'''
    Subscribes aruco tf wrt camera frame and computes an homogeneous transform as a 4x4 matrix
'''
def arucoCallback(data):
    if not rospy.is_shutdown():
        try:
            global aruco_parent_frame, aruco_id, T_CM, MarkerTobeUpdated
            aruco_parent_frame = data.header.frame_id

            for markerTf in data.transforms:
                tf_CM = markerTf.transform

                # aruco tf wrt camera frame
                p_CM = tf.transformations.translation_matrix(
                    [tf_CM.translation.x, tf_CM.translation.y, tf_CM.translation.z])
                R_CM = tf.transformations.quaternion_matrix(
                    [tf_CM.rotation.x, tf_CM.rotation.y, tf_CM.rotation.z, tf_CM.rotation.w])

                try:            # if aruco marker has been seen before
                    arucoIndex = aruco_id.index(markerTf.fiducial_id)
                    MarkerTobeUpdated[arucoIndex] = True
                    T_CM[arucoIndex] = np.matmul(p_CM, R_CM)
                except:         # aruco marker not seen before
                    aruco_id.append(markerTf.fiducial_id)
                    MarkerTobeUpdated.append(True)
                    T_CM.append(np.matmul(p_CM, R_CM))
                    # print("tf number = ", len(data.transforms))
                    # print("aruco_id = ", aruco_id)
                # print("T_CM = ", T_CM)
                # aruco_id = data.transforms[0].fiducial_id
                # MarkerTobeUpdated[arucoIndex] = True
            # print("MarkerTobeUpdated size = ", MarkerTobeUpdated)
        except:
            pass


'''
    Visualizes left and right target points 
'''
def show_points_in_rviz(marker_publisher, targets_pose_tf, ref_frame):

    marker = Marker(
                ns='aruco_box_command',
                type=Marker.POINTS,
                action=Marker.ADD,
                id=0,
                lifetime=rospy.Duration(),
                pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                scale=Vector3(0.05, 0.05, 0.05),
                header=Header(frame_id=ref_frame),
                color=ColorRGBA(0.0, 0.0, 1.0, 0.8),
                # lifetime=0,
                frame_locked=False)

    for tf in targets_pose_tf:
        pointPosition = Point()
        pointPosition.x = tf[0, -1]
        pointPosition.y = tf[1, -1]
        pointPosition.z = tf[2, -1]

        marker.points.append(pointPosition)

    marker_publisher.publish(marker)


if __name__ == "__main__":

    rospy.init_node('aruco_based_targets', anonymous=False)

    markerSubscriber = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, arucoCallback)
    rospy.sleep(1)

    # get params of references
    scenario = rospy.get_param("~scenario")

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

    arucoRate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            # get camera frame wrt world
            (eeTrans, eeRot) = listener.lookupTransform(globalFrame, aruco_parent_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        T_WT_list = []
        T_WTapp_list = []
        for markerIndex, updateFlag in enumerate(MarkerTobeUpdated):        # loop over found markers
            if updateFlag:                                                  # if marker has to be updated

                p_WC = tf.transformations.translation_matrix(eeTrans)       # tf camera wrt world
                R_WC = tf.transformations.quaternion_matrix(eeRot)
                T_WC = np.matmul(p_WC, R_WC)

                # tf of marker wrt world
                T_WM = np.matmul(T_WC, T_CM[markerIndex])

                # (pitch, yaw, roll) = euler_from_matrix(T_WM[:-1, :-1], axes='syzx')

                # print transformation
                # print("T_WM = ", T_WM)
                # print("markerTrans = ", T_WM[:-1, -1])
                # print("markerRot = ", T_WM[:-1, :-1])
                # print("eeRot euler (pitch, yaw, roll) = ", (pitch, yaw, roll))

                #################################################
                ############## BIMANUAL BOX GRASP ###############
                #################################################
                scenarioIsBox = scenario == "bimanual_box_grasp"     # aruco is vertical facing the robot
                if scenarioIsBox:
                    # check which axis of marker frame is vertical
                    isMyAxisCloserToVerticalThanMx = np.abs(T_WM[1, 1]) < np.abs(T_WM[1, 0])
                    if isMyAxisCloserToVerticalThanMx:
                        horizontalAxisIndex = 0     # the x axis of marker frame is horizontal pointing on the right
                    else:
                        horizontalAxisIndex = 1     # the y axis of marker frame is horizontal pointing on the left

                    # distances from marker origin for approach and grasp motion phases
                    approachDistanceFromMarkerOrigin = boxDimensions[horizontalAxisIndex] / 2 + safetyDistance + ballEeRadius
                    graspDistanceFromMarkerOrigin = boxDimensions[horizontalAxisIndex] / 2 + ballEeRadius - penaltyDistance
                    approachAndGraspDistances = [approachDistanceFromMarkerOrigin, graspDistanceFromMarkerOrigin]
                    motionNames = ['_approach', '_grasp']

                    for motionIndex, desiredDistance in enumerate(approachAndGraspDistances):   # loop over motions
                        # tf of left and right ee targets wrt the marker frame
                        T_MT_l = np.identity(4)
                        T_MT_r = np.identity(4)
                        T_MT_l[2, -1] = - boxDimensions[1] / 2
                        T_MT_r[2, -1] = - boxDimensions[1] / 2
                        if horizontalAxisIndex == 0:
                            T_MT_l[horizontalAxisIndex, -1] = - approachAndGraspDistances[motionIndex]
                            T_MT_r[horizontalAxisIndex, -1] = approachAndGraspDistances[motionIndex]
                        elif horizontalAxisIndex == 1:
                            T_MT_l[horizontalAxisIndex, -1] = approachAndGraspDistances[motionIndex]
                            T_MT_r[horizontalAxisIndex, -1] = - approachAndGraspDistances[motionIndex]

                        # tf of desired targets wrt world frame
                        T_WT_l = np.matmul(T_WM, T_MT_l)
                        T_WT_r = np.matmul(T_WM, T_MT_r)
                        T_WT_list = [T_WT_l, T_WT_r]

                        for armIndex, arm_tf in enumerate(T_WT_list):                           # loop over arms
                            tfName = eeNames[armIndex] + motionNames[motionIndex]
                            (roll, pitch, yaw) = tf.transformations.euler_from_matrix(arm_tf, 'sxyz')
                            targetBroadcaster.sendTransform(arm_tf[:-1, -1],
                                                            tf.transformations.quaternion_from_euler(roll, pitch, yaw),
                                                            rospy.Time.now(),
                                                            tfName,
                                                            globalFrame)
                    # lift phase based on last grasping target
                    T_WT_l[2, -1] += liftingPhase['liftHeight']
                    T_WT_r[2, -1] += liftingPhase['liftHeight']
                    T_WT_list = [T_WT_l, T_WT_r]

                    for armIndex, arm_tf in enumerate(T_WT_list):  # loop over arms
                        tfName = eeNames[armIndex] + '_lift'
                        (roll, pitch, yaw) = tf.transformations.euler_from_matrix(arm_tf, 'sxyz')
                        targetBroadcaster.sendTransform(arm_tf[:-1, -1],
                                                        tf.transformations.quaternion_from_euler(roll, pitch, yaw),
                                                        rospy.Time.now(),
                                                        tfName,
                                                        globalFrame)
                #################################################
                ################## DAGANA GRASP #################
                #################################################
                elif scenario == "dagana_grasp":
                    isMarkerSupport = (aruco_id[markerIndex] == supportSurface['markerId'])

                    # check which axis of marker frame is longitudinal
                    (roll, pitch, yaw) = euler_from_matrix(T_WM[:-1, :-1], axes='sxyz')

                    # broadcast tf
                    if isMarkerSupport:
                        # define matrix
                        T_MT = np.identity(4)
                        # transform of left and right ee targets wrt the marker frame
                        T_MT[0, -1] = supportSurface['targetWrtMarker'][0]
                        T_MT[1, -1] = supportSurface['targetWrtMarker'][1]
                        T_MT[2, -1] = supportSurface['targetWrtMarker'][2]

                        # print("T_MT = \n", T_MT)
                        tfName = eeNames[supportSurface['armIndex']] + '_support'
                    else:
                        # ASSUMPTION: fix marker always with -x pointing towards the object
                        T_MT = np.zeros((4, 4))
                        T_MT[-1, -1] = 1
                        T_MT[2, 2] = - 1  # aruco is horizontal facing upwards
                        T_MT[0, 1] = -1   # projection of Ty on Mx
                        T_MT[1, 0] = -1   # projection of Tx on My

                        # transform of left and right ee targets wrt the marker frame
                        T_MT[0, -1] = reachingPhase['targetWrtMarker'][0]
                        T_MT[1, -1] = reachingPhase['targetWrtMarker'][1]
                        T_MT[2, -1] = reachingPhase['targetWrtMarker'][2]

                        # print("T_MT = \n", T_MT)
                        tfName = eeNames[armIndex] + '_target'

                    # transform of target wrt world frame
                    T_WT = np.matmul(T_WM, T_MT)
                    # print("T_WT = \n", T_WT[:3, :3])

                    (roll, pitch, yaw) = tf.transformations.euler_from_matrix(T_WT, 'sxyz')
                    targetBroadcaster.sendTransform(T_WT[:-1, -1],
                                     tf.transformations.quaternion_from_euler(roll, pitch, yaw),
                                     rospy.Time.now(),
                                     tfName,
                                     globalFrame)

                    # assign to a list
                    # T_WT_list = [T_WT]

                    # if a first target (Ap) has to be reached first
                    if not isMarkerSupport and approachingPhase['active']:
                        TMAp = T_MT
                        TMAp[0, -1] = approachingPhase['ApWrtMarker'][0]
                        TMAp[1, -1] = approachingPhase['ApWrtMarker'][1]
                        TMAp[2, -1] = approachingPhase['ApWrtMarker'][2]
                        T_WAp = np.matmul(T_WM, TMAp)       # transform to world

                        # broadcast tf
                        (roll, pitch, yaw) = tf.transformations.euler_from_matrix(T_WAp, 'sxyz')
                        targetBroadcaster.sendTransform(T_WAp[:-1, -1],
                                                        tf.transformations.quaternion_from_euler(roll, pitch, yaw),
                                                        rospy.Time.now(),
                                                        eeNames[armIndex] + '_approach',
                                                        globalFrame)

                        # overwrite list
                        # T_WT_list = [T_WAp, T_WT]
                        T_WT_list.append(T_WAp)
                    T_WT_list.append(T_WT)

                #################################################
                ################## WALL CONTACT #################
                #################################################
                elif scenario == "wall_contact":
                    # transform of left and right ee targets wrt the marker frame
                    T_MT = np.identity(4)
                    T_MT[0, -1] = reachingPhase['targetWrtMarker'][0]
                    T_MT[1, -1] = reachingPhase['targetWrtMarker'][1]
                    T_MT[2, -1] = reachingPhase['targetWrtMarker'][2] + ballEeRadius

                    # transform of target wrt world frame
                    T_WT = np.matmul(T_WM, T_MT)

                    # assign to a list
                    T_WT_list.append(T_WT)

                # print target tfs
                # print("T_MT_L = ", T_MT_l)
                # print("T_MT_r = ", T_MT_r)
                # print("T_WT_l = ", T_WT_l)
                # print("T_WT_r = ", T_WT_r)

                MarkerTobeUpdated[markerIndex] = False
                show_points_in_rviz(markerPublisher, T_WT_list, globalFrame)
                # show_points_in_rviz(markerPublisher, T_WTapp_list, globalFrame)
        arucoRate.sleep()

