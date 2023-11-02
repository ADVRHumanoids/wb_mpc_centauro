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

import rosbag
import matplotlib.pyplot as plt
import matplotlib

# bag = rosbag.Bag('/home/idadiotis/Desktop/trotting_n.bag')
# bag = rosbag.Bag('/home/idadiotis/Desktop/trot_update_p.bag')
# bag = rosbag.Bag('/home/idadiotis/Desktop/manipulation_default.bag')
# bag = rosbag.Bag('/home/idadiotis/Desktop/manipulation_default_dummy.bag')


# new plots
# bag = rosbag.Bag('/home/idadiotis/Desktop/trotting2.bag')
# bag = rosbag.Bag('/home/idadiotis/Desktop/trotting2_updateP.bag')
bag = rosbag.Bag('/home/idadiotis/Desktop/manipulation_default2.bag')

# create list of msgs
JointStateList = []
for topic, msg, t in bag.read_messages(topics=['/xbotcore/joint_states']):
    JointStateList.append(msg)

JointTorqueTermsList = []
for topic, msg, t in bag.read_messages(topics=['/xbot/joint_torque_terms']):
    JointTorqueTermsList.append(msg)

bag.close()

# extract JointState.effort_reference and torque terms
cutFirstSamples = 1000
names = JointTorqueTermsList[0].name
effortReference = [i.reference for i in JointTorqueTermsList[cutFirstSamples:]]
positionRelatedTorque = [i.position_related for i in JointTorqueTermsList[cutFirstSamples:]]
velocityRelatedTorque = [i.velocity_related for i in JointTorqueTermsList[cutFirstSamples:]]
feedforwardTorque = [i.feedforward for i in JointTorqueTermsList[cutFirstSamples:]]

# wrap data in the order to be plotted
data = [effortReference, positionRelatedTorque, velocityRelatedTorque, feedforwardTorque]
dataNames = ['$τ_{ref}$', '$K_p * e_q$', '$K_d * e_{\dot{q}}$', '$τ_{ff}$']

# plot
jointsToPlot = ['hip_yaw_1', 'hip_pitch_1', 'knee_pitch_1', 'ankle_pitch_1', 'ankle_yaw_1', 'j_wheel_1']
# jointsToPlot = ['torso_yaw', 'j_arm1_1', 'j_arm1_2', 'j_arm1_3', 'j_arm1_4', 'j_arm1_5', 'j_arm1_6', 'j_arm1_7']
jointsIndicesToPlot = [names.index(i) for i in jointsToPlot]
# jointsToPlot = [names[i] for i in jointsIndicesToPlot]

# row and column labels
cols = [col for col in jointsToPlot]
rows = ['{}'.format(row) for row in dataNames]

fig, axes = plt.subplots(nrows=len(data), ncols=len(jointsToPlot), figsize=(14, 8))
matplotlib.rc('xtick', labelsize=14)
matplotlib.rc('ytick', labelsize=14)

for num, tau in enumerate(data):
    for joint_num, joint in enumerate(jointsToPlot):
        axes[num, joint_num].plot([sample[jointsIndicesToPlot[joint_num]] for sample in tau], linewidth=2)
        axes[num, joint_num].grid()

# axes[3, 0].plot([sample[jointsIndicesToPlot[0]] for sample in data[2]], linewidth=2, linestyle="--")
# axes[3, 0].plot([sample[jointsIndicesToPlot[0]] for sample in data[1]], linewidth=2, linestyle="-.")
# axes[0, 0].text(2200, -10, 'lift off', 'o')
axes[0, 0].annotate('LO', (1200, -10), color='r')
axes[0, 0].annotate('TD', (1500, 125), color='r')

# labels
for ax in axes[-1].flat:
    ax.set(xlabel='Samples')
for ax, col in zip(axes[0], cols):
    ax.set_title(col, size='large', y=1)
for ax, row in zip(axes[:,0], rows):
    ax.set_ylabel(row, rotation=90, fontsize=20)
# title
plt.suptitle('Trotting with $K_p = 50, K_d = 100$, constant position equilibrium\n at homing and feedforward from inverse dynamics', y=0.99, fontsize=20)
# fig.tight_layout()
fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
# plt.show()

# plot arms joints
armJointsToPlot = ['torso_yaw', 'j_arm1_1', 'j_arm1_2', 'j_arm1_3', 'j_arm1_4', 'j_arm1_5', 'j_arm1_6', 'j_arm1_7']
jointsIndicesToPlot = [names.index(i) for i in armJointsToPlot]
# row and column labels
cols = [col for col in armJointsToPlot]
rows = ['{}'.format(row) for row in dataNames]

fig, axes = plt.subplots(nrows=len(data), ncols=len(armJointsToPlot), figsize=(14, 8))
matplotlib.rc('xtick', labelsize=14)
matplotlib.rc('ytick', labelsize=14)

for num, tau in enumerate(data):
    for joint_num, joint in enumerate(armJointsToPlot):
        axes[num, joint_num].plot([sample[jointsIndicesToPlot[joint_num]] for sample in tau], linewidth=2)
        axes[num, joint_num].grid()
axes[0, 0].annotate('LO', (1200, -10), color='r')
axes[0, 0].annotate('TD', (1500, 125), color='r')

# labels
for ax in axes[-1].flat:
    ax.set(xlabel='Samples')
for ax, col in zip(axes[0], cols):
    ax.set_title(col, size='large', y=1)
for ax, row in zip(axes[:,0], rows):
    ax.set_ylabel(row, rotation=90, fontsize=20)
# title
plt.suptitle('Trotting with $K_p = 50, K_d = 100$, constant position equilibrium\n at homing and feedforward from inverse dynamics', y=0.99, fontsize=20)
# fig.tight_layout()
fig.subplots_adjust(top=0.9, left=0.1, right=0.98)
# plt.show()

# plot only velocity-related and ff
plt.figure()
matplotlib.rc('xtick', labelsize=14)
matplotlib.rc('ytick', labelsize=14)
colors = ['g', 'r', 'b']
linestyles = ['-', '--', '-.']
for num in range(len(jointsToPlot)):
    plt.subplot(1, len(jointsToPlot), num + 1)
    for i, torque in enumerate(data[2:]):
        plt.plot([sample[jointsIndicesToPlot[num]] for sample in torque], linestyle=linestyles[i], linewidth=2-i,
                 color=colors[i], label=dataNames[len(data[2:]) + i])
    plt.grid()
    plt.legend()
    plt.title(jointsToPlot[num])
    plt.xlabel('Samples', fontsize=20)
    plt.ylabel('$[N*m]$', fontsize=20)
plt.show()

exit()