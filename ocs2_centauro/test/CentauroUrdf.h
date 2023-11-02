
#pragma once

// clang-format off
/* Centauro URDF string */
static constexpr auto centauroUrdf = R"(
<robot name="centauro">
   <!--
   ROS urdf definition of the CENTAURO humanoid robot by Istituto Italiano di Tecnologia

   Author: Enrico Mingo Hoffman
 -->
   <!-- scaling facto for available peack torque link side, from 0 to 1 -->
   <material name="black">
     <color rgba="0.4 0.4 0.4 1"/>
   </material>
   <material name="dark">
     <color rgba="0.1 0.1 0.1 1"/>
   </material>
   <material name="dark_grey">
     <color rgba="0.2 0.2 0.2 1"/>
   </material>
   <!-- TODO CHECK THIS VALUE -->
   <!-- ??? -->
   <!-- ??? -->
   <!-- Origin -->
   <!-- <xacro:property name="hip_yaw_lower" value="${[-2.827,-2.583, -2.583 ,-2.827]}"/>
 <xacro:property name="hip_yaw_upper" value="${[ 2.583, 2.827,  2.827 , 2.583]}"/> -->
   <!--link name="base_link"/yiannis-->
   <link name="base_link">
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/pelvis.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/pelvis.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>

     <!-- Fixed joint to add dummy inertia link -->
     <joint name="base_to_base_inertia" type="fixed">
         <parent link="base_link"/>
         <child link="base_link_inertia"/>
         <origin rpy="0 0 0" xyz="0 0 0"/>
     </joint>
     <!-- Dummy inertia link, because KDL cannot have inertia on the base link -->
     <link name="base_link_inertia">
       <inertial>
           <origin xyz="-0.0016320207 0.0026534004000000003 0.079167428"/>
           <mass value="26.584529"/>
           <inertia ixx="0.39195477" ixy="0.024781258999999996" ixz="0.017108606999999998" iyy="1.0000708999999999" iyz="0.0052627106000000005" izz="1.0679988999999999"/>
       </inertial>
     </link>

   <link name="imu_link"/>
   <joint name="imu_joint" type="fixed">
     <parent link="base_link"/>
     <origin rpy="0.0 0.0 0.0" xyz="0.248316 0.0 0.015"/>
     <child link="imu_link"/>
   </joint>
   <!--joint name="base_joint" type="fixed">
     <parent link="base_link"/>
     <child link="base_link"/>
     <origin rpy="0 0 0" xyz="0 0 0"/>
   </jointyiannis-->
   <!-- torso links -->
   <link name="torso_2">
     <inertial>
       <origin xyz="-0.019497794 0.0045317835 0.13768283"/>
       <mass value="12.553731"/>
       <inertia ixx="0.063643392" ixy="8.939e-05" ixz="-0.00086873" iyy="0.02680235" iyz="-4.657e-05" izz="0.04743015"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/link1_no_head.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/link1_no_head.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /torso links -->
   <!-- torso joints -->
   <joint name="torso_yaw" type="revolute">
     <parent link="base_link"/>
     <origin xyz="0.2 0.0 0.256"/>
     <child link="torso_2"/>
     <axis xyz="0 0 1"/>
     <limit effort="147" lower="-2.618" upper="2.618" velocity="5.7"/>
     <!-- TODO -->
   </joint>
   <!-- /torso joints -->
   <!-- /macro arms -->
   <!-- LINKS -->
   <!-- shoulder yaw-roll-->
   <link name="arm1_1">
     <inertial>
       <origin xyz="-0.0074457212 0.03410796 0.00010978102"/>
       <mass value="1.9628675"/>
       <inertia ixx="0.0053547717" ixy="0.00036428926" ixz="1.5089568e-05" iyy="0.0033923328" iyz="5.5692312e-05" izz="0.0068921413"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/ShoulderPitch.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/ShoulderPitch.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- shoulder yaw-roll-->
   <!-- shoulder roll-pitch-->
   <link name="arm1_2">
     <inertial>
       <origin xyz="0.058142302 5.7450803e-05 -0.077477683"/>
       <mass value="1.8595811"/>
       <inertia ixx="0.013776643" ixy="3.7788675e-05" ixz="0.0037690171" iyy="0.015677464" iyz="-9.4893549e-06" izz="0.0046317657"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/ShoulderRoll.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/ShoulderRoll.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- shoulder roll-pitch-->
   <!-- shoulder-elbow -->
   <link name="arm1_3">
     <inertial>
       <origin xyz="0.014625194 0.0008172672 -0.028333545"/>
       <mass value="1.6678109"/>
       <inertia ixx="0.0064480435" ixy="-0.00015639093" ixz="0.0012205359" iyy="0.0073372077" iyz="8.9941532e-05" izz="0.0036738448"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/ShoulderYaw.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/ShoulderYaw.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /shoulder-elbow -->
   <!-- elbow yaw-pitch -->
   <link name="arm1_4">
     <inertial>
       <origin xyz="-0.0076833067 -0.040302205 -0.043492779"/>
       <mass value="1.3157289"/>
       <inertia ixx="0.004330394" ixy="-0.00011737391" ixz="-0.00041923199" iyy="0.0038539919" iyz="-0.00079573038" izz="0.0017594689"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/Elbow.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/Elbow.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /elbow yaw-pitch -->
   <!-- elbow-wrist -->
   <link name="arm1_5">
     <inertial>
       <origin xyz="-0.00011079615 0.011590836 -0.07816026"/>
       <mass value="1.4908547"/>
       <inertia ixx="0.0085692128" ixy="1.7856252e-05" ixz="1.9379365e-05" iyy="0.0077454159" iyz="-0.00032860094" izz="0.0027441921"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/Forearm.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/Forearm.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /elbow-wrist -->
   <!-- wrist yaw-pitch -->
   <link name="arm1_6">
     <inertial>
       <origin xyz="-4.6502396e-06 -0.038014094 -0.069926878"/>
       <mass value="1.1263612"/>
       <inertia ixx="0.0051871784" ixy="2.724437e-05" ixz="2.2833496e-06" iyy="0.0048037789" iyz="-0.00072165653" izz="0.0012771388"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/ForearmPitch.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/ForearmPitch.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /wrist yaw-pitch -->
   <link name="arm1_7">
     <inertial>
       <origin xyz="0.0 0.0 0.03127057"/>
       <mass value="0.27966428"/>
       <inertia ixx="0.00014415192" ixy="9.8651826e-08" ixz="0" iyy="0.0001441701" iyz="0" izz="6.0500616e-05"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/ForearmYaw.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/ForearmYaw.STL" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /LINKS -->
   <!-- JOINTS -->
   <!-- shoulder yaw -->
   <joint name="j_arm1_1" type="revolute">
     <parent link="torso_2"/>
     <child link="arm1_1"/>
     <origin rpy="0.174533 0.0 -0.5235988" xyz="0.0457475 0.169137 0.116626"/>
     <axis xyz="0 1 0"/>
     <limit effort="147.0" lower="-3.312" upper="1.615" velocity="3.86"/>
   </joint>
   <!-- /shoulder yaw -->
   <!-- shoulder roll -->
   <joint name="j_arm1_2" type="revolute">
     <parent link="arm1_1"/>
     <child link="arm1_2"/>
     <origin rpy="-0.1745327 0.0 0.0" xyz="-0.09015 0.062 0.0"/>
     <axis xyz="1 0 0"/>
     <limit effort="147.0" lower="0.02" upper="3.431" velocity="3.86"/>
   </joint>
   <!-- /shoulder roll -->
   <!-- shoulder pitch-->
   <joint name="j_arm1_3" type="revolute">
     <parent link="arm1_2"/>
     <child link="arm1_3"/>
     <origin xyz="0.09015 0.0 -0.21815"/>
     <axis xyz="0 0 1"/>
     <limit effort="147.0" lower="-2.552" upper="2.566" velocity="6.06"/>
   </joint>
   <!-- /shoulder pitch -->
   <!-- elbow yaw -->
   <joint name="j_arm1_4" type="revolute">
     <parent link="arm1_3"/>
     <child link="arm1_4"/>
     <origin xyz="0.045 0.05515 -0.074"/>
     <axis xyz="0 1 0"/>
     <limit effort="147.0" lower="-2.465" upper="0.28" velocity="6.06"/>
   </joint>
   <!-- /elbow yaw -->
   <!-- elbow pitch-->
   <joint name="j_arm1_5" type="revolute">
     <parent link="arm1_4"/>
     <child link="arm1_5"/>
     <origin xyz="-0.015 -0.05515 -0.095"/>
     <axis xyz="0 0 1"/>
     <limit effort="55.0" lower="-2.569" upper="2.562" velocity="11.72"/>
   </joint>
   <!-- /elbow pitch-->
   <!-- wrist yaw-->
   <joint name="j_arm1_6" type="revolute">
     <parent link="arm1_5"/>
     <child link="arm1_6"/>
     <origin xyz="0.0 0.049 -0.156"/>
     <axis xyz="0 1 0"/>
     <limit effort="55.0" lower="-1.529" upper="1.509" velocity="11.72"/>
   </joint>
   <!-- /wrist yaw-->
   <!-- wrist pitch-->
   <joint name="j_arm1_7" type="revolute">
     <parent link="arm1_6"/>
     <child link="arm1_7"/>
     <origin xyz="0.0 -0.049 -0.174"/>
     <axis xyz="0 0 1"/>
     <limit effort="28.32" lower="-2.565" upper="2.569" velocity="20.35"/>
   </joint>
   <!-- wrist pitch-->
   <!-- force-troque sensor -->
   <!-- end-effector -->
   <!-- wrist pitch-->
   <joint name="j_ft_1" type="fixed">
     <parent link="arm1_7"/>
     <child link="ft_arm1"/>
     <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.0"/>
     <axis xyz="0 0 1"/>
     <limit effort="0" lower="0.0" upper="0.0" velocity="0"/>
   </joint>
   <!-- wrist pitch-->
   <link name="ft_arm1">
     <inertial>
       <origin xyz="8.2161929e-05 0.0026334189 -0.016465877"/>
       <mass value="0.37827821"/>
       <inertia ixx="0.00023321716" ixy="2.3681435e-06" ixz="-1.8039102e-06" iyy="0.00020626291" iyz="3.158829e-05" izz="0.00027399881"/>
     </inertial>
     <!--visual>
          <origin xyz="0.0 0.0 0.0"  rpy="${PI} 0.0 0.0"/>
    <geometry>
       <mesh filename="package://centauro_urdf/meshes/Wrist.STL" scale="0.001 ${-rot*0.001} 0.001" />
    </geometry>
       </visual-->
     <!--collision>
          <origin xyz="0.0 0.0 0.0"  rpy="${PI} 0.0 0.0"/>
    <geometry>
       <mesh filename="package://centauro_urdf/meshes/simple/Wrist.STL" scale="0.001 ${-rot*0.001} 0.001" />
    </geometry>
       </collision-->
   </link>
   <!-- LINKS -->
   <!-- shoulder yaw-roll-->
   <link name="arm2_1">
     <inertial>
       <origin xyz="-0.0074457212 -0.03410796 0.00010978102"/>
       <mass value="1.9628675"/>
       <inertia ixx="0.0053547717" ixy="-0.00036428926" ixz="1.5089568e-05" iyy="0.0033923328" iyz="-5.5692312e-05" izz="0.0068921413"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/ShoulderPitch.STL" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/ShoulderPitch.STL" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- shoulder yaw-roll-->
   <!-- shoulder roll-pitch-->
   <link name="arm2_2">
     <inertial>
       <origin xyz="0.058142302 -5.7450803e-05 -0.077477683"/>
       <mass value="1.8595811"/>
       <inertia ixx="0.013776643" ixy="-3.7788675e-05" ixz="0.0037690171" iyy="0.015677464" iyz="9.4893549e-06" izz="0.0046317657"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/ShoulderRoll.STL" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/ShoulderRoll.STL" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- shoulder roll-pitch-->
   <!-- shoulder-elbow -->
   <link name="arm2_3">
     <inertial>
       <origin xyz="0.014625194 -0.0008172672 -0.028333545"/>
       <mass value="1.6678109"/>
       <inertia ixx="0.0064480435" ixy="0.00015639093" ixz="0.0012205359" iyy="0.0073372077" iyz="-8.9941532e-05" izz="0.0036738448"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/ShoulderYaw.STL" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/ShoulderYaw.STL" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /shoulder-elbow -->
   <!-- elbow yaw-pitch -->
   <link name="arm2_4">
     <inertial>
       <origin xyz="-0.0076833067 0.040302205 -0.043492779"/>
       <mass value="1.3157289"/>
       <inertia ixx="0.004330394" ixy="0.00011737391" ixz="-0.00041923199" iyy="0.0038539919" iyz="0.00079573038" izz="0.0017594689"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/Elbow.STL" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/Elbow.STL" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /elbow yaw-pitch -->
   <!-- elbow-wrist -->
   <link name="arm2_5">
     <inertial>
       <origin xyz="-0.00011079615 -0.011590836 -0.07816026"/>
       <mass value="1.4908547"/>
       <inertia ixx="0.0085692128" ixy="-1.7856252e-05" ixz="1.9379365e-05" iyy="0.0077454159" iyz="0.00032860094" izz="0.0027441921"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/Forearm.STL" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/Forearm.STL" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /elbow-wrist -->
   <!-- wrist yaw-pitch -->
   <link name="arm2_6">
     <inertial>
       <origin xyz="-4.6502396e-06 0.038014094 -0.069926878"/>
       <mass value="1.1263612"/>
       <inertia ixx="0.0051871784" ixy="-2.724437e-05" ixz="2.2833496e-06" iyy="0.0048037789" iyz="0.00072165653" izz="0.0012771388"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/ForearmPitch.STL" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/ForearmPitch.STL" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /wrist yaw-pitch -->
   <link name="arm2_7">
     <inertial>
       <origin xyz="0.0 -0.0 0.03127057"/>
       <mass value="0.27966428"/>
       <inertia ixx="0.00014415192" ixy="-9.8651826e-08" ixz="0" iyy="0.0001441701" iyz="0" izz="6.0500616e-05"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/ForearmYaw.STL" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/ForearmYaw.STL" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!-- /LINKS -->
   <!-- JOINTS -->
   <!-- shoulder yaw -->
   <joint name="j_arm2_1" type="revolute">
     <parent link="torso_2"/>
     <child link="arm2_1"/>
     <origin rpy="-0.174533 0.0 0.5235988" xyz="0.0457475 -0.169137 0.116626"/>
     <axis xyz="0 1 0"/>
     <limit effort="147.0" lower="-3.3458" upper="1.6012" velocity="3.86"/>
   </joint>
   <!-- /shoulder yaw -->
   <!-- shoulder roll -->
   <joint name="j_arm2_2" type="revolute">
     <parent link="arm2_1"/>
     <child link="arm2_2"/>
     <origin rpy="0.1745327 0.0 0.0" xyz="-0.09015 -0.062 0.0"/>
     <axis xyz="1 0 0"/>
     <limit effort="147.0" lower="-3.4258" upper="-0.0138" velocity="3.86"/>
   </joint>
   <!-- /shoulder roll -->
   <!-- shoulder pitch-->
   <joint name="j_arm2_3" type="revolute">
     <parent link="arm2_2"/>
     <child link="arm2_3"/>
     <origin xyz="0.09015 0.0 -0.21815"/>
     <axis xyz="0 0 1"/>
     <limit effort="147.0" lower="-2.5614" upper="2.5606" velocity="6.06"/>
   </joint>
   <!-- /shoulder pitch -->
   <!-- elbow yaw -->
   <joint name="j_arm2_4" type="revolute">
     <parent link="arm2_3"/>
     <child link="arm2_4"/>
     <origin xyz="0.045 -0.05515 -0.074"/>
     <axis xyz="0 1 0"/>
     <limit effort="147.0" lower="-2.4794" upper="0.2886" velocity="6.06"/>
   </joint>
   <!-- /elbow yaw -->
   <!-- elbow pitch-->
   <joint name="j_arm2_5" type="revolute">
     <parent link="arm2_4"/>
     <child link="arm2_5"/>
     <origin xyz="-0.015 0.05515 -0.095"/>
     <axis xyz="0 0 1"/>
     <limit effort="55.0" lower="-2.5394" upper="2.5546" velocity="11.72"/>
   </joint>
   <!-- /elbow pitch-->
   <!-- wrist yaw-->
   <joint name="j_arm2_6" type="revolute">
     <parent link="arm2_5"/>
     <child link="arm2_6"/>
     <origin xyz="0.0 -0.049 -0.156"/>
     <axis xyz="0 1 0"/>
     <limit effort="55.0" lower="-1.5154" upper="1.5156" velocity="11.72"/>
   </joint>
   <!-- /wrist yaw-->
   <!-- wrist pitch-->
   <joint name="j_arm2_7" type="revolute">
     <parent link="arm2_6"/>
     <child link="arm2_7"/>
     <origin xyz="0.0 0.049 -0.174"/>
     <axis xyz="0 0 1"/>
     <limit effort="28.32" lower="-2.5554" upper="2.5686" velocity="20.35"/>
   </joint>
   <!-- wrist pitch-->
   <!-- force-troque sensor -->
   <!-- end-effector -->
   <!-- wrist pitch-->
   <joint name="j_ft_2" type="fixed">
     <parent link="arm2_7"/>
     <child link="ft_arm2"/>
     <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.0"/>
     <axis xyz="0 0 1"/>
     <limit effort="0" lower="0.0" upper="0.0" velocity="0"/>
   </joint>
   <!-- wrist pitch-->
   <link name="ft_arm2">
     <inertial>
       <origin xyz="8.2161929e-05 -0.0026334189 -0.016465877"/>
       <mass value="0.37827821"/>
       <inertia ixx="0.00023321716" ixy="-2.3681435e-06" ixz="-1.8039102e-06" iyy="0.00020626291" iyz="-3.158829e-05" izz="0.00027399881"/>
     </inertial>
     <!--visual>
          <origin xyz="0.0 0.0 0.0"  rpy="${PI} 0.0 0.0"/>
    <geometry>
       <mesh filename="package://centauro_urdf/meshes/Wrist.STL" scale="0.001 ${-rot*0.001} 0.001" />
    </geometry>
       </visual-->
     <!--collision>
          <origin xyz="0.0 0.0 0.0"  rpy="${PI} 0.0 0.0"/>
    <geometry>
       <mesh filename="package://centauro_urdf/meshes/simple/Wrist.STL" scale="0.001 ${-rot*0.001} 0.001" />
    </geometry>
       </collision-->
   </link>
   <link name="ball1">
     <inertial>
       <origin rpy="0 0 0" xyz="-1.0072498e-05 3.7590658e-05 0.019772332"/>
       <mass value="1.27"/>
       <inertia ixx="0.0023061092" ixy="2.9181758e-07" ixz="-2.1246683999999997e-07" iyy="0.0023053155000000002" iyz="7.9290978e-07" izz="0.0030675615999999997"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry name="ball1_visual">
         <mesh filename="package://centauro_urdf/meshes/BallHand.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry name="ball1_collision">
         <mesh filename="package://centauro_urdf/meshes/simple/BallHand.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="ball1_tip"/>
   <joint name="j_arm1_8" type="fixed">
     <parent link="arm1_7"/>
     <child link="ball1"/>
     <origin rpy="0. 0. 0." xyz="0. 0. 0."/>
   </joint>
   <joint name="j_ball1_tip" type="fixed">
     <parent link="ball1"/>
     <child link="ball1_tip"/>
     <origin rpy="0 0 0" xyz="0 0 -0.07"/>
   </joint>
   <!-- add armX_8 link for legacy reasons (same as ballX) -->
   <joint name="j_ball1_fixed" type="fixed">
     <parent link="ball1"/>
     <child link="arm1_8"/>
   </joint>
   <link name="arm1_8">
     <inertial>
       <mass value="0"/>
       <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
     </inertial>
   </link>
   <link name="ball2">
     <inertial>
       <origin rpy="0 0 0" xyz="-1.0072498e-05 3.7590658e-05 0.019772332"/>
       <mass value="1.27"/>
       <inertia ixx="0.0023061092" ixy="2.9181758e-07" ixz="-2.1246683999999997e-07" iyy="0.0023053155000000002" iyz="7.9290978e-07" izz="0.0030675615999999997"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry name="ball2_visual">
         <mesh filename="package://centauro_urdf/meshes/BallHand.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry name="ball2_collision">
         <mesh filename="package://centauro_urdf/meshes/simple/BallHand.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="ball2_tip"/>
   <joint name="j_arm2_8" type="fixed">
     <parent link="arm2_7"/>
     <child link="ball2"/>
     <origin rpy="0. 0. 0." xyz="0. 0. 0."/>
   </joint>
   <joint name="j_ball2_tip" type="fixed">
     <parent link="ball2"/>
     <child link="ball2_tip"/>
     <origin rpy="0 0 0" xyz="0 0 -0.07"/>
   </joint>
   <!-- add armX_8 link for legacy reasons (same as ballX) -->
   <joint name="j_ball2_fixed" type="fixed">
     <parent link="ball2"/>
     <child link="arm2_8"/>
   </joint>
   <link name="arm2_8">
     <inertial>
       <mass value="0"/>
       <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
     </inertial>
   </link>
   <!-- LINKS -->
   <link name="neck_1">
     <inertial>
       <origin xyz="0 0 0"/>
       <mass value="0.33878686"/>
       <!-- MASS/INERTIA TO DO -->
       <inertia ixx="0.0003718568" ixy="-9.42983e-05" ixz="-2.80644e-05" iyy="0.0001886879" iyz="-1.54534e-05" izz="0.0005023443"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/head-base_mesh.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/head-base_mesh.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="velodyne_motor">
     <inertial>
       <origin xyz="0 0 0"/>
       <mass value="0.33878686"/>
       <!-- MASS/INERTIA TO DO -->
       <inertia ixx="0.0003718568" ixy="-9.42983e-05" ixz="-2.80644e-05" iyy="0.0001886879" iyz="-1.54534e-05" izz="0.0005023443"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/lidar_mesh.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/lidar_mesh.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="d435_head_motor">
     <inertial>
       <origin xyz="0 0 0"/>
       <mass value="0.33878686"/>
       <!-- MASS/INERTIA TO DO -->
       <inertia ixx="0.0003718568" ixy="-9.42983e-05" ixz="-2.80644e-05" iyy="0.0001886879" iyz="-1.54534e-05" izz="0.0005023443"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/neck-pitch_mesh.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/simple/neck-pitch_mesh.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <!--JOINTS -->
   <joint name="neck_base" type="fixed">
     <parent link="torso_2"/>
     <origin rpy="0 0 0" xyz="0.0030637045 0.0061607221 0.158"/>
     <child link="neck_1"/>
     <axis xyz="0 0 1"/>
   </joint>
   <joint name="velodyne_joint" type="fixed">
     <parent link="neck_1"/>
     <origin rpy="0 0 0" xyz="0.0822724 0 0.242"/>
     <child link="velodyne_motor"/>
     <!--axis xyz="0 -1 0"/>
     <limit effort="35" lower="-1.1" upper="1.1" velocity="5.7"/>
     <dynamics friction="10.0"/-->
   </joint>
   <joint name="d435_head_joint" type="fixed">
     <parent link="neck_1"/>
     <origin rpy="0 0 0" xyz="0.06 0 0.1375"/>
     <child link="d435_head_motor"/>
     <!--axis xyz="0 -1 0"/>
     <limit effort="35" lower="-1.1" upper="1.1" velocity="5.7"/>
     <dynamics friction="10.0"/-->
   </joint>
   <material name="D435_head_camera_aluminum">
     <color rgba="0.5 0.5 0.5 1"/>
   </material>
   <!-- camera body, with origin at bottom screw mount -->
   <joint name="D435_head_camera_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0.03015 0 -0.0125"/>
     <parent link="d435_head_motor"/>
     <child link="D435_head_camera_bottom_screw_frame"/>
   </joint>
   <link name="D435_head_camera_bottom_screw_frame"/>
   <joint name="D435_head_camera_link_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0.0175 0.0125"/>
     <parent link="D435_head_camera_bottom_screw_frame"/>
     <child link="D435_head_camera_link"/>
   </joint>
   <link name="D435_head_camera_link">
     <visual>
       <origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0.0149 -0.0175 0"/>
       <geometry>
         <!-- <box size="${d435i_cam_width} ${d435i_cam_height} ${d435i_cam_depth}"/> -->
         <mesh filename="package://realsense2_description/meshes/d435.dae"/>
         <!--<mesh filename="package://realsense2_description/meshes/d435/d435.dae"/>-->
       </geometry>
       <material name="D435_head_camera_aluminum"/>
     </visual>
     <collision>
       <origin rpy="0 0 0" xyz="0 -0.0175 0"/>
       <geometry>
         <box size="0.02505 0.09 0.025"/>
       </geometry>
     </collision>
     <inertial>
       <!-- The following are not reliable values, and should not be used for modeling -->
       <mass value="0.564"/>
       <origin xyz="0 0 0"/>
       <inertia ixx="0.003881243" ixy="0.0" ixz="0.0" iyy="0.000498940" iyz="0.0" izz="0.003879257"/>
     </inertial>
   </link>
   <!-- camera depth joints and links -->
   <joint name="D435_head_camera_depth_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0 0"/>
     <parent link="D435_head_camera_link"/>
     <child link="D435_head_camera_depth_frame"/>
   </joint>
   <link name="D435_head_camera_depth_frame"/>
   <joint name="D435_head_camera_depth_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435_head_camera_depth_frame"/>
     <child link="D435_head_camera_depth_optical_frame"/>
   </joint>
   <link name="D435_head_camera_depth_optical_frame"/>
   <!-- camera left IR joints and links -->
   <joint name="D435_head_camera_left_ir_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0.0 0"/>
     <parent link="D435_head_camera_depth_frame"/>
     <child link="D435_head_camera_left_ir_frame"/>
   </joint>
   <link name="D435_head_camera_left_ir_frame"/>
   <joint name="D435_head_camera_left_ir_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435_head_camera_left_ir_frame"/>
     <child link="D435_head_camera_left_ir_optical_frame"/>
   </joint>
   <link name="D435_head_camera_left_ir_optical_frame"/>
   <!-- camera right IR joints and links -->
   <joint name="D435_head_camera_right_ir_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 -0.05 0"/>
     <parent link="D435_head_camera_depth_frame"/>
     <child link="D435_head_camera_right_ir_frame"/>
   </joint>
   <link name="D435_head_camera_right_ir_frame"/>
   <joint name="D435_head_camera_right_ir_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435_head_camera_right_ir_frame"/>
     <child link="D435_head_camera_right_ir_optical_frame"/>
   </joint>
   <link name="D435_head_camera_right_ir_optical_frame"/>
   <!-- camera color joints and links -->
   <joint name="D435_head_camera_color_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0.015 0"/>
     <parent link="D435_head_camera_depth_frame"/>
     <child link="D435_head_camera_color_frame"/>
   </joint>
   <link name="D435_head_camera_color_frame"/>
   <joint name="D435_head_camera_color_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435_head_camera_color_frame"/>
     <child link="D435_head_camera_color_optical_frame"/>
   </joint>
   <link name="D435_head_camera_color_optical_frame"/>
   <!-- camera accel joints and links -->
   <joint name="D435_head_camera_accel_joint" type="fixed">
     <origin rpy="0 0 0" xyz="-0.00174 0.00552 0.0176"/>
     <parent link="D435_head_camera_link"/>
     <child link="D435_head_camera_accel_frame"/>
   </joint>
   <link name="D435_head_camera_accel_frame"/>
   <joint name="D435_head_camera_accel_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435_head_camera_accel_frame"/>
     <child link="D435_head_camera_accel_optical_frame"/>
   </joint>
   <link name="D435_head_camera_accel_optical_frame"/>
   <!-- camera gyro joints and links -->
   <joint name="D435_head_camera_gyro_joint" type="fixed">
     <origin rpy="0 0 0" xyz="-0.00174 0.00552 0.0176"/>
     <parent link="D435_head_camera_link"/>
     <child link="D435_head_camera_gyro_frame"/>
   </joint>
   <link name="D435_head_camera_gyro_frame"/>
   <joint name="D435_head_camera_gyro_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435_head_camera_gyro_frame"/>
     <child link="D435_head_camera_gyro_optical_frame"/>
   </joint>
   <link name="D435_head_camera_gyro_optical_frame"/>
   <gazebo reference="D435_head_camera_link">
     <self_collide>0</self_collide>
     <enable_wind>0</enable_wind>
     <kinematic>0</kinematic>
     <!-- <gravity>1</gravity> -->
     <!--<mu>1</mu>-->
     <mu2>1</mu2>
     <fdir1>0 0 0</fdir1>
     <!--<slip1>0</slip1>
       <slip2>0</slip2>-->
     <kp>1e+13</kp>
     <kd>1</kd>
     <!--<max_vel>0.01</max_vel>
       <min_depth>0</min_depth>-->
     <sensor name="D435_head_cameracolor" type="camera">
       <camera name="D435_head_camera">
         <horizontal_fov>1.2112585008840648</horizontal_fov>
         <image>
           <width>1280</width>
           <height>720</height>
           <format>RGB_INT8</format>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.007</stddev>
         </noise>
       </camera>
       <always_on>1</always_on>
       <update_rate>30</update_rate>
       <visualize>False</visualize>
     </sensor>
     <sensor name="D435_head_cameraired1" type="camera">
       <camera name="D435_head_camera">
         <horizontal_fov>1.4870205226991688</horizontal_fov>
         <image>
           <width>1280</width>
           <height>720</height>
           <format>L_INT8</format>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.05</stddev>
         </noise>
       </camera>
       <always_on>1</always_on>
       <update_rate>90</update_rate>
       <visualize>false</visualize>
     </sensor>
     <sensor name="D435_head_cameraired2" type="camera">
       <camera name="D435_head_camera">
         <horizontal_fov>1.4870205226991688</horizontal_fov>
         <image>
           <width>1280</width>
           <height>720</height>
           <format>L_INT8</format>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.05</stddev>
         </noise>
       </camera>
       <always_on>1</always_on>
       <update_rate>90</update_rate>
       <visualize>false</visualize>
     </sensor>
     <sensor name="D435_head_cameradepth" type="depth">
       <camera name="D435_head_camera">
         <horizontal_fov>1.2112585008840648</horizontal_fov>
         <image>
           <width>1280</width>
           <height>720</height>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.100</stddev>
         </noise>
       </camera>
       <always_on>1</always_on>
       <update_rate>30</update_rate>
       <visualize>false</visualize>
     </sensor>
     <sensor name="D435_head_cameraaccel" type="imu">
       <always_on>true</always_on>
       <update_rate>300</update_rate>
       <topic>D435_head_camera/accel/sample</topic>
       <plugin filename="librealsense_gazebo_accel_sensor.so" name="D435/D435_head_camera_accel_plugin">
         <topicName>D435_head_camera/accel/sample</topicName>
         <bodyName>D435_head_camera_link</bodyName>
         <updateRateHZ>300</updateRateHZ>
         <gaussianNoise>0.1</gaussianNoise>
         <xyzOffset>0 0 0</xyzOffset>
         <rpyOffset>0 0 0</rpyOffset>
         <frameName>D435_head_camera_accel_optical_frame</frameName>
         <initialOrientationAsReference>false</initialOrientationAsReference>
       </plugin>
       <pose>0 0 0 0 0 0</pose>
     </sensor>
     <sensor name="D435_head_cameragyro" type="imu">
       <always_on>true</always_on>
       <update_rate>300</update_rate>
       <topic>D435_head_camera/gyro/sample</topic>
       <plugin filename="librealsense_gazebo_gyro_sensor.so" name="D435/D435_head_camera_gyro_plugin">
         <topicName>D435_head_camera/gyro/sample</topicName>
         <bodyName>D435_head_camera_link</bodyName>
         <updateRateHZ>300</updateRateHZ>
         <gaussianNoise>0.1</gaussianNoise>
         <xyzOffset>0 0 0</xyzOffset>
         <rpyOffset>0 0 0</rpyOffset>
         <frameName>D435_head_camera_gyro_optical_frame</frameName>
         <initialOrientationAsReference>false</initialOrientationAsReference>
       </plugin>
     </sensor>
   </gazebo>
   <gazebo>
     <plugin filename="librealsense_gazebo_plugin.so" name="D435/D435_head_camera">
       <prefix>D435_head_camera</prefix>
       <!-- Color camera settings -->
       <colorUpdateRate>30</colorUpdateRate>
       <colorTopicName>color/image_raw</colorTopicName>
       <colorOpticalframeName>D435_head_camera_color_optical_frame</colorOpticalframeName>
       <colorCameraInfoTopicName>color/camera_info</colorCameraInfoTopicName>
       <!-- Infrared camera settings -->
       <infraredUpdateRate>90</infraredUpdateRate>
       <infrared1TopicName>infra1/image_rect_raw</infrared1TopicName>
       <infrared2TopicName>infra2/image_rect_raw</infrared2TopicName>
       <infrared1CameraInfoTopicName>infra1/camera_info</infrared1CameraInfoTopicName>
       <infrared2CameraInfoTopicName>infra2/camera_info</infrared2CameraInfoTopicName>
       <infrared1OpticalframeName>D435_head_camera_left_ir_optical_frame</infrared1OpticalframeName>
       <infrared2OpticalframeName>D435_head_camera_right_ir_optical_frame</infrared2OpticalframeName>
       <!-- Depth camera settings -->
       <rangeMinDepth>0.2</rangeMinDepth>
       <rangeMaxDepth>10.0</rangeMaxDepth>
       <depthUpdateRate>30</depthUpdateRate>
       <depthTopicName>aligned_depth_to_color/image_raw</depthTopicName>
       <depthCameraInfoTopicName>aligned_depth_to_color/camera_info</depthCameraInfoTopicName>
       <depthOpticalframeName>D435_head_camera_color_optical_frame</depthOpticalframeName>
       <!-- Pointlcloud settings -->
       <pointCloud>True</pointCloud>
       <pointCloudTopicName>depth/color/points</pointCloudTopicName>
       <pointCloudCutoff>0.5</pointCloudCutoff>
     </plugin>
   </gazebo>
   <joint name="velodyne_calib_base_mount_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0 0"/>
     <parent link="velodyne_motor"/>
     <child link="velodyne_calib_base_link"/>
   </joint>
   <link name="velodyne_calib_base_link">
     <inertial>
       <mass value="0.83"/>
       <origin xyz="0 0 0.03585"/>
       <inertia ixx="0.000908059425" ixy="0" ixz="0" iyy="0.000908059425" iyz="0" izz="0.0011049624"/>
     </inertial>
     <visual>
       <geometry>
         <mesh filename="package://velodyne_description/meshes/VLP16_base_1.dae"/>
       </geometry>
     </visual>
     <visual>
       <geometry>
         <mesh filename="package://velodyne_description/meshes/VLP16_base_2.dae"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0 0 0" xyz="0 0 0.03585"/>
       <geometry>
         <cylinder length="0.0717" radius="0.0516"/>
       </geometry>
     </collision>
   </link>
   <joint name="velodyne_calib_base_scan_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0 0.0377"/>
     <parent link="velodyne_calib_base_link"/>
     <child link="velodyne_calib"/>
   </joint>
   <link name="velodyne_calib">
     <inertial>
       <mass value="0.01"/>
       <origin xyz="0 0 0"/>
       <inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/>
     </inertial>
     <visual>
       <origin xyz="0 0 -0.0377"/>
       <geometry>
         <mesh filename="package://velodyne_description/meshes/VLP16_scan.dae"/>
       </geometry>
     </visual>
   </link>
   <!-- Gazebo requires the velodyne_gazebo_plugins package -->
   <gazebo reference="velodyne_calib">
     <sensor name="velodyne_calib-VLP16" type="ray">
       <pose>0 0 0 0 0 0</pose>
       <visualize>false</visualize>
       <update_rate>10</update_rate>
       <ray>
         <scan>
           <horizontal>
             <samples>440</samples>
             <resolution>1</resolution>
             <min_angle>-3.141592653589793</min_angle>
             <max_angle>3.141592653589793</max_angle>
           </horizontal>
           <vertical>
             <samples>16</samples>
             <resolution>1</resolution>
             <min_angle>-0.2617993877991494</min_angle>
             <max_angle> 0.2617993877991494</max_angle>
           </vertical>
         </scan>
         <range>
           <min>0.3</min>
           <max>131.0</max>
           <resolution>0.001</resolution>
         </range>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.0</stddev>
         </noise>
       </ray>
       <plugin filename="libgazebo_ros_velodyne_laser.so" name="gazebo_ros_laser_controller">
         <topicName>/velodyne_points</topicName>
         <frameName>velodyne_calib</frameName>
         <organize_cloud>False</organize_cloud>
         <min_range>0.9</min_range>
         <max_range>130.0</max_range>
         <gaussianNoise>0.008</gaussianNoise>
       </plugin>
     </sensor>
   </gazebo>
   <material name="D435i_camera_aluminum">
     <color rgba="0.5 0.5 0.5 1"/>
   </material>
   <!-- camera body, with origin at bottom screw mount -->
   <joint name="D435i_camera_joint" type="fixed">
     <origin rpy="0 0.5251508 0" xyz="0.27187 0 0.0215997"/>
     <parent link="base_link"/>
     <child link="D435i_camera_bottom_screw_frame"/>
   </joint>
   <link name="D435i_camera_bottom_screw_frame"/>
   <joint name="D435i_camera_link_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0.0175 0.0125"/>
     <parent link="D435i_camera_bottom_screw_frame"/>
     <child link="D435i_camera_link"/>
   </joint>
   <link name="D435i_camera_link">
     <visual>
       <origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0.0149 -0.0175 0"/>
       <geometry>
         <!-- <box size="${d435i_cam_width} ${d435i_cam_height} ${d435i_cam_depth}"/> -->
         <mesh filename="package://realsense2_description/meshes/d435.dae"/>
         <!--<mesh filename="package://realsense2_description/meshes/d435/d435.dae"/>-->
       </geometry>
       <material name="D435i_camera_aluminum"/>
     </visual>
     <collision>
       <origin rpy="0 0 0" xyz="0 -0.0175 0"/>
       <geometry>
         <box size="0.02505 0.09 0.025"/>
       </geometry>
     </collision>
     <inertial>
       <!-- The following are not reliable values, and should not be used for modeling -->
       <mass value="0.564"/>
       <origin xyz="0 0 0"/>
       <inertia ixx="0.003881243" ixy="0.0" ixz="0.0" iyy="0.000498940" iyz="0.0" izz="0.003879257"/>
     </inertial>
   </link>
   <!-- camera depth joints and links -->
   <joint name="D435i_camera_depth_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0 0"/>
     <parent link="D435i_camera_link"/>
     <child link="D435i_camera_depth_frame"/>
   </joint>
   <link name="D435i_camera_depth_frame"/>
   <joint name="D435i_camera_depth_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435i_camera_depth_frame"/>
     <child link="D435i_camera_depth_optical_frame"/>
   </joint>
   <link name="D435i_camera_depth_optical_frame"/>
   <!-- camera left IR joints and links -->
   <joint name="D435i_camera_left_ir_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0.0 0"/>
     <parent link="D435i_camera_depth_frame"/>
     <child link="D435i_camera_left_ir_frame"/>
   </joint>
   <link name="D435i_camera_left_ir_frame"/>
   <joint name="D435i_camera_left_ir_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435i_camera_left_ir_frame"/>
     <child link="D435i_camera_left_ir_optical_frame"/>
   </joint>
   <link name="D435i_camera_left_ir_optical_frame"/>
   <!-- camera right IR joints and links -->
   <joint name="D435i_camera_right_ir_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 -0.05 0"/>
     <parent link="D435i_camera_depth_frame"/>
     <child link="D435i_camera_right_ir_frame"/>
   </joint>
   <link name="D435i_camera_right_ir_frame"/>
   <joint name="D435i_camera_right_ir_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435i_camera_right_ir_frame"/>
     <child link="D435i_camera_right_ir_optical_frame"/>
   </joint>
   <link name="D435i_camera_right_ir_optical_frame"/>
   <!-- camera color joints and links -->
   <joint name="D435i_camera_color_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0.015 0"/>
     <parent link="D435i_camera_depth_frame"/>
     <child link="D435i_camera_color_frame"/>
   </joint>
   <link name="D435i_camera_color_frame"/>
   <joint name="D435i_camera_color_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435i_camera_color_frame"/>
     <child link="D435i_camera_color_optical_frame"/>
   </joint>
   <link name="D435i_camera_color_optical_frame"/>
   <!-- camera accel joints and links -->
   <joint name="D435i_camera_accel_joint" type="fixed">
     <origin rpy="0 0 0" xyz="-0.00174 0.00552 0.0176"/>
     <parent link="D435i_camera_link"/>
     <child link="D435i_camera_accel_frame"/>
   </joint>
   <link name="D435i_camera_accel_frame"/>
   <joint name="D435i_camera_accel_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435i_camera_accel_frame"/>
     <child link="D435i_camera_accel_optical_frame"/>
   </joint>
   <link name="D435i_camera_accel_optical_frame"/>
   <!-- camera gyro joints and links -->
   <joint name="D435i_camera_gyro_joint" type="fixed">
     <origin rpy="0 0 0" xyz="-0.00174 0.00552 0.0176"/>
     <parent link="D435i_camera_link"/>
     <child link="D435i_camera_gyro_frame"/>
   </joint>
   <link name="D435i_camera_gyro_frame"/>
   <joint name="D435i_camera_gyro_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="D435i_camera_gyro_frame"/>
     <child link="D435i_camera_gyro_optical_frame"/>
   </joint>
   <link name="D435i_camera_gyro_optical_frame"/>
   <gazebo reference="D435i_camera_link">
     <self_collide>0</self_collide>
     <enable_wind>0</enable_wind>
     <kinematic>0</kinematic>
     <!-- <gravity>1</gravity> -->
     <!--<mu>1</mu>-->
     <mu2>1</mu2>
     <fdir1>0 0 0</fdir1>
     <!--<slip1>0</slip1>
       <slip2>0</slip2>-->
     <kp>1e+13</kp>
     <kd>1</kd>
     <!--<max_vel>0.01</max_vel>
       <min_depth>0</min_depth>-->
     <sensor name="D435i_cameracolor" type="camera">
       <camera name="D435i_camera">
         <horizontal_fov>1.2112585008840648</horizontal_fov>
         <image>
           <width>1280</width>
           <height>720</height>
           <format>RGB_INT8</format>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.007</stddev>
         </noise>
       </camera>
       <always_on>1</always_on>
       <update_rate>30</update_rate>
       <visualize>True</visualize>
     </sensor>
     <sensor name="D435i_cameraired1" type="camera">
       <camera name="D435i_camera">
         <horizontal_fov>1.4870205226991688</horizontal_fov>
         <image>
           <width>1280</width>
           <height>720</height>
           <format>L_INT8</format>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.05</stddev>
         </noise>
       </camera>
       <always_on>1</always_on>
       <update_rate>90</update_rate>
       <visualize>false</visualize>
     </sensor>
     <sensor name="D435i_cameraired2" type="camera">
       <camera name="D435i_camera">
         <horizontal_fov>1.4870205226991688</horizontal_fov>
         <image>
           <width>1280</width>
           <height>720</height>
           <format>L_INT8</format>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.05</stddev>
         </noise>
       </camera>
       <always_on>1</always_on>
       <update_rate>90</update_rate>
       <visualize>false</visualize>
     </sensor>
     <sensor name="D435i_cameradepth" type="depth">
       <camera name="D435i_camera">
         <horizontal_fov>1.2112585008840648</horizontal_fov>
         <image>
           <width>1280</width>
           <height>720</height>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
         <noise>
           <type>gaussian</type>
           <mean>0.0</mean>
           <stddev>0.100</stddev>
         </noise>
       </camera>
       <always_on>1</always_on>
       <update_rate>30</update_rate>
       <visualize>false</visualize>
     </sensor>
     <sensor name="D435i_cameraaccel" type="imu">
       <always_on>true</always_on>
       <update_rate>300</update_rate>
       <topic>D435i_camera/accel/sample</topic>
       <plugin filename="librealsense_gazebo_accel_sensor.so" name="D435i/D435i_camera_accel_plugin">
         <topicName>D435i_camera/accel/sample</topicName>
         <bodyName>D435i_camera_link</bodyName>
         <updateRateHZ>300</updateRateHZ>
         <gaussianNoise>0.1</gaussianNoise>
         <xyzOffset>0 0 0</xyzOffset>
         <rpyOffset>0 0 0</rpyOffset>
         <frameName>D435i_camera_accel_optical_frame</frameName>
         <initialOrientationAsReference>false</initialOrientationAsReference>
       </plugin>
       <pose>0 0 0 0 0 0</pose>
     </sensor>
     <sensor name="D435i_cameragyro" type="imu">
       <always_on>true</always_on>
       <update_rate>300</update_rate>
       <topic>D435i_camera/gyro/sample</topic>
       <plugin filename="librealsense_gazebo_gyro_sensor.so" name="D435i/D435i_camera_gyro_plugin">
         <topicName>D435i_camera/gyro/sample</topicName>
         <bodyName>D435i_camera_link</bodyName>
         <updateRateHZ>300</updateRateHZ>
         <gaussianNoise>0.1</gaussianNoise>
         <xyzOffset>0 0 0</xyzOffset>
         <rpyOffset>0 0 0</rpyOffset>
         <frameName>D435i_camera_gyro_optical_frame</frameName>
         <initialOrientationAsReference>false</initialOrientationAsReference>
       </plugin>
     </sensor>
   </gazebo>
   <gazebo>
     <plugin filename="librealsense_gazebo_plugin.so" name="D435i/D435i_camera">
       <prefix>D435i_camera</prefix>
       <!-- Color camera settings -->
       <colorUpdateRate>30</colorUpdateRate>
       <colorTopicName>color/image_raw</colorTopicName>
       <colorOpticalframeName>D435i_camera_color_optical_frame</colorOpticalframeName>
       <colorCameraInfoTopicName>color/camera_info</colorCameraInfoTopicName>
       <!-- Infrared camera settings -->
       <infraredUpdateRate>90</infraredUpdateRate>
       <infrared1TopicName>infra1/image_rect_raw</infrared1TopicName>
       <infrared2TopicName>infra2/image_rect_raw</infrared2TopicName>
       <infrared1CameraInfoTopicName>infra1/camera_info</infrared1CameraInfoTopicName>
       <infrared2CameraInfoTopicName>infra2/camera_info</infrared2CameraInfoTopicName>
       <infrared1OpticalframeName>D435i_camera_left_ir_optical_frame</infrared1OpticalframeName>
       <infrared2OpticalframeName>D435i_camera_right_ir_optical_frame</infrared2OpticalframeName>
       <!-- Depth camera settings -->
       <rangeMinDepth>0.2</rangeMinDepth>
       <rangeMaxDepth>10.0</rangeMaxDepth>
       <depthUpdateRate>30</depthUpdateRate>
       <depthTopicName>aligned_depth_to_color/image_raw</depthTopicName>
       <depthCameraInfoTopicName>aligned_depth_to_color/camera_info</depthCameraInfoTopicName>
       <depthOpticalframeName>D435i_camera_color_optical_frame</depthOpticalframeName>
       <!-- Pointlcloud settings -->
       <pointCloud>True</pointCloud>
       <pointCloudTopicName>depth/color/points</pointCloudTopicName>
       <pointCloudCutoff>0.5</pointCloudCutoff>
     </plugin>
   </gazebo>
   <material name="T265_camera_aluminum">
     <color rgba="0.5 0.5 0.5 1"/>
   </material>
   <!-- camera body, with origin at camera_pose_frame -->
   <joint name="T265_camera_pose_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0.29245 0.00775 0.063"/>
     <parent link="base_link"/>
     <child link="T265_camera_pose_frame"/>
   </joint>
   <link name="T265_camera_pose_frame">
     <visual>
       <origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0 0 0"/>
       <geometry>
         <mesh filename="package://realsense_gazebo_description/meshes/t265_full.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="T265_camera_aluminum"/>
     </visual>
     <collision>
       <origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0 0 0"/>
       <geometry>
         <mesh filename="package://realsense_gazebo_description/meshes/t265_simplified.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
     <inertial>
       <!-- The following are not reliable values, and should not be used for modeling -->
       <mass value="0.068024"/>
       <origin xyz="0 0 0"/>
       <inertia ixx="0.0000039782" ixy="0.0" ixz="0.000000034641" iyy="0.000065045" iyz="0.0" izz="0.000067499"/>
     </inertial>
   </link>
   <!-- camera body, with origin at camera_link -->
   <joint name="T265_camera_link_joint" type="fixed">
     <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="T265_camera_pose_frame"/>
     <child link="T265_camera_link"/>
   </joint>
   <link name="T265_camera_link"/>
   <joint name="T265_camera_fisheye1_rgb_joint" type="fixed">
     <origin rpy="0 1.5707963267948966 1.5707963267948966" xyz="-0.0411 0.0 0.0"/>
     <parent link="T265_camera_link"/>
     <child link="T265_camera_fisheye1_rgb_frame"/>
   </joint>
   <link name="T265_camera_fisheye1_rgb_frame"/>
   <joint name="T265_camera_fisheye2_rgb_joint" type="fixed">
     <origin rpy="0 1.5707963267948966 1.5707963267948966" xyz="0.0229 0.0 0.0"/>
     <parent link="T265_camera_link"/>
     <child link="T265_camera_fisheye2_rgb_frame"/>
   </joint>
   <link name="T265_camera_fisheye2_rgb_frame"/>
   <joint name="T265_camera_fisheye1_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="T265_camera_fisheye1_rgb_frame"/>
     <child link="T265_camera_fisheye1_optical_frame"/>
   </joint>
   <link name="T265_camera_fisheye1_optical_frame"/>
   <joint name="T265_camera_fisheye2_optical_joint" type="fixed">
     <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
     <parent link="T265_camera_fisheye2_rgb_frame"/>
     <child link="T265_camera_fisheye2_optical_frame"/>
   </joint>
   <link name="T265_camera_fisheye2_optical_frame"/>
   <!-- IMU joints and links -->
   <joint name="T265_camera_accel_joint" type="fixed">
     <origin rpy="0 3.141592653589793 0" xyz="-0.0311 0 0.00655"/>
     <parent link="T265_camera_link"/>
     <child link="T265_camera_accel_frame"/>
   </joint>
   <link name="T265_camera_accel_frame"/>
   <joint name="T265_camera_accel_optical_joint" type="fixed">
     <origin rpy="0 3.141592653589793 0" xyz="-0.0311 0 0.00655"/>
     <parent link="T265_camera_accel_frame"/>
     <child link="T265_camera_accel_optical_frame"/>
   </joint>
   <link name="T265_camera_accel_optical_frame"/>
   <joint name="T265_camera_gyro_joint" type="fixed">
     <origin rpy="0 3.141592653589793 0" xyz="-0.0311 0 0.00655"/>
     <parent link="T265_camera_link"/>
     <child link="T265_camera_gyro_frame"/>
   </joint>
   <link name="T265_camera_gyro_frame"/>
   <joint name="T265_camera_gyro_optical_joint" type="fixed">
     <origin rpy="0 0 0" xyz="0 0 0"/>
     <parent link="T265_camera_gyro_frame"/>
     <child link="T265_camera_gyro_optical_frame"/>
   </joint>
   <link name="T265_camera_gyro_optical_frame"/>
   <!-- Load parameters to model's main link-->
   <gazebo reference="T265_camera_fisheye1_rgb_frame">
     <self_collide>0</self_collide>
     <enable_wind>0</enable_wind>
     <kinematic>0</kinematic>
     <!-- <gravity>1</gravity> -->
     <!--<mu>1</mu>-->
     <mu2>1</mu2>
     <fdir1>0 0 0</fdir1>
     <!--<slip1>0</slip1>
       <slip2>0</slip2>-->
     <kp>1e+13</kp>
     <kd>1</kd>
     <!--<max_vel>0.01</max_vel>
       <min_depth>0</min_depth>-->
     <sensor name="T265_camerafisheye1" type="wideanglecamera">
       <!-- <origin xyz="0 0 0" rpy="0 ${M_PI/2} 0"/> -->
       <update_rate>30</update_rate>
       <camera>
         <horizontal_fov>1.95</horizontal_fov>
         <image>
           <width>848</width>
           <height>800</height>
         </image>
         <clip>
           <near>0.2</near>
           <far>100</far>
         </clip>
         <lens>
           <!-- type element is mandatory -->
           <type>orthographic</type>
           <!-- if it is set to `true` your horizontal FOV will remain as defined -->
           <!-- othervise it depends on lens type and custom function, if there is one -->
           <scale_to_hfov>true</scale_to_hfov>
           <!-- clip everything that is outside of this angle -->
           <!-- eye balled value for these cameras-->
           <cutoff_angle>2.20</cutoff_angle>
           <!-- resolution of the cubemap texture, the highter it is - the sharper is your image -->
           <env_texture_size>1024</env_texture_size>
         </lens>
       </camera>
       <plugin filename="libgazebo_ros_camera.so" name="T265_camerafisheye1">
         <alwaysOn>true</alwaysOn>
         <updateRate>30</updateRate>
         <cameraName>T265/T265_camera/fisheye1</cameraName>
         <imageTopicName>image_raw</imageTopicName>
         <cameraInfoTopicName>camera_info</cameraInfoTopicName>
         <frameName>T265_camera_fisheye1_optical_frame</frameName>
         <hackBaseline>0.07</hackBaseline>
         <distortionK1>-0.00040996368625201285</distortionK1>
         <distortionK2>0.03653175011277199</distortionK2>
         <distortionK3>0.0</distortionK3>
         <distortionP1>-0.034823670983314514</distortionP1>
         <distortionP2>0.0052825710736215115</distortionP2>
         <Cx>416.22</Cx>
         <Cy>402.1</Cy>
         <CxPrime>416.22</CxPrime>
       </plugin>
     </sensor>
   </gazebo>
   <gazebo reference="T265_camera_fisheye2_rgb_frame">
     <self_collide>0</self_collide>
     <enable_wind>0</enable_wind>
     <kinematic>0</kinematic>
     <!-- <gravity>1</gravity> -->
     <!--<mu>1</mu>-->
     <mu2>1</mu2>
     <fdir1>0 0 0</fdir1>
     <!--<slip1>0</slip1>
       <slip2>0</slip2>-->
     <kp>1e+13</kp>
     <kd>1</kd>
     <!--<max_vel>0.01</max_vel>
       <min_depth>0</min_depth>-->
     <sensor name="T265_camerafisheye2" type="wideanglecamera">
       <!-- <origin xyz="0 0 0" rpy="0 ${M_PI/2} 0"/> -->
       <update_rate>30</update_rate>
       <camera>
         <horizontal_fov>2.84</horizontal_fov>
         <image>
           <width>848</width>
           <height>800</height>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
         <lens>
           <!-- type element is mandatory -->
           <type>orthographic</type>
           <!-- if it is set to `true` your horizontal FOV will remain as defined -->
           <!-- othervise it depends on lens type and custom function, if there is one -->
           <scale_to_hfov>true</scale_to_hfov>
           <!-- clip everything that is outside of this angle -->
           <!-- eye balled value for these cameras-->
           <cutoff_angle>2.20</cutoff_angle>
           <!-- resolution of the cubemap texture, the highter it is - the sharper is your image -->
           <env_texture_size>1024</env_texture_size>
         </lens>
       </camera>
       <always_on>1</always_on>
       <plugin filename="libgazebo_ros_camera.so" name="T265_camerafisheye2">
         <alwaysOn>true</alwaysOn>
         <updateRate>30</updateRate>
         <cameraName>T265/T265_camera/fisheye2</cameraName>
         <imageTopicName>image_raw</imageTopicName>
         <cameraInfoTopicName>camera_info</cameraInfoTopicName>
         <frameName>T265_camera_fisheye2_optical_frame</frameName>
         <hackBaseline>0.07</hackBaseline>
         <distortionK1>-0.00040996368625201285</distortionK1>
         <distortionK2>0.03653175011277199</distortionK2>
         <distortionK3>0.0</distortionK3>
         <distortionP1>-0.034823670983314514</distortionP1>
         <distortionP2>0.0052825710736215115</distortionP2>
       </plugin>
     </sensor>
     <sensor name="T265_cameraaccel" type="imu">
       <always_on>true</always_on>
       <update_rate>62.5</update_rate>
       <plugin filename="librealsense_gazebo_accel_sensor.so" name="T265/T265_camera_accel_plugin">
         <topicName>T265_camera/accel/sample</topicName>
         <bodyName>T265_camera_pose_frame</bodyName>
         <updateRateHZ>62.5</updateRateHZ>
         <gaussianNoise>0.1</gaussianNoise>
         <xyzOffset>0 0 0</xyzOffset>
         <rpyOffset>0 0 0</rpyOffset>
         <frameName>T265_camera_accel_optical_frame</frameName>
         <initialOrientationAsReference>false</initialOrientationAsReference>
       </plugin>
       <pose>0 0 0 0 0 0</pose>
     </sensor>
     <sensor name="T265_cameragyro" type="imu">
       <always_on>true</always_on>
       <update_rate>200</update_rate>
       <topic>T265/T265_camera/gyro/sample</topic>
       <plugin filename="librealsense_gazebo_gyro_sensor.so" name="T265/T265_camera_gyro_plugin">
         <topicName>T265_camera/gyro/sample</topicName>
         <bodyName>T265_camera_pose_frame</bodyName>
         <updateRateHZ>200</updateRateHZ>
         <gaussianNoise>0.1</gaussianNoise>
         <xyzOffset>0 0 0</xyzOffset>
         <rpyOffset>0 0 0</rpyOffset>
         <frameName>T265_camera_gyro_optical_frame</frameName>
         <initialOrientationAsReference>false</initialOrientationAsReference>
       </plugin>
     </sensor>
   </gazebo>
   <gazebo>
     <!-- Odom from SLAM -->
     <plugin filename="libgazebo_ros_p3d.so" name="p3d_base_controller">
       <alwaysOn>true</alwaysOn>
       <updateRate>262</updateRate>
       <bodyName>T265_camera_pose_frame</bodyName>
       <topicName>T265/T265_camera/odom/sample</topicName>
       <gaussianNoise>0.01</gaussianNoise>
       <frameName>world</frameName>
       <xyzOffsets>0.0 0.0 0.25</xyzOffsets>
       <rpyOffsets>0.0 0.0 0.0</rpyOffsets>
     </plugin>
   </gazebo>
   <!-- /macro legs -->
   <link name="hip1_1">
     <inertial>
       <origin xyz="0.0003595855 0.033338818000000006 -0.059190309"/>
       <mass value="2.3349153"/>
       <!-- org: 2 -->
       <inertia ixx="0.0068405374" ixy="1.8572633e-05" ixz="-4.8325972e-05" iyy="0.0026261815" iyz="0.0001564273" izz="0.0066933354"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/hip-yaw.stl" scale="0.001 -0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/hip-yaw.stl" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="hip2_1">
     <inertial>
       <origin xyz="-4.6285952e-06 -0.2214019 0.03316685"/>
       <mass value="3.4572118"/>
       <!-- org: 4.5 -->
       <inertia ixx="0.050469853" ixy="-7.784938699999999e-06" ixz="-9.220580999999999e-06" iyy="0.008181596999999999" iyz="0.0059796848" izz="0.048213808999999996"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/hip-pitch.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/hip-pitch.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="knee_1">
     <inertial>
       <origin xyz="0.00012063381 0.14620674 0.029740711000000003"/>
       <mass value="2.1199315"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.018426402999999997" ixy="2.8226635999999997e-05" ixz="-2.9204348e-05" iyy="0.0047685615" iyz="-0.002640771" izz="0.017793722"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/knee.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/knee.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="ankle1_1">
     <inertial>
       <origin xyz="0.00012360094 -0.06274712 -0.0049068453"/>
       <mass value="1.4023368"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.0049928829" ixy="5.762355699999999e-06" ixz="1.9606394e-05" iyy="0.0017621464" iyz="0.00045315091" izz="0.0048706328"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/ankle-pitch.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/ankle-pitch.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="ankle2_1">
     <inertial>
       <origin xyz="0.0018496706 -0.00051322371 -0.06730924"/>
       <mass value="1.7656714"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.0073976286" ixy="9.7557687e-07" ixz="-0.0002295255" iyy="0.0054434358" iyz="3.6337754e-05" izz="0.0043055053"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/ankle-yaw.stl" scale="0.001 -0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/ankle-yaw.stl" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="wheel_1">
     <inertial>
       <origin xyz="0.0 0.0 -0.0031155904"/>
       <mass value="1.11164"/>
       <inertia ixx="0.0028174054" ixy="0.0" ixz="-0.0" iyy="0.0028175648" iyz="-0.0" izz="0.0041817161999999995"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/wheel.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
       <material name="black"/>
     </visual>
     <collision>
       <origin rpy="0 0 0" xyz="0 0 0"/>
       <geometry>
         <cylinder length="0.11" radius="0.078"/>
       </geometry>
     </collision>
   </link>
   <!-- *****************************-->
   <link name="contact_1"/>
   <joint name="contact_joint_1" type="fixed">
     <parent link="ankle2_1"/>
     <child link="contact_1"/>
     <origin rpy="0 0 0" xyz="0 0 -0.176"/>
   </joint>
   <!-- ******************************** -->
   <joint name="hip_yaw_1" type="revolute">
     <parent link="base_link"/>
     <child link="hip1_1"/>
     <origin rpy="0.0 0.0 0.0" xyz="0.25 0.125 -0.056600000000000004"/>
     <axis xyz="0 0 -1"/>
     <limit effort="304" lower="-2.03" upper="2.51" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="hip_pitch_1" type="revolute">
     <parent link="hip1_1"/>
     <child link="hip2_1"/>
     <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 0.1146 -0.0625"/>
     <axis xyz="0 0 -1"/>
     <limit effort="304" lower="-2.0226" upper="2.02748" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="knee_pitch_1" type="revolute">
     <parent link="hip2_1"/>
     <child link="knee_1"/>
     <origin rpy="3.14159265359 0.0 0.0" xyz="0.0 -0.3 0.1182"/>
     <axis xyz="0 0 -1"/>
     <limit effort="304" lower="-2.4056" upper="2.3994" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="ankle_pitch_1" type="revolute">
     <parent link="knee_1"/>
     <child link="ankle1_1"/>
     <origin rpy="3.14159265359 0.0 0.0" xyz="0.0 0.2 0.10015"/>
     <axis xyz="0 0 -1"/>
     <limit effort="147" lower="-2.4056" upper="2.3184" velocity="8.1"/>
     <!-- 98 Nm, 1 rad/s-->
   </joint>
   <joint name="ankle_yaw_1" type="revolute">
     <parent link="ankle1_1"/>
     <child link="ankle2_1"/>
     <origin rpy="-1.57079632679 0.0 0.0" xyz="0.0 -0.139 -0.00085"/>
     <axis xyz="0 0 -1"/>
     <limit effort="35" lower="-2.5626" upper="2.5384" velocity="20.0"/>
     <!-- 23 Nm, 1 rad/s-->
   </joint>
   <joint name="j_wheel_1" type="revolute">
     <parent link="ankle2_1"/>
     <child link="wheel_1"/>
     <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 -0.0 -0.09834999999999999"/>
     <axis xyz="0 0 -1"/>
     <limit effort="35" velocity="20.0" lower="-62.83" upper="62.83"/>
     <!-- 23 Nm, 1 rad/s-->
   </joint>
   <link name="hip1_2">
     <inertial>
       <origin xyz="0.0003595855 -0.033338818000000006 -0.059190309"/>
       <mass value="2.3349153"/>
       <!-- org: 2 -->
       <inertia ixx="0.0068405374" ixy="-1.8572633e-05" ixz="-4.8325972e-05" iyy="0.0026261815" iyz="-0.0001564273" izz="0.0066933354"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/hip-yaw.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/hip-yaw.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="hip2_2">
     <inertial>
       <origin xyz="-4.6285952e-06 -0.2214019 -0.03316685"/>
       <mass value="3.4572118"/>
       <!-- org: 4.5 -->
       <inertia ixx="0.050469853" ixy="-7.784938699999999e-06" ixz="9.220580999999999e-06" iyy="0.008181596999999999" iyz="-0.0059796848" izz="0.048213808999999996"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/hip-pitch.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/hip-pitch.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="knee_2">
     <inertial>
       <origin xyz="0.00012063381 0.14620674 -0.029740711000000003"/>
       <mass value="2.1199315"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.018426402999999997" ixy="2.8226635999999997e-05" ixz="2.9204348e-05" iyy="0.0047685615" iyz="0.002640771" izz="0.017793722"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/knee.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/knee.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="ankle1_2">
     <inertial>
       <origin xyz="0.00012360094 -0.06274712 0.0049068453"/>
       <mass value="1.4023368"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.0049928829" ixy="5.762355699999999e-06" ixz="-1.9606394e-05" iyy="0.0017621464" iyz="-0.00045315091" izz="0.0048706328"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/ankle-pitch.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/ankle-pitch.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="ankle2_2">
     <inertial>
       <origin xyz="0.0018496706 0.00051322371 -0.06730924"/>
       <mass value="1.7656714"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.0073976286" ixy="-9.7557687e-07" ixz="-0.0002295255" iyy="0.0054434358" iyz="-3.6337754e-05" izz="0.0043055053"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/ankle-yaw.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/ankle-yaw.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="wheel_2">
     <inertial>
       <origin xyz="0.0 0.0 0.0031155904"/>
       <mass value="1.11164"/>
       <inertia ixx="0.0028174054" ixy="0.0" ixz="0.0" iyy="0.0028175648" iyz="0.0" izz="0.0041817161999999995"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/wheel.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="black"/>
     </visual>
     <collision>
       <origin rpy="0 0 0" xyz="0 0 0"/>
       <geometry>
         <cylinder length="0.11" radius="0.078"/>
       </geometry>
     </collision>
   </link>
   <!-- *****************************-->
   <link name="contact_2"/>
   <joint name="contact_joint_2" type="fixed">
     <parent link="ankle2_2"/>
     <child link="contact_2"/>
     <origin rpy="0 0 0" xyz="0 0 -0.176"/>
   </joint>
   <!-- ******************************** -->
   <joint name="hip_yaw_2" type="revolute">
     <parent link="base_link"/>
     <child link="hip1_2"/>
     <origin rpy="0.0 0.0 0.0" xyz="0.25 -0.125 -0.056600000000000004"/>
     <axis xyz="0 0 -1"/>
     <limit effort="304" lower="-2.51" upper="2.03" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="hip_pitch_2" type="revolute">
     <parent link="hip1_2"/>
     <child link="hip2_2"/>
     <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 -0.1146 -0.0625"/>
     <axis xyz="0 0 1"/>
     <limit effort="304" lower="-2.0156" upper="2.0354" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="knee_pitch_2" type="revolute">
     <parent link="hip2_2"/>
     <child link="knee_2"/>
     <origin rpy="3.14159265359 0.0 0.0" xyz="0.0 -0.3 -0.1182"/>
     <axis xyz="0 0 1"/>
     <limit effort="304" lower="-2.4006" upper="2.4034" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="ankle_pitch_2" type="revolute">
     <parent link="knee_2"/>
     <child link="ankle1_2"/>
     <origin rpy="3.14159265359 0.0 0.0" xyz="0.0 0.2 -0.10015"/>
     <axis xyz="0 0 1"/>
     <limit effort="147" lower="-2.3266" upper="2.3914" velocity="8.1"/>
     <!-- 98 Nm, 1 rad/s-->
   </joint>
   <joint name="ankle_yaw_2" type="revolute">
     <parent link="ankle1_2"/>
     <child link="ankle2_2"/>
     <origin rpy="-1.57079632679 0.0 0.0" xyz="0.0 -0.139 0.00085"/>
     <axis xyz="0 0 -1"/>
     <limit effort="35" lower="-2.5546" upper="2.5484" velocity="20.0"/>
     <!-- 23 Nm, 1 rad/s-->
   </joint>
   <joint name="j_wheel_2" type="revolute">
     <parent link="ankle2_2"/>
     <child link="wheel_2"/>
     <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 0.0 -0.09834999999999999"/>
     <axis xyz="0 0 1"/>
     <limit effort="35" velocity="20.0" lower="-62.83" upper="62.83"/>
     <!-- 23 Nm, 1 rad/s-->
   </joint>
   <link name="hip1_3">
     <inertial>
       <origin xyz="0.0003595855 0.033338818000000006 -0.059190309"/>
       <mass value="2.3349153"/>
       <!-- org: 2 -->
       <inertia ixx="0.0068405374" ixy="1.8572633e-05" ixz="-4.8325972e-05" iyy="0.0026261815" iyz="0.0001564273" izz="0.0066933354"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/hip-yaw.stl" scale="0.001 -0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/hip-yaw.stl" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="hip2_3">
     <inertial>
       <origin xyz="-4.6285952e-06 -0.2214019 0.03316685"/>
       <mass value="3.4572118"/>
       <!-- org: 4.5 -->
       <inertia ixx="0.050469853" ixy="-7.784938699999999e-06" ixz="-9.220580999999999e-06" iyy="0.008181596999999999" iyz="0.0059796848" izz="0.048213808999999996"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/hip-pitch.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/hip-pitch.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="knee_3">
     <inertial>
       <origin xyz="0.00012063381 0.14620674 0.029740711000000003"/>
       <mass value="2.1199315"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.018426402999999997" ixy="2.8226635999999997e-05" ixz="-2.9204348e-05" iyy="0.0047685615" iyz="-0.002640771" izz="0.017793722"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/knee.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/knee.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="ankle1_3">
     <inertial>
       <origin xyz="0.00012360094 -0.06274712 -0.0049068453"/>
       <mass value="1.4023368"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.0049928829" ixy="5.762355699999999e-06" ixz="1.9606394e-05" iyy="0.0017621464" iyz="0.00045315091" izz="0.0048706328"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/ankle-pitch.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/ankle-pitch.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="ankle2_3">
     <inertial>
       <origin xyz="0.0018496706 -0.00051322371 -0.06730924"/>
       <mass value="1.7656714"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.0073976286" ixy="9.7557687e-07" ixz="-0.0002295255" iyy="0.0054434358" iyz="3.6337754e-05" izz="0.0043055053"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/ankle-yaw.stl" scale="0.001 -0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/ankle-yaw.stl" scale="0.001 -0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="wheel_3">
     <inertial>
       <origin xyz="0.0 0.0 -0.0031155904"/>
       <mass value="1.11164"/>
       <inertia ixx="0.0028174054" ixy="0.0" ixz="-0.0" iyy="0.0028175648" iyz="-0.0" izz="0.0041817161999999995"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/wheel.stl" scale="0.001 0.001 -0.001"/>
       </geometry>
       <material name="black"/>
     </visual>
     <collision>
       <origin rpy="0 0 0" xyz="0 0 0"/>
       <geometry>
         <cylinder length="0.11" radius="0.078"/>
       </geometry>
     </collision>
   </link>
   <!-- *****************************-->
   <link name="contact_3"/>
   <joint name="contact_joint_3" type="fixed">
     <parent link="ankle2_3"/>
     <child link="contact_3"/>
     <origin rpy="0 0 0" xyz="0 0 -0.176"/>
   </joint>
   <!-- ******************************** -->
   <joint name="hip_yaw_3" type="revolute">
     <parent link="base_link"/>
     <child link="hip1_3"/>
     <origin rpy="0.0 0.0 0.0" xyz="-0.25 0.125 -0.056600000000000004"/>
     <axis xyz="0 0 -1"/>
     <limit effort="304" lower="-2.51" upper="2.03" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="hip_pitch_3" type="revolute">
     <parent link="hip1_3"/>
     <child link="hip2_3"/>
     <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 0.1146 -0.0625"/>
     <axis xyz="0 0 -1"/>
     <limit effort="304" lower="-2.0126" upper="2.0384" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="knee_pitch_3" type="revolute">
     <parent link="hip2_3"/>
     <child link="knee_3"/>
     <origin rpy="3.14159265359 0.0 0.0" xyz="0.0 -0.3 0.1182"/>
     <axis xyz="0 0 -1"/>
     <limit effort="304" lower="-2.4056" upper="2.4064" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="ankle_pitch_3" type="revolute">
     <parent link="knee_3"/>
     <child link="ankle1_3"/>
     <origin rpy="3.14159265359 0.0 0.0" xyz="0.0 0.2 0.10015"/>
     <axis xyz="0 0 -1"/>
     <limit effort="147" lower="-2.4696" upper="2.4464" velocity="8.1"/>
     <!-- 98 Nm, 1 rad/s-->
   </joint>
   <joint name="ankle_yaw_3" type="revolute">
     <parent link="ankle1_3"/>
     <child link="ankle2_3"/>
     <origin rpy="-1.57079632679 0.0 0.0" xyz="0.0 -0.139 -0.00085"/>
     <axis xyz="0 0 -1"/>
     <limit effort="35" lower="-2.5606" upper="2.5454" velocity="20.0"/>
     <!-- 23 Nm, 1 rad/s-->
   </joint>
   <joint name="j_wheel_3" type="revolute">
     <parent link="ankle2_3"/>
     <child link="wheel_3"/>
     <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 -0.0 -0.09834999999999999"/>
     <axis xyz="0 0 -1"/>
     <limit effort="35" velocity="20.0" lower="-62.83" upper="62.83"/>
     <!-- 23 Nm, 1 rad/s-->
   </joint>
   <link name="hip1_4">
     <inertial>
       <origin xyz="0.0003595855 -0.033338818000000006 -0.059190309"/>
       <mass value="2.3349153"/>
       <!-- org: 2 -->
       <inertia ixx="0.0068405374" ixy="-1.8572633e-05" ixz="-4.8325972e-05" iyy="0.0026261815" iyz="-0.0001564273" izz="0.0066933354"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/hip-yaw.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/hip-yaw.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="hip2_4">
     <inertial>
       <origin xyz="-4.6285952e-06 -0.2214019 -0.03316685"/>
       <mass value="3.4572118"/>
       <!-- org: 4.5 -->
       <inertia ixx="0.050469853" ixy="-7.784938699999999e-06" ixz="9.220580999999999e-06" iyy="0.008181596999999999" iyz="-0.0059796848" izz="0.048213808999999996"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/hip-pitch.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/hip-pitch.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="knee_4">
     <inertial>
       <origin xyz="0.00012063381 0.14620674 -0.029740711000000003"/>
       <mass value="2.1199315"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.018426402999999997" ixy="2.8226635999999997e-05" ixz="2.9204348e-05" iyy="0.0047685615" iyz="0.002640771" izz="0.017793722"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/knee.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/knee.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="ankle1_4">
     <inertial>
       <origin xyz="0.00012360094 -0.06274712 0.0049068453"/>
       <mass value="1.4023368"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.0049928829" ixy="5.762355699999999e-06" ixz="-1.9606394e-05" iyy="0.0017621464" iyz="-0.00045315091" izz="0.0048706328"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/ankle-pitch.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/ankle-pitch.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="ankle2_4">
     <inertial>
       <origin xyz="0.0018496706 0.00051322371 -0.06730924"/>
       <mass value="1.7656714"/>
       <!-- org: 3.5, foot un modeled-->
       <inertia ixx="0.0073976286" ixy="-9.7557687e-07" ixz="-0.0002295255" iyy="0.0054434358" iyz="-3.6337754e-05" izz="0.0043055053"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/ankle-yaw.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="dark_grey"/>
     </visual>
     <collision>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/collision/ankle-yaw.stl" scale="0.001 0.001 0.001"/>
       </geometry>
     </collision>
   </link>
   <link name="wheel_4">
     <inertial>
       <origin xyz="0.0 0.0 0.0031155904"/>
       <mass value="1.11164"/>
       <inertia ixx="0.0028174054" ixy="0.0" ixz="0.0" iyy="0.0028175648" iyz="0.0" izz="0.0041817161999999995"/>
     </inertial>
     <visual>
       <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
       <geometry>
         <mesh filename="package://centauro_urdf/meshes/v2/wheel.stl" scale="0.001 0.001 0.001"/>
       </geometry>
       <material name="black"/>
     </visual>
     <collision>
       <origin rpy="0 0 0" xyz="0 0 0"/>
       <geometry>
         <cylinder length="0.11" radius="0.078"/>
       </geometry>
     </collision>
   </link>
   <!-- *****************************-->
   <link name="contact_4"/>
   <joint name="contact_joint_4" type="fixed">
     <parent link="ankle2_4"/>
     <child link="contact_4"/>
     <origin rpy="0 0 0" xyz="0 0 -0.176"/>
   </joint>
   <!-- ******************************** -->
   <joint name="hip_yaw_4" type="revolute">
     <parent link="base_link"/>
     <child link="hip1_4"/>
     <origin rpy="0.0 0.0 0.0" xyz="-0.25 -0.125 -0.056600000000000004"/>
     <axis xyz="0 0 -1"/>
     <limit effort="304" lower="-2.03" upper="2.51" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="hip_pitch_4" type="revolute">
     <parent link="hip1_4"/>
     <child link="hip2_4"/>
     <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 -0.1146 -0.0625"/>
     <axis xyz="0 0 1"/>
     <limit effort="304" lower="-2.0326" upper="2.0524" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="knee_pitch_4" type="revolute">
     <parent link="hip2_4"/>
     <child link="knee_4"/>
     <origin rpy="3.14159265359 0.0 0.0" xyz="0.0 -0.3 -0.1182"/>
     <axis xyz="0 0 1"/>
     <limit effort="304" lower="-2.4176" upper="2.3884" velocity="8.8"/>
     <!-- 200 Nm, 1 rad/s-->
   </joint>
   <joint name="ankle_pitch_4" type="revolute">
     <parent link="knee_4"/>
     <child link="ankle1_4"/>
     <origin rpy="3.14159265359 0.0 0.0" xyz="0.0 0.2 -0.10015"/>
     <axis xyz="0 0 1"/>
     <limit effort="147" lower="-2.3426" upper="2.3934" velocity="8.1"/>
     <!-- 98 Nm, 1 rad/s-->
   </joint>
   <joint name="ankle_yaw_4" type="revolute">
     <parent link="ankle1_4"/>
     <child link="ankle2_4"/>
     <origin rpy="-1.57079632679 0.0 0.0" xyz="0.0 -0.139 0.00085"/>
     <axis xyz="0 0 -1"/>
     <limit effort="35" lower="-2.6046" upper="2.5514" velocity="20.0"/>
     <!-- 23 Nm, 1 rad/s-->
   </joint>
   <joint name="j_wheel_4" type="revolute">
     <parent link="ankle2_4"/>
     <child link="wheel_4"/>
     <origin rpy="1.57079632679 0.0 0.0" xyz="0.0 0.0 -0.09834999999999999"/>
     <axis xyz="0 0 1"/>
     <limit effort="35" velocity="20.0" lower="-62.83" upper="62.83"/>
     <!-- 23 Nm, 1 rad/s-->
   </joint>
   <!-- ************ Control frames ************ -->
   <!--link name="world"/-->
   <!--joint name="reference" type="floating">
     <parent link="world"/>
     <child link="base_link"/>
   </joint-->
</robot>
)";  // clang-format on
