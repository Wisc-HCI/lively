export const panda = `<?xml version="1.0" ?>
<!-- =================================================================================== -->
<!-- |    This document was autogenerated by xacro from franka_description/robots/panda_arm_hand.urdf.xacro | -->
<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
<!-- =================================================================================== -->
<robot name="panda">
  <link name="panda_link0">
  <visual>
    <geometry>
    <mesh filename="package://franka_ros/franka_description/meshes/visual/link0.dae"/>
  </geometry>
</visual>
<collision>
  <origin rpy="0 0 0" xyz="-0.04 0 0.06"/>
  <geometry>
    <box size="0.23 0.2 0.15"/>
  </geometry>
</collision>
  </link>
  <link name="panda_link1">
    <visual>
      <geometry>
        <mesh filename="package://franka_ros/franka_description/meshes/visual/link1.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0.0 -0.13"/>
      <geometry>
        <capsule length="0.06" radius="0.06"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 0"/>
      <geometry>
        <capsule length="0.135" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="3.875e-03 2.081e-03 -0.1750"/>
      <mass value="4.970684"/>
      <inertia ixx="7.0337e-01" ixy="-1.3900e-04" ixz="6.7720e-03" iyy="7.0661e-01" iyz="1.9169e-02" izz="9.1170e-03"/>
    </inertial>
  </link>
  <joint name="panda_joint1" type="revolute">
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/>
    <origin rpy="0 0 0" xyz="0 0 0.333"/>
    <parent link="panda_link0"/>
    <child link="panda_link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="87" lower="-2.8973" upper="2.8973" velocity="2.1750"/>
  </joint>
  <link name="panda_link2">
    <visual>
      <geometry>
        <mesh filename="package://franka_ros/franka_description/meshes/visual/link2.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="-1.5707963267948966 0 0" xyz="0 -0.18 0.0"/>
      <geometry>
        <capsule length="0.11" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-3.141e-03 -2.872e-02 3.495e-03"/>
      <mass value="0.646926"/>
      <inertia ixx="7.9620e-03" ixy="-3.9250e-03" ixz="1.0254e-02" iyy="2.8110e-02" iyz="7.0400e-04" izz="2.5995e-02"/>
    </inertial>
  </link>
  <joint name="panda_joint2" type="revolute">
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-1.7628" soft_upper_limit="1.7628"/>
    <origin rpy="-1.5707963267948966 0 0" xyz="0 0 0"/>
    <parent link="panda_link1"/>
    <child link="panda_link2"/>
    <axis xyz="0 0 1"/>
    <limit effort="87" lower="-1.7628" upper="1.7628" velocity="2.1750"/>
  </joint>
  <link name="panda_link3">
    <visual>
      <geometry>
        <mesh filename="package://franka_ros/franka_description/meshes/visual/link3.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0.08 0.0 0"/>
      <geometry>
        <capsule length="0.11" radius="0.055"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="2.7518e-02 3.9252e-02 -6.6502e-02"/>
      <mass value="3.228604"/>
      <inertia ixx="3.7242e-02" ixy="-4.7610e-03" ixz="-1.1396e-02" iyy="3.6155e-02" iyz="-1.2805e-02" izz="1.0830e-02"/>
    </inertial>
  </link>
  <joint name="panda_joint3" type="revolute">
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/>
    <origin rpy="1.5707963267948966 0 0" xyz="0 -0.316 0"/>
    <parent link="panda_link2"/>
    <child link="panda_link3"/>
    <axis xyz="0 0 1"/>
    <limit effort="87" lower="-2.8973" upper="2.8973" velocity="2.1750"/>
  </joint>
  <link name="panda_link4">
    <visual>
      <geometry>
        <mesh filename="package://franka_ros/franka_description/meshes/visual/link4.dae"/>
      </geometry>
    </visual>
    <inertial>
      <origin rpy="0 0 0" xyz="-5.317e-02 1.04419e-01 2.7454e-02"/>
      <mass value="3.587895"/>
      <inertia ixx="2.5853e-02" ixy="7.7960e-03" ixz="-1.3320e-03" iyy="1.9552e-02" iyz="8.6410e-03" izz="2.8323e-02"/>
    </inertial>
  </link>
  <joint name="panda_joint4" type="revolute">
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-3.0718" soft_upper_limit="-0.0698"/>
    <origin rpy="1.5707963267948966 0 0" xyz="0.0825 0 0"/>
    <parent link="panda_link3"/>
    <child link="panda_link4"/>
    <axis xyz="0 0 1"/>
    <limit effort="87" lower="-3.0718" upper="-0.0698" velocity="2.1750"/>
  </joint>
  <link name="panda_link5">
    <visual>
      <geometry>
        <mesh filename="package://franka_ros/franka_description/meshes/visual/link5.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0 -0.27"/>
      <geometry>
        <capsule length="0.09" radius="0.055"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="-0.2 0 0" xyz="0 0.08 -0.1"/>
      <geometry>
        <box size="0.07 0.07 0.2"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0.0 0.04 0"/>
      <geometry>
        <capsule length="0.1" radius="0.045"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-1.1953e-02 4.1065e-02 -3.8437e-02"/>
      <mass value="1.225946"/>
      <inertia ixx="3.5549e-02" ixy="-2.1170e-03" ixz="-4.0370e-03" iyy="2.9474e-02" iyz="2.2900e-04" izz="8.6270e-03"/>
    </inertial>
  </link>
  <joint name="panda_joint5" type="revolute">
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/>
    <origin rpy="-1.5707963267948966 0 0" xyz="-0.0825 0.384 0"/>
    <parent link="panda_link4"/>
    <child link="panda_link5"/>
    <axis xyz="0 0 1"/>
    <limit effort="12" lower="-2.8973" upper="2.8973" velocity="2.6100"/>
  </joint>
  <link name="panda_link6">
    <visual>
      <geometry>
        <mesh filename="package://franka_ros/franka_description/meshes/visual/link6.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0.09 0.014 0.0"/>
      <geometry>
        <cylinder length="0.12" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="6.0149e-02 -1.4117e-02 -1.0517e-02"/>
      <mass value="1.666555"/>
      <inertia ixx="1.9640e-03" ixy="1.0900e-04" ixz="-1.1580e-03" iyy="4.3540e-03" iyz="3.4100e-04" izz="5.4330e-03"/>
    </inertial>
  </link>
  <joint name="panda_joint6" type="revolute">
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-0.0175" soft_upper_limit="3.7525"/>
    <origin rpy="1.5707963267948966 0 0" xyz="0 0 0"/>
    <parent link="panda_link5"/>
    <child link="panda_link6"/>
    <axis xyz="0 0 1"/>
    <limit effort="12" lower="-0.0175" upper="3.7525" velocity="2.6100"/>
  </joint>
  <link name="panda_link7">
    <visual>
      <geometry>
        <mesh filename="package://franka_ros/franka_description/meshes/visual/link7.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0.005 0.07"/>
      <geometry>
        <cylinder length="0.04" radius="0.045"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="0 0 0.8" xyz="0.04 0.04 0.08"/>
      <geometry>
        <box size="0.07 0.07 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="1.0517e-02 -4.252e-03 6.1597e-02"/>
      <mass value="7.35522e-01"/>
      <inertia ixx="1.2516e-02" ixy="-4.2800e-04" ixz="-1.1960e-03" iyy="1.0027e-02" iyz="-7.4100e-04" izz="4.8150e-03"/>
    </inertial>
  </link>
  <joint name="panda_joint7" type="revolute">
    <safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/>
    <origin rpy="1.5707963267948966 0 0" xyz="0.088 0 0"/>
    <parent link="panda_link6"/>
    <child link="panda_link7"/>
    <axis xyz="0 0 1"/>
    <limit effort="12" lower="-2.8973" upper="2.8973" velocity="2.6100"/>
  </joint>
  <link name="panda_link8">
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.0"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
  </link>
  <joint name="panda_joint8" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0.107"/>
    <parent link="panda_link7"/>
    <child link="panda_link8"/>
  </joint>
  <joint name="panda_hand_joint" type="fixed">
    <parent link="panda_link8"/>
    <child link="panda_hand"/>
    <origin rpy="0 0 -0.7853981633974483" xyz="0 0 0"/>
  </joint>
  <link name="panda_hand">
    <visual>
      <geometry>
        <mesh filename="package://franka_ros/franka_description/meshes/visual/hand.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.03"/>
      <geometry>
        <box size="0.04 0.2 0.08"/>
      </geometry>
    </collision>
    <inertial>
        <origin rpy="0 0 0" xyz="0 0.0015244 0.0275912"/>
        <mass value="0.73"/>
        <inertia ixx="0.00278560230025" ixy="0.0" ixz="0.0" iyy="0.000400033405336" iyz="0.0" izz="0.00256378041832"/>
      </inertial>
  </link>
  <link name="panda_leftfinger">
    <visual>
      <geometry>
        <mesh filename="package://franka_ros/franka_description/meshes/visual/finger.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0.012 0.033"/>
      <geometry>
        <box size="0.02 0.025 0.041"/>
      </geometry>
    </collision>
    <inertial>
        <origin rpy="0 0 0" xyz="0 0.0145644 0.0227941"/>
        <mass value="0.1"/>
        <inertia ixx="3.01220925051e-05" ixy="0.0" ixz="0.0" iyy="2.95873808038e-05" iyz="0.0" izz="6.95125211657e-06"/>
    </inertial>
  </link>
  <link name="panda_rightfinger">
    <visual>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://franka_ros/franka_description/meshes/visual/finger.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 -0.012 0.033"/>
      <geometry>
        <box size="0.02 0.025 0.041"/>
      </geometry>
    </collision>
    <inertial>
        <origin rpy="0 0 3.141592653589793" xyz="0 -0.0145644 0.0227941"/>
        <mass value="0.1"/>
        <inertia ixx="3.01220925051e-05" ixy="0.0" ixz="0.0" iyy="2.95873808038e-05" iyz="0.0" izz="6.95125211657e-06"/>
      </inertial>
  </link>
  <joint name="panda_finger_joint1" type="prismatic">
    <parent link="panda_hand"/>
    <child link="panda_leftfinger"/>
    <origin rpy="0 0 0" xyz="0 0 0.0584"/>
    <axis xyz="0 1 0"/>
    <limit effort="20" lower="0.0" upper="0.04" velocity="0.2"/>
  </joint>
  <joint name="panda_finger_joint2" type="prismatic">
    <parent link="panda_hand"/>
    <child link="panda_rightfinger"/>
    <origin rpy="0 0 0" xyz="0 0 0.0584"/>
    <axis xyz="0 -1 0"/>
    <limit effort="20" lower="0.0" upper="0.04" velocity="0.2"/>
    <mimic joint="panda_finger_joint1"/>
  </joint>
</robot>`;

export const ur3e = `<?xml version="1.0" ?>
<!-- =================================================================================== -->
<!-- |    This document was autogenerated by xacro from src/universal_robot/ur_description/urdf/ur3.xacro | -->
<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
<!-- =================================================================================== -->
<robot name="ur3_robot">
  <!--
    Base UR robot series xacro macro.

    NOTE: this is NOT a URDF. It cannot directly be loaded by consumers
    expecting a flattened '.urdf' file. See the top-level '.xacro' for that
    (but note: that .xacro must still be processed by the xacro command).

    For use in '.launch' files: use one of the 'load_urX.launch' convenience
    launch files.

    This file models the base kinematic chain of a UR robot, which then gets
    parameterised by various configuration files to convert it into a UR3(e),
    UR5(e), UR10(e) or UR16e.

    NOTE: the default kinematic parameters (ie: link lengths, frame locations,
    offets, etc) do not correspond to any particular robot. They are defaults
    only. There WILL be non-zero offsets between the Forward Kinematics results
    in TF (ie: robot_state_publisher) and the values reported by the Teach
    Pendant.

    For accurate (and robot-specific) transforms, the 'kinematics_parameters_file'
    parameter MUST point to a .yaml file containing the appropriate values for
    the targetted robot.

    If using the UniversalRobots/Universal_Robots_ROS_Driver, follow the steps
    described in the readme of that repository to extract the kinematic
    calibration from the controller and generate the required .yaml file.

    Main author of the migration to yaml configs: Ludovic Delval.

    Contributors to previous versions (in no particular order):

     - Felix Messmer
     - Kelsey Hawkins
     - Wim Meeussen
     - Shaun Edwards
     - Nadia Hammoudeh Garcia
     - Dave Hershberger
     - G. vd. Hoorn
     - Philip Long
     - Dave Coleman
     - Miguel Prada
     - Mathias Luedtke
     - Marcel Schnirring
     - Felix von Drigalski
     - Felix Exner
     - Jimmy Da Silva
     - Ajit Krisshna N L
     - Muhammad Asif Rana
  -->
  <!--
    NOTE: the macro defined in this file is NOT part of the public API of this
          package. Users CANNOT rely on this file being available, or stored in
          this location. Nor can they rely on the existence of the macro.
  -->
  <!-- links: main serial chain -->
  <link name="base_link">
    <visual>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0.0"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/base.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0.04"/>
      <geometry>
        <cylinder length="0.09" radius="0.065"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0030531654454" ixy="0.0" ixz="0.0" iyy="0.0030531654454" iyz="0.0" izz="0.005625"/>
    </inertial>
  </link>
  <link name="shoulder_link">
    <visual>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/shoulder.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.11" radius="0.065"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.008093163429399999" ixy="0.0" ixz="0.0" iyy="0.008093163429399999" iyz="0.0" izz="0.005625"/>
    </inertial>
  </link>
  <link name="upper_arm_link">
    <visual>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.12"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/upperarm.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="-0.127 0 0.12"/>
      <geometry>
        <cylinder length="0.145" radius="0.05"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="0 0 -1.5707963267948966" xyz="0 0 0.125"/>
      <geometry>
        <cylinder length="0.11" radius="0.05"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="0 0 -1.5707963267948966" xyz="-0.245 0 0.125"/>
      <geometry>
        <cylinder length="0.11" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.42"/>
      <origin rpy="0 1.5707963267948966 0" xyz="-0.121825 0.0 0.12"/>
      <inertia ixx="0.021728483221103233" ixy="0.0" ixz="0.0" iyy="0.021728483221103233" iyz="0.0" izz="0.00961875"/>
    </inertial>
  </link>
  <link name="forearm_link">
    <visual>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.027"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/forearm.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="-0.07 0 0.027"/>
      <geometry>
        <cylinder length="0.20" radius="0.04"/>
      </geometry>
    </collision>
    <collision>
      <origin rpy="0 0 -1.5707963267948966" xyz="-0.215 0 0.027"/>
      <geometry>
        <cylinder length="0.095" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.26"/>
      <origin rpy="0 1.5707963267948966 0" xyz="-0.1066 0.0 0.027"/>
      <inertia ixx="0.0065445675821719194" ixy="0.0" ixz="0.0" iyy="0.0065445675821719194" iyz="0.0" izz="0.00354375"/>
    </inertial>
  </link>
  <link name="wrist_1_link">
    <visual>
      <!-- TODO: Move this to a parameter -->
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.104"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/wrist1.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.095" radius="0.035"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.002084999166" ixy="0.0" ixz="0.0" iyy="0.002084999166" iyz="0.0" izz="0.00225"/>
    </inertial>
  </link>
  <link name="wrist_2_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 -0.08535"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/wrist2.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 -0.008 0.005"/>
      <geometry>
        <cylinder length="0.1" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.002084999166" ixy="0.0" ixz="0.0" iyy="0.002084999166" iyz="0.0" izz="0.00225"/>
    </inertial>
  </link>
  <link name="wrist_3_link">
    <visual>
      <origin rpy="1.5707963267948966 0 0" xyz="0 -0.0 -0.081"/>
      <geometry>
        <mesh filename="package://ur_description/meshes/ur3/visual/wrist3.dae"/>
      </geometry>
      <material name="LightGrey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.005 0 -0.015"/>
      <geometry>
        <cylinder length="0.04" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.35"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 -0.02"/>
      <inertia ixx="0.00013626661215999998" ixy="0.0" ixz="0.0" iyy="0.00013626661215999998" iyz="0.0" izz="0.0001792"/>
    </inertial>
  </link>
  <!-- joints: main serial chain -->
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link_inertia"/>
    <child link="shoulder_link"/>
    <origin rpy="0 0 0" xyz="0 0 0.1519"/>
    <axis xyz="0 0 1"/>
    <limit effort="56.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin rpy="1.570796327 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="56.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin rpy="0 0 0" xyz="-0.24365 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="28.0" lower="-3.141592653589793" upper="3.141592653589793" velocity="3.141592653589793"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="wrist_1_joint" type="revolute">
    <parent link="forearm_link"/>
    <child link="wrist_1_link"/>
    <origin rpy="0 0 0" xyz="-0.21325 0 0.11235"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="6.283185307179586"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="wrist_2_joint" type="revolute">
    <parent link="wrist_1_link"/>
    <child link="wrist_2_link"/>
    <origin rpy="1.570796327 0 0" xyz="0 -0.08535 -1.750557762378351e-11"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="6.283185307179586"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link"/>
    <child link="wrist_3_link"/>
    <origin rpy="1.570796326589793 3.141592653589793 3.141592653589793" xyz="0 0.0819 -1.679797079540562e-11"/>
    <axis xyz="0 0 1"/>
    <limit effort="12.0" lower="-6.283185307179586" upper="6.283185307179586" velocity="6.283185307179586"/>
    <dynamics damping="0" friction="0"/>
  </joint>
  <!-- ROS-Industrial 'base' frame: base_link to UR 'Base' Coordinates transform -->
  <link name="base"/>
  <joint name="base_link-base_fixed_joint" type="fixed">
    <!-- Note the rotation over Z of pi radians: as base_link is REP-103
           aligned (ie: has X+ forward, Y+ left and Z+ up), this is needed
           to correctly align 'base' with the 'Base' coordinate system of
           the UR controller.
      -->
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="base"/>
  </joint>
  <!-- ROS-Industrial 'flange' frame: attachment point for EEF models -->
  <link name="flange"/>
  <joint name="wrist_3-flange" type="fixed">
    <parent link="wrist_3_link"/>
    <child link="flange"/>
    <origin rpy="0 -1.5707963267948966 -1.5707963267948966" xyz="0 0 0"/>
  </joint>
  <!-- ROS-Industrial 'tool0' frame: all-zeros tool frame -->
  <link name="tool0"/>
  <joint name="flange-tool0" type="fixed">
    <!-- default toolframe: X+ left, Y+ up, Z+ front -->
    <origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0 0 0"/>
    <parent link="flange"/>
    <child link="tool0"/>
  </joint>
  <joint name="robotiq_85_base_joint" type="fixed">
    <parent link="flange"/>
    <child link="robotiq_85_base_link"/>
  </joint>

  <link name="robotiq_85_base_link">
    <visual>
      <geometry>
        <mesh filename="package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.07 0 0"/>
      <geometry>
        <box size="0.12 0.12 0.07"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.636951" />
      <origin xyz="0.0 0.0 0.0" />
      <inertia ixx = "0.000380" ixy = "0.000000" ixz = "0.000000"
           iyx = "0.000000" iyy = "0.001110" iyz = "0.000000"
           izx = "0.000000" izy = "0.000000" izz = "0.001171" />
    </inertial>
  </link>

  <joint name="robotiq_85_left_knuckle_joint" type="revolute">
    <parent link="robotiq_85_base_link"/>
    <child link="robotiq_85_left_knuckle_link"/>
    <axis xyz="0 0 1"/>
    <origin rpy="3.14159265359 0.0 0.0" xyz="0.05490451627 0.03060114443 0.0"/>
    <limit effort="1000" lower="0.0" upper="0.80285" velocity="0.5"/>
  </joint>
  <joint name="robotiq_85_right_knuckle_joint" type="continuous">
    <parent link="robotiq_85_base_link"/>
    <child link="robotiq_85_right_knuckle_link"/>
    <axis xyz="0 0 1"/>
    <origin rpy="0.0 0.0 0.0" xyz="0.05490451627 -0.03060114443 0.0"/>
    <limit effort="1000" lower="-3.14" upper="3.14" velocity="100.0"/>
    <mimic joint="robotiq_85_left_knuckle_joint"/>
  </joint>
  <link name="robotiq_85_left_knuckle_link">
    <visual>
      <geometry>
        <mesh filename="package://robotiq_85_description/meshes/visual/robotiq_85_knuckle_link.dae"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.018491"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.000009" ixy="-0.000001" ixz="0.000000" iyy="0.000001" iyz="0.000000" izz="0.000010"/>
    </inertial>
  </link>
  <link name="robotiq_85_right_knuckle_link">
    <visual>
      <geometry>
        <mesh filename="package://robotiq_85_description/meshes/visual/robotiq_85_knuckle_link.dae"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.018491"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.000009" ixy="-0.000001" ixz="0.000000" iyy="0.000001" iyz="0.000000" izz="0.000010"/>
    </inertial>
  </link>

  <joint name="robotiq_85_left_finger_joint" type="fixed">
    <parent link="robotiq_85_left_knuckle_link"/>
    <child link="robotiq_85_left_finger_link"/>
    <origin rpy="0 0 0" xyz="-0.00408552455 -0.03148604435 0.0"/>
  </joint>
  <joint name="robotiq_85_right_finger_joint" type="fixed">
    <parent link="robotiq_85_right_knuckle_link"/>
    <child link="robotiq_85_right_finger_link"/>
    <origin rpy="0 0 0" xyz="-0.00408552455 -0.03148604435 0.0"/>
  </joint>
  <link name="robotiq_85_left_finger_link">
    <visual>
      <geometry>
        <mesh filename="package://robotiq_85_description/meshes/visual/robotiq_85_finger_link.dae"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.027309"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.000003" ixy="-0.000002" ixz="0.000000" iyy="0.000021" iyz="0.000000" izz="0.000020"/>
    </inertial>
  </link>
  <link name="robotiq_85_right_finger_link">
    <visual>
      <geometry>
        <mesh filename="package://robotiq_85_description/meshes/visual/robotiq_85_finger_link.dae"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.027309"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.000003" ixy="-0.000002" ixz="0.000000" iyy="0.000021" iyz="0.000000" izz="0.000020"/>
    </inertial>
  </link>

  <joint name="robotiq_85_left_inner_knuckle_joint" type="continuous">
    <parent link="robotiq_85_base_link"/>
    <child link="robotiq_85_left_inner_knuckle_link"/>
    <axis xyz="0 0 1"/>
    <origin rpy="3.14159265359 0.0 0.0" xyz="0.06142 0.0127 0"/>
    <limit effort="0.1" lower="-3.14" upper="3.14" velocity="100.0"/>
    <mimic joint="robotiq_85_left_knuckle_joint" offset="0"/>
  </joint>
  <joint name="robotiq_85_right_inner_knuckle_joint" type="continuous">
    <parent link="robotiq_85_base_link"/>
    <child link="robotiq_85_right_inner_knuckle_link"/>
    <axis xyz="0 0 1"/>
    <origin rpy="0 0 0" xyz="0.06142 -0.0127 0"/>
    <limit effort="0.1" lower="-3.14" upper="3.14" velocity="100.0"/>
    <mimic joint="robotiq_85_left_knuckle_joint" offset="0"/>
  </joint>
  <link name="robotiq_85_left_inner_knuckle_link">
    <visual>
      <geometry>
        <mesh filename="package://robotiq_85_description/meshes/visual/robotiq_85_inner_knuckle_link.dae"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.029951"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.000039" ixy="0.000000" ixz="0.000000" iyy="0.000005" iyz="0.000000" izz="0.000035"/>
    </inertial>
  </link>
  <link name="robotiq_85_right_inner_knuckle_link">
    <visual>
      <geometry>
        <mesh filename="package://robotiq_85_description/meshes/visual/robotiq_85_inner_knuckle_link.dae"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.029951"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.000039" ixy="0.000000" ixz="0.000000" iyy="0.000005" iyz="0.000000" izz="0.000035"/>
    </inertial>
  </link>

  <joint name="robotiq_85_left_finger_tip_joint" type="continuous">
    <parent link="robotiq_85_left_inner_knuckle_link"/>
    <child link="robotiq_85_left_finger_tip_link"/>
    <axis xyz="0 0 1"/>
    <origin rpy="0.0 0.0 0.0" xyz="0.04303959807 -0.03759940821 0.0"/>
    <limit effort="0.1" lower="-3.14" upper="3.14" velocity="100.0"/>
    <mimic joint="robotiq_85_left_knuckle_joint" multiplier="-1"/>
  </joint>
  <joint name="robotiq_85_right_finger_tip_joint" type="continuous">
    <parent link="robotiq_85_right_inner_knuckle_link"/>
    <child link="robotiq_85_right_finger_tip_link"/>
    <axis xyz="0 0 1"/>
    <origin rpy="0.0 0.0 0.0" xyz="0.04303959807 -0.03759940821  0.0"/>
    <limit effort="0.1" lower="-3.14" upper="3.14" velocity="100.0"/>
    <mimic joint="robotiq_85_left_knuckle_joint" multiplier="-1"/>
  </joint>
  <link name="robotiq_85_left_finger_tip_link">
    <visual>
      <geometry>
        <mesh filename="package://robotiq_85_description/meshes/visual/robotiq_85_finger_tip_link.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.03 0 0"/>
      <geometry>
        <box size="0.03 0.013 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.019555"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.000002" ixy="0.000000" ixz="0.000000" iyy="0.000005" iyz="0.000000" izz="0.000006"/>
    </inertial>
  </link>
  <link name="robotiq_85_right_finger_tip_link">
    <visual>
      <geometry>
        <mesh filename="package://robotiq_85_description/meshes/visual/robotiq_85_finger_tip_link.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.03 0 0"/>
      <geometry>
        <box size="0.03 0.013 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.019555"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.000002" ixy="0.000000" ixz="0.000000" iyy="0.000005" iyz="0.000000" izz="0.000006"/>
    </inertial>
  </link>
</robot>`;
