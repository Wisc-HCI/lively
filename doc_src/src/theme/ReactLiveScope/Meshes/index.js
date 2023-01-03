//----------------------------------------------Pepper

// import PepperCollisionCameraBase from "./Pepper/collision/Camera_base";
// import PepperCollisionCameraBody from "./Pepper/collision/Camera_body";
// import PepperCollisionCameraVisor from "./Pepper/collision/Camera_visor";
import PepperCollisionHeadPitch from "./Pepper/collision/HeadPitch";
import PepperCollisionHeadYaw from "./Pepper/collision/HeadYaw";
import PepperCollisionHipPitch from "./Pepper/collision/HipPitch";
import PepperCollisionHipRoll from "./Pepper/collision/HipRoll";
import PepperCollisionKneePitch from "./Pepper/collision/KneePitch";
import PepperCollisionLElbowRoll from "./Pepper/collision/LElbowRoll";
import PepperCollisionLElbowYaw from "./Pepper/collision/LElbowYaw";
import PepperCollisionLFinger11 from "./Pepper/collision/LFinger11";
import PepperCollisionLFinger12 from "./Pepper/collision/LFinger12";
import PepperCollisionLFinger13 from "./Pepper/collision/LFinger13";
import PepperCollisionLFinger21 from "./Pepper/collision/LFinger21";
import PepperCollisionLFinger22 from "./Pepper/collision/LFinger22";
import PepperCollisionLFinger23 from "./Pepper/collision/LFinger23";
import PepperCollisionLFinger31 from "./Pepper/collision/LFinger31";
import PepperCollisionLFinger32 from "./Pepper/collision/LFinger32";
import PepperCollisionLFinger33 from "./Pepper/collision/LFinger33";
import PepperCollisionLFinger41 from "./Pepper/collision/LFinger41";
import PepperCollisionLFinger42 from "./Pepper/collision/LFinger42";
import PepperCollisionLFinger43 from "./Pepper/collision/LFinger43";
import PepperCollisionLShoulderPitch from "./Pepper/collision/LShoulderPitch";
import PepperCollisionLShoulderRoll from "./Pepper/collision/LShoulderRoll";
import PepperCollisionLThumb1 from "./Pepper/collision/LThumb1";
import PepperCollisionLThumb2 from "./Pepper/collision/LThumb2";
import PepperCollisionLWristYaw from "./Pepper/collision/LWristYaw";
import PepperCollisionRElbowRoll from "./Pepper/collision/RElbowRoll";
import PepperCollisionRElbowYaw from "./Pepper/collision/RElbowYaw";
import PepperCollisionRFinger11 from "./Pepper/collision/RFinger11";
import PepperCollisionRFinger12 from "./Pepper/collision/RFinger12";
import PepperCollisionRFinger13 from "./Pepper/collision/RFinger13";
import PepperCollisionRFinger21 from "./Pepper/collision/RFinger21";
import PepperCollisionRFinger22 from "./Pepper/collision/RFinger22";
import PepperCollisionRFinger23 from "./Pepper/collision/RFinger23";
import PepperCollisionRFinger31 from "./Pepper/collision/RFinger31";
import PepperCollisionRFinger32 from "./Pepper/collision/RFinger32";
import PepperCollisionRFinger33 from "./Pepper/collision/RFinger33";
import PepperCollisionRFinger41 from "./Pepper/collision/RFinger41";
import PepperCollisionRFinger42 from "./Pepper/collision/RFinger42";
import PepperCollisionRFinger43 from "./Pepper/collision/RFinger43";
import PepperCollisionRShoulderPitch from "./Pepper/collision/RShoulderPitch";
import PepperCollisionRShoulderRoll from "./Pepper/collision/RShoulderRoll";
import PepperCollisionRThumb1 from "./Pepper/collision/RThumb1";
import PepperCollisionRThumb2 from "./Pepper/collision/RThumb2";
import PepperCollisionRWristYaw from "./Pepper/collision/RWristYaw";
//import PepperCollisionT265 from "./Pepper/collision/T265";
import PepperCollisionTorso from "./Pepper/collision/Torso";
import PepperCollisionWheelB from "./Pepper/collision/WheelB";
import PepperCollisionWheelFR from "./Pepper/collision/WheelFR";
import PepperCollisionWheelFL from "./Pepper/collision/WheelFL";

//import PepperVisualD435 from "./Pepper/visual/D435";
import PepperVisualHeadPitch from "./Pepper/visual/HeadPitch";
import PepperVisualHeadYaw from "./Pepper/visual/HeadYaw";
import PepperVisualHipPitch from "./Pepper/visual/HipPitch";
import PepperVisualHipRoll from "./Pepper/visual/HipRoll";
import PepperVisualKneePitch from "./Pepper/visual/KneePitch";
import PepperVisualLElbowRoll from "./Pepper/visual/LElbowRoll";
import PepperVisualLElbowYaw from "./Pepper/visual/LElbowYaw";
import PepperVisualLFinger11 from "./Pepper/visual/LFinger11";
import PepperVisualLFinger12 from "./Pepper/visual/LFinger12";
import PepperVisualLFinger13 from "./Pepper/visual/LFinger13";
import PepperVisualLFinger21 from "./Pepper/visual/LFinger21";
import PepperVisualLFinger22 from "./Pepper/visual/LFinger22";
import PepperVisualLFinger23 from "./Pepper/visual/LFinger23";
import PepperVisualLFinger31 from "./Pepper/visual/LFinger31";
import PepperVisualLFinger32 from "./Pepper/visual/LFinger32";
import PepperVisualLFinger33 from "./Pepper/visual/LFinger33";
import PepperVisualLFinger41 from "./Pepper/visual/LFinger41";
import PepperVisualLFinger42 from "./Pepper/visual/LFinger42";
import PepperVisualLFinger43 from "./Pepper/visual/LFinger43";
import PepperVisualLShoulderPitch from "./Pepper/visual/LShoulderPitch";
import PepperVisualLShoulderRoll from "./Pepper/visual/LShoulderRoll";
import PepperVisualLThumb1 from "./Pepper/visual/LThumb1";
import PepperVisualLThumb2 from "./Pepper/visual/LThumb2";
import PepperVisualLWristYaw from "./Pepper/visual/LWristYaw";
import PepperVisualRElbowRoll from "./Pepper/visual/RElbowRoll";
import PepperVisualRElbowYaw from "./Pepper/visual/LElbowYaw";
import PepperVisualRFinger11 from "./Pepper/visual/RFinger11";
import PepperVisualRFinger12 from "./Pepper/visual/RFinger12";
import PepperVisualRFinger13 from "./Pepper/visual/RFinger13";
import PepperVisualRFinger21 from "./Pepper/visual/RFinger21";
import PepperVisualRFinger22 from "./Pepper/visual/RFinger22";
import PepperVisualRFinger23 from "./Pepper/visual/RFinger23";
import PepperVisualRFinger31 from "./Pepper/visual/RFinger31";
import PepperVisualRFinger32 from "./Pepper/visual/RFinger32";
import PepperVisualRFinger33 from "./Pepper/visual/RFinger33";
import PepperVisualRFinger41 from "./Pepper/visual/RFinger41";
import PepperVisualRFinger42 from "./Pepper/visual/RFinger42";
import PepperVisualRFinger43 from "./Pepper/visual/RFinger43";
import PepperVisualRShoulderPitch from "./Pepper/visual/RShoulderPitch";
import PepperVisualRShoulderRoll from "./Pepper/visual/RShoulderRoll";
import PepperVisualRThumb1 from "./Pepper/visual/RThumb1";
import PepperVisualRThumb2 from "./Pepper/visual/RThumb2";
import PepperVisualRWristYaw from "./Pepper/visual/RWristYaw";
import PepperVisualTorso from "./Pepper/visual/Torso";
import PepperVisualWheelB from "./Pepper/visual/WheelB";
import PepperVisualWheelFL from "./Pepper/visual/WheelFL";
import PepperVisualWheelFR from "./Pepper/visual/WheelFR";


//---------------------------------------------- Panda
import Finger from './Panda/finger';
import Hand from './Panda/hand';
import Link0 from './Panda/link0';
import Link1 from './Panda/link1';
import Link2 from './Panda/link2';
import Link3 from './Panda/link3';
import Link4 from './Panda/link4';
import Link5 from './Panda/link5';
import Link6 from './Panda/link6';
import Link7 from './Panda/link7';
//---------------------------------------------- Robotiq2f85
import RobotiqCollision85BaseLink from
'./Robotiq2f85/collision/robotiq_arg2f_85_base_link';
import RobotiqCollision85InnerFinger from
'./Robotiq2f85/collision/robotiq_arg2f_85_inner_finger';
import RobotiqCollision85InnerKnuckle from
'./Robotiq2f85/collision/robotiq_arg2f_85_inner_knuckle';
import RobotiqCollision85OuterFinger from
'./Robotiq2f85/collision/robotiq_arg2f_85_outer_finger';
import RobotiqCollision85OuterKnuckle from
'./Robotiq2f85/collision/robotiq_arg2f_85_outer_knuckle';
import RobotiqCollisionBaseLink from
'./Robotiq2f85/collision/robotiq_arg2f_base_link';
import RobotiqVisual85BaseLink from
'./Robotiq2f85/visual/robotiq_arg2f_85_base_link';
import RobotiqVisual85InnerFinger from
'./Robotiq2f85/visual/robotiq_arg2f_85_inner_finger';
import RobotiqVisual85InnerKnuckle from
'./Robotiq2f85/visual/robotiq_arg2f_85_inner_knuckle';
import RobotiqVisual85OuterFinger from
'./Robotiq2f85/visual/robotiq_arg2f_85_outer_finger';
import RobotiqVisual85OuterKnuckle from
'./Robotiq2f85/visual/robotiq_arg2f_85_outer_knuckle';
import RobotiqVisual85Pad from
'./Robotiq2f85/visual/robotiq_arg2f_85_pad';
import RobotiqVisualGripper from
'./Robotiq2f85/visual/robotiq_gripper_coupling';
//---------------------------------------------- RobotiqWisc
import RobotiqWiscVisualBaseLink from 
'./RobotiqWisc/visual/robotiq_85_base_link';
import RobotiqWiscVisualKnuckleLink from 
'./RobotiqWisc/visual/robotiq_85_knuckle_link';
import RobotiqWiscVisualFingerLink from 
'./RobotiqWisc/visual/robotiq_85_finger_link';
import RobotiqWiscVisualFingerTipLink from 
'./RobotiqWisc/visual/robotiq_85_finger_tip_link';
import RobotiqWiscVisualInnerKnuckleLink from 
'./RobotiqWisc/visual/robotiq_85_inner_knuckle_link';
import RobotiqWiscCollisionBaseLink from 
'./RobotiqWisc/collision/robotiq_85_base_link';
import RobotiqWiscCollisionKnuckleLink from 
'./RobotiqWisc/collision/robotiq_85_knuckle_link';
import RobotiqWiscCollisionFingerLink from 
'./RobotiqWisc/collision/robotiq_85_finger_link';
import RobotiqWiscCollisionFingerTipLink from 
'./RobotiqWisc/collision/robotiq_85_finger_tip_link';
import RobotiqWiscCollisionInnerKnuckleLink from 
'./RobotiqWisc/collision/robotiq_85_inner_knuckle_link';
//-------------------------------------------------NAO
import NaoHeadPitch from './Nao/HeadPitch';
import NaoHeadYaw from './Nao/HeadYaw';
import NaoLAnklePitch from './Nao/LAnklePitch';
import NaoLAnkleRoll from './Nao/LAnkleRoll';
import NaoLElbowRoll from './Nao/LElbowRoll';
import NaoLFinger11 from './Nao/LFinger11';
import NaoLFinger12 from './Nao/LFinger12';
import NaoLFinger13 from './Nao/LFinger13';
import NaoLFinger21 from './Nao/LFinger21';
import NaoLFinger22 from './Nao/LFinger22';
import NaoLFinger23 from './Nao/LFinger23';
import NaoLHipPitch from './Nao/LHipPitch';
import NaoLHipRoll from './Nao/LHipRoll';
import NaoLHipYawPitch from './nao/LHipYawPitch';
import NaoLKneePitch from './nao/LKneePitch';
import NaoLShoulderPitch from './nao/LShoulderPitch';
import NaoLShoulderRoll from './nao/LShoulderRoll';
import NaoLThumb1 from './nao/LThumb1';
import NaoLThumb2 from './nao/LThumb2';
import NaoLWristYaw from './nao/LWristYaw';
import NaoRAnklePitch from './nao/RAnklePitch';
import NaoRAnkleRoll from './nao/RAnkleRoll';
import NaoRElbowRoll from './nao/RElbowRoll';
import NaoRFinger11 from './nao/RFinger11';
import NaoRFinger12 from './nao/RFinger12';
import NaoRFinger13 from './nao/RFinger13';
import NaoRFinger21 from './nao/RFinger21';
import NaoRFinger22 from './nao/RFinger22';
import NaoRFinger23 from './nao/RFinger23';
import NaoRHipPitch from './nao/RHipPitch';
import NaoRHipRoll from './nao/RHipRoll';
import NaoRHipYawPitch from './nao/RHipYawPitch';
import NaoRKneePitch from './nao/RKneePitch';
import NaoRShoulderPitch from './nao/RShoulderPitch';
import NaoRShoulderRoll from './nao/RShoulderRoll';
import NaoRThumb1 from './nao/RThumb1';
import NaoRThumb2 from './nao/RThumb2';
import NaoRWristYaw from './nao/RWristYaw';
import NaoTorso from './nao/Torso';




//-------------------------------------------------Baxter
import Pedestal_Link_Collision from
'./Baxter/base/Pedestal_link_collision';
import PEDESTAL from
'./Baxter/base/PEDESTAL';
import H0 from
'./Baxter/head/H0';
import H1 from
'./Baxter/head/H1';
import E1 from
'./Baxter/lower_elbow/E1';
import W1 from
'./Baxter/lower_forearm/W1';
import Base_Link_Collision from
'./Baxter/torso/Base_link_collision';
import Base_Link from
'./Baxter/torso/Base_link';
import E0 from
'./Baxter/upper_elbow/E0';
import W0 from
'./Baxter/upper_forearm/W0';
import S0 from
'./Baxter/upper_shoulder/S0';
import S1 from
'./Baxter/lower_shoulder/S1';
import W2 from
'./Baxter/wrist/W2';
//--------------------------------------------------Ur3
import Ur3Base from './Ur3/visual/base';
import Ur3Forearm from './Ur3/visual/forearm';
import Ur3Shoulder from './Ur3/visual/shoulder';
import Ur3Upperarm from './Ur3/visual/upperarm';
import Ur3Wrist1 from './Ur3/visual/wrist1';
import Ur3Wrist2 from './Ur3/visual/wrist2';
import Ur3Wrist3 from './Ur3/visual/wrist3';
import Ur3BaseCollision from './Ur3/collision/base';
import Ur3ForearmCollision from './Ur3/collision/forearm';
import Ur3ShoulderCollision from './Ur3/collision/shoulder';
import Ur3UpperarmCollision from './Ur3/collision/upperarm';
import Ur3Wrist1Collision from './Ur3/collision/wrist1';
import Ur3Wrist2Collision from './Ur3/collision/wrist2';
import Ur3Wrist3Collision from './Ur3/collision/wrist3';
//--------------------------------------------------Ur5
import Ur5Base from './Ur5/base';
import Ur5Forearm from './Ur5/forearm';
import Ur5Shoulder from './Ur5/shoulder';
import Ur5Upperarm from './Ur5/upperarm';
import Ur5Wrist1 from './Ur5/wrist1';
import Ur5Wrist2 from './Ur5/wrist2';
import Ur5Wrist3 from './Ur5/wrist3';
//---------------------------------------------------Ur10
import Ur10Base from './Ur10/base';
import Ur10Forearm from './Ur10/forearm';
import Ur10Shoulder from './Ur10/shoulder';
import Ur10Upperarm from './Ur10/upperarm';
import Ur10Wrist1 from './Ur10/wrist1';
import Ur10Wrist2 from './Ur10/wrist2';
import Ur10Wrist3 from './Ur10/wrist3';
//---------------------------------------------------Other
import Benchy from './Other/3DBenchy';
import FlatArrow from './Other/Arrow';
import Box from './Other/Box';
import Collision_Box from './Other/Collision-Box';
import Collision_Mk2_Printer from './Other/Collision-Mk2-Printer';
import Collision_Pedestal from './Other/Collision-Pedestal';
import Collision_Table from './Other/Collision-Table';
import InfoPhycon from './Other/InfoPhycon';
import LocationMarker from './Other/LocationMarker';
import MK2Printer from './Other/MK2Printer';
import OpenWaypointMarker from './Other/OpenWaypointMarker';
import Pedestal from './Other/Pedestal';
import Table from './Other/Table';
import WarningPhycon from './Other/WarningPhycon';
import Tag from './Other/Tag';
import Flag from './Other/Flag';
import Blade from './Other/Blade';
import HandleL from './Other/HandleL';
import HandleR from './Other/HandleR';
import Knife from './Other/Knife';
import Conveyor from './Other/Conveyor';
import ConveyorCollision from './Other/ConveyorCollision';
import TransportJig from './Other/TransportJig';
import AssemblyJig from './Other/AssemblyJig';
import AssemblyJigCollision from './Other/AssemblyJigCollision';
import BladeWithTransportJig from './Other/BladeWithTransportJig';
import KnifeWithTransportJig from './Other/KnifeWithTransportJig';
import ConveyorDispatcher from './Other/ConveyorDispatcher';
import ConveyorReceiver from './Other/ConveyorReceiver';
import ConveyorDispatcherCollision from './Other/ConveyorDispatcherCollision';
import ConveyorReceiverCollision from './Other/ConveyorReceiverCollision';

const MeshLookupTable = {
    // 'sphere':Sphere,
    // 'cube':Cube,
    // 'cylinder':Cylinder,
    // 'arrow':Arrow,
    'flatarrow':FlatArrow,
    'warning':WarningPhycon,
    'info':InfoPhycon,
    'tag':Tag,
    'flag':Flag,
    'blade':Blade,
    'knife':Knife,
    'handle_l':HandleL,
    'handle_r':HandleR,
    'conveyor':Conveyor,
    'conveyor_collision':ConveyorCollision,
    'transport_jig':TransportJig,
    'assembly_jig':AssemblyJig,
    'assembly_jig_collision':AssemblyJigCollision,
    'blade_with_transport_jig':BladeWithTransportJig,
    'knife_with_transport_jig':KnifeWithTransportJig,
    'conveyor_dispatcher':ConveyorDispatcher,
    'conveyor_receiver':ConveyorReceiver,
    'conveyor_dispatcher_collision':ConveyorDispatcherCollision,
    'conveyor_receiver_collision':ConveyorReceiverCollision,
    //------------------------------------------------------Nao
    "package://nao_meshes/meshes/V40/HeadYaw.dae" : NaoHeadYaw,
    "package://nao_meshes/meshes/V40/HeadPitch.dae" : NaoHeadPitch,
    "package://nao_meshes/meshes/V40/LHipYawPitch.dae" : NaoLHipYawPitch,
    "package://nao_meshes/meshes/V40/LHipRoll.dae" : NaoLHipRoll,
    "package://nao_meshes/meshes/V40/LHipPitch.dae" : NaoLHipPitch,
    "package://nao_meshes/meshes/V40/LKneePitch.dae" : NaoLKneePitch,
    "package://nao_meshes/meshes/V40/LAnklePitch.dae" : NaoLAnklePitch,
    "package://nao_meshes/meshes/V40/LAnkleRoll.dae" : NaoLAnkleRoll,
    "package://nao_meshes/meshes/V40/RHipYawPitch.dae" : NaoRHipYawPitch,
    "package://nao_meshes/meshes/V40/RHipRoll.dae" : NaoRHipRoll,
    "package://nao_meshes/meshes/V40/RHipPitch.dae" : NaoRHipPitch,
    "package://nao_meshes/meshes/V40/RKneePitch.dae" : NaoRKneePitch,
    "package://nao_meshes/meshes/V40/RAnklePitch.dae" : NaoRAnklePitch,
    "package://nao_meshes/meshes/V40/RAnkleRoll.dae" : NaoRAnkleRoll,
    "package://nao_meshes/meshes/V40/Torso.dae" : NaoTorso,
    "package://nao_meshes/meshes/V40/LShoulderPitch.dae" : NaoLShoulderPitch,
    "package://nao_meshes/meshes/V40/LShoulderRoll.dae" : NaoLShoulderRoll,
    "package://nao_meshes/meshes/V40/LElbowRoll.dae" : NaoLElbowRoll,
    "package://nao_meshes/meshes/V40/LWristYaw.dae" : NaoLWristYaw,
    "package://nao_meshes/meshes/V40/RShoulderPitch.dae" : NaoRShoulderPitch,
    "package://nao_meshes/meshes/V40/RShoulderRoll.dae" : NaoRShoulderRoll,
    "package://nao_meshes/meshes/V40/RElbowRoll.dae" : NaoRElbowRoll,
    "package://nao_meshes/meshes/V40/RWristYaw.dae" : NaoRWristYaw,
    "package://nao_meshes/meshes/V40/RFinger13.dae" : NaoRFinger13,
    "package://nao_meshes/meshes/V40/RFinger12.dae" : NaoRFinger12,
    "package://nao_meshes/meshes/V40/LFinger21.dae" : NaoLFinger21,
    "package://nao_meshes/meshes/V40/LFinger13.dae" : NaoLFinger13,
    "package://nao_meshes/meshes/V40/LFinger11.dae" : NaoLFinger11,
    "package://nao_meshes/meshes/V40/RFinger21.dae" : NaoRFinger21,
    "package://nao_meshes/meshes/V40/RFinger22.dae" : NaoRFinger22,
    "package://nao_meshes/meshes/V40/LFinger22.dae" : NaoLFinger22,
    "package://nao_meshes/meshes/V40/LFinger21.dae" : NaoLFinger21,
    "package://nao_meshes/meshes/V40/LFinger12.dae" : NaoLFinger12,
    "package://nao_meshes/meshes/V40/RFinger23.dae" : NaoRFinger23,
    "package://nao_meshes/meshes/V40/RFinger11.dae" : NaoRFinger11,
    "package://nao_meshes/meshes/V40/LFinger23.dae" : NaoLFinger23,
    "package://nao_meshes/meshes/V40/LThumb1.dae" : NaoLThumb1,
    "package://nao_meshes/meshes/V40/RThumb1.dae" : NaoRThumb1,
    "package://nao_meshes/meshes/V40/RThumb2.dae" : NaoRThumb2, 
    "package://nao_meshes/meshes/V40/LThumb2.dae" : NaoLThumb2,


    //------------------------------------------------------Pepper
    "package://pepper_description/meshes/HeadYaw.dae" : PepperVisualHeadYaw,
    "package://pepper_description/meshes/HeadYaw_0.10.stl" : PepperCollisionHeadYaw,
    "package://pepper_description/meshes/HeadPitch.dae" : PepperVisualHeadPitch,
    "package://pepper_description/meshes/HeadPitch_0.10.stl" : PepperCollisionHeadPitch,
    "package://pepper_description/meshes/HipRoll.dae" : PepperVisualHipRoll,
    "package://pepper_description/meshes/HipRoll_0.10.stl" : PepperCollisionHipRoll,
    "package://pepper_description/meshes/HipPitch.dae" : PepperVisualHipPitch,
    "package://pepper_description/meshes/HipPitch_0.10.stl" : PepperCollisionHipPitch,
    "package://pepper_description/meshes/KneePitch.dae" : PepperVisualKneePitch,
    "package://pepper_description/meshes/KneePitch_0.10.stl" : PepperCollisionKneePitch,
    "package://pepper_description/meshes/Torso.dae" : PepperVisualTorso,
    "package://pepper_description/meshes/Torso_0.10.stl" : PepperCollisionTorso,
    "package://pepper_description/meshes/LShoulderPitch.dae" : PepperVisualLShoulderPitch,
    "package://pepper_description/meshes/LShoulderPitch_0.10.stl" : PepperCollisionLShoulderPitch,
    "package://pepper_description/meshes/LShoulderRoll.dae" : PepperVisualLShoulderRoll,
    "package://pepper_description/meshes/LShoulderRoll_0.10.stl" : PepperCollisionLShoulderRoll,
    "package://pepper_description/meshes/LElbowYaw.dae" : PepperVisualLElbowYaw,
    "package://pepper_description/meshes/LElbowYaw_0.10.stl" : PepperCollisionLElbowYaw,
    "package://pepper_description/meshes/LElbowRoll.dae" : PepperVisualLElbowRoll,
    "package://pepper_description/meshes/LElbowRoll_0.10.stl" : PepperCollisionLElbowRoll,
    "package://pepper_description/meshes/LWristYaw.dae" : PepperVisualLWristYaw,
    "package://pepper_description/meshes/LWristYaw_0.10.stl" : PepperCollisionLWristYaw,
    "package://pepper_description/meshes/RShoulderPitch.dae" : PepperVisualRShoulderPitch,
    "package://pepper_description/meshes/RShoulderPitch_0.10.stl" : PepperCollisionRShoulderPitch,
    "package://pepper_description/meshes/RShoulderRoll.dae" : PepperVisualRShoulderRoll,
    "package://pepper_description/meshes/RShoulderRoll_0.10.stl" : PepperCollisionRShoulderRoll,
    "package://pepper_description/meshes/RElbowYaw.dae" : PepperVisualRElbowYaw,
    "package://pepper_description/meshes/RElbowYaw_0.10.stl": PepperCollisionRElbowYaw,
    "package://pepper_description/meshes/RElbowRoll.dae" : PepperVisualRElbowRoll,
    "package://pepper_description/meshes/RElbowRoll_0.10.stl" : PepperCollisionRElbowRoll,
    "package://pepper_description/meshes/RWristYaw.dae" : PepperVisualRWristYaw,
    "package://pepper_description/meshes/RWristYaw_0.10.stl" : PepperCollisionRWristYaw,
    "package://pepper_description/meshes/RFinger41.dae" : PepperVisualRFinger41,
    "package://pepper_description/meshes/RFinger41_0.10.stl" : PepperCollisionRFinger41,
    "package://pepper_description/meshes/LFinger42.dae" : PepperVisualLFinger42,
    "package://pepper_description/meshes/LFinger42_0.10.stl" : PepperCollisionLFinger42,
    "package://pepper_description/meshes/RFinger12.dae" : PepperVisualRFinger12,
    "package://pepper_description/meshes/RFinger12_0.10.stl" : PepperCollisionRFinger12,
    "package://pepper_description/meshes/LFinger33.dae" : PepperVisualLFinger33,
    "package://pepper_description/meshes/LFinger33_0.10.stl" : PepperCollisionLFinger33,
    "package://pepper_description/meshes/RFinger31.dae" : PepperVisualRFinger31,
    "package://pepper_description/meshes/RFinger31_0.10.stl" : PepperCollisionRFinger31,
    "package://pepper_description/meshes/LFinger21.dae" : PepperVisualLFinger21,
    "package://pepper_description/meshes/LFinger21_0.10.stl" : PepperCollisionLFinger21,
    "package://pepper_description/meshes/RFinger32.dae" : PepperVisualRFinger32,
    "package://pepper_description/meshes/RFinger32_0.10.stl" : PepperCollisionRFinger32,
    "package://pepper_description/meshes/LFinger13.dae" : PepperVisualLFinger13,
    "package://pepper_description/meshes/LFinger13_0.10.stl" : PepperCollisionLFinger13,
    "package://pepper_description/meshes/LFinger32.dae" : PepperVisualLFinger32,
    "package://pepper_description/meshes/LFinger32_0.10.stl" : PepperCollisionLFinger32,
    "package://pepper_description/meshes/LFinger11.dae" : PepperVisualLFinger11,
    "package://pepper_description/meshes/LFinger11_0.10.stl" : PepperCollisionLFinger11,
    "package://pepper_description/meshes/RFinger22.dae" : PepperVisualRFinger22,
    "package://pepper_description/meshes/RFinger22_0.10.stl" : PepperCollisionRFinger22,
    "package://pepper_description/meshes/RFinger13.dae" : PepperVisualRFinger13,
    "package://pepper_description/meshes/RFinger13_0.10.stl" : PepperCollisionRFinger13,
    "package://pepper_description/meshes/LFinger22.dae" : PepperVisualLFinger22,
    "package://pepper_description/meshes/LFinger22_0.10.stl" : PepperCollisionLFinger22,
    "package://pepper_description/meshes/RFinger21.dae" : PepperVisualRFinger21,
    "package://pepper_description/meshes/RFinger21_0.10.stl" : PepperCollisionRFinger21,
    "package://pepper_description/meshes/LFinger41.dae" : PepperVisualLFinger41,
    "package://pepper_description/meshes/LFinger41_0.10.stl" : PepperCollisionLFinger41,
    "package://pepper_description/meshes/LFinger12.dae" : PepperVisualLFinger12,
    "package://pepper_description/meshes/LFinger12_0.10.stl" : PepperCollisionLFinger12,
    "package://pepper_description/meshes/RFinger23.dae" : PepperVisualRFinger23,
    "package://pepper_description/meshes/RFinger23_0.10.stl" : PepperCollisionRFinger23,
    "package://pepper_description/meshes/RFinger11.dae" : PepperVisualRFinger11,
    "package://pepper_description/meshes/RFinger11_0.10.stl" : PepperCollisionRFinger11,
    "package://pepper_description/meshes/LFinger23.dae" : PepperVisualLFinger23,
    "package://pepper_description/meshes/LFinger23_0.10.stl" : PepperCollisionLFinger23,
    "package://pepper_description/meshes/LFinger43.dae" : PepperVisualLFinger43,
    "package://pepper_description/meshes/LFinger43_0.10.stl" : PepperCollisionLFinger43,
    "package://pepper_description/meshes/RFinger43.dae" : PepperVisualRFinger43,
    "package://pepper_description/meshes/RFinger43_0.10.stl" : PepperCollisionRFinger43,
    "package://pepper_description/meshes/RFinger42.dae" : PepperVisualRFinger42,
    "package://pepper_description/meshes/RFinger42_0.10.stl" : PepperCollisionRFinger42,
    "package://pepper_description/meshes/LFinger31.dae" : PepperVisualLFinger31,
    "package://pepper_description/meshes/LFinger31_0.10.stl" : PepperCollisionLFinger31,
    "package://pepper_description/meshes/RFinger33.dae" : PepperVisualRFinger33,
    "package://pepper_description/meshes/RFinger33_0.10.stl" : PepperCollisionRFinger33,
    "package://pepper_description/meshes/LThumb1.dae" : PepperVisualLThumb1,
    "package://pepper_description/meshes/LThumb1_0.10.stl" : PepperCollisionLThumb1,
    "package://pepper_description/meshes/RThumb2.dae" : PepperVisualRThumb2,
    "package://pepper_description/meshes/RThumb2_0.10.stl" : PepperCollisionRThumb2,
    "package://pepper_description/meshes/RThumb1.dae" : PepperVisualRThumb1,
    "package://pepper_description/meshes/RThumb1_0.10.stl" : PepperCollisionRThumb1,
    "package://pepper_description/meshes/LThumb2.dae" : PepperVisualLThumb2,
    "package://pepper_description/meshes/LThumb2_0.10.stl" : PepperCollisionLThumb2,
    "package://pepper_description/meshes/WheelFL.dae" : PepperVisualWheelFL,
    "package://pepper_description/meshes/WheelFL_0.10.stl" : PepperCollisionWheelFL,
    "package://pepper_description/meshes/WheelB.dae" : PepperVisualWheelB,
    "package://pepper_description/meshes/WheelB_0.10.stl" : PepperCollisionWheelB,
    "package://pepper_description/meshes/WheelFR.dae" : PepperVisualWheelFR,
    "package://pepper_description/meshes/WheelFR_0.10.stl" : PepperCollisionWheelFR,
    //"package://pepper_description/meshes/d435.dae" : PepperVisualD435,
    // "package://pepper_description/meshes/camera_body.stl" : PepperCollisionCameraBody,
    // "package://pepper_description/meshes/camera_base.stl" : PepperCollisionCameraBase,
    // "package://pepper_description/meshes/camera_visor.stl" : PepperCollisionCameraVisor,
    //"package://pepper_description/meshes/t265.stl" : PepperCollisionT265,

    //------------------------------------------------------Panda
    'package://franka_ros/franka_description/meshes/visual/finger.dae': Finger,
    'package://franka_ros/franka_description/meshes/visual/hand.dae': Hand,
    'package://franka_ros/franka_description/meshes/visual/link0.dae': Link0,
    'package://franka_ros/franka_description/meshes/visual/link1.dae': Link1, // missing
    'package://franka_ros/franka_description/meshes/visual/link2.dae': Link2,
    'package://franka_ros/franka_description/meshes/visual/link3.dae': Link3,
    'package://franka_ros/franka_description/meshes/visual/link4.dae': Link4,
    'package://franka_ros/franka_description/meshes/visual/link5.dae': Link5,
    'package://franka_ros/franka_description/meshes/visual/link6.dae': Link6,
    'package://franka_ros/franka_description/meshes/visual/link7.dae': Link7,
    //---------------------------------------------------------Robotiq2f85
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/collision/robotiq_arg2f_85_base_link.stl': RobotiqCollision85BaseLink,
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/collision/robotiq_arg2f_85_inner_finger.dae': RobotiqCollision85InnerFinger,// huge
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/collision/robotiq_arg2f_85_inner_knuckle.dae': RobotiqCollision85InnerKnuckle,// tooo huge
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/collision/robotiq_arg2f_85_outer_finger.dae': RobotiqCollision85OuterFinger,// too huge
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/collision/robotiq_arg2f_85_outer_knuckle.dae': RobotiqCollision85OuterKnuckle,//too huge
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/collision/robotiq_arg2f_base_link.stl': RobotiqCollisionBaseLink, // too huge
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/visual/robotiq_arg2f_85_base_link.dae': RobotiqVisual85BaseLink,
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/visual/robotiq_arg2f_85_inner_finger.dae': RobotiqVisual85InnerFinger, // too huge
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/visual/robotiq_arg2f_85_inner_knuckle.dae': RobotiqVisual85InnerKnuckle, // too huge
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/visual/robotiq_arg2f_85_outer_finger.dae': RobotiqVisual85OuterFinger,// huge
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/visual/robotiq_arg2f_85_outer_knuckle.dae': RobotiqVisual85OuterKnuckle, //huge
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/visual/robotiq_arg2f_85_pad.dae': RobotiqVisual85Pad,// huge
    'package://robotiq/robotiq_2f_85_gripper_visualization/meshes/visual/robotiq_gripper_coupling.stl': RobotiqVisualGripper, //huge
    //---------------------------------------------------------RobotiqWisc
    'package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae':RobotiqWiscVisualBaseLink,
    'package://robotiq_85_description/meshes/visual/robotiq_85_knuckle_link.dae':RobotiqWiscVisualKnuckleLink,
    'package://robotiq_85_description/meshes/visual/robotiq_85_finger_link.dae':RobotiqWiscVisualFingerLink,
    'package://robotiq_85_description/meshes/visual/robotiq_85_inner_knuckle_link.dae':RobotiqWiscVisualInnerKnuckleLink,
    'package://robotiq_85_description/meshes/visual/robotiq_85_finger_tip_link.dae':RobotiqWiscVisualFingerTipLink,
    'package://robotiq_85_description/meshes/collision/robotiq_85_base_link.stl':RobotiqWiscCollisionBaseLink,
    'package://robotiq_85_description/meshes/collision/robotiq_85_knuckle_link.stl':RobotiqWiscCollisionKnuckleLink,
    'package://robotiq_85_description/meshes/collision/robotiq_85_finger_link.stl':RobotiqWiscCollisionFingerLink,
    'package://robotiq_85_description/meshes/collision/robotiq_85_inner_knuckle_link.stl':RobotiqWiscCollisionInnerKnuckleLink,
    'package://robotiq_85_description/meshes/collision/robotiq_85_finger_tip_link.stl':RobotiqWiscCollisionFingerTipLink,
    //------------------------------------------------------------------Baxter
     'package://baxter_common/baxter_description/meshes/base/PEDESTAL.DAE':PEDESTAL,
     'package://baxter_common/baxter_description/meshes/base/pedestal_link_collision.DAE':Pedestal_Link_Collision,
     'package://baxter_common/baxter_description/meshes/head/H0.DAE' : H0,
     'package://baxter_common/baxter_description/meshes/head/H1.DAE' : H1,
     'package://baxter_common/baxter_description/meshes/lower_elbow/E1.DAE' : E1,
     'package://baxter_common/baxter_description/meshes/lower_forearm/W1.DAE': W1,
     'package://baxter_common/baxter_description/meshes/lower_shoulder/S1.DAE':S1,
     'package://baxter_common/baxter_description/meshes/torso/base_link.DAE':Base_Link,
     'package://baxter_common/baxter_description/meshes/torso/base_link_collision.DAE' : Base_Link_Collision,
     'package://baxter_common/baxter_description/meshes/upper_elbow/E0.DAE':E0,
     'package://baxter_common/baxter_description/meshes/upper_forearm/W0.DAE':W0,
     'package://baxter_common/baxter_description/meshes/upper_shoulder/S0.DAE':S0,
     'package://baxter_common/baxter_description/meshes/wrist/W2.DAE': W2,
     //---------------------------------------------------------------------Ur3
      'package://ur_description/meshes/ur3/visual/base.dae' : Ur3Base,
      'package://ur_description/meshes/ur3/visual/forearm.dae': Ur3Forearm,
      'package://ur_description/meshes/ur3/visual/shoulder.dae' : Ur3Shoulder,
      'package://ur_description/meshes/ur3/visual/upperarm.dae' : Ur3Upperarm,
      'package://ur_description/meshes/ur3/visual/wrist1.dae' : Ur3Wrist1, 
      'package://ur_description/meshes/ur3/visual/wrist2.dae' : Ur3Wrist2,
      'package://ur_description/meshes/ur3/visual/wrist3.dae' : Ur3Wrist3,
      'package://ur_description/meshes/ur3/collision/base.stl' : Ur3BaseCollision,
      'package://ur_description/meshes/ur3/collision/forearm.stl': Ur3ForearmCollision,
      'package://ur_description/meshes/ur3/collision/shoulder.stl' : Ur3ShoulderCollision,
      'package://ur_description/meshes/ur3/collision/upperarm.stl' : Ur3UpperarmCollision,
      'package://ur_description/meshes/ur3/collision/wrist1.stl' : Ur3Wrist1Collision,
      'package://ur_description/meshes/ur3/collision/wrist2.stl' : Ur3Wrist2Collision,
      'package://ur_description/meshes/ur3/collision/wrist3.stl' : Ur3Wrist3Collision,
      //--------------------------------------------------------------------Ur5
      'package://ur_description/meshes/ur5/visual/base.dae' : Ur5Base,
      'package://ur_description/meshes/ur5/visual/forearm.dae': Ur5Forearm,
      'package://ur_description/meshes/ur5/visual/shoulder.dae' : Ur5Shoulder,
      'package://ur_description/meshes/ur5/visual/upperarm.dae' : Ur5Upperarm,
      'package://ur_description/meshes/ur5/visual/wrist1.dae' : Ur5Wrist1,
      'package://ur_description/meshes/ur5/visual/wrist2.dae' : Ur5Wrist2,
      'package://ur_description/meshes/ur5/visual/wrist3.dae' : Ur5Wrist3,
      //--------------------------------------------------------------------Ur10
       'package://ur_description/meshes/ur10/visual/base.dae' : Ur10Base,
       'package://ur_description/meshes/ur10/visual/forearm.dae': Ur10Forearm,
       'package://ur_description/meshes/ur10/visual/shoulder.dae' : Ur10Shoulder,
       'package://ur_description/meshes/ur10/visual/upperarm.dae' : Ur10Upperarm,
       'package://ur_description/meshes/ur10/visual/wrist1.dae' : Ur10Wrist1,
       'package://ur_description/meshes/ur10/visual/wrist2.dae' : Ur10Wrist2,
       'package://ur_description/meshes/ur10/visual/wrist3.dae' : Ur10Wrist3,
       //-------------------------------------------------------------------Other
       'package://app/meshes/3DBenchy.stl': Benchy,
       'package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/MK2-Printer/MK2-Printer.stl':MK2Printer,
       'package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/MK2-Printer.stl': Collision_Mk2_Printer,
       'package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/Box/Box.stl': Box,
       'package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Box.stl': Collision_Box,
       'package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/ur3e-Pedestal/Pedestal.stl':Pedestal,
       'package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Pedestal.stl': Collision_Pedestal,
       'package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/Table/Table.stl':Table,
       'package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Table.stl': Collision_Table ,
       //'package://app/meshes/InfoPhycon.stl': InfoPhycon, // not showing
       'package://app/meshes/LocationMarker.stl': LocationMarker,
       'package://app/meshes/OpenWaypointMarker.stl': OpenWaypointMarker, //
  }

  export default MeshLookupTable