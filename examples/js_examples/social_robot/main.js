import { pepper } from './urdfs.js';
import { Solver } from '@people_and_robots/lively';
import { mapValues } from 'lodash';

const initialRootBounds = [
  // An exmaple of root bounds
  { value: 0.0, delta: 0.0 },
  { value: 0.25, delta: 0.0 },
  { value: 0.5, delta: 0.0 },
  { value: 0.0, delta: 0.0 },
  { value: 0.0, delta: 0.0 },
  { value: 0.0, delta: 0.0 },
];

const l_arm_joints = {
  LShoulderPitch: 0,
  LShoulderRoll: 1.1,
  LElbowYaw: -1.88,
  LElbowRoll: -0.7,
  LWristYaw: 0.9,
  LHand: 1,
};

const initialObjectives = {
  // some lively objective examples. Notice for JavaScript, you do not need to import anything for objective. Simply construct an object
  smoothness: {
    name: 'MySmoothnessObjective',
    type: 'SmoothnessMacro',
    weight: 25,
    joints: true,
    origin: false,
    links: false,
  },
  collision: {
    name: 'MyCollisionDetection',
    type: 'CollisionAvoidance',
    weight: 0.5,
  },
  jointLimit: {
    name: 'MyJointLimit',
    type: 'JointLimits',
    weight: 5,
  },
  torsoPosition: {
    name: 'Torso Position',
    type: 'PositionMatch',
    link: 'torso',
    weight: 10,
  },
  rHandPosition: {
    name: 'R Hand Position',
    type: 'PositionMatch',
    link: 'r_gripper',
    weight: 10,
  },
  rHandOrientation: {
    name: 'R Hand Orientation',
    type: 'OrientationMatch',
    link: 'r_gripper',
    weight: 10,
  },
  headOrientation: {
    name: 'Gaze',
    type: 'OrientationMatch',
    link: 'Head',
    weight: 7,
  },
  idleGaze: {
    name: 'Idle Gaze',
    type: 'OrientationLiveliness',
    link: 'Head',
    weight: 20,
    frequency: 10,
  },
  LShoulderPitch: {
    name: 'LShoulderPitch',
    joint: 'LShoulderPitch',
    type: 'JointMatch',
    weight: 10,
  },
  LShoulderRoll: {
    name: 'LShoulderRoll',
    joint: 'LShoulderRoll',
    type: 'JointMatch',
    weight: 10,
  },
  LElbowYaw: {
    name: 'LElbowYaw',
    joint: 'LElbowYaw',
    type: 'JointMatch',
    weight: 10,
  },
  LElbowRoll: {
    name: 'LElbowRoll',
    joint: 'LElbowRoll',
    type: 'JointMatch',
    weight: 10,
  },
  LWristYaw: {
    name: 'LWristYaw',
    joint: 'LWristYaw',
    type: 'JointMatch',
    weight: 10,
  },
  LHand: {
    name: 'LHand',
    joint: 'LHand',
    type: 'JointMatch',
    weight: 10,
  },
};

const newSolver = new Solver(
  pepper,
  initialObjectives,
  initialRootBounds,
  null,
  null,
  null,
  null,
  null
);

newSolver.computeAverageDistanceTable();
const d = new Date();
let jointGoals = mapValues(l_arm_joints, (j) => ({ Scalar: j }));

let time = d.getTime(); // Get the time in milliseconds
jointGoals.LElbowRoll.Scalar = 0.21 * Math.sin(time / 1000) - 1.13;
let goals = {
  torsoPosition: {
    Translation: [0.0, 0.0, 0.85],
  },
  rHandPosition: {
    Translation: [0.0, -0.216, 0.546],
  },
  rHandOrientation: {
    Rotation: [0.536, 0.455, -0.435, 0.5616],
  },
  idleGaze: {
    Size: [0.01, 0.0, 0.1],
  },
  ...jointGoals,
};
const newState = newSolver.solve(goals, {}, time / 1000);

document.querySelector('#app').innerHTML = `
  <div>
   ${JSON.stringify(newState)}
  </div>`;

console.log(newState);
