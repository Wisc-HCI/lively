import { panda, ur3e } from './urdfs.js';
import { Solver } from '@people_and_robots/lively';

const initialRootBounds = [
  // An exmaple of root bounds
  { value: 0.0, delta: 0.0 },
  { value: 0.25, delta: 0.0 },
  { value: 0.5, delta: 0.0 },
  { value: 0.0, delta: 0.0 },
  { value: 0.0, delta: 0.0 },
  { value: 0.0, delta: 0.0 },
];

const initialObjectives = {
  // some lively objective examples. Notice for JavaScript, you do not need to import anything for objective. Simply construct an object
  smoothness: {
    name: 'MySmoothnessObjective',
    type: 'SmoothnessMacro',
    weight: 20,
    joints: true,
    origin: false,
    links: true,
  },
  collision: {
    name: 'MyCollisionDetection',
    type: 'CollisionAvoidance',
    weight: 5,
  },
  jointLimit: {
    name: 'MyJointLimit',
    type: 'JointLimits',
    weight: 5,
  },
  //collision, jointlimit
  positionLively: {
    name: 'MyLivelinessObjective',
    type: 'PositionLiveliness',
    weight: 15,
    link: 'panda_hand',
    frequency: 7,
  },
  position: {
    name: 'MyPositionObjective',
    type: 'PositionMatch',
    weight: 10,
    link: 'panda_hand',
  },
  orientationLively: {
    name: 'MyOrientationObjective',
    type: 'OrientationLiveliness',
    weight: 15,
    link: 'panda_link2',
    frequency: 7,
  },
  finger_joint_control: {
    name: 'FingerJointControl',
    type: 'JointMatch',
    weight: 25,
    joint: 'panda_finger_joint1',
  },
};

const initialEnvShapes = [
  {
    type: 'Cylinder', // The Cylinder here is an example of static environmental shape. This shape will be not able to be moved or deleted.
    name: 'pill',
    frame: 'world',
    physical: true,
    length: 0.3,
    radius: 0.2,
    localTransform: {
      translation: [-0.8, 0.0, 0.1],
      rotation: [1.0, 0.0, 0.0, 0.0],
    }, // [x, y, z, w] ordering for quaternion
  },
];

const collision_settings = {
  // This is an example of customized collision_settings
  dMax: 0.1,
  r: 0.0,
  aMax: 2.0,
  timeBudget: 100,
  timed: false,
};

const newSolver = new Solver(
  panda,
  initialObjectives,
  initialRootBounds,
  initialEnvShapes,
  null,
  null,
  null,
  collision_settings
);

newSolver.computeAverageDistanceTable();
const d = new Date();
let time = d.getTime(); // Get the time used in Math.sin
let goal = {
  // An goal example with defined Scalar and Size values for the lively objectives
  position: {
    Translation: [0.6, 0, 0.6],
  },
  finger_joint_control: {
    Scalar: 0.02,
  },
  positionLively: {
    Size: [0.07, 0.05, 0.08],
  },
  orientationLively: {
    Size: [0.07, 0.05, 0.08],
  },
  jointLimit: {
    Scalar: 0.02,
  },
};
const newState = newSolver.solve(goal, {}, time / 1000);

document.querySelector('#app').innerHTML = `
  <div>
   ${JSON.stringify(newState)}
  </div>`;

console.log(newState);
