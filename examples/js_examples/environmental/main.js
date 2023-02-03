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

const shapeUpdates = [
  {
    Add: {
      id: 'env-box', // must be an unique id
      shape: {
        type: 'Box', //can be 'Cylinder', 'Capsule', or 'Sphere'
        name: 'box', // name can be arbitrary
        frame: 'world', // or 'world'
        physical: true, // physical collision
        x: 0.25,
        y: 0.25,
        z: 0.25, // dimension of the box
        localTransform: {
          translation: [0.6, 0.05, 0.15],
          rotation: [0.0, 0.0, 0.0, 1.0],
        },
      },
    },
  },
  {
    Move: {
      id: 'env-box',
      transform: {
        translation: [0.6, 0.05, 0.15],
        rotation: [0.0, 0.0, 0.0, 1.0],
      },
    },
  },
  { Delete: 'env-box' },
];

const initialObjectives = {
  // some objective examples. Notice for JavaScript, you do not need to import anything for objective. Simply construct an object
  smoothness: {
    name: 'MySmoothnessObjective',
    type: 'SmoothnessMacro',
    weight: 15,
    joints: true,
    origin: false,
    links: true,
  },
  collision: {
    // The main objective that allows the robot to avoid collision within the links, as well as with the environmental objects.
    name: 'MyCollisionDetection',
    type: 'CollisionAvoidance',
    weight: 3,
  },
  jointLimit: {
    name: 'MyJointLimit',
    type: 'JointLimits',
    weight: 5,
  },
  position: {
    // The main objective that allows the pand hand to follow the position defined by the sphere transform control visual in the scene.
    name: 'MyPositionMatchObjective',
    type: 'PositionMatch',
    link: 'tool0',
    weight: 15,
  },
  orientation: {
    // The main objective that allows the pand hand to follow the orientation defined by the arrow transform control visual in the scene.
    name: 'MyOrientationMatchObjective',
    type: 'OrientationMatch',
    link: 'tool0',
    weight: 10,
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
  ur3e,
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
const newState = newSolver.solve(goal, {}, time / 1000, shapeUpdates);

document.querySelector('#app').innerHTML = `
  <div>
   ${JSON.stringify(newState)}
  </div>`;

console.log(newState);
