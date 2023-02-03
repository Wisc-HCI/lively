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
  smoothness: {
    // An example objective (smoothness macro)
    name: 'MySmoothnessObjective',
    type: 'SmoothnessMacro',
    weight: 5,
  },
  collision: {
    // An example objective (collision avoidance)
    name: 'MyCollisionDetection',
    type: 'CollisionAvoidance',
    weight: 5,
  },
  jointLimit: {
    // An example objective (joint limit)
    name: 'MyJointLimit',
    type: 'JointLimits',
    weight: 5,
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
const newState = newSolver.solve({}, {}, 0.0);

document.querySelector('#app').innerHTML = `
  <div>
   ${JSON.stringify(newState)}
  </div>`;

console.log(newState);
