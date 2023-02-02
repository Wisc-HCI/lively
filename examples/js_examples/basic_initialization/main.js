import { panda } from './urdfs.js';
import { Solver } from '@people_and_robots/lively';

const newSolver = new Solver(panda, {
  smoothness: {
    // An example objective (smoothness macro)
    name: 'MySmoothnessObjective',
    type: 'SmoothnessMacro',
    weight: 5,
  },
});

const newState = newSolver.solve({}, {}, 0.0);

document.querySelector('#app').innerHTML = `
  <div>
   ${JSON.stringify(newState)}
  </div>`

console.log(newState);
