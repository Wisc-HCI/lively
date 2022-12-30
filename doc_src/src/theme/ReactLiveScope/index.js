import React from 'react';
// Add react-live imports you need here
import init, {Solver} from '../../../../pkg';
import {panda, ur3e} from './urdfs';
import { Tree } from './Tree'

const ReactLiveScope = {
  React,
  ...React,
  lively: {init,Solver},
  urdfs: {panda,ur3e},
  Tree
};




export default ReactLiveScope;
