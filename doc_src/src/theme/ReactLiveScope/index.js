import React from 'react';
// Add react-live imports you need here
import init, {Solver} from '../../../../pkg';
import {panda, ur3e} from './urdfs';
import { Tree } from './Tree';
import { Button } from './Button';
import { RobotViewer } from './RobotViewer';
import { useLively } from './useLively';

const ReactLiveScope = {
  React,
  ...React,
  lively: {init,Solver},
  urdfs: {panda,ur3e},
  Tree,
  Button,
  RobotViewer,
  useLively
};




export default ReactLiveScope;
