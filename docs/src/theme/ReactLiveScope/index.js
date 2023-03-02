import React from 'react';
// Add react-live imports you need here
import {Solver} from '../../../../pkg';
import {panda, ur3e, ur5e, pepper} from './urdfs';
import { Tree } from './Tree';
import { Button } from './Button';
import { RobotViewer } from './RobotViewer';
import { useLively } from './useLively';
import lodash from 'lodash';

const ReactLiveScope = {
  React,
  ...React,
  lively: {Solver},
  urdfs: {panda,ur3e,ur5e,pepper},
  Tree,
  Button,
  RobotViewer,
  useLively,
  lodash
};




export default ReactLiveScope;
