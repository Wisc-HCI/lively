import React from 'react';
import init, {Solver} from '../../../../pkg-web';
// import {Solver} from '@people_and_robots/lively';
import {panda, ur3e, ur5e, pepper} from './urdfs';
import { Tree } from './Tree';
import { Button } from './Button';
import { RobotViewer } from './RobotViewer';
import lodash from 'lodash';

const ReactLiveScope = {
    React,
    ...React,
    lively: {Solver},
    urdfs: {panda,ur3e,ur5e,pepper},
    Tree,
    Button,
    RobotViewer,
    lodash
};

export default ReactLiveScope;
