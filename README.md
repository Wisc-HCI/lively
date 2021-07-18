[![PyPI version](https://img.shields.io/pypi/v/lively_tk)](https://badge.fury.io/py/lively_tk)
![Upload Python Package](https://github.com/Wisc-HCI/lively_tk/workflows/Upload%20Python%20Package/badge.svg)
# LivelyTK

## About

LivelyTK Package

The LivelyTK framework provides a highly configurable toolkit for commanding robots in mixed modalities while incorporating liveliness motions. It is adapted from [RelaxedIK](https://github.com/uwgraphics/relaxed_ik_core) framework.

To configure a robot, the easiest method is to use the LivelyStudio interface in the [lively_tk_ros](https://github.com/Wisc-HCI/lively_tk_ros) repository, which is a wizard for configuring the robot.

Once you have a config json file, you can create a `Config` object from the loaded data using `parse_config_data`:

```
import yaml
from datetime import datetime
from lively_tk import parse_config_data, LivelyIK

with open('path/to/config_file.json') as handle:
    config_data = yaml.safe_load(handle)

config = parse_config_data(config_data)
```

This config can be used to initialize the LivelyTK solver:

```
solver = Solver(config)
```

This solver can then be used when providing goals to the solver's `solve` method:

```
timestamp = datetime.utcnow().timestamp()
inputs = [
    ObjectiveInput(weight=5.0,vector=[1,2,0]),
    ObjectiveInput(weight=3.0),
    ObjectiveInput(weight=1.0,quaternion=[1,0,0,0])
]
base_position, joint_values, _ = solver.solve(inputs,timestamp,max_retries=2)

```

The fields for solve are as follows:
1. `goals`: a list of ObjectiveInput objects.
2. `time`: (float) The current time. If no liveliness objectives are used, this has no effect.
3. `world`: [NOT IMPLEMENTED]. Update the current model of the world to handle real-time collision avoidance.
4. `max_retries`: (int) Number of random restarts to perform when searching for a solution.
5. `max_iterations`: (int) Number of iterations for each round of search.
6. `only_core`: (bool) Ignore liveliness objectives and disable the second liveliness optimization. *Note: Not advised to switch within runs. Only use this flag if you are not using liveliness objectives and want a slight speed-up.*
7. `return_frames`: (bool) Return the positions and orientations of each joint (structured specified in joint_names in config) as the third output field.

If you need to restart a run, you can execute the reset method of the solver, and provide the starting base position and joint values:

```
solver.reset([0,0,0],[1,0,1.2,0.4,0.1])
```

## Contributing

To build, download and `cd` to this directory. Then run:

```
# If you just want to install:
python3 setup.py install


# Or if you are developing:
python3 setup.py develop

# If you are developing and need to rebuild:
python3 setup.py clean && python3 setup.py develop
```

You will need this installed to use the ROS2 LivelyIK Package.
