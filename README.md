This solver can then be used when providing goals to the solver's `solve` method:

```
timestamp = datetime.utcnow().timestamp()
inputs = [
    ObjectiveInput(weight=5.0,vector=[1,2,0]),
    ObjectiveInput(weight=3.0),
    ObjectiveInput(weight=1.0,quaternion=[1,0,0,0])
]
base_position, joint_values = solver.solve(inputs,timestamp,max_retries=2)

```

The fields for solve are as follows:
1. `goals`: a list of ObjectiveInput objects.
2. `time`: a float indicating the current time. If no liveliness objectives are used, this has no effect.
3. `world`: [NOT IMPLEMENTED]. Update the current model of the world to handle real-time collision avoidance.
4. `max_retries`: Number of random restarts to perform when searching for a solution.
5. `max_iterations`: Number of iterations for each round of search.
6. `only_core`: Ignore liveliness objectives and disable the second liveliness optimization. *Note: Not advised to switch within runs. Only use this flag if you are not using liveliness objectives and want a slight speed-up.*

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