[![PyPI version](https://img.shields.io/pypi/v/lively_tk)](https://badge.fury.io/py/lively_tk)
![Upload Python Package](https://github.com/Wisc-HCI/lively_tk/workflows/Upload%20Python%20Package/badge.svg)
# LivelyTK v0.10.0 (beta)

_NOTE: Since LivelyTK is still in beta, the design is subject to change and should not be considered final!_

## About

LivelyTK Package

The LivelyTK framework provides a highly configurable toolkit for commanding robots in mixed modalities while incorporating liveliness motions. It is adapted from [RelaxedIK](https://github.com/uwgraphics/relaxed_ik_core) framework, and compatible with Python and Javascript/Node.

To configure a robot, the easiest method is to use the LivelyStudio interface in the [lively_tk_ros](https://github.com/Wisc-HCI/lively_tk_ros) repository, which is a wizard for configuring the robot.

## Configuring

Configuring of LivelyTK is centered on the Solver class, which you can instantiate in the following ways:

_python_
```python
from lively_tk import Solver, PositionMatchObjective, OrientationMatchObjective, SmoothnessMacroObjective, CollisionAvoidanceObjective, State, Transform, ScalarRange, BoxShape
solver = Solver(
    urdf='<?xml version="1.0" ?><robot name="panda">...</robot>', # Full urdf as a string
    objectives={
        "PositionMatchObjective" : PositionMatchObjective(name="EE Position",link="panda_hand",weight=50),
        "OrientationMatchObjective" :  OrientationMatchObjective(name="EE Rotation",link="panda_hand",weight=25),
        "SmoothnessMacroObjective":SmoothnessMacroObjective(name="General Smoothness",weight=10),
        "CollisionAvoidanceObjective":CollisionAvoidanceObjective(name="Collision Avoidance",weight=10)
        ...
    }, 
    root_bounds=[
        ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0), # Translational, (x, y, z)
        ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0)  # Rotational, (r, p, y)
    ],
    shapes=[
        BoxShape(name="Table",frame="world",physical=True,x=2,y=1,z=1.2,local_transform=Transform.isometry())
    ], 
    initial_state=State(origin=Transform.identity(),joints={"panda_joint1":0.0,"panda_joint2":0.0,...}), # Optional
    only_core=False, # Only use this flag if you are not using liveliness objectives and want a slight speed-up.
    max_retries=1, # Number of times the solution is attempted (default 1)
    max_iterations=150, # Number of iterations per try (default 150)
    collision_settings = CollisionSettingInfo(
        d_max = 0.3, 
        r = 0.0, 
        a_max = 2.0, 
        time_budget = 100, 
        timed = True),

)
```

_javascript_
```javascript
import {Solver} from "@people_and_robots/lively_tk";

let solver = new Solver(
    urdf = '<?xml version="1.0" ?><robot name="panda">...</robot>', // Full urdf as a string
    objectives = {
            "eePosition": {
              type: "PositionMatch",
              name: "EE Position",
              link: attachmentLink,
              weight: 50,
            },
            "eeRotation": {
              type: "OrientationMatch",
              name: "EE Rotation",
              link: attachmentLink,
              weight: 25,
            },
            "collision": {
              type: "CollisionAvoidance",
              name: "Collision Avoidance",
              weight: COLLISION_WEIGHT,
            },
    },
    root_bounds = [
        { value: basePose.position.x, delta: 0.0 },
        { value: basePose.position.y, delta: 0.0 },
        { value: basePose.position.z, delta: 0.0 }, // Translational
        { value: baseEuler[0], delta: 0.0 },
        { value: baseEuler[1], delta: 0.0 },
        { value: baseEuler[2], delta: 0.0 }, // Rotational
      ],
    shapes = [{
        type:'Box', //can be 'Cylinder', 'Capsule', or 'Sphere'
        name:'camera attachment',
        frame: 'panda_hand', // or 'world'
        physical: true,
        x:0.5,y:0.5,z:0.2,
        localTransform: {translation:[0.0,0.0,0.0],rotation:[0.0,0.0,0.0,1.0]} // [x, y, z, w] ordering for quaternion
    }
    ], 
    initial_state= {origin:{translation:[0,0,0],rotation:[1,0,0,0]},joints:{panda_joint1:0.0,panda_joint2:0.0,...}}, // Optional
    only_core=False, # Only use this flag if you are not using liveliness objectives and want a slight speed-up.
    max_retries=1, # Number of times the solution is attempted (default 1)
    max_iterations=150 # Number of iterations per try (default 150)
    collision_settings = {dMax : 0.3, r : 0.0, aMax : 2.0, timeBudget : 100, timed : true}
)
```

## Collision 
The livelytk framework provides collision detection between the robot and environmental objects as well as within the shapes of robot parts. The collision detection functionality is implemented by a time-efficient algorithm that governs collision detection through a time-limited or error-limited routine that reduces the number of collision queries between the shapes by estimating the "pairwise relative spatial coherence", which is defined below:


> "Guaranteed upper and lower bounds on signed distance are computed between each active pair of shapes in the scene. In this work, these bounds are computed by assessing how relative transforms between shapes change over time, a novel technique we call Pairwise Relative Spatial Coherence[^1]".


#### Collision Settings

This is a parameter `collision_settings` that allows the user to customize the collision checking system while configuring a `Solver` 

`d_max`:
A lower bound distance parameter that controls the distance that queries the collision detection between shapes. The distance by default is 0.3 meteres. This means that pairwise collision queries will only happen when two shapes are within 0.3 meters. Increasing/Decreasing the distance will result in a greater/smaller distance for the collision queries which will lead to greater/smaller number of collision queries.

`r`:
A scalar value between 0 and 1 that controls how cautious the collision estimate will be. The value is 0 by default which is the most cautious and accurate. The estimate will be more optimistic when the value approaches 1.

`a_max`:
A upper bound distance parameter that, similar to d_max, determines if a shape pair should be included or excluded in collision checking. The value is 2.0 meters by default. Increasing/Decreasing the distance will result in a greater/smaller distance for the collision queries which will lead to greater/smaller number of collision queries.

`time_budget`:
A time parameter that will be used in the collision checking. The value is 100 microseconds by default. Increase the value will result in a slower but more accurate proximity approximiation.

`timed`:
A boolean parameter that determines which routine will be used for collision checking. The value is true by default. Timed-limited routine will be used if true, error-limited routine will be used if false.

_python_
```python
CollisionSettingInfo(
        d_max = 0.3, 
        r = 0.0, 
        a_max = 2.0, 
        time_budget = 100, 
        timed = True),
```

_javascript_
```javascript
let collision_settings = {dMax : 0.3, r : 0.0, aMax : 2.0, timeBudget : 100, timed : true}
```

## Resetting 

In both the Javascript and Python interfaces, the `Solver` class has a `reset` method that allows the user to reset the state of the solver given some new objective weights and a new robot state. In this case, the robot state only needs to supply the `joints` and `origin` field, as shown in the initialization example.

_python_
```python
solver.reset(state=State(origin=Transform.identity(),joints={"panda_joint1":0.0,"panda_joint2":0.0,...}),weights={})
```

_javascript_
```javascript
solver.reset(
    {origin:{translation:[0,0,0],rotation:[1,0,0,0]},joints:{panda_joint1:0.0,panda_joint2:0.0,...}}, // New starting state
    {} // Can define new weights here
)
```



## Solving

The `Solver` class has a `solve` method that represents the core functionality of the LivelyTK interface. At a high level, it accepts the following fields:

1. `goals`: A look-up table of goal-type objects. The key of the look-up table should match with that of the objectives to which the goals are corresponded.
2. `weights`: A look-up table of floats, order corresponding to the order of the objectives. The key of the look-up table should match with that of the objectives to which the weights are corresponded.
3. `time`: (float) The current time. If no liveliness objectives are used, this has no effect.
4. `shapes`: A list of shape objects. 

The `solve` method returns a fully-filled `State` object



#### Goals

There are a variety of different "goal" types that can be provided. Think of these as settings that you would like to achieve (e.g. a PositionMatch objective accepts a `Translation` goal). 

**Translation**
The translation goal is used by the `PositionMatch`, `PositionMirroring`, and `OriginPositionMatch` objectives.

_python_
```python
goal = Translation(x:1.0,y:0.0,z:0.5)
```

_javascript_
```javascript
let goal = {Translation:[1.0,0.0,0.5]}
```

**Rotation**

The rotation goal is used by the `OrientationMatch`, `OrientationMirroring`, and `OriginOrientationMatch` objectives.

_python_
```python
goal = Rotation(w:0.707,x:0.0,y:0.0,z:0.707)
```

_javascript_
```javascript
let goal = {Rotation:[0.707,0.0,0.0,0.707]} // [x, y, z, w] ordering
```

**Scalar**

The scalar goal is used by the `JointMatch`, `JointMirroring`, `DistanceMatch`, `JointLiveliness`, and `RelativeMotionLiveliness` objectives.

_python_
```python
goal = 0.5
```

_javascript_
```javascript
let goal = {Scalar:0.5}
```

**Size**

The size goal is used by the `PositionLiveliness`, `OrientationLiveliness`, `OriginPositionLiveliness`, and `OriginOrientationLiveliness` objectives.

_python_
```python
goal = Size(x:1.0,y:0.1,z:0.5)
```

_javascript_
```javascript
let goal = {Size:[1.0,0.1,0.5]}
```

**Ellipse**

The ellipse goal is used by the `PositionBounding` objective.

_python_
```python
goal = Ellipse(
    translation=Translation(x:1.0,y:0.0,z:0.4),
    rotation=Rotation(w:0.707,x:0.0,y:0.0,z:0.707),
    size=Size(x:0.1,y:0.1,z:0.2)
)
```

_javascript_
```javascript
let goal = {Ellipse: {
    pose: {translation: [1.0,0.0,0.4], rotation: [0.707,0.0,0.0,0.707]}, // [x, y, z, w] ordering for quaternion
    size: [0.1,0.1,0.2]
}
```

**RotationRange**

The rotation range goal is used by the `RotationBounding` objective.

_python_
```python
goal = RotationRange(
    rotation=Rotation(w:0.707,x:0.0,y:0.0,z:0.707),
    delta=0.4
)
```

_javascript_
```javascript
let goal = {RotationRange: {
    rotation: [0.707,0.0,0.0,0.707], // [x, y, z, w] ordering for quaternion
    delta:0.4
}
```

**ScalarRange**

The scalar range goal is used by the `JointBounding` objective.

_python_
```python
goal = ScalarRange(value=0.0,delta=0.4)
```

_javascript_
```javascript
let goal = {ScalarRange: {value:0.0,delta:0.4}
```

### Shapes

There are 4 different Shape classes: Box, Sphere, Capsule, and Cylinder. The `name` field is entirely for your own usage, and the `frame` field indicates what robot link the shape is attached to (by default "world"). The `physical` field indicates whether the shape presents a collision, and should be factored into collision avoidance. Otherwise, the shape is simply tracked in the state if close enough.

**Box**

_python_
```python
shape = BoxShape(
    name="camera attachment",
    frame='panda_hand',
    physical=True,
    x=0.5,y=0.5,z=0.2,
    local_transform=Transform.identity())
```

_javascript_
```javascript
let shape = {
    type:'Box',
    name:'camera attachment',
    frame: 'panda_hand',
    physical: true,
    x:0.5,y:0.5,z:0.2,
    localTransform: {translation:[0.0,0.0,0.0],rotation:[0.0,0.0,0.0,1.0]} // [x, y, z, w] ordering for quaternion
    }
```

**Sphere**

_python_
```python
shape = SphereShape(
    name="bouncy ball",
    frame='world',
    physical=True,
    radius=0.1,
    local_transform=Transform.identity())
```

_javascript_
```javascript
let shape = {
    type:'Sphere',
    name:'bouncy ball',
    frame: 'world',
    physical: true,
    radius:0.1,
    localTransform: {translation:[0.0,0.0,0.0],rotation:[0.0,0.0,0.0,1.0]} // [x, y, z, w] ordering for quaternion
    }
```

**Capsule**

_python_
```python
shape = CapsuleShape(
    name="pill",
    frame='world',
    physical=True,
    length=0.2,
    radius=0.1,
    local_transform=Transform.identity())
```

_javascript_
```javascript
let shape = {
    type:'Capsule',
    name:'pill',
    frame: 'world',
    physical: true,
    length:0.2,
    radius:0.1,
    localTransform: {translation:[0.0,0.0,0.0],rotation:[1.0,0.0,0.0,0.0]} // [x, y, z, w] ordering for quaternion
    }
```

**Cylinder**

_python_
```python
shape = CylinderShape(
    name="zone",
    frame='world',
    physical=False,
    length=0.2,
    radius=0.1,
    local_transform=Transform.identity())
```

_javascript_
```javascript
let shape = {
    type:'Cylinder',
    name:'zone',
    frame: 'world',
    physical: false,
    length:0.2,
    radius:0.1,
    localTransform: {translation:[0.0,0.0,0.0],rotation:[0.0,0.0,0.0,1.0]} // [x, y, z, w] ordering for quaternion
    }
```

#### State

The `State` object is the response back after calling `solve`. It contains the state of the robot in terms of joint and frames, as well as some diagnostic information regarding proximity of various shapes and the center-of-mass of the robot. 

**Origin**

The transform of the root of the robot. This is useful if the root node is movable. Exported as a Transform class (python) or object containing `translation` and `rotation` fields (javascript).

**Joints**

A lookup table of the joint values for each movable joint. Exported as a dictionary (python) or object (javascript).

**Frames**

A lookup table of the positions/orientations for each link. Exported as a dictionary (python) or object (javascript) of transform objects keyed by the link name.

**Proximity**

A list of all shapes that are currently tracked and that reach close enough proximity to potentially factor into collision detection. Exported as a ProximityInfo class (python) or object (javascript) with the following attributes:

1. `shape1` (`string`) The name of the first shape (note, if the robot is initialized with this shape, and it is attached to a robot link, the name is the link it is attached to.)
2. `shape2` (`string`) The name of the second shape (note, if the robot is initialized with this shape, and it is attached to a robot link, the name is the link it is attached to.)
3. `distance` (`float`/`number`/`None`/`null`) The distance recorded between the two shapes. May possibly be `None` or `null` if not tracked, and is zero or negative if the shapes are intersecting. 
4. `points` (`None`/`set of two points`) The closest points on two different shapes that are used determined whether these two shapes are close though to cause a collision based on the distance between the points on these two shapes. 
5. `physical` (`bool`) True if both shapes are physical, otherwise False.



1. If the distance between points on two different shapes is smaller than or equal to `0.0`, two shapes are `intersecting` with each other. 
2. If the distance between points on two different shapes is bigger than `0.0` but smaller than a `user defined distance`, two shapes are `with in margin` with each other within `user defined distance`.

**Center of Mass**

A translation (vector) indicating the current center of mass of the robot.







## Contributing


**Python Instructions**

To build, download and `cd` to this directory. Then run:

```bash
# If you just want to install:
python3 setup.py install


# Or if you are developing:
python3 setup.py develop

# If you are developing and need to rebuild:
python3 setup.py clean && python3 setup.py develop
```

You will need this installed to use the ROS2 LivelyIK Package.

**Javascript Instructions**

To build, download and `cd` to this directory. Then run:

```bash
# Build the javascript bundle
wasm-pack build --scope people_and_robots --target web -- --features jsbindings

# Pack
wasm-pack pack

# Publish
wasm-pack publish --access=public
```

## References
[^1]:Rakita, Daniel, Bilge Mutlu, and Michael Gleicher. "PROXIMA: An Approach for Time or Accuracy Budgeted Collision Proximity Queries." Proceedings of Robotics: Science and Systems (RSS). 2022. http://www.roboticsproceedings.org/rss18/p043.pdf


