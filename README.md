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
    objectives=[
        PositionMatchObjective(name="EE Position",link="panda_hand",weight=50),
        OrientationMatchObjective(name="EE Rotation",link="panda_hand",weight=25),
        SmoothnessMacroObjective(name="General Smoothness",weight=10),
        CollisionAvoidanceObjective(name="Collision Avoidance",weight=10)
        ...
    ], 
    root_bounds=[
        ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0), # Translational
        ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0)  # Rotational
    ],
    shapes=[
        BoxShape(name="Table",frame="world",physical=True,x=2,y=1,z=1.2,local_transform=Transform.isometry())
    ], 
    initial_state=State(origin=Transform.identity(),joints={"panda_joint1":0.0,"panda_joint2":0.0,...}), # Optional
    only_core=False, # Only use this flag if you are not using liveliness objectives and want a slight speed-up.
    max_retries=1, # Number of times the solution is attempted (default 1)
    max_iterations=150 # Number of iterations per try (default 150)
)
```

_javascript_
```javascript
import {
    Solver, PositionMatchObjective, OrientationMatchObjective, 
    SmoothnessMacroObjective, CollisionAvoidanceObjective, 
    State, Transform, ScalarRange, BoxShape} from "@people_and_robots/lively_tk";

let solver = new Solver(
    '<?xml version="1.0" ?><robot name="panda">...</robot>', // Full urdf as a string
    [
        {type:'PositionMatch',name:"EE Position",link:"panda_hand",weight:50},
        {type:'OrientationMatch',name:"EE Rotation",link:"panda_hand",weight:25},
        {type:'SmoothnessMacro',name:"General Smoothness",weight:10},
        {type:'CollisionAvoidance',name:"Collision Avoidance",weight:10}
        ...
    ], 
    [
        {value:0.0,delta:0.0},{value:0.0,delta:0.0},{value:0.0,delta:0.0}, // Translational
        {value:0.0,delta:0.0},{value:0.0,delta:0.0},{value:0.0,delta:0.0}  // Rotational
    ],
    [
        {type:'Box',name="Table",frame:"world",physical:True,x:2,y:1,z:1.2,localTransform:{translation:[0,0,0],rotation:[1,0,0,0]}}
    ], 
    {origin:{translation:[0,0,0],rotation:[1,0,0,0]},joints:{panda_joint1:0.0,panda_joint2:0.0,...}}, // Optional
    false, // Only use this flag if you are not using liveliness objectives and want a slight speed-up.
    1, // Number of times the solution is attempted (default 1)
    150 // Number of iterations per try (default 150)
)
```

## Resetting 

In both the Javascript and Python interfaces, the `Solver` class has a `reset` method that allows the user to reset the state of the solver given some new objective weights and a new robot state. In this case, the robot state only needs to supply the `joints` and `origin` field, as shown in the initialization example.

_python_
```python
solver.reset(state=State(origin=Transform.identity(),joints={"panda_joint1":0.0,"panda_joint2":0.0,...}),weights=[50.0,30.0,20.0,10.0])
```

_javascript_
```javascript
solver.reset(
    {origin:{translation:[0,0,0],rotation:[1,0,0,0]},joints:{panda_joint1:0.0,panda_joint2:0.0,...}}, // New starting state
    [50.0,30.0,20.0,10.0] // New starting weights
)
```

## Solving

The `Solver` class has a `solve` method that represents the core functionality of the LivelyTK interface. At a high level, it accepts the following fields:

1. `goals`: A list of goal-type objects.
2. `weights`: A list of floats, order corresponding to the order of the objectives.
3. `time`: (float) The current time. If no liveliness objectives are used, this has no effect.
4. `shapes`: A list of shape objects. 

The `solve` method returns a fully-filled `State` object

### Goals

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
let goal = {Rotation:[0.707,0.0,0.0,0.707]}
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
    pose: {translation: [1.0,0.0,0.4], rotation: [0.707,0.0,0.0,0.707]},
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
    rotation: [0.707,0.0,0.0,0.707],
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
    local_transform: {translation:[0.0,0.0,0.0],rotation:[1.0,0.0,0.0,0.0]}
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
    local_transform: {translation:[0.0,0.0,0.0],rotation:[1.0,0.0,0.0,0.0]}
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
    local_transform: {translation:[0.0,0.0,0.0],rotation:[1.0,0.0,0.0,0.0]}
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
    local_transform: {translation:[0.0,0.0,0.0],rotation:[1.0,0.0,0.0,0.0]}
    }
```

### State

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
3. `distance` (`float`/`number`/`None`/`null`) The distance recorded between the two shapes. May possibly be `None` or `null` if not tracked, and is zero if the shapes are intersecting. 
4. `points` (`None`/`set of two points`) The closest points on two different shapes that are used determined whether these two shapes are close though to cause a collision based on the distance between the points on these two shapes. `points` will be `None` and `distance` will be `0.0` if two shapes are `interesecting` and colliding. `points` will return a `set of two points` and `distance` will be greater than `0.0` but smaller than `1.0` if two shapes are close enough(smaller than 1.0) to potentially cause a collision but not yet colliding. Any distance between two points bigger than `1.0` are not considered for collision detection. 
5. `physical` (`bool`) True if both shapes are physical, otherwise False.



1. If the distance between points on two different shapes is `0.0`, two shapes are `intersecting` with each other. 
2. If the distance between points on two different shapes is bigger than `0.0` but smaller than `1.0`, two shapes are `with in margin` with each other.

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
