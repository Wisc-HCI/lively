"use strict";(self.webpackChunklively_documentation=self.webpackChunklively_documentation||[]).push([[53],{1109:e=>{e.exports=JSON.parse('{"pluginId":"default","version":"current","label":"Next","banner":null,"badge":false,"noIndex":false,"className":"docs-version-current","isLast":true,"docsSidebars":{"docs":[{"type":"link","label":"Overview","href":"/docs/API/","docId":"API/index"},{"type":"category","label":"Solver","items":[{"type":"link","label":"Initialization","href":"/docs/API/Solver/initialization","docId":"API/Solver/initialization"},{"type":"category","label":"Methods","items":[{"type":"link","label":"Solving","href":"/docs/API/Solver/Methods/solve","docId":"API/Solver/Methods/solve"},{"type":"link","label":"Resetting","href":"/docs/API/Solver/Methods/reset","docId":"API/Solver/Methods/reset"},{"type":"link","label":"Collision Normalization","href":"/docs/API/Solver/Methods/collision_normalization","docId":"API/Solver/Methods/collision_normalization"}],"collapsed":true,"collapsible":true,"href":"/docs/category/methods"},{"type":"link","label":"Properties","href":"/docs/API/Solver/Properties/","docId":"API/Solver/Properties/index"}],"collapsed":true,"collapsible":true,"href":"/docs/API/Solver/"},{"type":"category","label":"Shapes","items":[{"type":"link","label":"Box","href":"/docs/API/Shapes/box","docId":"API/Shapes/box"},{"type":"link","label":"Sphere","href":"/docs/API/Shapes/sphere","docId":"API/Shapes/sphere"},{"type":"link","label":"Cylinder","href":"/docs/API/Shapes/cylinder","docId":"API/Shapes/cylinder"},{"type":"link","label":"Capsule","href":"/docs/API/Shapes/capsule","docId":"API/Shapes/capsule"}],"collapsed":true,"collapsible":true,"href":"/docs/API/Shapes/"},{"type":"link","label":"State","href":"/docs/API/state","docId":"API/state"},{"type":"link","label":"Goals","href":"/docs/API/Goals/goal","docId":"API/Goals/goal"},{"type":"category","label":"Info","items":[{"type":"link","label":"Transform Info","href":"/docs/API/Info/transformInfo","docId":"API/Info/transformInfo"},{"type":"link","label":"Mimic Info","href":"/docs/API/Info/mimicInfo","docId":"API/Info/mimicInfo"},{"type":"link","label":"Joint Info","href":"/docs/API/Info/jointInfo","docId":"API/Info/jointInfo"},{"type":"link","label":"Link Info","href":"/docs/API/Info/linkInfo","docId":"API/Info/linkInfo"},{"type":"link","label":"Proximity Info","href":"/docs/API/Info/proximityInfo","docId":"API/Info/proximityInfo"},{"type":"link","label":"Collision Settings","href":"/docs/API/Info/collisionSettingInfo","docId":"API/Info/collisionSettingInfo"}],"collapsed":true,"collapsible":true,"href":"/docs/API/Info/"},{"type":"category","label":"Objectives","items":[{"type":"link","label":"Base","href":"/docs/API/Objectives/base","docId":"API/Objectives/base"},{"type":"link","label":"Bounding","href":"/docs/API/Objectives/bounding","docId":"API/Objectives/bounding"},{"type":"link","label":"Matching","href":"/docs/API/Objectives/matching","docId":"API/Objectives/matching"},{"type":"link","label":"Mirroring","href":"/docs/API/Objectives/mirroring","docId":"API/Objectives/mirroring"},{"type":"link","label":"Liveliness","href":"/docs/API/Objectives/liveliness","docId":"API/Objectives/liveliness"},{"type":"link","label":"Forces","href":"/docs/API/Objectives/forces","docId":"API/Objectives/forces"}],"collapsed":true,"collapsible":true,"href":"/docs/API/Objectives/"},{"type":"link","label":"Collision","href":"/docs/API/collision","docId":"API/collision"}],"tutorials":[{"type":"link","label":"Overview","href":"/docs/Tutorials/","docId":"Tutorials/index"},{"type":"link","label":"Installation","href":"/docs/Tutorials/installation","docId":"Tutorials/installation"},{"type":"category","label":"Basic Usage","items":[{"type":"link","label":"Basic Initialization","href":"/docs/Tutorials/initialization","docId":"Tutorials/initialization"},{"type":"link","label":"Advanced Initialization","href":"/docs/Tutorials/advanced_initialization","docId":"Tutorials/advanced_initialization"},{"type":"link","label":"Solving","href":"/docs/Tutorials/solving","docId":"Tutorials/solving"},{"type":"link","label":"Adding Liveliness","href":"/docs/Tutorials/liveliness","docId":"Tutorials/liveliness"},{"type":"link","label":"Environment Modeling","href":"/docs/Tutorials/environment","docId":"Tutorials/environment"}],"collapsed":true,"collapsible":true,"href":"/docs/category/basic-usage"},{"type":"category","label":"Customized Usage","items":[{"type":"link","label":"Adding Objectives","href":"/docs/Tutorials/adding_objectives","docId":"Tutorials/adding_objectives"},{"type":"link","label":"Adding Goals","href":"/docs/Tutorials/adding_goals","docId":"Tutorials/adding_goals"}],"collapsed":true,"collapsible":true,"href":"/docs/category/customized-usage"}]},"docs":{"API/collision":{"id":"API/collision","title":"Collision","description":"Lively implements the PROXIMA collision detection algorithm, which allows for time-efficient collision","sidebar":"docs"},"API/Goals/goal":{"id":"API/Goals/goal","title":"Goals","description":"There are a variety of different \\"goal\\" types that can be provided. Think of these as settings that you would like to achieve (e.g. a PositionMatch objective accepts a Translation goal).","sidebar":"docs"},"API/index":{"id":"API/index","title":"Overview","description":"- The Lively framework provides a highly configurable toolkit for commanding robots in mixed modalities while incorporating liveliness motions. It is adapted from RelaxedIK framework, and compatible with Python and Javascript/Node.","sidebar":"docs"},"API/Info/collisionSettingInfo":{"id":"API/Info/collisionSettingInfo","title":"Collision Settings","description":"This is a parameter collision_settings that allows the user to customize the collision checking system while configuring a Solver","sidebar":"docs"},"API/Info/index":{"id":"API/Info/index","title":"Info","description":"Lively provides a set of data structures that contain information that can be provided to the solver or are returned from the solver.","sidebar":"docs"},"API/Info/jointInfo":{"id":"API/Info/jointInfo","title":"Joint Info","description":"JointInfo contains information about a joint such as name, type, lower bound, upper bound, parent link, or child link. It is generated through the parsing of the input URDF and is accessible as a property or getter of the solver.","sidebar":"docs"},"API/Info/linkInfo":{"id":"API/Info/linkInfo","title":"Link Info","description":"LinkInfo captures information such as name, and parent joint. It is generated through the parsing of the input URDF and is accessible as a property or getter of the solver.","sidebar":"docs"},"API/Info/mimicInfo":{"id":"API/Info/mimicInfo","title":"Mimic Info","description":"MimicInfo is an optional subset of JointInfo is present when the given joint mimics another. Contains information about the identity of the other joint, as well as multiplier scaling and offset values.","sidebar":"docs"},"API/Info/proximityInfo":{"id":"API/Info/proximityInfo","title":"Proximity Info","description":"ProximityInfo data is included in the state and captures information about the pairwise distances and collision state when two objects in the scene are near one another.","sidebar":"docs"},"API/Info/transformInfo":{"id":"API/Info/transformInfo","title":"Transform Info","description":"TransformInfo captures the world and local transformation of a link, joint, or environmental object. This is commonly included within the State data structure, with one value for each link of the robot.","sidebar":"docs"},"API/Objectives/base":{"id":"API/Objectives/base","title":"Base","description":"Base objectives revolve around the fluidity of robot motion by limiting rapid changes and considering possible collisions between the links of the robot.","sidebar":"docs"},"API/Objectives/bounding":{"id":"API/Objectives/bounding","title":"Bounding","description":"Bounding Objectives limit the space within which joints can assume angles and links can move or be oriented.","sidebar":"docs"},"API/Objectives/forces":{"id":"API/Objectives/forces","title":"Forces","description":"Forces Objectievs proximate the physcial forces applied to the robot.","sidebar":"docs"},"API/Objectives/index":{"id":"API/Objectives/index","title":"Objective","description":"Lively allows for a wide range of robot with which users program robot motion. These 24 properties, which serve as building blocks for defining the behavior and motion of the robot, fit into five categories:","sidebar":"docs"},"API/Objectives/liveliness":{"id":"API/Objectives/liveliness","title":"Liveliness","description":"Liveliness Objectives allow adding smooth, coordinated motion to joint angles or link positions/orientations.","sidebar":"docs"},"API/Objectives/matching":{"id":"API/Objectives/matching","title":"Matching","description":"Matching Objectives specify exact positions and orientations of links or angles of joints, while bounding behavior properties set limits.","sidebar":"docs"},"API/Objectives/mirroring":{"id":"API/Objectives/mirroring","title":"Mirroring","description":"Mirroring objectives allow users to mirror the current state of a link\'s position or orientation in a different link, or the current angle of one joint in another.","sidebar":"docs"},"API/Shapes/box":{"id":"API/Shapes/box","title":"Box","description":"A 6-sided cuboid that captures colliders that have cuboid properties.","sidebar":"docs"},"API/Shapes/capsule":{"id":"API/Shapes/capsule","title":"Capsule","description":"A 3D capsule shape that captures colliders with curvilinear geometric properties.","sidebar":"docs"},"API/Shapes/cylinder":{"id":"API/Shapes/cylinder","title":"Cylinder","description":"A 3D cylinder shape that captures colliders with curvilinear geometric properties.","sidebar":"docs"},"API/Shapes/index":{"id":"API/Shapes/index","title":"Shapes","description":"There are 4 different Shape classes: Box, Sphere, Cylinder, and Capsule. The name field is entirely for your own usage, and the frame field indicates what robot link the shape is attached to (by default \\"world\\"). The physical field indicates whether the shape presents a collision, and should be factored into collision avoidance. Otherwise, the shape is simply tracked in the state if close enough.","sidebar":"docs"},"API/Shapes/sphere":{"id":"API/Shapes/sphere","title":"Sphere","description":"A 3D sphere that captures colliders that have ball-like properties.","sidebar":"docs"},"API/Solver/index":{"id":"API/Solver/index","title":"Solver","description":"Introduction","sidebar":"docs"},"API/Solver/initialization":{"id":"API/Solver/initialization","title":"Initialization","description":"Configuring of Lively is centered on the Solver class, which you can instantiate with the following parameters:","sidebar":"docs"},"API/Solver/Methods/collision_normalization":{"id":"API/Solver/Methods/collision_normalization","title":"Collision Normalization","description":"Collision Detection is a computational-heavy calculation that allows for collision and proximity detection for robots and environmental shapes.","sidebar":"docs"},"API/Solver/Methods/reset":{"id":"API/Solver/Methods/reset","title":"Resetting","description":"In both the Javascript and Python interfaces, the Solver class has a reset method that allows the user to reset the state of the solver given new objective, weights and a new robot state. In this case, the robot state only needs to supply the joints and origin field, as shown in the initialization example.","sidebar":"docs"},"API/Solver/Methods/solve":{"id":"API/Solver/Methods/solve","title":"Solving","description":"The Solver class has a solve method that represents the core functionality of the Lively interface. At a high level, it returns a fully-filled state object and accepts the following fields:","sidebar":"docs"},"API/Solver/Properties/index":{"id":"API/Solver/Properties/index","title":"Properties","description":"| Property | Description |","sidebar":"docs"},"API/state":{"id":"API/state","title":"State","description":"The State object is the response back after calling solve.","sidebar":"docs"},"Tutorials/adding_goals":{"id":"Tutorials/adding_goals","title":"Adding Goals","description":"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!","sidebar":"tutorials"},"Tutorials/adding_objectives":{"id":"Tutorials/adding_objectives","title":"Adding Objectives","description":"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!","sidebar":"tutorials"},"Tutorials/advanced_initialization":{"id":"Tutorials/advanced_initialization","title":"Advanced Initialization","description":"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!","sidebar":"tutorials"},"Tutorials/environment":{"id":"Tutorials/environment","title":"Environment Modeling","description":"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!","sidebar":"tutorials"},"Tutorials/index":{"id":"Tutorials/index","title":"Overview","description":"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!","sidebar":"tutorials"},"Tutorials/initialization":{"id":"Tutorials/initialization","title":"Basic Initialization","description":"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!","sidebar":"tutorials"},"Tutorials/installation":{"id":"Tutorials/installation","title":"Installation","description":"","sidebar":"tutorials"},"Tutorials/liveliness":{"id":"Tutorials/liveliness","title":"Adding Liveliness","description":"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!","sidebar":"tutorials"},"Tutorials/solving":{"id":"Tutorials/solving","title":"Solving","description":"NOTE: Since Lively is still in beta, the design is subject to change and should not be considered final!","sidebar":"tutorials"}}}')}}]);