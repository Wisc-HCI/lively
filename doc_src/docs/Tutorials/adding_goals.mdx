# Adding Goals

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

:::note
Since Lively is still in beta, the design is subject to change and should not be considered final!
:::

A developer may desire to create a lifelike behavior that exhibits positional and rotational motion around an offset `focal point`,
as if inspecting the properties of an object located there. Doing so
requires the addition of an new [`goal`](../API/Goals/goal.mdx) type, which would encode the
`focal length` to maintain the position of the focus, and the amount
of `rotational/translational` movement allowed. The [`objective’s`](../API/Objectives/) `𝑐𝑎𝑙𝑙`
method would use these goals and a Perlin noise generator function
to project the needed position and orientation in space to achieve
the specified rotation around the focus at a given time and compute
the radial and translational distance from those values, returning a
cost value. The resulting [`objective`](../API/Objectives/) would attempt to produce poses
that adhered to this dynamic pattern as a function of time.

To demonstrate how to create additional lifelike behavior mentioned above, the developer will have to makes changes to the following files:
- [`src/objectives/liveliness/perlin.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/liveliness/perlin.rs)
- [`src/objectives/objective.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/objective.rs)
- [`src/utils/goals.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/utils/goals.rs)
- [`src/utils/info.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/utils/info.rs)

We will name the new objective: `PerspectiveLivelinessObjective` and the new goal: `Cone`.

<Tabs>
  <TabItem value="perlin" label="src/objectives/liveliness/perlin.rs">

```rust
    ...
    [repr(C)]
    #[derive(Serialize, Deserialize, Clone, Debug, Default)] #[cfg_attr(feature = "pybindings", pyclass)]
    pub struct PerspectiveLivelinessObjective {
        // Adds position liveliness to the specified link
        pub name: String,
        pub weight: f64,
        pub link: String,
        pub frequency: f64,

        // Goal Value (shape of noise)
        #[serde(skip)]
        pub goal: Cone,
        #[serde(skip)]
        pub time: Option<f64>,

        // Inaccessible
        #[serde(skip)]
        pub noise: Vector3<f64>,
        #[serde(skip, default = "get_default_perlin")]
        pub perlin: Perlin,
        #[serde(skip, default = "get_default_offsets")]
        pub offsets: [f64; 3],
    }

    impl PerspectiveLivelinessObjective {
        pub fn new(name: String, weight: f64, link: String, frequency: f64) -> Self {
            let mut rng: ThreadRng = thread_rng();
            let seed: u32 = rng.gen();
            let perlin: Perlin = Perlin::new().set_seed(seed);
            let offsets: [f64; 3] = [
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000)),
            ];
        Self {
            name,
            weight,
            link,
            frequency,
            goal: Cone::default(),
            time: None,
            noise: vector![0.0, 0.0, 0.0],
            perlin,
            offsets,
            }
        }
    }

    impl Callable<Cone> for PerspectiveLivelinessObjective {
        fn call(&self, v: &Vars, state: &State) -> f64 {
            //implementation ommitted
            ...
            return ;
         }

        fn update(&mut self, time: f64) {
            //implementation ommitted
        }

        fn set_goal(&mut self, goal: Cone) {
            self.goal = goal;
        }

        fn set_weight(&mut self, weight: f64) {
            self.weight = weight;
        }
    }

    #[cfg(feature = "pybindings")]
    #[pymethods]
    impl PerspectiveLivelinessObjective {
        #[new]
        pub fn from_python(name:String,weight:f64,link:String,frequency:f64) -> Self {
            PerspectiveLivelinessObjective::new(name,weight,link,frequency)
        }

        #[getter]
        pub fn get_name(&self) -> PyResult<String> {
            Ok(self.name.clone())
        }

        #[getter]
        pub fn get_weight(&self) -> PyResult<f64> {
            Ok(self.weight.clone())
        }

        #[getter]
        pub fn get_link(&self) -> PyResult<String> {
            Ok(self.link.clone())
        }

        #[getter]
        pub fn get_frequency(&self) -> PyResult<f64> {
            Ok(self.frequency.clone())
        }
    }
    ...
```

  </TabItem>

  <TabItem value="objective" label="src/objectives/objective.rs">

```rust
    pub enum Objective {
        ...
        LinkVelocityMinimization(LinkVelocityMinimizationObjective),
        LinkAccelerationMinimization(LinkAccelerationMinimizationObjective),
        LinkJerkMinimization(LinkJerkMinimizationObjective),
        RelativeMotionLiveliness(RelativeMotionLivelinessObjective),
        //The additional objectives (PerspectiveLivelinessObjective)
        PerspectiveLiveliness(PerspectiveLivelinessObjective),
        //--------------------------------------------------
        Gravity(GravityObjective),
        ...
    }

    impl Objective {
        pub fn get_type(&self) -> String {
            // Returns a string value for each variant. Useful in debugging.
            match self {
                ...
                Self::Gravity(_obj) => return String::from("GravityObjective"),
                Self::SmoothnessMacro(_obj) => return String::from("SmoothnessMacroObjective"),
                Self::DistanceMatch(_obj) => return String::from("DistanceMatchObjective"),
                //The additional objectives (PerspectiveLivelinessObjective)
                Self::PerspectiveLiveliness(_obj) => return String::from("PerspectiveLivelinessObjective")
                //---------------------------------------------------------
            }
        }

        pub fn call(&self,v: &Vars,state: &State) -> f64 {
            // A switch that passes along the `call` method to the inner objective.
            match self {
                ...
                Self::Gravity(obj) => obj.call(v,state),
                Self::SmoothnessMacro(obj) => obj.call(v,state),
                Self::DistanceMatch(obj) => obj.call(v,state),
                //The additional objectives (PerspectiveLivelinessObjective)
                Self::PerspectiveLiveliness(obj) => obj.call(v,state)
                //---------------------------------------------------
            }
        }

        pub fn update(&mut self, time: f64) {
            // For time-sensitive objectives, include them here.
            match self {
                Self::PositionLiveliness(obj) => obj.update(time),
                Self::OrientationLiveliness(obj) => obj.update(time),
                Self::JointLiveliness(obj) => obj.update(time),
                Self::RelativeMotionLiveliness(obj) => obj.update(time),
                //The additional objectives (PerspectiveLivelinessObjective)
                Self::PerspectiveLiveliness(obj) => obj.update(time),
                //--------------------------------------------------
                _ => {}
            }
        }

        pub fn set_weight(&mut self, weight: f64) {
            // Set the weight for the inner objective
            match self {
                ...
                Self::Gravity(obj) => obj.set_weight(weight),
                Self::SmoothnessMacro(obj) => obj.set_weight(weight),
                Self::DistanceMatch(obj) =>  obj.set_weight(weight),
                //The additional objectives (PerspectiveLivelinessObjective)
                Self::PerspectiveLiveliness(obj) => obj.set_weight(weight)
                //--------------------------------------------------------
            }
        }

        pub fn get_goal(&self) -> Option<Goal> {
            // get the goal for the inner objective. Useful for debugging.
            match self {
                ...
                Self::Gravity(_obj) => return None,
                Self::SmoothnessMacro(_obj) => return None,
                Self::DistanceMatch(obj) => return Some(Goal::Scalar(obj.goal)),
                //The additional objectives (PerspectiveLivelinessObjective)
                Self::PerspectiveLiveliness(obj) => return Some(Goal::Cone(obj.goal))
                //-------------------------------------------------------------------
            }
        }

        pub fn set_goal(&mut self, goal: &Goal) {
            // Set the goal for the inner objective. This matches based on Objective and Goal variant.
            match (goal,self) {
                ...
                (Goal::Ellipse(ellipse_goal),Self::PositionBounding(obj)) => obj.set_goal(*ellipse_goal),
                (Goal::RotationRange(rotation_range_goal),Self::OrientationBounding(obj)) => obj.set_goal(*rotation_range_goal),
                (Goal::ScalarRange(scalar_range_goal),Self::JointBounding(obj)) => obj.set_goal(*scalar_range_goal),
                //The additional goal (Cone) and objectives (PerspectiveLivelinessObjective)
                (Goal::Cone(cone_goal), Self::PerspectiveLiveliness(obj)) => obj.set_goal(*cone_goal),
                //-----------------------------------------------------------------------------------
                (g,o) => {
                    println!("Unexpected goal {:?} provided for Objective {:?}",g,o.clone())
                }
            }
        }
    }


    #[cfg(feature = "pybindings")]
    impl IntoPy<PyObject> for Objective {
        fn into_py(self, py: Python) -> PyObject {
            match self {
                ...
                Self::Gravity(obj) => obj.into_py(py),
                Self::SmoothnessMacro(obj) => obj.into_py(py),
                Self::DistanceMatch(obj) => obj.into_py(py),
                //The additional objectives (PerspectiveLivelinessObjective) for python binding
                Self::PerspectiveLiveliness(obj) => obj.into_py(py),
                //-------------------------------------------------
            }
        }
    }
```

  </TabItem>

  <TabItem value="goals" label="src/utils/goals.rs">

```rust
    ...
    #[repr(C)]
    #[derive(Serialize,Deserialize,Clone,Debug)]
    pub enum Goal {
        ...
        RotationRange(RotationRange),
        ScalarRange(ScalarRange),
        //The additional goal (Cone)
        Cone(Cone)
        //--------
    }

    #[cfg(feature = "pybindings")]
    impl IntoPy<PyObject> for Goal {
        fn into_py(self, py: Python) -> PyObject {
            match self {
                ...
                Self::RotationRange(obj) => obj.into_py(py),
                Self::ScalarRange(obj) => obj.into_py(py),
                //The additional goal (Cone)
                Self::Cone(obj) => obj.into_py(py)
                //-----------------------------
            }
        }
    }

    #[cfg(feature = "pybindings")]
    impl FromPyObject<'_> for Goal {
        fn extract(ob: &'_ PyAny) -> PyResult<Self> {
            ...
            if let Ok(ob) = ScalarRange::extract(ob) {
                return Ok(Self::ScalarRange(ob))
            }
            //The additional goal (Cone)
            if let Ok(ob) = Cone::extract(ob){
                return Ok(Self::Cone(ob))
            }
            //-------------------------------


            return Ok(Self::Scalar(0.0));
        }
    }
    ...
```

  </TabItem>

  <TabItem value="info" label="src/utils/info.rs">

```rust
    ...
    //The additional struct for the new goal (Cone)
    #[repr(C)]
    #[derive(Serialize, Deserialize, Clone, Debug, Default, Copy)]
    #[cfg_attr(feature = "pybindings", pyclass)]
    pub struct Cone {
        pub focal_point : Vector3<f64>,
        pub focal_length : f64,
        pub delta: f64,
    }

    impl Cone {
        pub fn new(focal_point: Vector3<f64>, focal_length: f64,delta: f64) -> Self {
            Self { focal_point,focal_length, delta }
        }
    }

    #[cfg(feature = "pybindings")]
    #[pymethods]
    impl Cone {
        #[new]
        pub fn from_python(focal_point: PyPoint3, focal_length: f64, delta: f64) -> Self {
            Self::new(focal_point.value, focal_length, delta)
        }

        fn as_str(&self) -> String {
            format!("Cone: {{focal_point: {:?}, focal_length: {:?}, delta: {:?} }}",
                self.focal_point,
                self.focal_length,
                self.delta
            )
        }

        pub fn __str__(&self) -> PyResult<String> {
            Ok(self.as_str())
        }

        pub fn __repr__(&self) -> PyResult<String> {
            Ok(self.as_str())
        }

        #[getter]
        pub fn get_focal_point(&self) -> PyResult<PyPoint3>{
            Ok(PyPoint3{value: self.focal_point})
        }

        #[getter]
        pub fn get_focal_length(&self) -> PyResult<f64>{
            Ok(self.focal_length)
        }

        #[getter]
        pub fn get_delta(&self) -> PyResult<f64>{
            Ok(self.delta)
        }
    }
    //--------------------------------------
    ...
```

  </TabItem>

</Tabs>
