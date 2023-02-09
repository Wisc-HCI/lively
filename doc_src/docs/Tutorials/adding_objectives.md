# Adding Objectives

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

:::note
Since Lively is still in beta, the design is subject to change and should not be considered final!
:::


`Lively` can be greatly extended through the development of additional [`objectives`](../API/Objectives). In order to add your own new [`objectives`](../API/Objectives), there are three files you will have to modify. 
In the example below, an additional `CenterOfMassMatchObjective` is created. Because the robot state already includes a vector representing the [`center-of-mass`](../API/state)
of the robot, it is straightforward to create a new [`objectives`](../API/Objectives) of `CenterOfMassMatch`,
which could be useful in cases where the robot’s balance must
be maintained, or as a way to center the robot near its base. The changes are made in [`src/lib.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/lib.rs), 
[`src/objectives/objective.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/objective.rs), and [`src/objectives/core/matching.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/core/matching.rs). 


:::note
`CenterOfMassMatchObjective` is a type of [`matching objective`](../API/Objectives/matching.mdx) and we make the change to [`src/objectives/core/matching.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/core/matching.rs). Depending on the type of [`objective`](../API/Objectives) you want to make, you will have to make corresponding changes to 
- [`src/objectives/core/base.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/core/base.rs)
- [`src/objectives/core/bounding.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/core/bounding.rs)
- [`src/objectives/matching.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/core/matching.rs)
- [`src/objectives/core/mirroring.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/core/mirroring.rs)
- [`src/objectives/liveliness/forces.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/liveliness/forces.rs)
- [`src/objectives/liveliness/perlin.rs`](https://github.com/Wisc-HCI/lively/blob/master/src/objectives/liveliness/perlin.rs)
:::


<Tabs>
  <TabItem value="lib.rs" label="src/lib.rs">

```rust
    ...
    m.add_class::<objectives::core::bounding::PositionBoundingObjective>()?;
    m.add_class::<objectives::core::bounding::OrientationBoundingObjective>()?;
    m.add_class::<objectives::core::bounding::JointBoundingObjective>()?;
    // An example of additional objectives (CenterOfMassMatchingObjective)
    m.add_class::<objectives::core::matching::CenterOfMassMatchObjective>()?;
    // ------------------------------------------------------------------
    m.add_class::<objectives::core::matching::PositionMatchObjective>()?;
    m.add_class::<objectives::core::matching::OrientationMatchObjective>()?;
    m.add_class::<objectives::core::matching::JointMatchObjective>()?;
    m.add_class::<objectives::core::matching::DistanceMatchObjective>()?;
    ...
```

  </TabItem>

  <TabItem value="objective.rs" label="src/objectives/objective.rs">

```rust
    pub enum Objective {
    // An example of additional objectives (CenterOfMassMatchingObjective)
    CenterOfMassMatch(CenterOfMassMatchObjective),
    //------------------------------------------
    PositionMatch(PositionMatchObjective),
    OrientationMatch(OrientationMatchObjective),
    PositionLiveliness(PositionLivelinessObjective),
    ...
    }

    impl Objective {
        pub fn get_type(&self) -> String {
        // Returns a string value for each variant. Useful in debugging.
            match self {
                // An example of additional objectives (CenterOfMassMatchingObjective)
                Self::CenterOfMassMatch(_obj)=> return String::from("CenterOfMassObjective"),
                //-------------------------------------------------------------------------
                Self::PositionMatch(_obj) => return String::from("PositionMatchObjective"),
                Self::OrientationMatch(_obj) => return String::from("OrientationnMatchObjective"),
                ...
                }
        }
        pub fn call(&self,v: &Vars,state: &State) -> f64 {
            // A switch that passes along the `call` method to the inner objective.
            match self {
                // An example of additional objectives (CenterOfMassMatchingObjective)
                Self::CenterOfMassMatch(obj) => obj.call(v,state),
                //-----------------------------------------------
                Self::PositionMatch(obj) => obj.call(v,state),
                Self::OrientationMatch(obj) => obj.call(v,state),
                ...
                }
        }
        pub fn update(&mut self, time: f64) {
        // For time-sensitive objectives, include them here.
            match self {
                 // An example of additional objectives (CenterOfMassMatchingObjective)
                Self::CenterOfMassMatch(obj) => obj.update(time),
                //----------------------------------------------
                Self::PositionLiveliness(obj) => obj.update(time),
                ...
                }
            }
        pub fn set_weight(&mut self, weight: f64) {
        // Set the weight for the inner objective
            match self {
                 // An example of additional objectives (CenterOfMassMatchingObjective)
                Self::CenterOfMassMatch(obj) => obj.set_weight(weight),
                //----------------------------------------------------
                Self::PositionMatch(obj) => obj.set_weight(weight),
                Self::OrientationMatch(obj) => obj.set_weight(weight),
                Self::PositionLiveliness(obj) => obj.set_weight(weight),
                ...
                }
            }
        pub fn get_goal(&self) -> Option<Goal> {
        // get the goal for the inner objective. Useful for debugging.
            match self {
                // An example of additional objectives (CenterOfMassMatchingObjective)
                Self::CenterOfMassMatch(obj) => return Some(Goal::Translation(Translation3::from(obj.goal))),
                //------------------------------------------------------------------------------------------
                Self::PositionMatch(obj) => return Some(Goal::Translation(Translation3::from(obj.goal))),
                Self::OrientationMatch(obj) => return Some(Goal::Rotation(obj.goal)),
                ...
                }
            }
        pub fn set_goal(&mut self, goal: &Goal) {
            // Set the goal for the inner objective. This matches based on Objective and Goal variant.
            match (goal,self) {
             // An example of additional objectives (CenterOfMassMatchingObjective)
            (Goal::Translation(translation_goal),Self::CenterOfMassMatch(obj)) => obj.set_goal(translation_goal.vector),
            //---------------------------------------------------------------------------------------------------------
            (Goal::Translation(translation_goal),Self::PositionMatch(obj)) => obj.set_goal(translation_goal.vector),
            (Goal::Translation(translation_goal),Self::PositionMirroring(obj)) => obj.set_goal(translation_goal.vector),
            ...
            }
        }

        #[cfg(feature = "pybindings")]
        impl IntoPy<PyObject> for Objective {
        fn into_py(self, py: Python) -> PyObject {
            match self {
                // An example of additional objectives (CenterOfMassMatchingObjective)
                Self::CenterOfMassMatch(obj) => obj.into_py(py),
                //--------------------------------------------
			    Self::PositionMatch(obj) => obj.into_py(py),
			    Self::OrientationMatch(obj) => obj.into_py(py),
                ...
                }
            }
         }
    }


```

  </TabItem>

  <TabItem value="matching.rs" label="src/objectives/core/matching.rs">

```rust
// An example of additional objectives (CenterOfMassMatchingObjective)
#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct CenterOfMassMatchObjective {
    pub name: String,
    pub weight: f64,
    // Goal Value
    #[serde(skip)]
    pub goal: Vector3<f64>,
}

impl CenterOfMassMatchObjective {
    pub fn new(name: String, weight: f64) -> Self {
        Self {
            name,
            weight,
            goal: vector![0.0, 0.0, 0.0],
        }
    }
}

impl Callable<Vector3<f64>> for CenterOfMassMatchObjective {
    fn call(&self, _v: &Vars, state: &State) -> f64 {

        let x_val = ((state.center_of_mass- self.goal).norm()).abs();
        // println!("matching value is {:?}", groove_loss(x_val, 0., 2, 0.1, 10.0, 2));
        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2);
    }

    fn set_goal(&mut self, goal: Vector3<f64>) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl CenterOfMassMatchObjective {
    #[new]
    pub fn from_python(name:String,weight:f64) -> Self {
        CenterOfMassObjective::new(name,weight)
    }

    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }

}

//---------------------------------------------------------
```

  </TabItem>

</Tabs>
