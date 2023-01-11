use crate::objectives::objective::{groove_loss, Callable};
use crate::utils::state::State;
use crate::utils::vars::Vars;
use nalgebra::geometry::UnitQuaternion;
use nalgebra::{vector, Vector3};
use serde::{Deserialize, Serialize};
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct PositionMatchObjective {
    pub name: String,
    pub weight: f64,
    pub link: String,
    // Goal Value
    #[serde(skip)]
    pub goal: Vector3<f64>,
}

impl PositionMatchObjective {
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {
            name,
            weight,
            link,
            goal: vector![0.0, 0.0, 0.0],
        }
    }
}

impl Callable<Vector3<f64>> for PositionMatchObjective {
    fn call(&self, _v: &Vars, state: &State) -> f64 {
        // Get the link transform from frames
        let link_translation = state.get_link_transform(&self.link).translation.vector;

        let x_val = (link_translation - self.goal).norm();
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
impl PositionMatchObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,link:String) -> Self {
        PositionMatchObjective::new(name,weight,link)
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
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct OrientationMatchObjective {
    pub name: String,
    pub weight: f64,
    pub link: String,
    // Goal Value
    #[serde(skip)]
    pub goal: UnitQuaternion<f64>,
}

impl OrientationMatchObjective {
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {
            name,
            weight,
            link,
            goal: UnitQuaternion::identity(),
        }
    }
}

impl Callable<UnitQuaternion<f64>> for OrientationMatchObjective {
    fn call(&self, _v: &Vars, state: &State) -> f64 {
        // Get the link transform from frames
        let link_rotation = state.get_link_transform(&self.link).rotation;

        let x_val = link_rotation.angle_to(&self.goal);

        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2);
    }

    fn set_goal(&mut self, goal: UnitQuaternion<f64>) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl OrientationMatchObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,link:String) -> Self {
        OrientationMatchObjective::new(name,weight,link)
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
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct JointMatchObjective {
    // Sets a joint to a value given in scalar goal
    pub name: String,
    pub weight: f64,
    pub joint: String,
    // Goal Value
    #[serde(skip)]
    pub goal: f64,
}

impl JointMatchObjective {
    pub fn new(name: String, weight: f64, joint: String) -> Self {
        Self {
            name,
            weight,
            joint,
            goal: 0.0,
        }
    }
}

impl Callable<f64> for JointMatchObjective {
    fn call(&self, _v: &Vars, state: &State) -> f64 {
        let x_val = (self.goal - state.get_joint_position(&self.joint)).abs();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2);
    }

    fn set_goal(&mut self, goal: f64) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl JointMatchObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,joint:String) -> Self {
        JointMatchObjective::new(name,weight,joint)
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
    pub fn get_joint(&self) -> PyResult<String> {
        Ok(self.joint.clone())
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct DistanceMatchObjective {
    // Specify that the cartesian distance between two links is maintained
    pub name: String,
    pub weight: f64,
    pub link1: String,
    pub link2: String,
    // Goal Value
    #[serde(skip)]
    pub goal: f64,
}

impl DistanceMatchObjective {
    pub fn new(name: String, weight: f64, link1: String, link2: String) -> Self {
        Self {
            name,
            weight,
            link1,
            link2,
            goal: 0.0,
        }
    }
}

impl Callable<f64> for DistanceMatchObjective {
    fn call(&self, _v: &Vars, state: &State) -> f64 {
        let link1_translation = state.get_link_transform(&self.link1).translation.vector;
        let link2_translation = state.get_link_transform(&self.link2).translation.vector;
        let x_val = ((link1_translation - link2_translation).norm() - self.goal).abs();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_goal(&mut self, goal: f64) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl DistanceMatchObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,link1:String,link2:String) -> Self {
        DistanceMatchObjective::new(name,weight,link1,link2)
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
    pub fn get_link1(&self) -> PyResult<String> {
        Ok(self.link1.clone())
    }

    #[getter]
    pub fn get_link2(&self) -> PyResult<String> {
        Ok(self.link2.clone())
    }
}