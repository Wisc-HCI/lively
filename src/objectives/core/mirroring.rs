use serde::{Serialize, Deserialize};
use crate::utils::vars::Vars;
use crate::utils::state::State;
use crate::objectives::objective::{groove_loss,Callable};
use nalgebra::geometry::{UnitQuaternion};
use nalgebra::{Vector3, vector};
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct PositionMirroringObjective {
    // Matches the position between two joints, with a difference according to the Vector3 provided in goals.
    pub name: String,
    pub weight: f64,
    pub link1: String,
    pub link2: String,
    // Goal Value
    #[serde(skip)]
    pub goal: Vector3<f64>
}

impl PositionMirroringObjective {

    pub fn new(name: String, weight: f64, link1: String, link2: String) -> Self {
        Self { name, weight, link1, link2, goal: vector![0.0,0.0,0.0]}
    }
}

impl Callable<Vector3<f64>> for PositionMirroringObjective {

    fn call(
        &self,
        _v: &Vars,
        state: &State
    ) -> f64 {
        let link1_translation = state.get_link_transform(&self.link1).translation.vector;
        let link2_translation = state.get_link_transform(&self.link2).translation.vector;
        let x_val = ((link1_translation - link2_translation) - self.goal).norm();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
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
impl PositionMirroringObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,link1:String,link2:String) -> Self {
        PositionMirroringObjective::new(name,weight,link1,link2)
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

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct OrientationMirroringObjective {
    // Matches the orientation between two joints, with a difference according to the Quaternion provided in goals.
    pub name: String,
    pub weight: f64,
    pub link1: String,
    pub link2: String,
    // Goal Value
    #[serde(skip)]
    pub goal: UnitQuaternion<f64>
}

impl OrientationMirroringObjective {

    pub fn new(name: String, weight: f64, link1: String, link2: String) -> Self {
        Self { name, weight, link1, link2, goal: UnitQuaternion::identity()}
    }
}

impl Callable<UnitQuaternion<f64>> for OrientationMirroringObjective {

    fn call(
        &self,
        _v: &Vars,
        state: &State
    ) -> f64 {
        let link1_rotation = state.get_link_transform(&self.link1).rotation;
        let link2_rotation = state.get_link_transform(&self.link2).rotation;
        let x_val = link1_rotation.rotation_to(&link2_rotation).angle_to(&self.goal);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
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
impl OrientationMirroringObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,link1:String,link2:String) -> Self {
        OrientationMirroringObjective::new(name,weight,link1,link2)
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

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct JointMirroringObjective {
    // Match joint values according to the difference specified in goals
    pub name: String,
    pub weight: f64,
    pub joint1: String,
    pub joint2: String,
    // Goal Value
    #[serde(skip)]
    pub goal: f64
}

impl JointMirroringObjective {

    pub fn new(name: String, weight: f64, joint1: String, joint2: String) -> Self {
        Self { name, weight, joint1, joint2, goal: 0.0}
    }
}

impl Callable<f64> for JointMirroringObjective {

    fn call(
        &self,
        _v: &Vars,
        state: &State
    ) -> f64 {
        let joint1_position = state.get_joint_position(&self.joint1);
        let joint2_position = state.get_joint_position(&self.joint2);
        let x_val = ((joint1_position+self.goal)-joint2_position).abs();
        return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
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
impl JointMirroringObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,joint1:String,joint2:String) -> Self {
        JointMirroringObjective::new(name,weight,joint1,joint2)
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
    pub fn get_joint1(&self) -> PyResult<String> {
        Ok(self.joint1.clone())
    }

    #[getter]
    pub fn get_joint2(&self) -> PyResult<String> {
        Ok(self.joint2.clone())
    }
}