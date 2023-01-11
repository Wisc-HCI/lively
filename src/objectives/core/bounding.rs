use crate::objectives::objective::{groove_loss, Callable};
use crate::utils::info::{Ellipse, RotationRange, ScalarRange};
use crate::utils::state::State;
use crate::utils::vars::Vars;
use serde::{Deserialize, Serialize};
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct PositionBoundingObjective {
    // Bounds the position within a region according to the input provided in goals.
    pub name: String,
    pub weight: f64,
    pub link: String,
    // Goal Value
    #[serde(skip)]
    pub goal: Ellipse,
}

impl PositionBoundingObjective {
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {
            name,
            weight,
            link,
            goal: Ellipse::default(),
        }
    }
}

impl Callable<Ellipse> for PositionBoundingObjective {
    fn call(&self, _v: &Vars, state: &State) -> f64 {
        let position = state.get_link_transform(&self.link).translation.vector;
        let pos = self.goal.transform.inverse_transform_point(&position.into());
        let dist = pos[0].powi(2) / self.goal.size[0].powi(2)
            + pos[1].powi(2) / self.goal.size[1].powi(2)
            + pos[2].powi(2) / self.goal.size[2].powi(2);
        let cost = 1.0 / (1.0 + (-2.0 * dist + 4.0).exp()) + dist / 10.0;
        return self.weight * cost; //groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    fn set_goal(&mut self, goal: Ellipse) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PositionBoundingObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,link:String) -> Self {
        PositionBoundingObjective::new(name,weight,link)
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
pub struct OrientationBoundingObjective {
    // Bounds the orientation within a region on the unit sphere according to the input provided in goals.
    pub name: String,
    pub weight: f64,
    pub link: String,
    // Goal Value
    #[serde(skip)]
    pub goal: RotationRange,
}

impl OrientationBoundingObjective {
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {
            name,
            weight,
            link,
            goal: RotationRange::default(),
        }
    }
}

impl Callable<RotationRange> for OrientationBoundingObjective {
    fn call(&self, _v: &Vars, state: &State) -> f64 {
        let orientation = state.get_link_transform(&self.link).rotation;
        let angle_dist = orientation.angle_to(&self.goal.rotation);
        let x_val = (angle_dist - self.goal.delta).max(0.0);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2);
    }

    fn set_goal(&mut self, goal: RotationRange) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl OrientationBoundingObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,link:String) -> Self {
        OrientationBoundingObjective::new(name,weight,link)
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
pub struct JointBoundingObjective {
    // Bounds the position within a region according to the input provided in goals.
    pub name: String,
    pub weight: f64,
    pub joint: String,
    // Goal Value
    #[serde(skip)]
    pub goal: ScalarRange,
}

impl JointBoundingObjective {
    pub fn new(name: String, weight: f64, joint: String) -> Self {
        Self {
            name,
            weight,
            joint,
            goal: ScalarRange::default(),
        }
    }
}

impl Callable<ScalarRange> for JointBoundingObjective {
    fn call(&self, _v: &Vars, state: &State) -> f64 {
        let joint_value = state.get_joint_position(&self.joint);
        let penalty_cutoff: f64 = 0.9;
        let a = 0.05 / (penalty_cutoff.powi(50));
        if self.goal.delta <= 0.0 {
            let x_val: f64 = a * (joint_value - self.goal.value).abs().powi(50);
            return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2);
        } else {
            let l: f64 = self.goal.value - self.goal.delta;
            let u: f64 = self.goal.value + self.goal.delta;
            let r: f64 = (joint_value - l) / (u - l);
            let n: f64 = (2.0 * (r - 0.5).abs()).powi(50);
            let x_val: f64 = a * n.powi(50);
            return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2);
        }
    }

    fn set_goal(&mut self, goal: ScalarRange) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl JointBoundingObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,joint:String) -> Self {
        JointBoundingObjective::new(name,weight,joint)
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