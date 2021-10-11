use pyo3::prelude::*;
use crate::utils::goals::*;
use crate::utils::vars::Vars;
use crate::objectives::objective::{groove_loss};
use nalgebra::geometry::{Isometry3, Point3, Translation3, UnitQuaternion};
use nalgebra::{Vector3};

pub struct PositionBoundingObjective {
    // Bounds the position within a region according to the input provided in goals.
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link: String
    // Goal Value
    pub goal: (Translation3,UnitQuaternion,Vector3)
}
#[pymethods]
impl PositionBoundingObjective {
    #[new]
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {name, weight, link, goal: (Translation3::new(0.0,0.0,0.0),UnitQuaternion::identity(),vector![0.0,0.0,0.0])}
    }
}
impl PositionBoundingObjective {
    fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        const position = state.get_frame_transform(&self.link)).translation.vector;
        const transform = Isometry3::from_parts(self.goal.0, self.goal.1);
        const pos = transform.inverse_transform_vector(&position);
        const x_val = (pos[0].powi(1) / self.goal.2[0].powi(2)
                    + pos[1].powi(2) / self.goal.2[1].powi(2)
                    + pos[2].powi(2) / self.goal.2[2].powi(2))
                .powi(2)
        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct OrientationBoundingObjective {
    // Bounds the orientation within a region on the unit sphere according to the input provided in goals.
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link: String
    // Goal Value
    pub goal: (UnitQuaternion,f64)
}
#[pymethods]
impl OrientationBoundingObjective {
    #[new]
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {name, weight, link, goal: (UnitQuaternion::identity(),0.0)}
    }
}
impl OrientationBoundingObjective {
    fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        const orientation = state.get_frame_transform(&self.link)).rotation;
        const angle_dist = orientation.angle_to(goal.0);
        const x_val = (angle_dist-self.goal.1).max(0.0);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct JointBoundingObjective {
    // Bounds the position within a region according to the input provided in goals.
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link: String
    // Goal Value
    pub goal: (f64,f64)
}
#[pymethods]
impl JointBoundingObjective {
    #[new]
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self {name, weight, link, goal: (0.0,0.0)}
    }
}
impl JointBoundingObjective {
    fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        const position = state.get_frame_transform(&self.link)).translation.vector;
        const transform = Isometry3::from_parts(self.goal.0, self.goal.1);
        const pos = transform.inverse_transform_vector(&position);
        const x_val = (pos[0].powi(1) / self.goal.2[0].powi(2)
                    + pos[1].powi(2) / self.goal.2[1].powi(2)
                    + pos[2].powi(2) / self.goal.2[2].powi(2))
                .powi(2)
        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}
