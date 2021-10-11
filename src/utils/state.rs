use pyo3::prelude::*;
use std::collections::HashMap;
use nalgebra::geometry::{Isometry3};
use crate::geometry::*;

/*
A read-only struct that provides information about the origin, jointstate, and frames of a robot.
*/

#[pyclass]
#[derive(Clone,Debug)]
pub struct State {
    pub origin: Isometry3<f64>,
    pub joints: HashMap<String,f64>,
    pub frames: HashMap<String,Isometry3<f64>>,
    //pub collision: Option<Vec<CollisionInfo>>
}

#[pymethods]
impl State {
    #[new]
    pub fn from_python(py: Python, origin: Transform, joints: HashMap<String,f64>, frames: HashMap<String,Transform>) -> Self {
        let mut iso_frames: HashMap<String,Isometry3<f64>> = HashMap::new();
        for (key, value) in frames.iter() {
            iso_frames.insert(key.to_string(),value.get_isometry(py));
        }
        Self { origin: origin.get_isometry(py), joints, frames: iso_frames }
    }

    #[getter]
    pub fn get_origin(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.origin.translation })?,
            rotation: Py::new(py, Rotation { value: self.origin.rotation })?
        })
    }

    #[getter]
    pub fn get_joints(&self) -> PyResult<HashMap<String,f64>> {
        Ok(self.joints.clone())
    }

    #[getter]
    pub fn get_frames(&self, py: Python) -> PyResult<HashMap<String,Transform>> {
        let mut transform_frames: HashMap<String,Transform> = HashMap::new();
        for (key, iso) in self.frames.iter() {
            transform_frames.insert(key.to_string(), Transform {
                translation: Py::new(py, Translation { value: iso.translation })?,
                rotation: Py::new(py, Rotation { value: iso.rotation })?
            });
        }
        Ok(transform_frames)
    }
}


impl State {
    pub fn new(origin: Isometry3<f64>, joints: HashMap<String,f64>, frames: HashMap<String,Isometry3<f64>>) -> Self {
        Self { origin, joints, frames }
    }

    pub fn get_link_transform(&self, link: &String) {
        return self.frames.get(link).unwrap_or(Isometry3::identity())
    }

    pub fn get_joint_position(&self, joint: &String) {
        return self.joints.get(joint).unwrap_or(0.0)
    }
}