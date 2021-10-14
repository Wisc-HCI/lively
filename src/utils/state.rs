use pyo3::prelude::*;
use std::collections::HashMap;
use nalgebra::geometry::{Isometry3};
use crate::utils::geometry::*;
use crate::utils::info::*;

/*
A read-only struct that provides information about the origin, jointstate, and frames of a robot.
*/

#[pyclass]
#[derive(Clone,Debug)]
pub struct State {
    pub origin: Isometry3<f64>,
    pub joints: HashMap<String,f64>,
    pub frames: HashMap<String,Isometry3<f64>>,
    pub proximity: Vec<ProximityInfo>,
    default_joint_position: f64,
    default_frame_transform: Isometry3<f64>
}

#[pymethods]
impl State {
    #[new]
    pub fn from_python(py: Python, origin: Transform, joints: HashMap<String,f64>, frames: Option<HashMap<String,Transform>>, proximity: Option<Vec<ProximityInfo>>) -> Self {
        let mut iso_frames: HashMap<String,Isometry3<f64>> = HashMap::new();
        let mut proximity_vec: Vec<ProximityInfo> = vec![]; 
        match frames {
            Some(frame_data) => {
                for (key, value) in frame_data.iter() {
                    iso_frames.insert(key.to_string(),value.get_isometry(py));
                }
            },
            None => {}
        }
        match proximity {
            Some(proximity_data) => {
                proximity_vec = proximity_data
            },
            None => {}
        }
        Self { 
            origin: origin.get_isometry(py), 
            joints, frames: iso_frames, 
            proximity: proximity_vec,
            default_joint_position: 0.0,
            default_frame_transform: Isometry3::identity()
        }
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

    #[getter]
    pub fn get_proximity(&self) -> PyResult<Vec<ProximityInfo>> {
        Ok(self.proximity.clone())
    }
}


impl State {
    pub fn new(origin: Isometry3<f64>, joints: HashMap<String,f64>, frames: HashMap<String,Isometry3<f64>>, proximity: Vec<ProximityInfo>) -> Self {
        Self { origin, joints, 
            frames, proximity,
            default_joint_position: 0.0,
            default_frame_transform: Isometry3::identity() }
    }

    pub fn get_link_transform(&self, link: &String) -> Isometry3<f64> {
        return *self.frames.get(link).unwrap_or(&self.default_frame_transform)
    }

    pub fn get_joint_position(&self, joint: &String) -> f64 {
        return *self.joints.get(joint).unwrap_or(&self.default_joint_position)
    }
}