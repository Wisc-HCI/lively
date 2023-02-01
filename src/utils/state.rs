use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use nalgebra::geometry::{Isometry3};
use nalgebra::Vector3;
use crate::utils::info::*;
#[cfg(feature = "pybindings")]
use crate::utils::pyutils::*;
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "pybindings")]
use nalgebra::vector;
/*
A read-only struct that provides information about the origin, jointstate, and frames of a robot.
*/

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug)]
#[serde(rename_all = "camelCase")]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct State {
    #[serde(skip_deserializing)]
    pub timestamp: f64,
    pub origin: Isometry3<f64>,
    pub joints: HashMap<String,f64>,
    #[serde(skip_deserializing)]
    pub frames: HashMap<String,TransformInfo>,
    #[serde(default)]
    pub proximity: Vec<ProximityInfo>,
    #[serde(skip_deserializing)]
    pub center_of_mass: Vector3<f64>,
    #[serde(skip)]
    default_joint_position: f64,
    #[serde(skip,default="TransformInfo::default")]
    default_frame_transform: TransformInfo
}

impl State {
    pub fn new(
        origin: Isometry3<f64>, 
        joints: HashMap<String,f64>, 
        frames: HashMap<String,TransformInfo>, 
        proximity: Vec<ProximityInfo>,
        center_of_mass: Vector3<f64>,
        timestamp: f64
    ) -> Self {
        Self { timestamp, origin, joints, 
            frames, proximity,
            center_of_mass,
            default_joint_position: 0.0,
            default_frame_transform: TransformInfo::default() }
    }

    pub fn get_link_transform(&self, link: &String) -> Isometry3<f64> {
        return self.frames.get(link).unwrap_or(&self.default_frame_transform).world
    }

    pub fn get_joint_position(&self, joint: &String) -> f64 {
        return *self.joints.get(joint).unwrap_or(&self.default_joint_position)
    }

    pub fn stepped(&self, time: f64) -> Self {
        let mut stepped = self.clone();
        stepped.timestamp -= time;
        return stepped;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl State {
    #[new]
    pub fn from_python(py: Python, origin: PyTransform, joints: HashMap<String,f64>) -> Self {
        let frames: HashMap<String,TransformInfo> = HashMap::new();
        let proximity: Vec<ProximityInfo> = vec![]; 
        let center_of_mass: Vector3<f64> = vector![0.0,0.0,0.0];
        Self::new(
            origin.get_isometry(py), 
            joints, frames, 
            proximity,
            center_of_mass, 0.0
        )
    }
    fn as_str(&self) -> String {
        format!("State: {{origin: {:?}, joints:{:?}, frames:{:?}, proximity:{:?}, center_of_mass:{:?}, timestamp:{:?}}}", 
                self.origin, self.joints, self.frames, self.proximity, self.center_of_mass, self.timestamp)
    }

    pub fn __str__(&self) -> PyResult<String> {
        Ok(self.as_str())
    }

    pub fn __repr__(&self) -> PyResult<String> {
        Ok(self.as_str())
    }

    #[getter(origin)]
    pub fn get_origin_python(&self, py: Python) -> PyResult<PyTransform> {
        Ok(PyTransform {
            translation: Py::new(py, PyTranslation { value: self.origin.translation })?,
            rotation: Py::new(py, PyRotation { value: self.origin.rotation })?
        })
    }

    #[getter(joints)]
    pub fn get_joints_python(&self) -> PyResult<HashMap<String,f64>> {
        Ok(self.joints.clone())
    }

    #[getter(frames)]
    pub fn get_frames_python(&self) -> PyResult<HashMap<String,TransformInfo>> {
        let mut transform_frames: HashMap<String,TransformInfo> = HashMap::new();
        for (key, tfi) in self.frames.iter() {
            transform_frames.insert(key.to_string(), TransformInfo::from(tfi.clone()));
        }
        Ok(transform_frames)
    }

    #[getter(proximity)]
    pub fn get_proximity_python(&self) -> PyResult<Vec<ProximityInfo>> {
        Ok(self.proximity.iter().map(|p| ProximityInfo::from(p.clone())).collect())
    }

    #[getter(center_of_mass)]
    pub fn get_center_of_mass_python(&self) -> PyResult<PyTranslation> {
        Ok(PyTranslation::from(self.center_of_mass))
    }
}