use pyo3::prelude::*;
use pyo3::PyObjectProtocol;
use urdf_rs::{Mimic};
// use std::fmt::Display;

#[pyclass]
#[derive(Clone,Debug)]
pub struct MimicInfo {
    #[pyo3(get)]
    pub joint: String,
    #[pyo3(get)]
    pub multiplier: f64,
    #[pyo3(get)]
    pub offset: f64
}

#[pymethods]
impl MimicInfo {
    #[new]
    pub fn new(joint: String, multiplier: f64, offset: f64) -> Self {
        Self { joint, multiplier, offset }
    }
}

impl From<Mimic> for MimicInfo {
    fn from(mimic: Mimic) -> Self {
        let joint: String = mimic.joint;
        let multiplier: f64;
        let offset: f64;
        match mimic.multiplier {
            Some(value) => multiplier = value,
            None => multiplier = 1.0
        };
        match mimic.offset {
            Some(value) => offset = value,
            None => offset = 0.0
        }
        MimicInfo { joint, multiplier, offset }
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct JointInfo {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub joint_type: String,
    #[pyo3(get)]
    pub lower_bound: f64,
    #[pyo3(get)]
    pub upper_bound: f64,
    #[pyo3(get)]
    pub max_velocity: f64,
    #[pyo3(get)]
    pub axis: [f64; 3],
    #[pyo3(get)]
    pub mimic: Option<MimicInfo>
}

#[pymethods]
impl JointInfo {
    #[new]
    pub fn new(name: String, joint_type: String, lower_bound: f64, upper_bound: f64, max_velocity: f64, axis: [f64; 3], mimic: Option<MimicInfo>) -> Self {
        Self { name, joint_type, lower_bound, upper_bound, max_velocity, axis, mimic }
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct LinkInfo {
    #[pyo3(get)]
    pub name: String
}

#[pymethods]
impl LinkInfo {
    #[new]
    pub fn new(name: String) -> Self {
        Self { name }
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct ProximityInfo {
    #[pyo3(get)]
    pub frame1: String,
    #[pyo3(get)]
    pub frame2: String,
    #[pyo3(get)]
    pub distance: Option<f64>,
    #[pyo3(get)]
    pub scored: bool
}

#[pymethods]
impl ProximityInfo {
    #[new]
    pub fn new(frame1: String, frame2: String, distance: Option<f64>, scored: Option<bool>) -> Self {
        Self { frame1, frame2, distance, scored: scored.unwrap_or(true) }
    }
}

#[pyproto]
impl PyObjectProtocol for JointInfo {
    fn __str__(&self) -> PyResult<String> {
        Ok(format!("<Joint (name: {}, type: {}, bounds: [{},{}], max_vel: {}, axis: [{},{},{}], mimic: {})>", 
                    self.name, self.joint_type, self.lower_bound, self.upper_bound, 
                    self.max_velocity, self.axis[0], self.axis[1], self.axis[2], self.mimic.is_some()))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("<Joint (name: {}, type: {}, bounds: [{},{}], max_vel: {}, axis: [{},{},{}], mimic: {})>", 
                    self.name, self.joint_type, self.lower_bound, self.upper_bound, 
                    self.max_velocity, self.axis[0], self.axis[1], self.axis[2], self.mimic.is_some()))
    }
}

#[pyproto]
impl PyObjectProtocol for LinkInfo {
    fn __str__(&self) -> PyResult<String> {
        Ok(format!("<Link (name: {})>", self.name))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("<Link (name: {})>", self.name))
    }
}

#[pyproto]
impl PyObjectProtocol for ProximityInfo {
    fn __str__(&self) -> PyResult<String> {
        Ok(format!("<Proximity (frame1: {}, frame2: {}, distance: {}, scored: {})>", self.frame1, self.frame2, self.distance.unwrap_or(100.0), self.scored))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("<Proximity (frame1: {}, frame2: {}, distance: {}, scored: {})>", self.frame1, self.frame2, self.distance.unwrap_or(100.0), self.scored))
    }
}