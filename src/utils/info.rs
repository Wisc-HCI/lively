use pyo3::prelude::*;

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
    fn new(joint: String, multiplier: f64, offset: f64) -> Self {
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
    fn new(name: String, joint_type: String, lower_bound: f64, upper_bound: f64, max_velocity: f64, axis: [f64; 3], mimic: Option<MimicInfo>) -> Self {
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
    fn new(name: String) -> Self {
        Self { name }
    }
}