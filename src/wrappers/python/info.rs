use pyo3::prelude::*;
use crate::utils::info::{MimicInfo,LinkInfo,JointInfo,ProximityInfo};

#[pyclass(name="MimicInfo")]
#[derive(Clone,Debug)]
pub struct PyMimicInfo(MimicInfo);

#[pyclass(name="LinkInfo")]
#[derive(Clone,Debug)]
pub struct PyLinkInfo(LinkInfo);

#[pyclass(name="JointInfo")]
#[derive(Clone,Debug)]
pub struct PyJointInfo(JointInfo);

#[pyclass(name="ProximityInfo")]
#[derive(Clone,Debug)]
pub struct PyProximityInfo(ProximityInfo);

impl From<MimicInfo> for PyMimicInfo {
    fn from(mimicinfo:MimicInfo) -> PyMimicInfo {
        PyMimicInfo(mimicinfo)
    }
}

impl From<LinkInfo> for PyLinkInfo {
    fn from(linkinfo:LinkInfo) -> PyLinkInfo {
        PyLinkInfo(linkinfo)
    }
}

impl From<JointInfo> for PyJointInfo {
    fn from(jointinfo:JointInfo) -> PyJointInfo {
        PyJointInfo(jointinfo)
    }
}

impl From<ProximityInfo> for PyProximityInfo {
    fn from(proxinfo:ProximityInfo) -> PyProximityInfo {
        PyProximityInfo(proxinfo)
    }
}

impl From<PyProximityInfo> for ProximityInfo {
    fn from(pyproxinfo: PyProximityInfo) -> ProximityInfo {
        pyproxinfo.0
    }
}

#[pymethods]
impl PyMimicInfo {
    #[getter]
    pub fn get_joint(&self) -> PyResult<String> {
        Ok(self.0.joint.clone())
    }
    #[getter]
    pub fn get_multiplier(&self) -> PyResult<f64> {
        Ok(self.0.multiplier.clone())
    }
    #[getter]
    pub fn get_offset(&self) -> PyResult<f64> {
        Ok(self.0.offset.clone())
    }
}

#[pymethods]
impl PyJointInfo {
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }
    #[getter]
    pub fn get_joint_type(&self) -> PyResult<String> {
        Ok(self.0.joint_type.clone())
    }
    #[getter]
    pub fn get_lower_bound(&self) -> PyResult<f64> {
        Ok(self.0.lower_bound.clone())
    }
    #[getter]
    pub fn get_upper_bound(&self) -> PyResult<f64> {
        Ok(self.0.upper_bound.clone())
    }
    #[getter]
    pub fn get_max_velocity(&self) -> PyResult<f64> {
        Ok(self.0.max_velocity.clone())
    }
    #[getter]
    pub fn get_axis(&self) -> PyResult<[f64; 3]> {
        Ok(self.0.axis.clone())
    }
    #[getter]
    pub fn get_mimic(&self) -> PyResult<Option<PyMimicInfo>> {
        match &self.0.mimic {
            Some(mimic) => return Ok(Some(PyMimicInfo(mimic.clone()))),
            None => return Ok(None)
        }
    }
    fn __str__(&self) -> PyResult<String> {
        Ok(format!("<Joint (name: {}, type: {}, bounds: [{},{}], max_vel: {}, axis: [{},{},{}], mimic: {})>", 
                    self.0.name, self.0.joint_type, self.0.lower_bound, self.0.upper_bound, 
                    self.0.max_velocity, self.0.axis[0], self.0.axis[1], self.0.axis[2], self.0.mimic.is_some()))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("<Joint (name: {}, type: {}, bounds: [{},{}], max_vel: {}, axis: [{},{},{}], mimic: {})>", 
                    self.0.name, self.0.joint_type, self.0.lower_bound, self.0.upper_bound, 
                    self.0.max_velocity, self.0.axis[0], self.0.axis[1], self.0.axis[2], self.0.mimic.is_some()))
    }
}

#[pymethods]
impl PyLinkInfo {
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }
    #[getter]
    pub fn get_parent_joint(&self) -> PyResult<String> {
        Ok(self.0.parent_joint.clone())
    }
    fn __str__(&self) -> PyResult<String> {
        Ok(format!("<Link (name: {}, parent_joint: {})>", self.0.name, self.0.parent_joint))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("<Link (name: {}, parent_joint: {})>", self.0.name, self.0.parent_joint))
    }
}

#[pymethods]
impl PyProximityInfo {
    #[getter]
    pub fn get_frame1(&self) -> PyResult<String> {
        Ok(self.0.frame1.clone())
    }
    #[getter]
    pub fn get_frame2(&self) -> PyResult<String> {
        Ok(self.0.frame2.clone())
    }
    #[getter]
    pub fn get_distance(&self) -> PyResult<Option<f64>> {
        Ok(self.0.distance.clone())
    }
    fn __str__(&self) -> PyResult<String> {
        Ok(format!("<Proximity (frame1: {}, frame2: {}, distance: {})>", self.0.frame1, self.0.frame2, self.0.distance.unwrap_or(100.0)))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("<Proximity (frame1: {}, frame2: {}, distance: {})>", self.0.frame1, self.0.frame2, self.0.distance.unwrap_or(100.0)))
    }
}