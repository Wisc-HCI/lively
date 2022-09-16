#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "pybindings")]
use nalgebra::Isometry3;
#[cfg(feature = "pybindings")]
use crate::utils::info::{CollisionSettingInfo,TransformInfo,MimicInfo,LinkInfo,JointInfo,ProximityInfo,ShapeUpdate};
#[cfg(feature = "pybindings")]
use crate::utils::shapes::Shape;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::shapes::{PyShape};
#[cfg(feature = "pybindings")]
use crate::wrappers::python::geometry::{Translation,Rotation,Transform};

#[cfg(feature = "pybindings")]
#[pyclass(name="TransformInfo")]
#[derive(Clone,Debug)]
pub struct PyTransformInfo(TransformInfo);

#[cfg(feature = "pybindings")]
#[pyclass(name="CollisionSettingInfo")]
#[derive(Clone,Debug)]
pub struct PyCollisionSettingInfo(CollisionSettingInfo);

#[cfg(feature = "pybindings")]
#[pyclass(name="MimicInfo")]
#[derive(Clone,Debug)]
pub struct PyMimicInfo(MimicInfo);

#[cfg(feature = "pybindings")]
#[pyclass(name="LinkInfo")]
#[derive(Clone,Debug)]
pub struct PyLinkInfo(LinkInfo);

#[cfg(feature = "pybindings")]
#[pyclass(name="JointInfo")]
#[derive(Clone,Debug)]
pub struct PyJointInfo(JointInfo);

#[cfg(feature = "pybindings")]
#[pyclass(name="ProximityInfo")]
#[derive(Clone,Debug)]
pub struct PyProximityInfo(ProximityInfo);

#[cfg(feature = "pybindings")]
impl From<MimicInfo> for PyMimicInfo {
    fn from(mimicinfo:MimicInfo) -> PyMimicInfo {
        PyMimicInfo(mimicinfo)
    }
}

#[cfg(feature = "pybindings")]
impl From<LinkInfo> for PyLinkInfo {
    fn from(linkinfo:LinkInfo) -> PyLinkInfo {
        PyLinkInfo(linkinfo)
    }
}

#[cfg(feature = "pybindings")]
impl From<TransformInfo> for PyTransformInfo {
    fn from(transforminfo:TransformInfo) -> PyTransformInfo {
        PyTransformInfo(transforminfo)
    }
}

#[cfg(feature = "pybindings")]
impl From<CollisionSettingInfo> for PyCollisionSettingInfo {
    fn from(collisionsettinginfo:CollisionSettingInfo) -> PyCollisionSettingInfo {
        PyCollisionSettingInfo(collisionsettinginfo)
    }
}

#[cfg(feature = "pybindings")]
impl From<JointInfo> for PyJointInfo {
    fn from(jointinfo:JointInfo) -> PyJointInfo {
        PyJointInfo(jointinfo)
    }
}

#[cfg(feature = "pybindings")]
impl From<ProximityInfo> for PyProximityInfo {
    fn from(proxinfo:ProximityInfo) -> PyProximityInfo {
        PyProximityInfo(proxinfo)
    }
}

#[cfg(feature = "pybindings")]
impl From<PyProximityInfo> for ProximityInfo {
    fn from(pyproxinfo: PyProximityInfo) -> ProximityInfo {
        pyproxinfo.0
    }
}

#[cfg(feature = "pybindings")]
impl From<PyTransformInfo> for TransformInfo {
    fn from(pytransforminfo: PyTransformInfo) -> TransformInfo {
        pytransforminfo.0
    }
}

#[cfg(feature = "pybindings")]
impl From<PyCollisionSettingInfo> for CollisionSettingInfo {
    fn from(pycollisionsettinginfo: PyCollisionSettingInfo) -> CollisionSettingInfo {
        pycollisionsettinginfo.0
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyTransformInfo {
    #[getter]
    pub fn get_world(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform { translation: Py::new(py, Translation{value:self.0.world.translation})?, rotation: Py::new(py, Rotation{value:self.0.world.rotation})?})
    }
    #[getter]
    pub fn get_local(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform { translation: Py::new(py, Translation{value:self.0.local.translation})?, rotation: Py::new(py, Rotation{value:self.0.local.rotation})?})
    }
}

#[cfg(feature = "pybindings")]
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

#[cfg(feature = "pybindings")]
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
    #[getter]
    pub fn get_parent_link(&self) -> PyResult<String> {
        Ok(self.0.parent_link.clone())
    }
    #[getter]
    pub fn get_child_link(&self) -> PyResult<String> {
        Ok(self.0.parent_link.clone())
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

#[cfg(feature = "pybindings")]
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

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyProximityInfo {
    #[getter]
    pub fn get_shape1(&self) -> PyResult<String> {
        Ok(self.0.shape1.clone())
    }
    #[getter]
    pub fn get_shape2(&self) -> PyResult<String> {
        Ok(self.0.shape2.clone())
    }
    #[getter]
    pub fn get_distance(&self) -> PyResult<f64> {
        Ok(self.0.distance.clone())
    }
    #[getter]
    pub fn get_points(&self) -> PyResult<Option<([f64;3],[f64;3])>> {
        match self.0.points {
            Some((point1,point2)) => return Ok(Some(([point1.x,point1.y,point1.z],[point2.x,point2.y,point2.z]))),
            None => return Ok(None)
        }
    }
    #[getter]
    pub fn get_physical(&self) -> PyResult<bool> {
        Ok(self.0.physical.clone())
    }
    #[getter]
    pub fn get_loss(&self) -> PyResult<f64> {
        Ok(self.0.loss.clone())
    }

    #[getter]
    pub fn get_average_distance(&self) -> PyResult<Option<f64>> {
        Ok(self.0.average_distance)
    }

    fn __str__(&self) -> PyResult<String> {
        Ok(format!("<Proximity (shape1: {}, shape2: {}, distance: {})>", self.0.shape1, self.0.shape2, self.0.distance))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("<Proximity (shape1: {}, shape2: {}, distance: {})>", self.0.shape1, self.0.shape2, self.0.distance))
    }
}

#[cfg(feature = "pybindings")]
#[derive(Clone,Debug,FromPyObject)]
pub enum PyShapeUpdate {
    Add{
        id: String,
        shape: PyShape
    },
    Move{
        id: String,
        translation: Translation,
        rotation: Rotation
    },
    Delete(String)
}

// #[cfg(feature = "pybindings")]
// impl IntoPy<PyObject> for PyShapeUpdate {
//     fn into_py(self, py: Python) -> PyObject {
//         match self {
//             Self::Add{id,shape} => obj.into_py(py),
//             Self::Move{id,translation,rotation} => obj.into_py(py),
//             Self::Delete(obj) => obj.into_py(py)
//         }
//     }
// }

#[cfg(feature = "pybindings")]
impl From<PyShapeUpdate> for ShapeUpdate {
    fn from(pyshape:PyShapeUpdate) -> ShapeUpdate {
        match pyshape {
            PyShapeUpdate::Add{id,shape} => ShapeUpdate::Add{id,shape:Shape::from(shape)},
            PyShapeUpdate::Move{id,translation,rotation} => ShapeUpdate::Move{id,pose:Isometry3::from_parts(translation.value,rotation.value)},
            PyShapeUpdate::Delete(id) => ShapeUpdate::Delete(id)
        }
    }
}