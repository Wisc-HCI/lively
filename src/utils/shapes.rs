use pyo3::prelude::*;
use crate::geometry::*;
use nalgebra::geometry::{Isometry3};

#[pyclass]
#[derive(Clone,Debug,PartialEq)]
pub struct BoxObject {
    #[pyo3(get,set)]
    frame: String,
    #[pyo3(get,set)]
    x: f64,
    #[pyo3(get,set)]
    y: f64,
    #[pyo3(get,set)]
    z: f64,
    // Non-pyo3 attributes
    local_transform: Isometry3<f64>
    // handle: Whatever Parry3D/Rapier handle is needed
}

#[pymethods]
impl BoxObject {
    #[new]
    fn from_python(py: Python, frame:String, x: f64, y: f64, z: f64, local_transform: Transform) -> Self {
        return Self { frame, x, y, z, local_transform: local_transform.get_isometry(py) }
    }
    #[getter]
    fn get_local_transform(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.local_transform.translation })?,
            rotation: Py::new(py, Rotation { value: self.local_transform.rotation })?
        })
    }
    #[setter]
    fn set_local_transform(&mut self, py: Python, local_transform: Transform) -> PyResult<()> {
        self.local_transform = local_transform.get_isometry(py);
        Ok(())
    }
}

impl BoxObject {
    pub fn new(frame:String, x: f64, y: f64, z: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {frame, x, y, z, local_transform}
    }

    // pub fn set_frame_transform(&mut self, frame_transform: Isometry3) {
    //     self.world_transform = frame_transform * self.local_transform;
    //     self.bounding_shape = self.collision_shape.bounding_aabb(self.world_transform)
    // }
}

#[pyclass]
#[derive(Clone,Debug,PartialEq)]
pub struct CylinderObject {
    #[pyo3(get,set)]
    frame: String,
    #[pyo3(get,set)]
    length: f64,
    #[pyo3(get,set)]
    radius: f64,
    // Non-pyo3 attributes
    local_transform: Isometry3<f64>,
    // handle: Whatever Parry3D/Rapier handle is needed 
}

#[pymethods]
impl CylinderObject {
    #[new]
    fn from_python(py: Python, frame:String, length: f64, radius: f64, local_transform: Transform) -> Self {
        return Self { frame, length, radius, local_transform: local_transform.get_isometry(py) } 
    }
    #[getter]
    fn get_local_transform(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.local_transform.translation })?,
            rotation: Py::new(py, Rotation { value: self.local_transform.rotation })?
        })
    }
    #[setter]
    fn set_local_transform(&mut self, py: Python, local_transform: Transform) -> PyResult<()> {
        self.local_transform = local_transform.get_isometry(py);
        Ok(())
    }
}

impl CylinderObject {
    pub fn new(frame:String, length: f64, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {frame, length, radius, local_transform}
    }
}

#[pyclass]
#[derive(Clone,Debug,PartialEq)]
pub struct CapsuleObject {
    #[pyo3(get,set)]
    frame: String,
    #[pyo3(get,set)]
    length: f64,
    #[pyo3(get,set)]
    radius: f64,
    // Non-pyo3 attributes
    local_transform: Isometry3<f64>,
    // handle: Whatever Parry3D/Rapier handle is needed 
}

#[pymethods]
impl CapsuleObject {
    #[new]
    fn from_python(py: Python, frame:String, length: f64, radius: f64, local_transform: Transform) -> Self {
        return Self {frame, length, radius, local_transform: local_transform.get_isometry(py)}
    }
    #[getter]
    fn get_local_transform(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.local_transform.translation })?,
            rotation: Py::new(py, Rotation { value: self.local_transform.rotation })?
        })
    }
    #[setter]
    fn set_local_transform(&mut self, py: Python, local_transform: Transform) -> PyResult<()> {
        self.local_transform = local_transform.get_isometry(py);
        Ok(())
    }
}

impl CapsuleObject {
    pub fn new(frame:String, length: f64, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {frame, length, radius, local_transform}
    }
}

#[pyclass]
#[derive(Clone,Debug,PartialEq)]
pub struct SphereObject {
    #[pyo3(get,set)]
    frame: String,
    #[pyo3(get,set)]
    radius: f64,
    // Non-pyo3 attributes
    local_transform: Isometry3<f64>
    // handle: Whatever Parry3D/Rapier handle is needed 
}

#[pymethods]
impl SphereObject {
    #[new]
    fn from_python(py: Python, frame:String, radius: f64, local_transform: Transform) -> Self {
        Self { frame, radius, local_transform: local_transform.get_isometry(py) } 
    }
    #[getter]
    fn get_local_transform(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.local_transform.translation })?,
            rotation: Py::new(py, Rotation { value: self.local_transform.rotation })?
        })
    }
    #[setter]
    fn set_local_transform(&mut self, py: Python, local_transform: Transform) -> PyResult<()> {
        self.local_transform = local_transform.get_isometry(py);
        Ok(())
    }
}

impl SphereObject {
    pub fn new(frame:String, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {frame, radius, local_transform}
    }
}

#[derive(Clone,Debug,PartialEq,FromPyObject)]
pub enum CollisionObject {
    Box(BoxObject),
    Cylinder(CylinderObject),
    Sphere(SphereObject),
    Capsule(CapsuleObject)
}

impl IntoPy<PyObject> for CollisionObject {
    fn into_py(self, py: Python) -> PyObject {
        match self {
            Self::Box(obj) => obj.into_py(py),
            Self::Cylinder(obj) => obj.into_py(py),
            Self::Sphere(obj) => obj.into_py(py),
            Self::Capsule(obj) => obj.into_py(py)
        }
    }
}