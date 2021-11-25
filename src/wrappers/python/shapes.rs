#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "pybindings")]
use nalgebra::vector;
#[cfg(feature = "pybindings")]
use crate::utils::shapes::*;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::geometry::*;

#[cfg(feature = "pybindings")]
#[pyclass(name="BoxShape")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyBoxShape(BoxShape);

#[cfg(feature = "pybindings")]
#[pyclass(name="SphereShape")]
#[derive(Clone,Debug,PartialEq)]
pub struct PySphereShape(SphereShape);

#[cfg(feature = "pybindings")]
#[pyclass(name="CylinderShape")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyCylinderShape(CylinderShape);

#[cfg(feature = "pybindings")]
#[pyclass(name="CapsuleShape")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyCapsuleShape(CapsuleShape);

#[cfg(feature = "pybindings")]
#[pyclass(name="HullShape")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyHullShape(HullShape);

#[cfg(feature = "pybindings")]
#[pyclass(name="MeshShape")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyMeshShape(MeshShape);

#[cfg(feature = "pybindings")]
#[derive(Clone,Debug,FromPyObject)]
pub enum PyShape {
    Box(PyBoxShape),
    Cylinder(PyCylinderShape),
    Sphere(PySphereShape),
    Capsule(PyCapsuleShape)
}

#[cfg(feature = "pybindings")]
impl IntoPy<PyObject> for PyShape {
    fn into_py(self, py: Python) -> PyObject {
        match self {
            Self::Box(obj) => obj.into_py(py),
            Self::Cylinder(obj) => obj.into_py(py),
            Self::Sphere(obj) => obj.into_py(py),
            Self::Capsule(obj) => obj.into_py(py)
        }
    }
}

#[cfg(feature = "pybindings")]
impl From<PyShape> for Shape {
    fn from(pyshape:PyShape) -> Shape {
        match pyshape {
            PyShape::Box(shape) => Shape::Box(shape.0),
            PyShape::Cylinder(shape) => Shape::Cylinder(shape.0),
            PyShape::Capsule(shape) => Shape::Capsule(shape.0),
            PyShape::Sphere(shape) => Shape::Sphere(shape.0),
        }
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyBoxShape {
    #[new]
    fn new(py: Python, name: String, frame:String, physical:bool, x: f64, y: f64, z: f64, local_transform: Transform) -> Self {
        return Self(BoxShape{ name, frame, physical, x, y, z, local_transform: local_transform.get_isometry(py) })
    }
    #[getter]
    fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }
    #[getter]
    fn get_frame(&self) -> PyResult<String> {
        Ok(self.0.frame.clone())
    }
    #[getter]
    fn get_physical(&self) -> PyResult<bool> {
        Ok(self.0.physical.clone())
    }
    #[getter]
    fn get_x(&self) -> PyResult<f64> {
        Ok(self.0.x.clone())
    }
    #[getter]
    fn get_y(&self) -> PyResult<f64> {
        Ok(self.0.y.clone())
    }
    #[getter]
    fn get_z(&self) -> PyResult<f64> {
        Ok(self.0.z.clone())
    }
    #[getter]
    fn get_local_transform(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.0.local_transform.translation })?,
            rotation: Py::new(py, Rotation { value: self.0.local_transform.rotation })?
        })
    }
    #[setter]
    fn set_name(&mut self,value:String) -> PyResult<()> {
        self.0.name = value;
        Ok(())
    }
    #[setter]
    fn set_frame(&mut self,value:String) -> PyResult<()> {
        self.0.frame = value;
        Ok(())
    }
    #[setter]
    fn set_physical(&mut self,value:bool) -> PyResult<()> {
        self.0.physical = value;
        Ok(())
    }
    #[setter]
    fn set_x(&mut self,value:f64) -> PyResult<()> {
        self.0.x = value;
        Ok(())
    }
    #[setter]
    fn set_y(&mut self,value:f64) -> PyResult<()> {
        self.0.y = value;
        Ok(())
    }
    #[setter]
    fn set_z(&mut self,value:f64) -> PyResult<()> {
        self.0.z = value;
        Ok(())
    }
    #[setter]
    fn set_local_transform(&mut self, py: Python, local_transform: Transform) -> PyResult<()> {
        self.0.local_transform = local_transform.get_isometry(py);
        Ok(())
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PySphereShape {
    #[new]
    fn new(py: Python, name: String, frame:String, physical:bool, radius: f64, local_transform: Transform) -> Self {
        Self(SphereShape{name, frame, physical, radius, local_transform: local_transform.get_isometry(py)})
    }
    #[getter]
    fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }
    #[getter]
    fn get_frame(&self) -> PyResult<String> {
        Ok(self.0.frame.clone())
    }
    #[getter]
    fn get_physical(&self) -> PyResult<bool> {
        Ok(self.0.physical.clone())
    }
    #[getter]
    fn get_radius(&self) -> PyResult<f64> {
        Ok(self.0.radius.clone())
    }
    #[getter]
    fn get_local_transform(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.0.local_transform.translation })?,
            rotation: Py::new(py, Rotation { value: self.0.local_transform.rotation })?
        })
    }
    #[setter]
    fn set_name(&mut self,value:String) -> PyResult<()> {
        self.0.name = value;
        Ok(())
    }
    #[setter]
    fn set_frame(&mut self,value:String) -> PyResult<()> {
        self.0.frame = value;
        Ok(())
    }
    #[setter]
    fn set_physical(&mut self,value:bool) -> PyResult<()> {
        self.0.physical = value;
        Ok(())
    }
    #[setter]
    fn set_radius(&mut self,value:f64) -> PyResult<()> {
        self.0.radius = value;
        Ok(())
    }
    #[setter]
    fn set_local_transform(&mut self, py: Python, local_transform: Transform) -> PyResult<()> {
        self.0.local_transform = local_transform.get_isometry(py);
        Ok(())
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyCylinderShape {
    #[new]
    fn new(py: Python, name: String, frame:String, physical:bool, length: f64, radius: f64, local_transform: Transform) -> Self {
        return Self(CylinderShape{ name, frame, physical, length, radius, local_transform: local_transform.get_isometry(py) } )
    }
    #[getter]
    fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }
    #[getter]
    fn get_frame(&self) -> PyResult<String> {
        Ok(self.0.frame.clone())
    }
    #[getter]
    fn get_physical(&self) -> PyResult<bool> {
        Ok(self.0.physical.clone())
    }
    #[getter]
    fn get_length(&self) -> PyResult<f64> {
        Ok(self.0.length.clone())
    }
    #[getter]
    fn get_radius(&self) -> PyResult<f64> {
        Ok(self.0.radius.clone())
    }
    #[getter]
    fn get_local_transform(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.0.local_transform.translation })?,
            rotation: Py::new(py, Rotation { value: self.0.local_transform.rotation })?
        })
    }
    #[setter]
    fn set_name(&mut self,value:String) -> PyResult<()> {
        self.0.name = value;
        Ok(())
    }
    #[setter]
    fn set_frame(&mut self,value:String) -> PyResult<()> {
        self.0.frame = value;
        Ok(())
    }
    #[setter]
    fn set_physical(&mut self,value:bool) -> PyResult<()> {
        self.0.physical = value;
        Ok(())
    }
    #[setter]
    fn set_length(&mut self,value:f64) -> PyResult<()> {
        self.0.length = value;
        Ok(())
    }
    #[setter]
    fn set_radius(&mut self,value:f64) -> PyResult<()> {
        self.0.radius = value;
        Ok(())
    }
    #[setter]
    fn set_local_transform(&mut self, py: Python, local_transform: Transform) -> PyResult<()> {
        self.0.local_transform = local_transform.get_isometry(py);
        Ok(())
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyCapsuleShape {
    #[new]
    fn new(py: Python, name: String, frame:String, physical:bool, length: f64, radius: f64, local_transform: Transform) -> Self {
        return Self(CapsuleShape{name, frame, physical, length, radius, local_transform: local_transform.get_isometry(py)})
    }
    #[getter]
    fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }
    #[getter]
    fn get_frame(&self) -> PyResult<String> {
        Ok(self.0.frame.clone())
    }
    #[getter]
    fn get_physical(&self) -> PyResult<bool> {
        Ok(self.0.physical.clone())
    }
    #[getter]
    fn get_length(&self) -> PyResult<f64> {
        Ok(self.0.length.clone())
    }
    #[getter]
    fn get_radius(&self) -> PyResult<f64> {
        Ok(self.0.radius.clone())
    }
    #[getter]
    fn get_local_transform(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.0.local_transform.translation })?,
            rotation: Py::new(py, Rotation { value: self.0.local_transform.rotation })?
        })
    }
    #[setter]
    fn set_name(&mut self,value:String) -> PyResult<()> {
        self.0.name = value;
        Ok(())
    }
    #[setter]
    fn set_frame(&mut self,value:String) -> PyResult<()> {
        self.0.frame = value;
        Ok(())
    }
    #[setter]
    fn set_physical(&mut self,value:bool) -> PyResult<()> {
        self.0.physical = value;
        Ok(())
    }
    #[setter]
    fn set_length(&mut self,value:f64) -> PyResult<()> {
        self.0.length = value;
        Ok(())
    }
    #[setter]
    fn set_radius(&mut self,value:f64) -> PyResult<()> {
        self.0.radius = value;
        Ok(())
    }
    #[setter]
    fn set_local_transform(&mut self, py: Python, local_transform: Transform) -> PyResult<()> {
        self.0.local_transform = local_transform.get_isometry(py);
        Ok(())
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyMeshShape {
    #[new]
    fn new(py: Python, name: String, frame:String, physical:bool, filename: String, x: f64, y: f64, z: f64, local_transform: Transform) -> Self {
        return Self(MeshShape{name, frame, physical, filename, x, y, z, local_transform: local_transform.get_isometry(py)})
    }
    #[getter]
    fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }
    #[getter]
    fn get_frame(&self) -> PyResult<String> {
        Ok(self.0.frame.clone())
    }
    #[getter]
    fn get_physical(&self) -> PyResult<bool> {
        Ok(self.0.physical.clone())
    }
    #[getter]
    fn get_filename(&self) -> PyResult<String> {
        Ok(self.0.filename.clone())
    }
    #[getter]
    fn get_x(&self) -> PyResult<f64> {
        Ok(self.0.x.clone())
    }
    #[getter]
    fn get_y(&self) -> PyResult<f64> {
        Ok(self.0.y.clone())
    }
    #[getter]
    fn get_z(&self) -> PyResult<f64> {
        Ok(self.0.z.clone())
    }
    #[getter]
    fn get_local_transform(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.0.local_transform.translation })?,
            rotation: Py::new(py, Rotation { value: self.0.local_transform.rotation })?
        })
    }
    #[setter]
    fn set_name(&mut self,value:String) -> PyResult<()> {
        self.0.name = value;
        Ok(())
    }
    #[setter]
    fn set_frame(&mut self,value:String) -> PyResult<()> {
        self.0.frame = value;
        Ok(())
    }
    #[setter]
    fn set_physical(&mut self,value:bool) -> PyResult<()> {
        self.0.physical = value;
        Ok(())
    }
    #[setter]
    fn set_filename(&mut self,value:String) -> PyResult<()> {
        self.0.filename = value;
        Ok(())
    }
    #[setter]
    fn set_x(&mut self,value:f64) -> PyResult<()> {
        self.0.x = value;
        Ok(())
    }
    #[setter]
    fn set_y(&mut self,value:f64) -> PyResult<()> {
        self.0.y = value;
        Ok(())
    }
    #[setter]
    fn set_z(&mut self,value:f64) -> PyResult<()> {
        self.0.z = value;
        Ok(())
    }
    #[setter]
    fn set_local_transform(&mut self, py: Python, local_transform: Transform) -> PyResult<()> {
        self.0.local_transform = local_transform.get_isometry(py);
        Ok(())
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyHullShape {
    #[new]
    fn new(py: Python, name: String, frame:String, physical:bool, points: Vec<[f64;3]>, local_transform: Transform) -> Self {
        return Self(HullShape{
            name, frame, physical, 
            points: points.iter().map(|p| vector![p[0],p[1],p[2]]).collect(), 
            local_transform: local_transform.get_isometry(py)})
    }
    #[getter]
    fn get_name(&self) -> PyResult<String> {
        Ok(self.0.name.clone())
    }
    #[getter]
    fn get_frame(&self) -> PyResult<String> {
        Ok(self.0.frame.clone())
    }
    #[getter]
    fn get_physical(&self) -> PyResult<bool> {
        Ok(self.0.physical.clone())
    }
    #[getter]
    fn get_points(&self) -> PyResult<Vec<[f64;3]>> {
        Ok(self.0.points.iter().map(|p| [p.x,p.y,p.z]).collect())
    }
    #[getter]
    fn get_local_transform(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.0.local_transform.translation })?,
            rotation: Py::new(py, Rotation { value: self.0.local_transform.rotation })?
        })
    }
    #[setter]
    fn set_name(&mut self,value:String) -> PyResult<()> {
        self.0.name = value;
        Ok(())
    }
    #[setter]
    fn set_frame(&mut self,value:String) -> PyResult<()> {
        self.0.frame = value;
        Ok(())
    }
    #[setter]
    fn set_physical(&mut self,value:bool) -> PyResult<()> {
        self.0.physical = value;
        Ok(())
    }
    #[setter]
    fn set_points(&mut self,value:Vec<[f64;3]>) -> PyResult<()> {
        self.0.points = value.iter().map(|p| vector![p[0],p[1],p[2]]).collect();
        Ok(())
    }
    #[setter]
    fn set_local_transform(&mut self, py: Python, local_transform: Transform) -> PyResult<()> {
        self.0.local_transform = local_transform.get_isometry(py);
        Ok(())
    }
}