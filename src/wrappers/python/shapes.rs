use pyo3::prelude::*;
use crate::utils::shapes::*;
use crate::wrappers::python::geometry::*;

#[pyclass(name="BoxShape")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyBoxShape(BoxShape);

#[pyclass(name="SphereShape")]
#[derive(Clone,Debug,PartialEq)]
pub struct PySphereShape(SphereShape);

#[pyclass(name="CylinderShape")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyCylinderShape(CylinderShape);

#[pyclass(name="CapsuleShape")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyCapsuleShape(CapsuleShape);

#[pyclass(name="BoxZone")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyBoxZone(BoxZone);

#[pyclass(name="SphereZone")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PySphereZone(SphereZone);

#[pyclass(name="CylinderZone")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyCylinderZone(CylinderZone);

#[pyclass(name="CapsuleZone")] 
#[derive(Clone,Debug,PartialEq)]
pub struct PyCapsuleZone(CapsuleZone);

#[derive(Clone,Debug,FromPyObject)]
pub enum PyShape {
    Box(PyBoxShape),
    Cylinder(PyCylinderShape),
    Sphere(PySphereShape),
    Capsule(PyCapsuleShape)
}

#[derive(Clone,Debug,FromPyObject)]
pub enum PyZone {
    Box(PyBoxZone),
    Cylinder(PyCylinderZone),
    Sphere(PySphereZone),
    Capsule(PyCapsuleZone)
}

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

impl IntoPy<PyObject> for PyZone {
    fn into_py(self, py: Python) -> PyObject {
        match self {
            Self::Box(obj) => obj.into_py(py),
            Self::Cylinder(obj) => obj.into_py(py),
            Self::Sphere(obj) => obj.into_py(py),
            Self::Capsule(obj) => obj.into_py(py)
        }
    }
}

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

impl From<PyZone> for Zone {
    fn from(pyshape:PyZone) -> Zone {
        match pyshape {
            PyZone::Box(shape) => Zone::Box(shape.0),
            PyZone::Cylinder(shape) => Zone::Cylinder(shape.0),
            PyZone::Capsule(shape) => Zone::Capsule(shape.0),
            PyZone::Sphere(shape) => Zone::Sphere(shape.0),
        }
    }
}

#[pymethods]
impl PyBoxShape {
    #[new]
    fn new(py: Python, name: String, frame:String, x: f64, y: f64, z: f64, local_transform: Transform) -> Self {
        return Self(BoxShape{ name, frame, x, y, z, local_transform: local_transform.get_isometry(py) })
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

#[pymethods]
impl PySphereShape {
    #[new]
    fn new(py: Python, name: String, frame:String, radius: f64, local_transform: Transform) -> Self {
        Self(SphereShape{name, frame, radius, local_transform: local_transform.get_isometry(py)})
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

#[pymethods]
impl PyCylinderShape {
    #[new]
    fn new(py: Python, name: String, frame:String, length: f64, radius: f64, local_transform: Transform) -> Self {
        return Self(CylinderShape{ name, frame, length, radius, local_transform: local_transform.get_isometry(py) } )
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

#[pymethods]
impl PyCapsuleShape {
    #[new]
    fn new(py: Python, name: String, frame:String, length: f64, radius: f64, local_transform: Transform) -> Self {
        return Self(CapsuleShape{name, frame, length, radius, local_transform: local_transform.get_isometry(py)})
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

#[pymethods]
impl PyBoxZone {
    #[new]
    fn new(py: Python, name: String, frame: String, x: f64, y: f64, z: f64, local_transform: Transform) -> Self {
        return Self(BoxZone{ name, frame, x, y, z, local_transform: local_transform.get_isometry(py) })
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

#[pymethods]
impl PySphereZone {
    #[new]
    fn new(py: Python, name: String, frame: String, radius: f64, local_transform: Transform) -> Self {
        Self(SphereZone{name, frame, radius, local_transform: local_transform.get_isometry(py)})
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

#[pymethods]
impl PyCylinderZone {
    #[new]
    fn new(py: Python, name: String, frame: String, length: f64, radius: f64, local_transform: Transform) -> Self {
        return Self(CylinderZone{ name, frame, length, radius, local_transform: local_transform.get_isometry(py) }) 
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

#[pymethods]
impl PyCapsuleZone {
    #[new]
    fn new(py: Python, name: String, frame: String, length: f64, radius: f64, local_transform: Transform) -> Self {
        return Self(CapsuleZone{name, frame, length, radius, local_transform: local_transform.get_isometry(py)})
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