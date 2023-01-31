#[cfg(feature = "pybindings")]
use nalgebra::geometry::{Isometry3, Quaternion, Translation3, UnitQuaternion};
#[cfg(feature = "pybindings")]
use nalgebra::Vector3;
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "pybindings")]
use std::collections::HashMap;

#[cfg_attr(feature = "pybindings", pyclass(name = "Size"))]
#[derive(Clone, Debug)]
pub struct PySize {
    pub value: Vector3<f64>,
}

#[cfg_attr(feature = "pybindings", pyclass(name = "Point3"))]
#[derive(Clone, Debug)]
pub struct PyPoint3 {
    pub value: Vector3<f64>,
}

#[cfg_attr(feature = "pybindings", pyclass(name = "Translation"))]
#[derive(Clone, Debug)]
pub struct PyTranslation {
    pub value: Translation3<f64>,
}

#[cfg_attr(feature = "pybindings", pyclass(name = "Rotation"))]
#[derive(Clone, Debug)]
pub struct PyRotation {
    pub value: UnitQuaternion<f64>,
}

#[cfg_attr(feature = "pybindings", pyclass(name = "Transform"))]
#[derive(Clone, Debug)]
pub struct PyTransform {
    #[pyo3(get)]
    pub translation: Py<PyTranslation>,
    #[pyo3(get)]
    pub rotation: Py<PyRotation>,
}

#[cfg(feature = "pybindings")]
impl PyPoint3 {
    pub fn new(x: f64, y: f64, z:f64) -> Self{
        Self{
            value: Vector3::new(x, y, z),
        }
    }
}

#[cfg(feature = "pybindings")]
impl PySize {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            value: Vector3::new(x, y, z),
        }
    }
}
#[cfg(feature = "pybindings")]
impl PyTranslation {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            value: Translation3::new(x, y, z),
        }
    }
}
#[cfg(feature = "pybindings")]
impl PyRotation {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self {
            value: UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
        }
    }
}
#[cfg(feature = "pybindings")]
impl PyTransform {
    pub fn get_isometry(&self, py: Python) -> Isometry3<f64> {
        let t: PyTranslation = self.translation.extract(py).unwrap();
        let r: PyRotation = self.rotation.extract(py).unwrap();
        return Isometry3::from_parts(t.value, r.value);
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyPoint3 {
    #[new]
    pub fn from_python(x: f64, y: f64, z: f64) -> Self {
        Self::new(x, y, z)
    }
    #[getter]
    pub fn get_x(&self) -> PyResult<f64> {
        Ok(self.value.x)
    }
    #[getter]
    pub fn get_y(&self) -> PyResult<f64> {
        Ok(self.value.y)
    }
    #[getter]
    pub fn get_z(&self) -> PyResult<f64> {
        Ok(self.value.z)
    }
    #[setter]
    pub fn set_x(&mut self, value: f64) -> PyResult<()> {
        self.value.x = value;
        Ok(())
    }
    #[setter]
    pub fn set_y(&mut self, value: f64) -> PyResult<()> {
        self.value.y = value;
        Ok(())
    }
    #[setter]
    pub fn set_z(&mut self, value: f64) -> PyResult<()> {
        self.value.z = value;
        Ok(())
    }

    pub fn as_vec(&self) -> PyResult<[f64; 3]> {
        Ok([self.value.x, self.value.y, self.value.z])
    }
    pub fn as_dict(&self) -> PyResult<HashMap<&str, f64>> {
        let mut map: HashMap<&str, f64> = HashMap::new();
        map.insert(&"x", self.value.x);
        map.insert(&"y", self.value.y);
        map.insert(&"z", self.value.z);
        Ok(map)
    }

    fn __str__(&self) -> PyResult<String> {
        Ok(format!(
            "[{}, {}, {}]",
            self.value.x, self.value.y, self.value.z
        ))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!(
            "[{}, {}, {}]",
            self.value.x, self.value.y, self.value.z
        ))
    }
}



#[cfg(feature = "pybindings")]
#[pymethods]
impl PySize {
    #[new]
    pub fn from_python(x: f64, y: f64, z: f64) -> Self {
        Self::new(x, y, z)
    }
    #[getter]
    pub fn get_x(&self) -> PyResult<f64> {
        Ok(self.value.x)
    }
    #[getter]
    pub fn get_y(&self) -> PyResult<f64> {
        Ok(self.value.y)
    }
    #[getter]
    pub fn get_z(&self) -> PyResult<f64> {
        Ok(self.value.z)
    }
    #[setter]
    pub fn set_x(&mut self, value: f64) -> PyResult<()> {
        self.value.x = value;
        Ok(())
    }
    #[setter]
    pub fn set_y(&mut self, value: f64) -> PyResult<()> {
        self.value.y = value;
        Ok(())
    }
    #[setter]
    pub fn set_z(&mut self, value: f64) -> PyResult<()> {
        self.value.z = value;
        Ok(())
    }

    pub fn as_vec(&self) -> PyResult<[f64; 3]> {
        Ok([self.value.x, self.value.y, self.value.z])
    }
    pub fn as_dict(&self) -> PyResult<HashMap<&str, f64>> {
        let mut map: HashMap<&str, f64> = HashMap::new();
        map.insert(&"x", self.value.x);
        map.insert(&"y", self.value.y);
        map.insert(&"z", self.value.z);
        Ok(map)
    }

    fn __str__(&self) -> PyResult<String> {
        Ok(format!(
            "[{}, {}, {}]",
            self.value.x, self.value.y, self.value.z
        ))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!(
            "[{}, {}, {}]",
            self.value.x, self.value.y, self.value.z
        ))
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyTranslation {
    #[new]
    pub fn from_python(x: f64, y: f64, z: f64) -> Self {
        Self::new(x, y, z)
    }
    #[getter]
    pub fn get_x(&self) -> PyResult<f64> {
        Ok(self.value.vector.x)
    }
    #[getter]
    pub fn get_y(&self) -> PyResult<f64> {
        Ok(self.value.vector.y)
    }
    #[getter]
    pub fn get_z(&self) -> PyResult<f64> {
        Ok(self.value.vector.z)
    }
    #[setter]
    pub fn set_x(&mut self, value: f64) -> PyResult<()> {
        self.value.vector.x = value;
        Ok(())
    }
    #[setter]
    pub fn set_y(&mut self, value: f64) -> PyResult<()> {
        self.value.vector.y = value;
        Ok(())
    }
    #[setter]
    pub fn set_z(&mut self, value: f64) -> PyResult<()> {
        self.value.vector.z = value;
        Ok(())
    }

    pub fn as_vec(&self) -> PyResult<[f64; 3]> {
        Ok([
            self.value.vector.x,
            self.value.vector.y,
            self.value.vector.z,
        ])
    }
    pub fn as_dict(&self) -> PyResult<HashMap<&str, f64>> {
        let mut map: HashMap<&str, f64> = HashMap::new();
        map.insert(&"x", self.value.vector.x);
        map.insert(&"y", self.value.vector.y);
        map.insert(&"z", self.value.vector.z);
        Ok(map)
    }

    fn __str__(&self) -> PyResult<String> {
        Ok(format!(
            "[{}, {}, {}]",
            self.value.vector.x, self.value.vector.y, self.value.vector.z
        ))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!(
            "[{}, {}, {}]",
            self.value.vector.x, self.value.vector.y, self.value.vector.z
        ))
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyRotation {
    #[new]
    pub fn from_python(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self::new(w, x, y, z)
    }
    #[getter]
    pub fn get_w(&self) -> PyResult<f64> {
        Ok(self.value.coords[3])
    }
    #[getter]
    pub fn get_x(&self) -> PyResult<f64> {
        Ok(self.value.coords[0])
    }
    #[getter]
    pub fn get_y(&self) -> PyResult<f64> {
        Ok(self.value.coords[1])
    }
    #[getter]
    pub fn get_z(&self) -> PyResult<f64> {
        Ok(self.value.coords[2])
    }

    pub fn set(&mut self, w: f64, x: f64, y: f64, z: f64) -> PyResult<()> {
        self.value = UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));
        Ok(())
    }

    pub fn as_vec(&self) -> PyResult<[f64; 4]> {
        Ok([
            self.value.coords[3],
            self.value.coords[0],
            self.value.coords[1],
            self.value.coords[2],
        ])
    }
    pub fn as_dict(&self) -> PyResult<HashMap<&str, f64>> {
        let mut map: HashMap<&str, f64> = HashMap::new();
        map.insert(&"w", self.value.coords[3]);
        map.insert(&"x", self.value.coords[0]);
        map.insert(&"y", self.value.coords[1]);
        map.insert(&"z", self.value.coords[2]);
        Ok(map)
    }

    fn __str__(&self) -> PyResult<String> {
        Ok(format!(
            "[{}, {}, {}, {}]",
            self.value.coords[3], self.value.coords[0], self.value.coords[1], self.value.coords[2]
        ))
    }
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!(
            "[{}, {}, {}, {}]",
            self.value.coords[3], self.value.coords[0], self.value.coords[1], self.value.coords[2]
        ))
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyTransform {
    #[new]
    pub fn from_python(
        py: Python,
        translation: PyTranslation,
        rotation: PyRotation,
    ) -> PyResult<Self> {
        Ok(Self {
            translation: Py::new(py, translation)?,
            rotation: Py::new(py, rotation)?,
        })
    }

    #[staticmethod]
    pub fn identity(py: Python) -> PyResult<Self> {
        Ok(Self {
            translation: Py::new(py, PyTranslation::new(0.0, 0.0, 0.0))?,
            rotation: Py::new(py, PyRotation::new(1.0, 0.0, 0.0, 0.0))?,
        })
    }

    pub fn as_vecs(&self, py: Python) -> PyResult<([f64; 3], [f64; 4])> {
        let t: PyTranslation = self.translation.extract(py).unwrap();
        let r: PyRotation = self.rotation.extract(py).unwrap();
        Ok((
            [t.value.vector.x, t.value.vector.y, t.value.vector.z],
            [
                r.value.coords[3],
                r.value.coords[0],
                r.value.coords[1],
                r.value.coords[2],
            ],
        ))
    }
    pub fn as_dicts(&self, py: Python) -> PyResult<(HashMap<&str, f64>, HashMap<&str, f64>)> {
        let t: PyTranslation = self.translation.extract(py).unwrap();
        let r: PyRotation = self.rotation.extract(py).unwrap();
        let mut vec_map: HashMap<&str, f64> = HashMap::new();
        vec_map.insert(&"x", t.value.vector.x);
        vec_map.insert(&"y", t.value.vector.y);
        vec_map.insert(&"z", t.value.vector.z);
        let mut quat_map: HashMap<&str, f64> = HashMap::new();
        quat_map.insert(&"w", r.value.coords[3]);
        quat_map.insert(&"x", r.value.coords[0]);
        quat_map.insert(&"y", r.value.coords[1]);
        quat_map.insert(&"z", r.value.coords[2]);
        Ok((vec_map, quat_map))
    }
}

#[cfg(feature = "pybindings")]
impl From<Vector3<f64>> for PyTranslation {
    fn from(vector: Vector3<f64>) -> Self {
        Self {
            value: Translation3::new(vector.x, vector.y, vector.z),
        }
    }
}

// #[cfg(feature = "pybindings")]
// #[pymethods]
// impl Ellipse {
//     #[new]
//     pub fn from_python(translation: Translation, rotation: Rotation, size: Size) -> Self {
//         Self { translation, rotation, size }
//     }
// }

// #[cfg(feature = "pybindings")]
// #[pymethods]
// impl RotationRange {
//     #[new]
//     pub fn from_python(rotation: Rotation, delta: f64) -> Self {
//         Self { rotation, delta }
//     }
// }

// #[cfg(feature = "pybindings")]
// #[pymethods]
// impl ScalarRange {
//     #[new]
//     pub fn from_python(value: f64, delta: f64) -> Self {
//         Self { value, delta }
//     }
// }
