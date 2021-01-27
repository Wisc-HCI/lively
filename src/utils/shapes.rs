use pyo3::prelude::*;

#[pyclass]
#[derive(Clone,Debug)]
pub struct Cuboid {
    #[pyo3(get, set)]
    pub name: String,
    #[pyo3(get, set)]
    pub x_halflength: f64,
    #[pyo3(get, set)]
    pub y_halflength: f64,
    #[pyo3(get, set)]
    pub z_halflength: f64,
    #[pyo3(get, set)]
    pub rx: f64,
    #[pyo3(get, set)]
    pub ry: f64,
    #[pyo3(get, set)]
    pub rz: f64,
    #[pyo3(get, set)]
    pub tx: f64,
    #[pyo3(get, set)]
    pub ty: f64,
    #[pyo3(get, set)]
    pub tz: f64,
    #[pyo3(get, set)]
    pub is_dynamic: bool,
    #[pyo3(get, set)]
    pub coordinate_frame: Option<String>
}

#[pymethods]
impl Cuboid {
    #[new]
    fn new(name: String, x_halflength: f64, y_halflength: f64, z_halflength: f64,
           rx: f64, ry: f64, rz: f64, tx: f64, ty: f64, tz: f64, is_dynamic: bool, coordinate_frame: Option<String>) -> Self {
        Self { name, x_halflength, y_halflength, z_halflength, rx, ry, rz, tx, ty, tz, is_dynamic, coordinate_frame }
    }
}

#[pyclass]
#[derive(Clone,Debug)]
pub struct Sphere {
    #[pyo3(get, set)]
    pub name: String,
    #[pyo3(get, set)]
    pub radius: f64,
    #[pyo3(get, set)]
    pub tx: f64,
    #[pyo3(get, set)]
    pub ty: f64,
    #[pyo3(get, set)]
    pub tz: f64,
    #[pyo3(get, set)]
    pub is_dynamic: bool,
    #[pyo3(get, set)]
    pub coordinate_frame: Option<String>,
}

#[pymethods]
impl Sphere {
    #[new]
    fn new(name: String, radius: f64, tx: f64, ty: f64, tz: f64, is_dynamic: bool, coordinate_frame: Option<String>) -> Self {
        Self { name, radius, tx, ty, tz, is_dynamic, coordinate_frame }
    }
}

#[pyclass]
#[derive(Clone, Debug)]
pub struct PC {
    #[pyo3(get, set)]
    pub name: String,
    #[pyo3(get, set)]
    pub rx: f64,
    #[pyo3(get, set)]
    pub ry: f64,
    #[pyo3(get, set)]
    pub rz: f64,
    #[pyo3(get, set)]
    pub tx: f64,
    #[pyo3(get, set)]
    pub ty: f64,
    #[pyo3(get, set)]
    pub tz: f64,
    #[pyo3(get, set)]
    pub is_dynamic: bool,
    #[pyo3(get, set)]
    pub points: Vec<Sphere>
}

#[pymethods]
impl PC {
    #[new]
    pub fn new(name: String, rx: f64, ry: f64, rz: f64, tx: f64, ty: f64, tz: f64, is_dynamic: bool, points: Vec<Sphere>) -> Self {
        Self {name, rx, ry, rz, tx, ty, tz, is_dynamic, points}
    }
}
