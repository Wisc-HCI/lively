use pyo3::prelude::*;
use std::collections::HashMap;
use nalgebra::geometry::{Isometry3};
use nalgebra::{vector, Vector3};
use crate::utils::state::State;
use crate::utils::info::*;
use crate::wrappers::python::geometry::{*};
use crate::wrappers::python::info::{*};

#[pyclass(name="State")] 
#[derive(Clone,Debug)]
pub struct PyState(State);

#[pymethods]
impl PyState {
    #[new]
    pub fn new(py: Python, origin: Transform, joints: HashMap<String,f64>) -> Self {
        let frames: HashMap<String,Isometry3<f64>> = HashMap::new();
        let proximity: Vec<ProximityInfo> = vec![]; 
        let center_of_mass: Vector3<f64> = vector![0.0,0.0,0.0];
        Self(State::new(
            origin.get_isometry(py), 
            joints, frames, 
            proximity,
            center_of_mass
        ))
    }

    #[getter]
    pub fn get_origin(&self, py: Python) -> PyResult<Transform> {
        Ok(Transform {
            translation: Py::new(py, Translation { value: self.0.origin.translation })?,
            rotation: Py::new(py, Rotation { value: self.0.origin.rotation })?
        })
    }

    #[getter]
    pub fn get_joints(&self) -> PyResult<HashMap<String,f64>> {
        Ok(self.0.joints.clone())
    }

    #[getter]
    pub fn get_frames(&self, py: Python) -> PyResult<HashMap<String,Transform>> {
        let mut transform_frames: HashMap<String,Transform> = HashMap::new();
        for (key, iso) in self.0.frames.iter() {
            transform_frames.insert(key.to_string(), Transform {
                translation: Py::new(py, Translation { value: iso.translation })?,
                rotation: Py::new(py, Rotation { value: iso.rotation })?
            });
        }
        Ok(transform_frames)
    }

    #[getter]
    pub fn get_proximity(&self) -> PyResult<Vec<PyProximityInfo>> {
        Ok(self.0.proximity.iter().map(|p| PyProximityInfo::from(p.clone())).collect())
    }

    #[getter]
    pub fn get_center_of_mass(&self) -> PyResult<Translation> {
        Ok(Translation::from(self.0.center_of_mass))
    }
}

impl From<State> for PyState {
    fn from(state:State) -> PyState {
        PyState(state)
    }
}

impl From<PyState> for State {
    fn from(pystate:PyState) -> State {
        pystate.0
    }
}