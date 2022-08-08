#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "pybindings")]
use std::collections::HashMap;
// #[cfg(feature = "pybindings")]
// use nalgebra::geometry::{Isometry3};
#[cfg(feature = "pybindings")]
use nalgebra::{vector, Vector3};
#[cfg(feature = "pybindings")]
use crate::utils::state::State;
#[cfg(feature = "pybindings")]
use crate::utils::info::*;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::geometry::{*};
#[cfg(feature = "pybindings")]
use crate::wrappers::python::info::{*};

#[cfg(feature = "pybindings")]
#[pyclass(name="State")] 
#[derive(Clone,Debug)]
pub struct PyState(State);

#[cfg(feature = "pybindings")]
#[pymethods]
impl PyState {
    #[new]
    pub fn new(py: Python, origin: Transform, joints: HashMap<String,f64>) -> Self {
        let frames: HashMap<String,TransformInfo> = HashMap::new();
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
    pub fn get_frames(&self, py: Python) -> PyResult<HashMap<String,PyTransformInfo>> {
        let mut transform_frames: HashMap<String,PyTransformInfo> = HashMap::new();
        for (key, tfi) in self.0.frames.iter() {
            transform_frames.insert(key.to_string(), PyTransformInfo::from(tfi.clone()));
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

#[cfg(feature = "pybindings")]
impl From<State> for PyState {
    fn from(state:State) -> PyState {
        PyState(state)
    }
}

#[cfg(feature = "pybindings")]
impl From<PyState> for State {
    fn from(pystate:PyState) -> State {
        pystate.0
    }
}