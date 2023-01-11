use serde::{Serialize, Deserialize};
use crate::utils::vars::Vars;
use crate::utils::state::State;
use crate::objectives::objective::{groove_loss,Callable};
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct GravityObjective {
    pub name: String,
    pub weight: f64,
    pub link: String
}

impl GravityObjective {

    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self { name, weight, link}
    }
}

impl Callable<bool> for GravityObjective {

    fn call(
        &self,
        v: &Vars,
        state: &State
    ) -> f64 {
        let prev_position = v.history.prev1.get_link_transform(&self.link).translation.vector;
        let current_position = state.get_link_transform(&self.link).translation.vector;
        let x = current_position[2]-prev_position[2];
        let x_val = 1.0/(1.0+(-4.0*x+2.0).exp());
        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl GravityObjective {
    #[new]
    pub fn from_python(name:String,weight:f64,link:String) -> Self {
        GravityObjective::new(name,weight,link)
    }
    
    #[getter]
    pub fn get_name(&self) -> PyResult<String> {
        Ok(self.name.clone())
    }

    #[getter]
    pub fn get_weight(&self) -> PyResult<f64> {
        Ok(self.weight.clone())
    }

    #[getter]
    pub fn get_link(&self) -> PyResult<String> {
        Ok(self.link.clone())
    }
}