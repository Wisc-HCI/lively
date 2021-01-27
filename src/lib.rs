extern crate pyo3;
use pyo3::prelude::*;

pub mod utils;
pub mod spacetime;
pub mod groove;
pub mod relaxed_ik;
// pub mod relaxed_ik_wrapper;

use crate::utils::config::{*};
use crate::utils::shapes::{*};
use crate::utils::goals::{*};
use crate::spacetime::robot::Robot;
use crate::groove::vars::RelaxedIKVars;
use crate:: relaxed_ik::LivelyIK;

#[pymodule]
fn lively_ik_core(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<Config>()?;
    m.add_class::<GoalConfig>()?;
    m.add_class::<NNSpec>()?;
    m.add_class::<EnvironmentSpec>()?;
    m.add_class::<ObjectiveSpec>()?;
    m.add_class::<GoalSpec>()?;
    m.add_class::<Cuboid>()?;
    m.add_class::<Sphere>()?;
    m.add_class::<Robot>()?;
    m.add_class::<RelaxedIKVars>()?;
    m.add_class::<LivelyIK>()?;
    Ok(())
}
