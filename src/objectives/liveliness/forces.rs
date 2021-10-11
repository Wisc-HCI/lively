use crate::utils::vars::Vars;
use crate::objectives::objective::{groove_loss};

#[pyclass]
#[derive(Clone,Debug)]
pub struct GravityObjective {
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link: String
}

#[pymethods]
impl GravityObjective {
    #[new]
    pub fn new(name: String, weight: f64, link: String) -> Self {
        Self { name, weight, link}
    }
}

impl GravityObjective {
    fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool,
    ) -> f64 {
        let mut prev_position;
        if is_core {
            prev_position = v.history_core.prev1.get_frame_transform(&self.link).translation.vector;
        } else {
            prev_position = v.history.prev1.get_frame_transform(&self.link).translation.vector;
        }
        const current_position = state.get_frame_transform(&self.link).translation.vector;
        const x = (current_position[2]-prev_position[2])
        const x_val = 1.0/(1.0+f64::E.powf(4.0*x-2.0))
        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}
