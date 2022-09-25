use serde::{Serialize, Deserialize};
use crate::utils::vars::Vars;
use crate::utils::state::State;
use crate::objectives::objective::{groove_loss,Callable};

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
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
        state: &State,
        is_core: bool
    ) -> f64 {
        let prev_position;
        if is_core {
            prev_position = v.history_core.prev1.get_link_transform(&self.link).translation.vector;
        } else {
            prev_position = v.history.prev1.get_link_transform(&self.link).translation.vector;
        }
        let current_position = state.get_link_transform(&self.link).translation.vector;
        let x = current_position[2]-prev_position[2];
        let x_val = 1.0/(1.0+(-4.0*x+2.0).exp());
        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}
