use crate::utils::history::History;
use crate::utils::state::State;
use crate::utils::info::{*};

pub struct Vars {
    pub history: History,
    pub joints: Vec<JointInfo>,
    pub links: Vec<LinkInfo>
}

impl Vars {
    pub fn new(initial_state:&State, joints: Vec<JointInfo>, links: Vec<LinkInfo>) -> Self {
        Self {
            history: History::new(&initial_state),
            joints, links
        }
    }
}