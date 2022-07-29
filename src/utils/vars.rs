use crate::utils::history::History;
use crate::utils::state::State;
use crate::utils::info::{*};

pub struct Vars {
    pub state_core: State,
    pub history: History,
    pub history_core: History,
    pub joints: Vec<JointInfo>,
    pub links: Vec<LinkInfo>
}

impl Vars {
    pub fn new(initial_state:&State, joints: Vec<JointInfo>, links: Vec<LinkInfo>) -> Self {
        Self {
            state_core: initial_state.clone(),
            history: History::new(&initial_state),
            history_core: History::new(&initial_state),
            joints, links
        }
    }
}