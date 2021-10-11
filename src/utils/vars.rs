use crate::utils::history::History;
use crate::utils::robot_model::RobotModel;
use crate::utils::collision_manager::CollisionManager;
use crate::utils::state::State;
use crate::utils::shapes::{*};
use crate::utils::info::{*};

pub struct Vars {
    pub state_core: State,
    pub history: History,
    pub history_core: History,
    pub joints: Vec<JointInfo>,
    pub links: Vec<LinkInfo>,
    pub collision_manager: CollisionManager
}

impl Vars {
    pub fn new(initial_state:State, joints: Vec<JointInfo>, links: Vec<LinkInfo>, collision_manager: CollisionManager) -> Self {
        Self {
            state_core: initial_state.clone(),
            history: History::new(initial_state),
            history_core: History::new(initial_state),
            joints, links,
            collision_manager
        }
    }
}