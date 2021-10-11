use crate::utils::state::State;

pub struct History {
    pub prev1: State,
    pub prev2: State,
    pub prev3: State
}

impl History {
    pub fn new(state: State) -> Self {
        Self {
            prev1: state.clone(),
            prev2: state.clone(),
            prev3: state.clone()}
    }

    pub fn update(&mut self, new: State) {
        self.prev3 = self.prev2.clone();
        self.prev2 = self.prev1.clone();
        self.prev1 = new.clone();
    }

    pub fn reset(&mut self, new: State) {
        self.prev3 = new.clone();
        self.prev2 = new.clone();
        self.prev1 = new.clone();
    }
}
