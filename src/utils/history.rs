use crate::utils::state::State;

pub struct History {
    pub prev1: State,
    pub prev2: State,
    pub prev3: State
}

impl History {
    pub fn new(state: &State) -> Self {
        Self {
            prev1: state.clone(),
            prev2: state.stepped(1.0/100000.0),
            prev3: state.stepped(2.0/100000.0)}
    }

    pub fn update(&mut self, new: &State) {
        self.prev3 = self.prev2.clone();
        self.prev2 = self.prev1.clone();
        self.prev1 = new.clone();
    }

    pub fn reset(&mut self, new: &State) {
        self.prev3 = new.stepped(2.0/100000.0);
        self.prev2 = new.stepped(1.0/100000.0);
        self.prev1 = new.clone();
    }
}
