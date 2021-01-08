
pub struct History {
    pub prev1: Vec<f64>,
    pub prev2: Vec<f64>,
    pub prev3: Vec<f64>
}

impl History {
    pub fn new(state: Vec<f64>) -> Self {
        Self {prev1: state.clone(), prev2: state.clone(), prev3: state.clone()}
    }

    pub fn update(&mut self, new: Vec<f64>) {
        self.prev3 = self.prev2.clone();
        self.prev2 = self.prev1.clone();
        self.prev1 = new.clone();
    }
}
