use crate::objectives::objective::Objective;
use crate::utils::robot_model::RobotModel;
use crate::utils::vars::Vars;

#[derive(Debug)]
pub struct ObjectiveSet {
    pub objectives: Vec<Objective>
}

impl ObjectiveSet {
    pub fn new(objectives: &Vec<Objective>) -> Self {
        Self { objectives: objectives.clone() }
    }

    pub fn call(&self, robot_model: &RobotModel, vars: &Vars, x: &[f64], is_core: bool, is_last: bool, orig_x: &[f64]) -> f64 {
        let mut out = 0.0;
        let state;
        if x.contains(&f64::NAN) {
            state = robot_model.get_state(&x.to_vec(),is_last);
        } else {
            state = robot_model.get_state(&orig_x.to_vec(),is_last);
        }
        for i in 0..self.objectives.len() {
            out += self.objectives[i].call(&vars, &state, is_core);
        }
        out
    }

    pub fn gradient(&self, robot_model: &RobotModel, vars: &Vars, x: &[f64], is_core: bool, is_last: bool, orig_x: &[f64]) -> (f64, Vec<f64>) {
        // println!("Computing Gradient {:?}",x);
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let f_0 = self.call(robot_model, vars, x, is_core, is_last, orig_x);
        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.call(robot_model, vars, x_h.as_slice(), is_core, is_last, orig_x);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }
        (f_0, grad)
    }
}