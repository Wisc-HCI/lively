use crate::objectives::objective::Objective;
use crate::objectives::core::base::VelocityMinimizationObjective;
use crate::utils::robot_model::RobotModel;
use crate::utils::vars::Vars;

#[derive(Debug)]
pub struct ObjectiveSet {
    pub objectives: Vec<Objective>,
    pub baseline: VelocityMinimizationObjective
}

impl ObjectiveSet {
    pub fn new(objectives: &Vec<Objective>) -> Self {
        Self { objectives: objectives.clone(), baseline: VelocityMinimizationObjective::new("Baseline".into(),1.0)}
    }

    pub fn call(&self, robot_model: &RobotModel, vars: &Vars, x: &[f64], is_core: bool, is_last: bool) -> f64 {
        let state = robot_model.get_state(&x.to_vec(),is_last);
        let mut out = self.baseline.call(&vars,&state,is_core);
        for i in 0..self.objectives.len() {
            out += self.objectives[i].call(&vars, &state, is_core);
        }
        out
    }

    pub fn gradient(&self, robot_model: &RobotModel, vars: &Vars, x: &[f64], is_core: bool, is_last: bool) -> (f64, Vec<f64>) {
        println!("Computing Gradient {:?}",x);
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let f_0 = self.call(robot_model, vars, x, is_core, is_last);
        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.call(robot_model, vars, x_h.as_slice(), is_core, is_last);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }
        (f_0, grad)
    }
}