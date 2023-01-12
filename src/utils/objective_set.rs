use crate::objectives::objective::{Objective,Callable};
use crate::objectives::core::base::JointVelocityMinimizationObjective;
use crate::utils::robot_model::RobotModel;
use crate::utils::vars::Vars;
use std::collections::HashMap;

#[derive(Debug)]
pub struct ObjectiveSet {
    pub objectives: HashMap<String,Objective>,
    pub baseline: JointVelocityMinimizationObjective
}

impl ObjectiveSet {
    pub fn new(objectives: &HashMap<String,Objective>) -> Self {
        Self { objectives: objectives.clone(), baseline: JointVelocityMinimizationObjective::new("Baseline".into(),0.1)}
    }

    pub fn call(&self, robot_model: &RobotModel, vars: &Vars, x: &[f64], timestamp: f64) -> f64 {
        let state = robot_model.get_state(&x.to_vec(),true, timestamp);
        let mut out = self.baseline.call(&vars,&state);
        for (_,objective) in &self.objectives {
            out += objective.call(&vars, &state);
        }
        out
    }

    pub fn gradient(&self, robot_model: &RobotModel, vars: &Vars, x: &[f64], timestamp: f64) -> (f64, Vec<f64>) {
        // println!("Computing Gradient {:?}",x);
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let f_0 = self.call(robot_model, vars, x, timestamp);
        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.call(robot_model, vars, x_h.as_slice(), timestamp);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }
        (f_0, grad)
    }
}