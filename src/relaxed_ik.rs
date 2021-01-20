use pyo3::prelude::*;
use crate::groove::vars::RelaxedIKVars;
use crate::groove::groove::{OptimizationEngineOpen, OptimizationEngineNLopt};
use crate::groove::objective_master::ObjectiveMaster;
// use crate::utils::file_utils::{*};
// use crate::utils::subscriber_utils::EEPoseGoalsSubscriber;
// use crate::utils::transformations::{*};
// use crate::utils::yaml_utils::{*};
use crate::utils::config::{Config,EnvironmentSpec};
use crate::utils::goals::{Goal,GoalSpec};
use crate::utils::settings::{*};
// use crate::utils::sampler::ThreadSampler;
// use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use std::os::raw::{c_double, c_int};

#[repr(C)]
pub struct Opt {
    pub data: *const c_double,
    pub length: c_int,
}

#[pyclass]
pub struct LivelyIK {
    pub config: Config,
    pub vars: RelaxedIKVars,
    pub om: ObjectiveMaster,
    pub groove: OptimizationEngineOpen,
    pub groove_nlopt: OptimizationEngineNLopt
}

#[pymethods]
impl LivelyIK {
    #[new]
    fn new(config: Config) -> Self {
        println!("Creating RelaxedIKVars");
        let vars = RelaxedIKVars::new(config.clone());
        println!("Creating ObjectiveMaster");
        let mut om = ObjectiveMaster::new(config.clone());
        println!("Creating OptimizationEngine");
        let groove = OptimizationEngineOpen::new(vars.robot.num_dof.clone()+3);
        let groove_nlopt = OptimizationEngineNLopt::new();

        Self { config, vars, om, groove, groove_nlopt }
    }

    fn solve(&mut self, goals: Vec<GoalSpec>, time: f64, world: Option<EnvironmentSpec>, _precise: Option<bool>) -> PyResult<Vec<f64>> {
        // Will be the output of solve. N=num_dof
        let mut out_x = self.vars.xopt.clone();
        // Will be the output of solving for core. N=num_dof
        let mut out_x_core = self.vars.xopt_core.clone();
        // Will be the output of the optimization. N=num_dof+3
        let mut xopt = self.vars.offset.clone();
        // Will be the output of the core optimization. N=num_dof+3
        let mut xopt_core = vec![0.0,0.0,0.0];

        for i in 0..out_x.len() {
            xopt.push(out_x[i])
        }

        for i in 0..out_x_core.len() {
            xopt_core.push(out_x_core[i])
        }

        if goals.len() != self.config.objectives.len() {
            println!("Mismatch between goals (n={:?}) and objectives (n={:?})", goals.len(), self.config.objectives.len());
            return Ok(out_x);
        }

        for goal_idx in 0..goals.clone().len() {
            match goals[goal_idx].weight {
                Some(weight) => self.om.weight_priors[goal_idx] = weight,
                None => {}
            }
            match goals[goal_idx].value {
                // Only update if the goal is specified.
                Goal::None => {},
                _ => {self.vars.goals[goal_idx].value = goals[goal_idx].value}
            }
        }

        println!("Updated Goals {:?}",self.vars.goals);

        self.vars.liveliness.update(time);

        match world {
            // Update the collision world
            Some(env) => {

            },
            // Keep the same one as previous
            None => {}
        }

        let in_collision = false;//self.vars.update_collision_world();
        if !in_collision {
            if self.config.mode_environment == EnvironmentMode::ECAA {
                self.om.tune_weight_priors(&self.vars);
            }
            // Run without liveliness (core objectives)
            self.groove.optimize(&mut xopt_core, &self.vars, &self.om, 100, true);
            for i in 0..out_x_core.len() {
                out_x_core[i] = xopt_core[i+3];
            }
            self.vars.xopt_core = out_x_core.clone();
            self.vars.history_core.update(out_x_core.clone());
            self.vars.frames_core = self.vars.robot.get_frames(&xopt_core.clone());

            // Run with liveliness (all objectives)
            self.groove.optimize(&mut xopt, &self.vars, &self.om, 100, false);
            for i in 0..out_x_core.len() {
                out_x[i] = xopt[i+3];
            }
            self.vars.xopt = out_x.clone();
            self.vars.history.update(out_x.clone());
            self.vars.offset = vec![xopt[0],xopt[1],xopt[2]]
        }

        return Ok(out_x)
    }
}

// impl LivelyIK {
//
//     pub fn solve(&mut self, ee_sub: &EEPoseGoalsSubscriber) -> Vec<f64> {
//         let mut out_x = self.vars.xopt.clone();
//
//         if self.vars.rotation_mode_relative {
//             for i in 0..self.vars.robot.num_chains {
//                 self.vars.goal_positions[i] = self.vars.init_ee_positions[i] + ee_sub.pos_goals[i];
//                 self.vars.goal_quats[i] = ee_sub.quat_goals[i] * self.vars.init_ee_quats[i];
//             }
//         } else {
//             for i in 0..self.vars.robot.num_chains  {
//                 self.vars.goal_positions[i] = ee_sub.pos_goals[i].clone();
//                 self.vars.goal_quats[i] = ee_sub.quat_goals[i].clone();
//             }
//         }
//
//         let in_collision = self.vars.update_collision_world();
//         if !in_collision {
//             if self.vars.objective_mode == "ECAA" {
//                 self.om.tune_weight_priors(&self.vars);
//             }
//             self.groove.optimize(&mut out_x, &self.vars, &self.om, 100);
//             self.vars.update(out_x.clone());
//         }
//         out_x
//     }
//
//     pub fn solve_with_user_provided_goals(&mut self, pos_goals: Vec<Vec<f64>>, quat_goals: Vec<Vec<f64>>) -> Vec<f64> {
//         let mut ee_sub = EEPoseGoalsSubscriber::new();
//         for i in 0..pos_goals.len() {
//             ee_sub.pos_goals.push( Vector3::new( pos_goals[i][0], pos_goals[i][1], pos_goals[i][2] ) );
//             let tmp_quat = Quaternion::new(quat_goals[i][0], quat_goals[i][1], quat_goals[i][2], quat_goals[i][3]);
//             ee_sub.quat_goals.push( UnitQuaternion::from_quaternion(tmp_quat) );
//         }
//
//         self.solve(&ee_sub)
//     }
//
//     pub fn solve_precise(&mut self, ee_sub: &EEPoseGoalsSubscriber) -> (Vec<f64>) {
//         let mut out_x = self.vars.xopt.clone();
//
//         if self.vars.rotation_mode_relative {
//             for i in 0..self.vars.robot.num_chains {
//                 self.vars.goal_positions[i] = self.vars.init_ee_positions[i] + ee_sub.pos_goals[i];
//                 self.vars.goal_quats[i] = ee_sub.quat_goals[i] * self.vars.init_ee_quats[i];
//             }
//         } else {
//             for i in 0..self.vars.robot.num_chains  {
//                 self.vars.goal_positions[i] = ee_sub.pos_goals[i].clone();
//                 self.vars.goal_quats[i] = ee_sub.quat_goals[i].clone();
//             }
//         }
//
//         self.groove_nlopt.optimize(&mut out_x, &self.vars, &self.om, 200);
//
//         let mut max_pos_error = 0.0;
//         let mut max_rot_error = 0.0;
//         let ee_poses = self.vars.robot.get_ee_pos_and_quat_immutable(&out_x);
//         for i in 0..self.vars.robot.num_chains {
//             let pos_error = (self.vars.goal_positions[i] - ee_poses[i].0).norm();
//             let rot_error = angle_between(self.vars.goal_quats[i].clone(), ee_poses[i].1.clone());
//             if pos_error > max_pos_error { max_pos_error = pos_error; }
//             if rot_error > max_rot_error { max_rot_error = rot_error; }
//         }
//
//         while max_pos_error > 0.005 || max_rot_error > 0.005 {
//             let res = self.solve_randstart(ee_sub);
//             out_x = res.1.clone();
//             max_pos_error = 0.0; max_rot_error = 0.0;
//             let ee_poses = self.vars.robot.get_ee_pos_and_quat_immutable(&out_x);
//             for i in 0..self.vars.robot.num_chains {
//                 let pos_error = (self.vars.goal_positions[i] - ee_poses[i].0).norm();
//                 let rot_error = angle_between(self.vars.goal_quats[i].clone(), ee_poses[i].1.clone());
//                 if pos_error > max_pos_error { max_pos_error = pos_error; }
//                 if rot_error > max_rot_error { max_rot_error = rot_error; }
//             }
//         }
//
//         self.vars.update(out_x.clone());
//         self.vars.update_collision_world();
//
//         out_x
//     }
//
//     pub fn solve_randstart(&mut self, ee_sub: &EEPoseGoalsSubscriber) -> (bool, Vec<f64>) {
//         let mut out_x = self.vars.sampler.sample().data.as_vec().clone();
//
//         if self.vars.rotation_mode_relative {
//             for i in 0..self.vars.robot.num_chains {
//                 self.vars.goal_positions[i] = self.vars.init_ee_positions[i] + ee_sub.pos_goals[i];
//                 self.vars.goal_quats[i] = ee_sub.quat_goals[i] * self.vars.init_ee_quats[i];
//             }
//         } else {
//             for i in 0..self.vars.robot.num_chains  {
//                 self.vars.goal_positions[i] = ee_sub.pos_goals[i].clone();
//                 self.vars.goal_quats[i] = ee_sub.quat_goals[i].clone();
//             }
//         }
//
//         self.groove_nlopt.optimize(&mut out_x, &self.vars, &self.om, 200);
//
//         let mut max_pos_error = 0.0;
//         let mut max_rot_error = 0.0;
//         let ee_poses = self.vars.robot.get_ee_pos_and_quat_immutable(&out_x);
//         for i in 0..self.vars.robot.num_chains {
//             let pos_error = (self.vars.goal_positions[i] - ee_poses[i].0).norm();
//             let rot_error = angle_between(self.vars.goal_quats[i].clone(), ee_poses[i].1.clone());
//             if pos_error > max_pos_error {max_pos_error = pos_error;}
//             if rot_error > max_rot_error {max_rot_error = rot_error;}
//         }
//
//         if max_pos_error > 0.005 || max_rot_error > 0.005 {
//             return (false, out_x)
//         } else {
//             // self.vars.update(out_x.clone());
//             return (true, out_x)
//         }
//     }
// }
