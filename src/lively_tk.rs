use crate::groove::groove::{OptimizationEngineNLopt, OptimizationEngineOpen};
use crate::groove::objective_master::ObjectiveMaster;
use crate::groove::vars::RelaxedIKVars;
use pyo3::prelude::*;
use crate::utils::config::{Config, EnvironmentSpec};
use crate::utils::goals::{Goal, ObjectiveInput};
use rand::{thread_rng, Rng};
use std::os::raw::{c_double, c_int};

#[repr(C)]
pub struct Opt {
    pub data: *const c_double,
    pub length: c_int,
}

#[pyclass]
pub struct Solver {
    pub config: Config,
    pub vars: RelaxedIKVars,
    pub om: ObjectiveMaster,
    pub groove: OptimizationEngineOpen,
    pub groove_nlopt: OptimizationEngineNLopt,
}

#[pymethods]
impl Solver {
    #[new]
    fn new(config: Config) -> Self {
        // println!("Creating RelaxedIKVars");
        let vars = RelaxedIKVars::new(config.clone());
        // println!("Creating ObjectiveMaster");
        let om = ObjectiveMaster::new(config.clone());
        // println!("Creating OptimizationEngine");
        let groove = OptimizationEngineOpen::new(vars.robot.num_dof.clone() + 3);
        let groove_nlopt = OptimizationEngineNLopt::new();

        Self {
            config,
            vars,
            om,
            groove,
            groove_nlopt,
        }
    }

    fn reset(
        &mut self, 
        base_offset:Vec<f64>,
        joint_state:Vec<f64>
    ) -> PyResult<()> {
        let mut full_state = base_offset.clone();
        for i in 0..joint_state.len() {
            full_state.push(joint_state[i])
        }
        let frames = self.vars.robot.get_frames(&full_state);
        for _i in 0..3 {
            self.vars.history_core.update(full_state.clone());
            self.vars.history.update(full_state.clone());
            self.vars.xopt_core = joint_state.clone();
            self.vars.xopt = joint_state.clone();
            self.vars.frames_core = frames.clone();
            self.vars.offset_core = base_offset.clone();
            self.vars.offset = base_offset.clone();
        }
        self.om.weight_priors = self.config.default_weights();

        Ok(())
    }

    fn solve(
        &mut self,
        goals: Vec<ObjectiveInput>,
        time: f64,
        world: Option<EnvironmentSpec>,
        max_retries: Option<u64>,
        max_iterations: Option<usize>,
        only_core: Option<bool>
    ) -> PyResult<(Vec<f64>, Vec<f64>)> {
        // RNG Gen
        let mut rng = thread_rng();

        // Will be the output of solve. N=num_dof
        let mut out_x = self.vars.xopt.clone();
        // Will be the output of solving for core. N=num_dof
        let mut out_x_core = self.vars.xopt_core.clone();
        // Will be the output of the optimization. N=num_dof+3
        let mut xopt = self.vars.offset.clone();
        // Will be the output of the core optimization. N=num_dof+3
        let mut xopt_core = self.vars.offset_core.clone();

        let mut run_only_core = false;
        match only_core {
            Some(v) => run_only_core = v,
            None => {}
        }

        for i in 0..out_x.len() {
            xopt.push(out_x[i])
        }

        for i in 0..out_x_core.len() {
            xopt_core.push(out_x_core[i])
        }

        if goals.len() != self.config.objectives.len() {
            println!(
                "Mismatch between goals (n={:?}) and objectives (n={:?})",
                goals.len(),
                self.config.objectives.len()
            );
            return Ok((self.vars.offset.clone(), out_x));
        }

        for goal_idx in 0..goals.clone().len() {
            self.om.weight_priors[goal_idx] = goals[goal_idx].weight;
            match goals[goal_idx].value {
                // Only update if the goal is specified.
                Goal::None => {}
                _ => self.vars.goals[goal_idx].value = goals[goal_idx].value,
            }
        }

        // println!("Updated Goals {:?}",self.vars.goals);

        self.vars.liveliness.update(time);
        // println!("Lively Goals: {:?}",self.vars.liveliness.goals);

        match world {
            // Update the collision world
            Some(_env) => {}
            // Keep the same one as previous
            None => {}
        }

        let in_collision = false; //self.vars.update_collision_world();
        if !in_collision {
            // if self.config.mode_environment == EnvironmentMode::ECAA {
            //     // Right now, doing this causes errors because vars.env_collision.active_obstacles isn't populated
            //     self.om.tune_weight_priors(&self.vars);
            // }

            let mut best_xopt_core = xopt_core.clone();
            let mut best_cost = f64::INFINITY.clone();

            // If precise, run until error is sufficiently low, or the max_retries is met
            let max_tries: u64;
            match max_retries {
                Some(value) => max_tries = value + 1,
                None => max_tries = 1,
            }
            let max_iter: usize;
            match max_iterations {
                Some(value) => max_iter = value,
                None => max_iter = 150,
            }

            let mut try_count = 0;

            // Run without liveliness (core objectives)
            // Run until the max_retries is met, or the solution could be improved
            while try_count < max_tries && (try_count == 0 || best_cost > 250.0) {
                let try_cost =
                    self.groove
                        .optimize(&mut xopt_core, &self.vars, &self.om, max_iter, true);
                if try_cost < best_cost {
                    best_xopt_core = xopt_core.clone();
                    best_cost = try_cost;
                }
                try_count += 1;

                // Randomly generate a new starting point
                for i in 0..xopt_core.len() {
                    if self.vars.robot.upper_bounds[i] - self.vars.robot.lower_bounds[i] <= 0.0 {
                        xopt_core[i] = self.vars.robot.lower_bounds[i]
                    } else {
                        xopt_core[i] = rng.gen_range(
                            self.vars.robot.lower_bounds[i]..self.vars.robot.upper_bounds[i],
                        );
                    }
                }
            }
            // println!("Best cost (core): {:?}",best_cost);
            for i in 0..out_x_core.len() {
                out_x_core[i] = best_xopt_core[i + 3];
            }

            self.vars.xopt_core = out_x_core.clone();
            self.vars.history_core.update(best_xopt_core.clone());
            self.vars.offset_core = vec![best_xopt_core[0], best_xopt_core[1], best_xopt_core[2]];
            self.vars.frames_core = self.vars.robot.get_frames(&best_xopt_core.clone());

            if run_only_core {
                self.vars.xopt = out_x_core.clone();
                self.vars.history.update(best_xopt_core.clone());
                self.vars.offset = vec![best_xopt_core[0], best_xopt_core[1], best_xopt_core[2]];
            } else {
                // Run with liveliness (all objectives)
                let mut best_xopt = xopt.clone();
                best_cost = f64::INFINITY.clone();
                try_count = 0;

                // Run until the max_retries is met, or the solution could be improved
                while try_count < max_tries && (try_count == 0 || best_cost > 250.0) {
                    let try_cost = self
                        .groove
                        .optimize(&mut xopt, &self.vars, &self.om, max_iter, false);
                    if try_cost < best_cost {
                        best_xopt = xopt.clone();
                        best_cost = try_cost;
                    }
                    try_count += 1;
                    // Randomly generate a new starting point
                    for i in 0..xopt.len() {
                        if self.vars.robot.upper_bounds[i] - self.vars.robot.lower_bounds[i] <= 0.0 {
                            xopt[i] = self.vars.robot.lower_bounds[i]
                        } else {
                            xopt[i] = rng.gen_range(
                                self.vars.robot.lower_bounds[i]..self.vars.robot.upper_bounds[i],
                            );
                        }
                    }
                }
                // println!("Best cost: {:?}",best_cost);
                for i in 0..out_x.len() {
                    out_x[i] = best_xopt[i + 3];
                }

                self.vars.xopt = out_x.clone();
                self.vars.history.update(best_xopt.clone());
                self.vars.offset = vec![best_xopt[0], best_xopt[1], best_xopt[2]]
            }

        }

        // println!("OUTX-CORE {:?},\nOUTX {:?}",xopt_core,xopt);

        return Ok((self.vars.offset.clone(), self.vars.xopt.clone()));
    }
}