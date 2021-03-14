use crate::groove::objective::*;
use crate::groove::vars::RelaxedIKVars;
use crate::utils::config::*;
use crate::utils::settings::*;

pub struct ObjectiveMaster {
    pub objectives: Vec<Box<dyn ObjectiveTrait + Send>>,
    pub num_chains: usize,
    pub weight_priors: Vec<f64>,
    pub finite_diff_grad: bool,
}

impl ObjectiveMaster {
    pub fn new(config: Config) -> Self {
        let num_chains = config.joint_names.len();
        let mut objectives: Vec<Box<dyn ObjectiveTrait + Send>> = Vec::new();
        let _weight_priors: Vec<f64> = Vec::new();
        for i in 0..config.objectives.len() {
            let objective_spec = &config.objectives[i];
            // Match by variant
            match objective_spec.variant {
                // Position (Standard)
                ObjectiveVariant::PositionMatch => {
                    objectives.push(Box::new(PositionMatch::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                // Orientation (Standard)
                ObjectiveVariant::OrientationMatch => {
                    objectives.push(Box::new(OrientationMatch::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                // Position Liveliness
                ObjectiveVariant::PositionLiveliness => {
                    objectives.push(Box::new(PositionLiveliness::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                // Orientation Liveliness
                ObjectiveVariant::OrientationLiveliness => {
                    objectives.push(Box::new(OrientationLiveliness::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                // Position Mirroring
                ObjectiveVariant::PositionMirroring => {
                    objectives.push(Box::new(PositionMirroring::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                // Orientation Mirroring
                ObjectiveVariant::OrientationMirroring => {
                    objectives.push(Box::new(OrientationMirroring::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                // Position Bounding (TODO)
                ObjectiveVariant::PositionBounding => {
                    objectives.push(Box::new(PositionBounding::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                // Orientation Bounding (TODO)
                ObjectiveVariant::OrientationBounding => {
                    objectives.push(Box::new(OrientationBounding::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                // Joint Matching
                ObjectiveVariant::JointMatch => {
                    objectives.push(Box::new(JointMatch::new(i, objective_spec.indices.clone())));
                }
                // Joint Liveliness
                ObjectiveVariant::JointLiveliness => {
                    objectives.push(Box::new(JointLiveliness::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                // Joint Mirroring
                ObjectiveVariant::JointMirroring => {
                    objectives.push(Box::new(JointMirroring::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                // Joint Limits (Standard)
                ObjectiveVariant::JointLimits => {
                    objectives.push(Box::new(JointLimits));
                }
                // Self-Collision (Standard)
                ObjectiveVariant::NNSelfCollision => {
                    objectives.push(Box::new(NNSelfCollision));
                }
                // Environment Collision (Standard)
                ObjectiveVariant::EnvCollision => {
                    // TODO: FIX, since this argument is the arm index, not objective index
                    objectives.push(Box::new(EnvCollision::new(i)));
                }
                // Velocity Minimization (Standard)
                ObjectiveVariant::MinimizeVelocity => {
                    objectives.push(Box::new(MinimizeVelocity));
                }
                // Acceleration Minimization (Standard)
                ObjectiveVariant::MinimizeAcceleration => {
                    objectives.push(Box::new(MinimizeAcceleration));
                }
                // Jerk Minimization (Standard)
                ObjectiveVariant::MinimizeJerk => {
                    objectives.push(Box::new(MinimizeJerk));
                }
                ObjectiveVariant::RootPositionLiveliness => {
                    objectives.push(Box::new(RootPositionLiveliness::new(i)));
                }
                // Relative Motion Liveliness
                ObjectiveVariant::RelativeMotionLiveliness => {
                    objectives.push(Box::new(RelativeMotionLiveliness::new(
                        i,
                        objective_spec.indices.clone(),
                    )));
                }
                ObjectiveVariant::None => {}
            }
        }

        Self {
            objectives,
            num_chains,
            weight_priors: config.default_weights(),
            finite_diff_grad: true,
        } // fix this
    }

    pub fn tune_weight_priors(&mut self, vars: &RelaxedIKVars) {
        let a = 0.05;
        let cap = 0.001;
        for i in 0..self.num_chains {
            let mut score_max = 0.0;
            for (_option, score) in &vars.env_collision.active_obstacles[i] {
                if *score > score_max {
                    score_max = *score;
                }
            }
            // match ee quat goal objectives
            let weight_cur = self.weight_priors[3 * i + 1];
            let weight_delta = a / (a + score_max) - weight_cur;
            if weight_delta.abs() < cap {
                self.weight_priors[3 * i + 1] += weight_delta;
            } else {
                self.weight_priors[3 * i + 1] += cap * weight_delta / weight_delta.abs();
            }
        }
    }

    pub fn call(&self, x: &[f64], vars: &RelaxedIKVars, is_core: bool) -> f64 {
        self.__call(x, vars, is_core)
    }

    pub fn gradient(&self, x: &[f64], vars: &RelaxedIKVars, is_core: bool) -> (f64, Vec<f64>) {
        if self.finite_diff_grad {
            self.__gradient_finite_diff(x, vars, is_core)
        } else {
            self.__gradient(x, vars, is_core)
        }
    }

    pub fn gradient_finite_diff(
        &self,
        x: &[f64],
        vars: &RelaxedIKVars,
        is_core: bool,
    ) -> (f64, Vec<f64>) {
        self.__gradient_finite_diff(x, vars, is_core)
    }

    fn __call(&self, x: &[f64], vars: &RelaxedIKVars, is_core: bool) -> f64 {
        let mut out = 0.0;
        let frames = vars.robot.get_frames(x);
        for i in 0..self.objectives.len() {
            out += self.weight_priors[i] * self.objectives[i].call(x, vars, &frames, is_core);
        }
        out
    }

    fn __gradient(&self, x: &[f64], vars: &RelaxedIKVars, is_core: bool) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let mut obj = 0.0;

        let mut finite_diff_list: Vec<usize> = Vec::new();
        let mut f_0s: Vec<f64> = Vec::new();
        let frames_0 = vars.robot.get_frames(x);
        for i in 0..self.objectives.len() {
            if self.objectives[i].gradient_type() == 1 {
                let (local_obj, local_grad) =
                    self.objectives[i].gradient(x, vars, &frames_0, is_core);
                f_0s.push(local_obj);
                obj += self.weight_priors[i] * local_obj;
                for j in 0..local_grad.len() {
                    grad[j] += self.weight_priors[i] * local_grad[j];
                }
            } else if self.objectives[i].gradient_type() == 0 {
                finite_diff_list.push(i);
                let local_obj = self.objectives[i].call(x, vars, &frames_0, is_core);
                obj += self.weight_priors[i] * local_obj;
                f_0s.push(local_obj);
            }
        }

        if finite_diff_list.len() > 0 {
            for i in 0..x.len() {
                let mut x_h = x.to_vec();
                x_h[i] += 0.0000001;
                let frames_h = vars.robot.get_frames(x_h.as_slice());
                for j in &finite_diff_list {
                    let f_h = self.objectives[*j].call(x, vars, &frames_h, is_core);
                    grad[i] += self.weight_priors[*j] * ((-f_0s[*j] + f_h) / 0.0000001);
                }
            }
        }

        (obj, grad)
    }

    fn __gradient_finite_diff(
        &self,
        x: &[f64],
        vars: &RelaxedIKVars,
        is_core: bool,
    ) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let f_0 = self.call(x, vars, is_core);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.call(x_h.as_slice(), vars, is_core);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }

        (f_0, grad)
    }
}
