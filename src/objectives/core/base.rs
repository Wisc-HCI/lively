use crate::groove::vars::RelaxedIKVars;
use crate::groove::objective::{ObjectiveTrait, groove_loss, groove_loss_derivative};
use nalgebra::geometry::{Point3, UnitQuaternion};
use nalgebra::{one, Vector3};
use ncollide3d::{query, shape};
use std::ops::Deref;

pub struct NNSelfCollision;
impl ObjectiveTrait for NNSelfCollision {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let x_val = v.collision_nn.predict(&x.to_vec());
        groove_loss(x_val, 0., 2, 2.1, 0.0002, 4)
    }

    fn gradient(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> (f64, Vec<f64>) {
        let (x_val, mut grad) = v.collision_nn.gradient(&x.to_vec());
        let g_prime = groove_loss_derivative(x_val, 0., 2, 2.1, 0.0002, 4);
        for i in 0..grad.len() {
            grad[i] *= g_prime;
        }
        (x_val, grad)
    }

    fn gradient_type(&self) -> usize {
        return 0;
    }
}

pub struct EnvCollision {
    pub arm_idx: usize,
}
impl EnvCollision {
    pub fn new(arm_idx: usize) -> Self {
        Self { arm_idx }
    }
}
impl ObjectiveTrait for EnvCollision {
    fn call(
        &self,
        _x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        // let start = PreciseTime::now();\
        let mut x_val: f64 = 0.0;
        let link_radius = v.env_collision.link_radius;
        let penalty_cutoff: f64 = link_radius * 2.0;
        let a = penalty_cutoff.powi(2);
        for (option, _score) in &v.env_collision.active_obstacles[self.arm_idx] {
            if let Some(handle) = option {
                let mut sum: f64 = 0.0;
                let obstacle = v.env_collision.world.objects.get(*handle).unwrap();
                let last_elem = frames[self.arm_idx].0.len() - 1;
                for i in 0..last_elem {
                    let start_pt = Point3::from(frames[self.arm_idx].0[i]);
                    let end_pt = Point3::from(frames[self.arm_idx].0[i + 1]);
                    let segment = shape::Segment::new(start_pt, end_pt);
                    let segment_pos = one();
                    let dis = query::distance(
                        obstacle.position(),
                        obstacle.shape().deref(),
                        &segment_pos,
                        &segment,
                    ) - link_radius;
                    // println!("Obstacle: {}, Link: {}, Distance: {:?}", obstacle.data().name, i, dis);
                    sum += a / (dis + link_radius).powi(2);
                }
                // println!("OBJECTIVE -> {:?}, Sum: {:?}", obstacle.data().name, sum);
                x_val += sum;
            }
        }

        // let end = PreciseTime::now();
        // println!("Obstacles calculating takes {}", start.to(end));
        // println!("EnvCollision error: {:?}",x_val);
        groove_loss(x_val, 0., 2, 3.5, 0.00005, 4)
    }
}

pub struct JointLimits;
impl ObjectiveTrait for JointLimits {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        _is_core: bool,
    ) -> f64 {
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.9;
        let a = 0.05 / (penalty_cutoff.powi(50));
        for i in 0..v.robot.lower_bounds.len() {
            let l = v.robot.lower_bounds[i];
            let u = v.robot.upper_bounds[i];
            if u - l <= 0.0 {
                // In cases where the upper and lower limits are the same,
                // just compare the lower limit to the x value.
                sum += a * (x[i] - l).abs().powi(50);
            } else {
                // Otherwise, compare as normal
                let r = (x[i] - l) / (u - l);
                let n = 2.0 * (r - 0.5);
                sum += a * n.powi(50);
            }
        }
        // println!("JointLimits error: {:?}",sum);
        groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2)
    }
}

pub struct MinimizeVelocity;
impl ObjectiveTrait for MinimizeVelocity {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 3..x.len() {
            if is_core {
                x_val += (x[i] - v.history_core.prev1[i]).powi(2);
            } else {
                x_val += (x[i] - v.history.prev1[i]).powi(2);
            }
        }
        x_val = x_val.sqrt();
        // println!("MinimizeVelocity error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeBaseVelocity;
impl ObjectiveTrait for MinimizeBaseVelocity {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let pos1:Vector3<f64> = Vector3::new(x[0],x[1],x[2]);
        let pos2:Vector3<f64>;
        let x_val:f64;
        if is_core {
            pos2 = Vector3::new(v.history_core.prev1[0],
                                v.history_core.prev1[1],
                                v.history_core.prev1[2])

        } else {
            pos2 = Vector3::new(v.history.prev1[0],
                                v.history.prev1[1],
                                v.history.prev1[2]);
        }
        x_val = (pos1-pos2).norm().powi(2);
        // println!("MinimizeVelocity error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeAcceleration;
impl ObjectiveTrait for MinimizeAcceleration {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 3..x.len() {
            let v1: f64;
            let v2: f64;
            if is_core {
                v1 = x[i] - v.history_core.prev1[i];
                v2 = v.history_core.prev1[i] - v.history_core.prev2[i];
            } else {
                v1 = x[i] - v.history.prev1[i];
                v2 = v.history.prev1[i] - v.history.prev2[i];
            }
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        // println!("MinimizeAcceleration error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeBaseAcceleration;
impl ObjectiveTrait for MinimizeBaseAcceleration {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let x_val: f64;
        let pos1:Vector3<f64> = Vector3::new(x[0],x[1],x[2]);
        let pos2:Vector3<f64>;
        let pos3:Vector3<f64>;
        if is_core {
            pos2 = Vector3::new(v.history_core.prev1[0],
                                v.history_core.prev1[1],
                                v.history_core.prev1[2]);
            pos3 = Vector3::new(v.history_core.prev2[0],
                                v.history_core.prev2[1],
                                v.history_core.prev2[2]);
        } else {
            pos2 = Vector3::new(v.history.prev1[0],
                                v.history.prev1[1],
                                v.history.prev1[2]);
            pos3 = Vector3::new(v.history.prev2[0],
                                v.history.prev2[1],
                                v.history.prev2[2]);
        }
        x_val = ((pos1-pos2)-(pos2-pos3)).norm().powi(2);
        // println!("MinimizeAcceleration error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeJerk;
impl ObjectiveTrait for MinimizeJerk {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let mut x_val = 0.0;
        for i in 3..x.len() {
            let v1: f64;
            let v2: f64;
            let v3: f64;
            if is_core {
                v1 = x[i] - v.history_core.prev1[i];
                v2 = v.history_core.prev1[i] - v.history_core.prev2[i];
                v3 = v.history_core.prev2[i] - v.history_core.prev3[i];
            } else {
                v1 = x[i] - v.history.prev1[i];
                v2 = v.history.prev1[i] - v.history.prev2[i];
                v3 = v.history.prev2[i] - v.history.prev3[i];
            }
            x_val += ((v1 - v2) - (v2 - v3)).powi(2);
        }
        x_val = x_val.sqrt();
        // println!("MinimizeJerk error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeBaseJerk;
impl ObjectiveTrait for MinimizeBaseJerk {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        _frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let x_val: f64;
        let pos1:Vector3<f64> = Vector3::new(x[0],x[1],x[2]);
        let pos2:Vector3<f64>;
        let pos3:Vector3<f64>;
        let pos4:Vector3<f64>;
        if is_core {
            pos2 = Vector3::new(v.history_core.prev1[0],
                                v.history_core.prev1[1],
                                v.history_core.prev1[2]);
            pos3 = Vector3::new(v.history_core.prev2[0],
                                v.history_core.prev2[1],
                                v.history_core.prev2[2]);
            pos4 = Vector3::new(v.history_core.prev3[0],
                                v.history_core.prev3[1],
                                v.history_core.prev3[2]);
        } else {
            pos2 = Vector3::new(v.history.prev1[0],
                                v.history.prev1[1],
                                v.history.prev1[2]);
            pos3 = Vector3::new(v.history.prev2[0],
                                v.history.prev2[1],
                                v.history.prev2[2]);
            pos4 = Vector3::new(v.history_core.prev3[0],
                                v.history_core.prev3[1],
                                v.history_core.prev3[2]);
        }
        x_val = (((pos1-pos2) - (pos2-pos3)) - ((pos2-pos3) - (pos3-pos4))).norm().powi(2);
        // println!("MinimizeJerk error: {:?}",x_val);
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MacroSmoothness {
    velocity_objective: MinimizeVelocity,
    acceleration_objective: MinimizeAcceleration,
    jerk_objective: MinimizeJerk,
    base_velocity_objective: MinimizeBaseVelocity,
    base_acceleration_objective: MinimizeBaseAcceleration,
    base_jerk_objective: MinimizeBaseJerk,
}

impl MacroSmoothness {
    pub fn new() -> Self {
        Self {
            velocity_objective: MinimizeVelocity,
            acceleration_objective: MinimizeAcceleration,
            jerk_objective: MinimizeJerk,
            base_velocity_objective: MinimizeBaseVelocity,
            base_acceleration_objective: MinimizeBaseAcceleration,
            base_jerk_objective: MinimizeBaseJerk,
        }
    }
}

impl ObjectiveTrait for MacroSmoothness {
    fn call(
        &self,
        x: &[f64],
        v: &RelaxedIKVars,
        frames: &Vec<(Vec<Vector3<f64>>, Vec<UnitQuaternion<f64>>)>,
        is_core: bool,
    ) -> f64 {
        let velocity_cost = self.velocity_objective.call(x, v, frames, is_core);
        let acceleration_cost = self.acceleration_objective.call(x, v, frames, is_core);
        let jerk_cost = self.jerk_objective.call(x, v, frames, is_core);
        let base_velocity_cost = self.base_velocity_objective.call(x, v, frames, is_core);
        let base_acceleration_cost = self.base_acceleration_objective.call(x, v, frames, is_core);
        let base_jerk_cost = self.base_jerk_objective.call(x, v, frames, is_core);
        return 7.0 * velocity_cost + 2.0 * acceleration_cost + jerk_cost +
               14.0 * base_velocity_cost + 4.0 * base_acceleration_cost + 2.0 * base_jerk_cost;
    }
}
