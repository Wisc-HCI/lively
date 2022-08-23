use serde::{Serialize, Deserialize};
use nalgebra::{Vector3};
use crate::objectives::objective::groove_loss;
use crate::utils::vars::Vars;
use crate::utils::state::State;


#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct CollisionAvoidanceObjective {
    pub name: String,
    pub weight: f64
}

impl CollisionAvoidanceObjective {

    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }

    pub fn call(
        &self,
        _v: &Vars,
        state: &State,
        _is_core: bool
    ) -> f64 {
        let mut score: f64 = 0.0;
        for proximity_info in &state.proximity {
            if proximity_info.physical {
                score += proximity_info.loss
            }
        }
        //  println!("score is : {:?}" , score);
        return self.weight * score;
       
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct JointLimitsObjective {
    pub name: String,
    pub weight: f64
}

impl JointLimitsObjective {
    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        _is_core: bool
    ) -> f64 {
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.9;
        let a: f64 = 0.05 / (penalty_cutoff.powi(50));
        for joint in v.joints.iter() {
            let l: f64 = joint.lower_bound;
            let u: f64 = joint.upper_bound;
            let joint_value = state.get_joint_position(&joint.name);
            if u - l <= 0.0 {
                // In cases where the upper and lower limits are the same,
                // just compare the lower limit to the x value.
                sum += a * (joint_value - l).abs().powi(50);
            } else {
                // Otherwise, compare as normal
                let r: f64 = (joint_value - l) / (u - l);
                let n: f64 = 2.0 * (r - 0.5);
                sum += a * n.powi(50);
            }
        }
        // println!("JointLimits error: {:?}",sum);
        return self.weight * groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2)
    }
}

impl JointLimitsObjective {
    
    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct VelocityMinimizationObjective {
    pub name: String,
    pub weight: f64
}

impl VelocityMinimizationObjective {

    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }

    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool
    ) -> f64 {
        let mut x_val = 0.0;
        for joint in v.joints.iter() {
            let joint_value: f64 = state.get_joint_position(&joint.name);
            if is_core {
                x_val += (joint_value - v.history_core.prev1.get_joint_position(&joint.name)).powi(2);
            } else {
                x_val += (joint_value - v.history.prev1.get_joint_position(&joint.name)).powi(2);
            }
        }
        x_val = x_val.sqrt();
        // println!("VelocityMinimizationObjective error: {:?}",x_val);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct OriginVelocityMinimizationObjective {
    pub name: String,
    pub weight: f64
}

impl OriginVelocityMinimizationObjective {

    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }

    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool
    ) -> f64 {
        let past:Vector3<f64>;
        let x_val:f64;
        if is_core {
            past = v.history_core.prev1.origin.translation.vector
        } else {
            past = v.history.prev1.origin.translation.vector
        }
        x_val = (state.origin.translation.vector-past).norm().powi(2);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct AccelerationMinimizationObjective {
    pub name: String,
    pub weight: f64
}

impl AccelerationMinimizationObjective {

    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }

    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool
    ) -> f64 {
        let mut x_val = 0.0;
        for joint in v.joints.iter() {
            let joint_value: f64 = state.get_joint_position(&joint.name);
            let v1: f64;
            let v2: f64;
            if is_core {
                v1 = joint_value - v.history_core.prev1.get_joint_position(&joint.name);
                v2 = v.history_core.prev1.get_joint_position(&joint.name) - v.history_core.prev2.get_joint_position(&joint.name);
            } else {
                v1 = joint_value - v.history.prev1.get_joint_position(&joint.name);
                v2 = v.history.prev1.get_joint_position(&joint.name) - v.history.prev2.get_joint_position(&joint.name);
            }
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        // println!("AccelerationMinimizationObjective error: {:?}",x_val);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct OriginAccelerationMinimizationObjective {
    pub name: String,
    pub weight: f64
}

impl OriginAccelerationMinimizationObjective {

    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }

    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool
    ) -> f64 {
        let pos1 = state.origin.translation.vector;
        if is_core {
            let pos2: Vector3<f64> = v.history_core.prev1.origin.translation.vector;
            let pos3: Vector3<f64> = v.history_core.prev2.origin.translation.vector;
            let x_val: f64 = ((pos1-pos2)-(pos2-pos3)).norm().powi(2);
            return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
        } else {
            let pos2: Vector3<f64> = v.history.prev1.origin.translation.vector;
            let pos3: Vector3<f64> = v.history.prev2.origin.translation.vector;
            let x_val: f64 = ((pos1-pos2)-(pos2-pos3)).norm().powi(2);
            return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
        }
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct JerkMinimizationObjective {
    pub name: String,
    pub weight: f64
}

impl JerkMinimizationObjective {

    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }

    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool
    ) -> f64 {
        let mut x_val = 0.0;
        for joint in v.joints.iter() {
            let joint_value: f64 = state.get_joint_position(&joint.name);
            let v1: f64;
            let v2: f64;
            let v3: f64;
            if is_core {
                v1 = joint_value - v.history_core.prev1.get_joint_position(&joint.name);
                v2 = v.history_core.prev1.get_joint_position(&joint.name) - v.history_core.prev2.get_joint_position(&joint.name);
                v3 = v.history_core.prev2.get_joint_position(&joint.name) - v.history_core.prev3.get_joint_position(&joint.name);
            } else {
                v1 = joint_value - v.history.prev1.get_joint_position(&joint.name);
                v2 = v.history.prev1.get_joint_position(&joint.name) - v.history.prev2.get_joint_position(&joint.name);
                v3 = v.history.prev2.get_joint_position(&joint.name) - v.history.prev3.get_joint_position(&joint.name);
            }
            x_val += ((v1 - v2) - (v2 - v3)).powi(2);
        }
        x_val = x_val.sqrt();
        // println!("JerkMinimizationObjective error: {:?}",x_val);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug,Default)]
pub struct OriginJerkMinimizationObjective {
    pub name: String,
    pub weight: f64
}

impl OriginJerkMinimizationObjective {

    pub fn new(name: String, weight: f64) -> Self {
        Self {name, weight}
    }

    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool
    ) -> f64 {
        let x_val: f64;
        let pos1 = state.origin.translation.vector;
        let pos2:Vector3<f64>;
        let pos3:Vector3<f64>;
        let pos4:Vector3<f64>;
        if is_core {
            pos2 = v.history_core.prev1.origin.translation.vector;
            pos3 = v.history_core.prev2.origin.translation.vector;
            pos4 = v.history_core.prev3.origin.translation.vector;
        } else {
            pos2 = v.history.prev1.origin.translation.vector;
            pos3 = v.history.prev2.origin.translation.vector;
            pos4 = v.history.prev3.origin.translation.vector;
        }
        x_val = (((pos1-pos2) - (pos2-pos3)) - ((pos2-pos3) - (pos3-pos4))).norm().powi(2);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

#[repr(C)]
#[derive(Serialize,Deserialize,Clone,Debug)]
pub struct SmoothnessMacroObjective {
    pub name: String,
    pub weight: f64,
    #[serde(skip_serializing,default = "SmoothnessMacroObjective::default_velocity_objective")]
    velocity_objective: VelocityMinimizationObjective,
    #[serde(skip_serializing,default = "SmoothnessMacroObjective::default_acceleration_objective")]
    acceleration_objective: AccelerationMinimizationObjective,
    #[serde(skip_serializing,default = "SmoothnessMacroObjective::default_jerk_objective")]
    jerk_objective: JerkMinimizationObjective,
    #[serde(skip_serializing,default = "SmoothnessMacroObjective::default_origin_velocity_objective")]
    base_velocity_objective: OriginVelocityMinimizationObjective,
    #[serde(skip_serializing,default = "SmoothnessMacroObjective::default_origin_acceleration_objective")]
    base_acceleration_objective: OriginAccelerationMinimizationObjective,
    #[serde(skip_serializing,default = "SmoothnessMacroObjective::default_origin_jerk_objective")]
    base_jerk_objective: OriginJerkMinimizationObjective
}

impl SmoothnessMacroObjective {

    pub fn new(name: String, weight: f64) -> Self {
        Self {
            name: name.clone(),
            weight,
            velocity_objective: VelocityMinimizationObjective::new(format!("Macro {} Velocity",name),0.21),
            acceleration_objective: AccelerationMinimizationObjective::new(format!("Macro {} Accel",name),0.08),
            jerk_objective: JerkMinimizationObjective::new(format!("Macro {} Jerk",name),0.04),
            base_velocity_objective: OriginVelocityMinimizationObjective::new(format!("Macro {} Origin Velocity",name),0.47),
            base_acceleration_objective: OriginAccelerationMinimizationObjective::new(format!("Macro {} Origin Accel",name),0.15),
            base_jerk_objective: OriginJerkMinimizationObjective::new(format!("Macro {} Origin Jerk",name),0.05)
        }
    }

    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool
    ) -> f64 {
        let velocity_cost = self.velocity_objective.call(v, state, is_core);
        let acceleration_cost = self.acceleration_objective.call(v, state, is_core);
        let jerk_cost = self.jerk_objective.call(v, state, is_core);
        let base_velocity_cost = self.base_velocity_objective.call(v, state, is_core);
        let base_acceleration_cost = self.base_acceleration_objective.call(v, state, is_core);
        let base_jerk_cost = self.base_jerk_objective.call(v, state, is_core);
        return self.weight * (velocity_cost + acceleration_cost + jerk_cost +
               base_velocity_cost + base_acceleration_cost + base_jerk_cost);
    }

    pub fn default_velocity_objective() -> VelocityMinimizationObjective {
        VelocityMinimizationObjective::new("Macro Velocity".to_string(), 0.21)
    }

    pub fn default_acceleration_objective() -> AccelerationMinimizationObjective {
        AccelerationMinimizationObjective::new("Macro Acceleration".to_string(), 0.08)
    }

    pub fn default_jerk_objective() -> JerkMinimizationObjective {
        JerkMinimizationObjective::new("Macro Jerk".to_string(), 0.04)
    }

    pub fn default_origin_velocity_objective() -> OriginVelocityMinimizationObjective {
        OriginVelocityMinimizationObjective::new("Macro Origin Velocity".to_string(), 0.47)
    }

    pub fn default_origin_acceleration_objective() -> OriginAccelerationMinimizationObjective {
        OriginAccelerationMinimizationObjective::new("Macro Origin Acceleration".to_string(), 0.15)
    }

    pub fn default_origin_jerk_objective() -> OriginJerkMinimizationObjective {
        OriginJerkMinimizationObjective::new("Macro Origin Jerk".to_string(), 0.05)
    }
}
