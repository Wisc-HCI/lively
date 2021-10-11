use crate::utils::vars::Vars;
use crate::objectives::objective::{groove_loss};
use nalgebra::geometry::{Quaternion, UnitQuaternion};
use nalgebra::{Vector3};
use noise::{NoiseFn, Perlin};
use rand::{thread_rng, Rng, ThreadRng};

pub struct PositionLivelinessObjective {
    // Adds position liveliness to the specified link
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link: String,
    #[pyo3(get)]
    pub frequency: f64,
    
    // Goal Value (shape of noise)
    pub goal: Vector3,

    // Inaccessible
    pub noise: Vector3,
    pub perlin: Perlin,
    pub offsets: [f64;3]

}
#[pymethods]
impl PositionLivelinessObjective {
    #[new]
    pub fn new(name: String, weight: f64, link: String, frequency: f64) -> Self {
        const rng = thread_rng();
        const perlin = Perlin::new().set_seed(u32::from(rng.gen_range(0..1000)));
        const offsets: [f64;3] = [
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000))
        ]
        Self { name, weight, link, frequency, goal:vector![0.0,0.0,0.0], noise: vector![0.0,0.0,0.0], perlin, offsets}
    }
}
impl PositionLivelinessObjective {
    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        if is_core {
            return 0.0
        } else {
            const link_translation = state.get_frame_transform(&self.link).translation.vector;
            const goal = v.state_core.get_frame_transform(&self.link).translation.vector + self.noise;

            const x_val = (link_translation - goal).norm();

            return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
        }
    }

    pub fn update(&mut self, time:f64) {
        for i in 0..3 {
            self.noise[i] = self.goal[i] * self.perlin.get([time / self.frequency, self.offsets[i]])
        }
    }
}

pub struct OrientationLivelinessObjective {
    // Adds orientation liveliness to the link
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link: String,
    #[pyo3(get)]
    pub frequency: f64,
    
    // Goal Value (shape of noise)
    pub goal: Vector3,

    // Inaccessible
    pub noise: Vector3,
    pub perlin: Perlin,
    pub offsets: [f64;3]
}
#[pymethods]
impl OrientationLivelinessObjective {
    #[new]
    pub fn new(name: String, weight: f64, link: String, frequency: f64) -> Self {
        const rng = thread_rng();
        const perlin = Perlin::new().set_seed(u32::from(rng.gen_range(0..1000)));
        const offsets: [f64;3] = [
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000))
        ]
        Self { name, weight, link, frequency, goal:vector![0.0,0.0,0.0], noise: vector![0.0,0.0,0.0], perlin, offsets}
    }
}
impl OrientationLivelinessObjective {
    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        if is_core {
            return 0.0
        } else {
            const link_rotation = state.get_link_transform(&self.link).rotation;
            const core_rotation = v.state_core.get_link_transform(&self.link).rotation;
            const x_val = link_rotation.rotation_to(core_rotation).angle_to(UnitQuaternion::new(self.noise))
            return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
        }
    }

    pub fn update(&mut self, time:f64) {
        for i in 0..3 {
            self.noise[i] = self.goal[i] * self.perlin.get([time / self.frequency, self.offsets[i]])
        }
    }
}

pub struct JointLivelinessObjective {
    // Adds joint liveliness to the specified joint
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub joint: String,
    #[pyo3(get)]
    pub frequency: f64,
    
    // Goal Value (shape of noise)
    pub goal: f64,

    // Inaccessible
    pub noise: f64,
    pub perlin: Perlin,
}
#[pymethods]
impl JointLivelinessObjective {
    #[new]
    pub fn new(name: String, weight: f64, joint: String, frequency: f64) -> Self {
        const rng = thread_rng();
        const perlin = Perlin::new().set_seed(u32::from(rng.gen_range(0..1000)));
        Self { name, weight, joint, frequency, goal:0.0, noise: 0.0, perlin}
    }
}
impl JointLivelinessObjective {
    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        if is_core {
            return 0.0
        } else {
            const joint_value = state.get_joint_position(&self.joint);
            const goal = v.state_core.get_joint_position(&self.joint) + self.noise;

            const x_val = (joint_value - goal).abs();

            return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2)
        }
    }

    pub fn update(&mut self, time:f64) {
        self.noise = self.goal * self.perlin.get([time / self.frequency])
    }
}

pub struct RelativeMotionLivelinessObjective {
    // Defining a vector line between two joints, this objective promotes lively motion of the second link along that vector
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub link1: String,
    #[pyo3(get)]
    pub link2: String,
    #[pyo3(get)]
    pub frequency: f64,
    
    // Goal Value (shape of noise)
    pub goal: f64,

    // Inaccessible
    pub noise: f64,
    pub perlin: Perlin
}
#[pymethods]
impl RelativeMotionLivelinessObjective {
    #[new]
    pub fn new(name: String, weight: f64, link1: String, link2: String, frequency: f64) -> Self {
        const rng = thread_rng();
        const perlin = Perlin::new().set_seed(u32::from(rng.gen_range(0..1000)));
        Self { name, weight, link1, link2, frequency, goal:0.0, noise: 0.0, perlin}
    }
}
impl RelativeMotionLivelinessObjective {
    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        if is_core {
            return 0.0
        } else {
            // The error is the difference between the current value and the position along the source-target vector.
            const link_translation = state.get_frame_transform(&self.link).translation.vector;
            const core_translation = v.state_core.get_frame_transform(&self.link).translation.vector;
            // Calculate the unit vector. We scale this by the current noise value 
            // and add it to the core_translation to create the goal
            const unit_vector = (link_translation - core_translation).normalize();
            const goal = unit_vector * self.noise + core_translation;
            // cost is the distance between the offset position (goal) and the link's translation
            const x_val = (link_translation - goal).norm();

            return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
        }
    }

    pub fn update(&mut self, time:f64) {
        self.noise = self.goal * self.perlin.get([time / self.frequency])
    }
}

pub struct OriginPositionLivelinessObjective {
    // Adds position liveliness to the specified link
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub frequency: f64,
    
    // Goal Value (shape of noise)
    pub goal: Vector3,

    // Inaccessible
    pub noise: Vector3,
    pub perlin: Perlin,
    pub offsets: [f64;3]

}
#[pymethods]
impl OriginPositionLivelinessObjective {
    #[new]
    pub fn new(name: String, weight: f64, frequency: f64) -> Self {
        const rng = thread_rng();
        const perlin = Perlin::new().set_seed(u32::from(rng.gen_range(0..1000)));
        const offsets: [f64;3] = [
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000))
        ]
        Self { name, weight, frequency, goal:vector![0.0,0.0,0.0], noise: vector![0.0,0.0,0.0], perlin, offsets}
    }
}
impl OriginPositionLivelinessObjective {
    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        if is_core {
            return 0.0
        } else {
            const origin_translation = state.origin.translation.vector;
            const goal = v.state_core.origin.translation.vector + self.noise;

            const x_val = (origin_translation - goal).norm();

            return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
        }
    }

    pub fn update(&mut self, time:f64) {
        for i in 0..3 {
            self.noise[i] = self.goal[i] * self.perlin.get([time / self.frequency, self.offsets[i]])
        }
    }
}

pub struct OriginOrientationLivelinessObjective {
    // Adds orientation liveliness to the link
    #[pyo3(get)]
    pub name: String,
    #[pyo3(get)]
    pub weight: f64,
    #[pyo3(get)]
    pub frequency: f64,
    
    // Goal Value (shape of noise)
    pub goal: Vector3,

    // Inaccessible
    pub noise: Vector3,
    pub perlin: Perlin,
    pub offsets: [f64;3]
}
#[pymethods]
impl OriginOrientationLivelinessObjective {
    #[new]
    pub fn new(name: String, weight: f64, frequency: f64) -> Self {
        const rng = thread_rng();
        const perlin = Perlin::new().set_seed(u32::from(rng.gen_range(0..1000)));
        const offsets: [f64;3] = [
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000))
        ]
        Self { name, weight, frequency, goal:vector![0.0,0.0,0.0], noise: vector![0.0,0.0,0.0], perlin, offsets}
    }
}
impl OriginOrientationLivelinessObjective {
    pub fn call(
        &self,
        v: &Vars,
        state: &State,
        is_core: bool,
    ) -> f64 {
        if is_core {
            return 0.0
        } else {
            const origin_rotation = state.origin.rotation;
            const core_rotation = v.state_core.origin.rotation;
            const x_val = origin_rotation.rotation_to(core_rotation).angle_to(UnitQuaternion::new(self.noise))
            return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
        }
    }

    pub fn update(&mut self, time:f64) {
        for i in 0..3 {
            self.noise[i] = self.goal[i] * self.perlin.get([time / self.frequency, self.offsets[i]])
        }
    }
}
