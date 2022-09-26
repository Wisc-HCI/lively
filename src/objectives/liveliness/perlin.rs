use crate::objectives::objective::{groove_loss, Callable};
// use crate::utils::general::quaternion_exp;
use crate::utils::state::State;
use crate::utils::vars::Vars;
use nalgebra::geometry::UnitQuaternion;
use nalgebra::{vector, Vector3};
use noise::{NoiseFn, Perlin, Seedable};
use rand::rngs::ThreadRng;
use rand::{thread_rng, Rng};
use serde::{Deserialize, Serialize};

fn get_default_perlin() -> Perlin {
    let mut rng: ThreadRng = thread_rng();
    let seed: u32 = rng.gen();
    return Perlin::new().set_seed(seed);
}

fn get_default_offsets() -> [f64; 3] {
    let mut rng: ThreadRng = thread_rng();
    let offsets: [f64; 3] = [
        f64::from(rng.gen_range(0..1000)),
        f64::from(rng.gen_range(0..1000)),
        f64::from(rng.gen_range(0..1000)),
    ];
    return offsets;
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PositionLivelinessObjective {
    // Adds position liveliness to the specified link
    pub name: String,
    pub weight: f64,
    pub link: String,
    pub frequency: f64,

    // Goal Value (shape of noise)
    #[serde(skip)]
    pub goal: Vector3<f64>,
    #[serde(skip)]
    pub time: Option<f64>,

    // Inaccessible
    #[serde(skip)]
    pub noise: Vector3<f64>,
    #[serde(skip, default = "get_default_perlin")]
    pub perlin: Perlin,
    #[serde(skip, default = "get_default_offsets")]
    pub offsets: [f64; 3],
}

impl PositionLivelinessObjective {
    pub fn new(name: String, weight: f64, link: String, frequency: f64) -> Self {
        let mut rng: ThreadRng = thread_rng();
        let seed: u32 = rng.gen();
        let perlin: Perlin = Perlin::new().set_seed(seed);
        let offsets: [f64; 3] = [
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000)),
        ];
        Self {
            name,
            weight,
            link,
            frequency,
            goal: vector![0.0, 0.0, 0.0],
            time: None,
            noise: vector![0.0, 0.0, 0.0],
            perlin,
            offsets,
        }
    }
}

impl Callable<Vector3<f64>> for PositionLivelinessObjective {
    fn call(&self, v: &Vars, state: &State, _is_core: bool) -> f64 {
        let link_translation = state.get_link_transform(&self.link).translation.vector;
        let goal = v
            .history
            .prev1
            .get_link_transform(&self.link)
            .translation
            .vector
            + self.noise;

        let x_val = (link_translation - goal).norm();

        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2);
    }

    fn update(&mut self, time: f64) {
        let last_time = self.time.unwrap_or(time);
        for i in 0..3 {
            self.noise[i] = self.goal[i] * 10.0 * 
                * (self.perlin.get([time / self.frequency, self.offsets[i]])
                    - self
                        .perlin
                        .get([last_time / self.frequency, self.offsets[i]]))
        }
        self.time = Some(time);
        println!("noise {:?}",self.noise);
    }

    fn set_goal(&mut self, goal: Vector3<f64>) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct OrientationLivelinessObjective {
    // Adds orientation liveliness to the link
    pub name: String,
    pub weight: f64,
    pub link: String,
    pub frequency: f64,

    // Goal Value (shape of noise)
    #[serde(skip)]
    pub goal: Vector3<f64>,

    // Inaccessible
    #[serde(skip)]
    pub noise: UnitQuaternion<f64>,
    #[serde(skip, default = "get_default_perlin")]
    pub perlin: Perlin,
    #[serde(skip, default = "get_default_offsets")]
    pub offsets: [f64; 3],
}

impl OrientationLivelinessObjective {
    pub fn new(name: String, weight: f64, link: String, frequency: f64) -> Self {
        let mut rng: ThreadRng = thread_rng();
        let seed: u32 = rng.gen();
        let perlin: Perlin = Perlin::new().set_seed(seed);
        let offsets: [f64; 3] = [
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000)),
            f64::from(rng.gen_range(0..1000)),
        ];
        Self {
            name,
            weight,
            link,
            frequency,
            goal: vector![0.0, 0.0, 0.0],
            noise: UnitQuaternion::identity(),
            perlin,
            offsets,
        }
    }
}

impl Callable<Vector3<f64>> for OrientationLivelinessObjective {
    fn call(&self, v: &Vars, state: &State, _is_core: bool) -> f64 {
        let link_rotation = state.get_link_transform(&self.link).rotation;
        let prev_rotation = v.history.prev1.get_link_transform(&self.link).rotation;
        let x_val = link_rotation
            .rotation_to(&prev_rotation)
            .angle_to(&self.noise);
        return self.weight * groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2);
    }

    fn update(&mut self, time: f64) {
        // let noise_vec = vector![
        //     self.goal[0] * self.perlin.get([time / self.frequency, self.offsets[0]]),
        //     self.goal[1] * self.perlin.get([time / self.frequency, self.offsets[1]]),
        //     self.goal[2] * self.perlin.get([time / self.frequency, self.offsets[2]])
        // ];
        self.noise = UnitQuaternion::from_euler_angles(
            self.goal[0]
                * (self.perlin.get([time / self.frequency, self.offsets[0]])
                    - self
                        .perlin
                        .get([(time - 0.01) / self.frequency, self.offsets[0]])),
            self.goal[1]
                * (self.perlin.get([time / self.frequency, self.offsets[1]])
                    - self
                        .perlin
                        .get([(time - 0.01) / self.frequency, self.offsets[1]])),
            self.goal[2]
                * (self.perlin.get([time / self.frequency, self.offsets[2]])
                    - self
                        .perlin
                        .get([(time - 0.01) / self.frequency, self.offsets[2]])),
        )
    }

    fn set_goal(&mut self, goal: Vector3<f64>) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct JointLivelinessObjective {
    // Adds joint liveliness to the specified joint
    pub name: String,
    pub weight: f64,
    pub joint: String,
    pub frequency: f64,

    // Goal Value (shape of noise)
    #[serde(skip)]
    pub goal: f64,

    // Inaccessible
    #[serde(skip)]
    pub noise: f64,
    #[serde(skip, default = "get_default_perlin")]
    pub perlin: Perlin,
}

impl JointLivelinessObjective {
    pub fn new(name: String, weight: f64, joint: String, frequency: f64) -> Self {
        let mut rng: ThreadRng = thread_rng();
        let seed: u32 = rng.gen();
        let perlin: Perlin = Perlin::new().set_seed(seed);
        Self {
            name,
            weight,
            joint,
            frequency,
            goal: 0.0,
            noise: 0.0,
            perlin,
        }
    }
}

impl Callable<f64> for JointLivelinessObjective {
    fn call(&self, v: &Vars, state: &State, _is_core: bool) -> f64 {
        let joint_value = state.get_joint_position(&self.joint);
        let goal = v.history.prev1.get_joint_position(&self.joint) + self.noise;

        let x_val = (joint_value - goal).abs();

        return self.weight * groove_loss(x_val, 0.0, 2, 0.32950, 0.1, 2);
    }

    fn update(&mut self, time: f64) {
        self.noise = self.goal
            * (self.perlin.get([time / self.frequency, 0.0])
                - self.perlin.get([(time - 0.01) / self.frequency, 0.0]))
    }

    fn set_goal(&mut self, goal: f64) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}

#[repr(C)]
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct RelativeMotionLivelinessObjective {
    // Defining a vector line between two links (link1 and link2), this objective promotes lively motion of the second link along that vector
    pub name: String,
    pub weight: f64,
    pub link1: String,
    pub link2: String,
    pub frequency: f64,

    #[serde(skip)]
    pub goal: f64,

    // Inaccessible
    #[serde(skip)]
    pub noise: f64,
    #[serde(skip, default = "get_default_perlin")]
    pub perlin: Perlin,
}

impl RelativeMotionLivelinessObjective {
    pub fn new(name: String, weight: f64, link1: String, link2: String, frequency: f64) -> Self {
        let mut rng: ThreadRng = thread_rng();
        let seed: u32 = rng.gen();
        let perlin: Perlin = Perlin::new().set_seed(seed);
        Self {
            name,
            weight,
            link1,
            link2,
            frequency,
            goal: 0.0,
            noise: 0.0,
            perlin,
        }
    }
}

impl Callable<f64> for RelativeMotionLivelinessObjective {
    fn call(&self, v: &Vars, state: &State, _is_core: bool) -> f64 {
        // The error is the difference between the current value and the position along the source-target vector.
        let link2_translation = state.get_link_transform(&self.link2).translation.vector;
        let prev1_translation = v
            .history.prev1
            .get_link_transform(&self.link1)
            .translation
            .vector;
        let prev2_translation = v
            .history.prev1
            .get_link_transform(&self.link2)
            .translation
            .vector;
        // Calculate the unit vector. We scale this by the current noise value
        // and add it to the core_translation to create the goal
        let unit_vector = (prev2_translation - prev1_translation).normalize();
        let goal = unit_vector * self.noise + prev1_translation;
        // cost is the distance between the offset position (goal) and the link's translation
        let x_val = (link2_translation - goal).norm();

        return self.weight * groove_loss(x_val, 0., 2, 0.1, 10.0, 2);
    }

    fn update(&mut self, time: f64) {
        self.noise = self.goal * (self.perlin.get([time / self.frequency, 0.0])-self.perlin.get([(time-0.01) / self.frequency, 0.0]))
    }

    fn set_goal(&mut self, goal: f64) {
        self.goal = goal;
    }

    fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
}
