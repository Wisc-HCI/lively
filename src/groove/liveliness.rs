use noise::{NoiseFn, Perlin};
use rand::{thread_rng, Rng};
use nalgebra::Vector3;
use crate::utils::config::ObjectiveSpec;
use crate::utils::settings::ObjectiveVariant;
use crate::utils::goals::Goal;

#[derive(Clone,Debug)]
pub struct Liveliness {
    pub perlin: Perlin,
    pub goals: Vec<Goal>,
    pub seeds: Vec<Goal>,
    pub sizes: Vec<f64>,
    pub shapes: Vec<Vec<f64>>,
    pub freqs: Vec<f64>
}

impl Liveliness {
    pub fn new(objectives:Vec<ObjectiveSpec>) -> Self {
        let perlin: Perlin = Perlin::new();
        let mut rng = thread_rng();
        let mut goals:Vec<Goal> = Vec::new();
        let mut seeds:Vec<Goal> = Vec::new();
        let mut sizes:Vec<f64> = Vec::new();
        let mut shapes:Vec<Vec<f64>> = Vec::new();
        let mut freqs:Vec<f64> = Vec::new();
        for objective in objectives {
            let mut size:f64 = 1.0;
            let mut shape:Vec<f64> = vec![1.0,1.0,1.0];
            let mut freq:f64 = 1.0;
            match objective.variant {
                ObjectiveVariant::PositionLiveliness => {
                    // println!("PositionLiveliness added");
                    goals.push(Goal::Vector(Vector3::new(0.0,0.0,0.0)));
                    seeds.push(Goal::Vector(Vector3::new(f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)))));
                    match objective.shape {
                        Some(s) => shape = s.clone(),
                        None => {}
                    }
                    match objective.frequency {
                        Some(f) => freq = f.clone(),
                        None => {}
                    }
                    sizes.push(size);
                    shapes.push(shape);
                    freqs.push(freq);
                },
                ObjectiveVariant::OrientationLiveliness => {
                    // println!("OrientationLiveliness added");
                    goals.push(Goal::Vector(Vector3::new(0.0,0.0,0.0)));
                    seeds.push(Goal::Vector(Vector3::new(f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)))));
                    match objective.shape {
                        Some(s) => shape = s.clone(),
                        None => {}
                    }
                    match objective.frequency {
                        Some(f) => freq = f.clone(),
                        None => {}
                    }
                    sizes.push(size);
                    shapes.push(shape);
                    freqs.push(freq);
                },
                ObjectiveVariant::JointLiveliness => {
                    // println!("JointLiveliness added");
                    goals.push(Goal::Scalar(0.0));
                    seeds.push(Goal::Scalar(f64::from(rng.gen_range(0..1000))));
                    match objective.scale {
                        Some(s) => size = s.clone(),
                        None => {}
                    }
                    match objective.frequency {
                        Some(f) => freq = f.clone(),
                        None => {}
                    }
                    sizes.push(size);
                    shapes.push(shape);
                    freqs.push(freq);
                },
                ObjectiveVariant::RelativeMotionLiveliness => {
                    // println!("JointLiveliness added");
                    goals.push(Goal::Scalar(0.0));
                    seeds.push(Goal::Scalar(f64::from(rng.gen_range(0..1000))));
                    match objective.scale {
                        Some(s) => size = s.clone(),
                        None => {}
                    }
                    match objective.frequency {
                        Some(f) => freq = f.clone(),
                        None => {}
                    }
                    sizes.push(size);
                    shapes.push(shape);
                    freqs.push(freq);
                },
                ObjectiveVariant::RootPositionLiveliness => {
                    // println!("RootPositionLiveliness added");
                    goals.push(Goal::Vector(Vector3::new(0.0,0.0,0.0)));
                    seeds.push(Goal::Vector(Vector3::new(f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)))));
                    match objective.shape {
                        Some(s) => shape = s.clone(),
                        None => {}
                    }
                    match objective.frequency {
                        Some(f) => freq = f.clone(),
                        None => {}
                    }
                    sizes.push(size);
                    shapes.push(shape);
                    freqs.push(freq);
                },
                // None-Objectives or Non-Lively Objectives are ignored.
                ObjectiveVariant::None => {
                    // println!("Non-Liveliness (None) objective added");
                    goals.push(Goal::None);
                    seeds.push(Goal::None);
                    sizes.push(size);
                    shapes.push(shape);
                    freqs.push(1.0);
                },
                _ => {
                    // println!("Non-Liveliness objective added");
                    goals.push(Goal::None);
                    seeds.push(Goal::None);
                    sizes.push(size);
                    shapes.push(shape);
                    freqs.push(1.0);
                }
            }
        }
        Self {perlin, goals, seeds, sizes, shapes, freqs}
    }

    pub fn update(&mut self, time:f64) {
        for i in 0..self.goals.len() {
            match (self.goals[i],self.seeds[i]) {
                // If the goal is a scalar
                (Goal::Scalar(goal),Goal::Scalar(seed)) => {
                    self.goals[i] = Goal::Scalar(self.sizes[i]*self.perlin.get([time/self.freqs[i], seed, 500.0*((time/self.freqs[i]+seed)/500.0).sin()]));
                },
                // If the goal is a 3-vector
                (Goal::Vector(goal),Goal::Vector(seed)) => {
                    self.goals[i] = Goal::Vector(Vector3::new(self.shapes[i][0]*self.perlin.get([time/self.freqs[i], seed[0], 500.0*((time/self.freqs[i]+seed[0])/500.0).sin()]),
                                                              self.shapes[i][1]*self.perlin.get([time/self.freqs[i], seed[1], 500.0*((time/self.freqs[i]+seed[1])/500.0).sin()]),
                                                              self.shapes[i][2]*self.perlin.get([time/self.freqs[i], seed[2], 500.0*((time/self.freqs[i]+seed[2])/500.0).sin()])
                                                          ));
                },
                // Ignore anything else
                _ => {}
            }
        }
        // println!("{:#?}", self.goals);
    }
}
