use noise::{NoiseFn, Perlin};
use rand::{thread_rng, Rng};
use nalgebra::Vector3;
use crate::utils::config::ObjectiveSpec;
use crate::utils::settings::ObjectiveVariant;
use crate::utils::goals::Goal;

pub struct Liveliness {
    pub perlin: Perlin,
    pub goals: Vec<Goal>,
    pub seeds: Vec<Goal>,
    pub sizes: Vec<f64>,
    pub freqs: Vec<f64>
}

impl Liveliness {
    pub fn new(objectives:Vec<ObjectiveSpec>) -> Self {
        let perlin: Perlin = Perlin::new();
        let mut rng = thread_rng();
        let mut goals:Vec<Goal> = Vec::new();
        let mut seeds:Vec<Goal> = Vec::new();
        let mut sizes:Vec<f64> = Vec::new();
        let mut freqs:Vec<f64> = Vec::new();
        for objective in objectives {
            let mut size:f64 = 1.0;
            let mut freq:f64 = 1.0;
            match objective.variant {
                ObjectiveVariant::EEPositionLiveliness => {
                    goals.push(Goal::Vector(Vector3::new(0.0,0.0,0.0)));
                    seeds.push(Goal::Vector(Vector3::new(f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)))));
                    match objective.scale {
                        Some(s) => size = s.clone(),
                        None => {}
                    }
                    match objective.frequency {
                        Some(f) => freq = f.clone(),
                        None => {}
                    }
                    sizes.push(size);
                    freqs.push(freq);
                },
                ObjectiveVariant::EEOrientationLiveliness => {
                    goals.push(Goal::Vector(Vector3::new(0.0,0.0,0.0)));
                    seeds.push(Goal::Vector(Vector3::new(f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)))));
                    match objective.scale {
                        Some(s) => size = s.clone(),
                        None => {}
                    }
                    match objective.frequency {
                        Some(f) => freq = f.clone(),
                        None => {}
                    }
                    sizes.push(size);
                    freqs.push(freq);
                },
                ObjectiveVariant::JointLiveliness => {
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
                    freqs.push(freq);
                },
                ObjectiveVariant::RootPositionLiveliness => {
                    goals.push(Goal::Vector(Vector3::new(0.0,0.0,0.0)));
                    seeds.push(Goal::Vector(Vector3::new(f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)),f64::from(rng.gen_range(0..1000)))));
                    match objective.scale {
                        Some(s) => size = s.clone(),
                        None => {}
                    }
                    match objective.frequency {
                        Some(f) => freq = f.clone(),
                        None => {}
                    }
                    sizes.push(size);
                    freqs.push(freq);
                },
                // None-Objectives or Non-Lively Objectives are ignored.
                ObjectiveVariant::None => {
                    goals.push(Goal::None);
                    seeds.push(Goal::None);
                    sizes.push(1.0);
                    freqs.push(1.0);
                },
                _ => {
                    goals.push(Goal::None);
                    seeds.push(Goal::None);
                    sizes.push(1.0);
                    freqs.push(1.0);
                }
            }
        }
        Self {perlin, goals, seeds, sizes, freqs}
    }

    pub fn update(&mut self, time:f64) {
        for i in 0..self.goals.len() {
            match (self.goals[i],self.seeds[i]) {
                // If the goal is a scalar
                (Goal::Scalar(goal),Goal::Scalar(seed)) => {
                    self.goals[i] = Goal::Scalar(self.perlin.get([time/self.freqs[i], seed, 500.0*((time/self.freqs[i]+seed)/500.0).sin()]));
                },
                // If the goal is a 3-vector
                (Goal::Vector(goal),Goal::Vector(seed)) => {
                    self.goals[i] = Goal::Vector(Vector3::new(self.perlin.get([time/self.freqs[i], seed[0], 500.0*((time/self.freqs[i]+seed[0])/500.0).sin()]),
                                                              self.perlin.get([time/self.freqs[i], seed[1], 500.0*((time/self.freqs[i]+seed[1])/500.0).sin()]),
                                                              self.perlin.get([time/self.freqs[i], seed[2], 500.0*((time/self.freqs[i]+seed[2])/500.0).sin()])
                                                          ));
                },
                // Ignore anything else
                _ => {}
            }
        }
    }
}
