use noise::{Perlin};
use rand::prelude::*;
use crate::utils::config::ObjectiveSpec;
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
        let mut rng = rand::thread_rng();
        let mut goals:Vec<Goal> = Vec::new();
        let mut seeds:Vec<Goal> = Vec::new();
        let mut sizes:Vec<f64> = Vec::new();
        let mut freqs:Vec<f64> = Vec::new();
        for objective in objectives {
            let mut size:f64 = 1;
            let mut freq:f64 = 1;
            match objective.variant {
                ObjectiveVariant::EEPositionLiveliness => {
                    goals.push(Goal::Vector(Vector3::new(0,0,0)));
                    seeds.push(Goal::Vector(Vector3::new(rng.gen_range(0..1000),rng.gen_range(0..1000),rng.gen_range(0..1000))));
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
                    goals.push(Goal::Quaternion(UnitQuaternion::new(1,0,0,0)));
                    seeds.push(Goal::Vector(Vector3::new(rng.gen_range(0..1000),rng.gen_range(0..1000),rng.gen_range(0..1000))));
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
                    goals.push(Goal::Scalar(0);
                    seeds.push(Goal::Scalar(rng.gen_range(0..1000)));
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
                    sizes.push(1);
                    freqs.push(1);
                },
                _ => {
                    goals.push(Goal::None);
                    seeds.push(Goal::None);
                    sizes.push(1);
                    freqs.push(1);
                }
            }
        }
        Self {perlin, goals, seeds, sizes, freqs}
    }
    pub fn update(&mut self, time:f64) {
        for i in 0..self.goals {
            match (self.goals,self.seeds) {
                // If the goal is a scalar
                (Goal::Scalar(goal),Goal::Scalar(seed)) => {
                    self.goals[i] = Goal::Scalar(self.perlin.get([time/self.freqs[i], seed, 500*((time/self.freqs[i]+seed)/500).sin()]))
                },
                // If the goal is a 3-vector
                (Goal::Vector(goal),Goal::Vector(seed)) => {
                    self.goals[i] = Goal::Vector(Vector3::new(self.perlin.get([time/self.freqs[i], seed[0], 500*((time/self.freqs[i]+seed[0])/500).sin()]),
                                                              self.perlin.get([time/self.freqs[i], seed[1], 500*((time/self.freqs[i]+seed[1])/500).sin()]),
                                                              self.perlin.get([time/self.freqs[i], seed[2], 500*((time/self.freqs[i]+seed[2])/500).sin()])
                                                          ))
                },
                // If the goal is a quaternion
                (Goal::Quaternion(goal),Goal::Vector(seed)) => {
                    self.goals[i] = Goal::Quaternion(UnitQuaternion::from_euler_angles(
                                                              self.perlin.get([time/self.freqs[i], seed[0], 500*((time/self.freqs[i]+seed[0])/500).sin()]),
                                                              self.perlin.get([time/self.freqs[i], seed[1], 500*((time/self.freqs[i]+seed[1])/500).sin()]),
                                                              self.perlin.get([time/self.freqs[i], seed[2], 500*((time/self.freqs[i]+seed[2])/500).sin()])
                                                          ))
                },
                // Ignore anything else
                _ => {}
            }
        }
    }
}
