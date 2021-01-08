use nalgebra::{UnitQuaternion, Vector3, Point3};
// use crate::utils::yaml_utils::{*};
use crate::spacetime::robot::Robot;
use crate::groove::collision_nn::CollisionNN;
use crate::utils::sampler::ThreadRobotSampler;
use crate::utils::config::{*};
use crate::utils::settings::{*};
use crate::utils::history::History;
// use crate::utils::file_utils::{*};
use crate::utils::goals::GoalSpec;
use crate::groove::env_collision::{*};
use ncollide3d::pipeline::{*};
use ncollide3d::query::{*};
use ncollide3d::shape::{*};
// use time::PreciseTime;
use std::ops::Deref;

// #[derive(Clone, Debug)]
// pub struct Vars {
//     pub init_state: Vec<f64>,
//     pub xopt: Vec<f64>,
//     pub offset: Vec<f64>,
//     pub prev_state: Vec<f64>,
//     pub prev_state2: Vec<f64>,
//     pub prev_state3: Vec<f64>
// }
// impl Vars {
//     pub fn new(init_state: Vec<f64>) -> Self {
//         Vars{init_state: init_state.clone(), xopt: init_state.clone(), offset: vec![0,0,0], prev_state: init_state.clone(),
//             prev_state2: init_state.clone(), prev_state3: init_state.clone()}
//     }
//
//     pub fn update(&mut self, xopt: Vec<f64>) {
//         self.prev_state3 = self.prev_state2.clone();
//         self.prev_state2 = self.prev_state.clone();
//         self.prev_state = self.xopt.clone();
//         self.xopt = xopt.clone();
//     }
// }


pub struct RelaxedIKVars {
    pub robot: Robot,
    pub sampler: ThreadRobotSampler,
    pub init_state: Vec<f64>,
    pub xopt: Vec<f64>,
    pub xopt_core: Vec<f64>,
    pub offset: Vec<f64>,
    pub history: History,
    pub history_core: History,
    pub goals: Vec<GoalSpec>,
    pub init_ee_positions: Vec<Vector3<f64>>,
    pub init_ee_quats: Vec<UnitQuaternion<f64>>,
    pub collision_nn: CollisionNN,
    pub env_collision: RelaxedIKEnvCollision,
    pub control_mode: ControlMode,
    pub environment_mode: EnvironmentMode,
    pub objective_variants: Vec<ObjectiveVariant>
}
impl RelaxedIKVars {
    pub fn new(config: Config) -> Self {
        let mut robot = Robot::new(config.clone());
        let num_chains = config.joint_names.len();
        let sampler = ThreadRobotSampler::new(robot.clone());

        let mut goals: Vec<GoalSpec> = Vec::new();
        for goal_spec in &config.goals {
            if goal_spec.name == "default" {
                goals = goal_spec.goals.clone();
            }
        }
        let init_ee_positions = robot.get_ee_positions(config.starting_config.as_slice());
        let init_ee_quats = robot.get_ee_quats(config.starting_config.as_slice());

        let collision_nn = CollisionNN::new(config.nn_main.clone());

        let frames = robot.get_frames(&config.starting_config.clone());
        let env_collision = RelaxedIKEnvCollision::new(config.clone(), &frames);

        let environment_mode = config.mode_environment;
        let control_mode = config.mode_control;

        let mut objective_variants: Vec<ObjectiveVariant> = Vec::new();
        for objective in config.objectives {
            objective_variants.push(objective.variant)
        }

        RelaxedIKVars{robot, sampler, init_state: config.starting_config.clone(), xopt: config.starting_config.clone(),
            xopt_core: config.starting_config.clone(), offset: vec![0.0,0.0,0.0],
            history: History::new(config.starting_config.clone()), history_core: History::new(config.starting_config.clone()),
            goals, init_ee_positions, init_ee_quats, control_mode, collision_nn,
            env_collision, environment_mode, objective_variants}
    }

    pub fn update_collision_world(&mut self) -> bool {
        let frames = self.robot.get_frames(&self.xopt);
        self.env_collision.update_links(&frames);
        for event in self.env_collision.world.proximity_events() {
            let c1 = self.env_collision.world.objects.get(event.collider1).unwrap();
            let c2 = self.env_collision.world.objects.get(event.collider2).unwrap();
            if event.new_status == Proximity::Intersecting {
                println!("===== {:?} Intersecting of {:?} =====", c1.data().name, c2.data().name);
            } else if event.new_status == Proximity::WithinMargin {
                println!("===== {:?} WithinMargin of {:?} =====", c1.data().name, c2.data().name);
                if c1.data().link_data.is_link {
                    let arm_idx = c1.data().link_data.arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider2) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider2).unwrap();
                        if !links.contains(&event.collider1) {
                            links.push(event.collider1);
                        }
                    } else {
                        let links: Vec<CollisionObjectSlabHandle> = vec![event.collider1];
                        self.env_collision.active_pairs[arm_idx].insert(event.collider2, links);
                    }
                } else if c2.data().link_data.is_link {
                    let arm_idx = c2.data().link_data.arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider1) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider1).unwrap();
                        if !links.contains(&event.collider2) {
                            links.push(event.collider2);
                        }
                    } else {
                        let links: Vec<CollisionObjectSlabHandle> = vec![event.collider2];
                        self.env_collision.active_pairs[arm_idx].insert(event.collider1, links);
                    }
                }
            } else {
                println!("===== {:?} Disjoint of {:?} =====", c1.data().name, c2.data().name);
                if c1.data().link_data.is_link {
                    let arm_idx = c1.data().link_data.arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider2) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider2).unwrap();
                        if links.contains(&event.collider1) {
                            let index = links.iter().position(|x| *x == event.collider1).unwrap();
                            links.remove(index);
                        }
                        if links.len() == 0 {
                            self.env_collision.active_pairs[arm_idx].remove(&event.collider2);
                        }
                    }
                } else if c2.data().link_data.is_link {
                    let arm_idx = c2.data().link_data.arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider1) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider1).unwrap();
                        if links.contains(&event.collider2) {
                            let index = links.iter().position(|x| *x == event.collider2).unwrap();
                            links.remove(index);
                        }
                        if links.len() == 0 {
                            self.env_collision.active_pairs[arm_idx].remove(&event.collider1);
                        }
                    }
                }
            }
            // self.print_active_pairs();
        }

        self.env_collision.world.update();

        let link_radius = self.env_collision.link_radius;
        let penalty_cutoff: f64 = link_radius * 2.0;
        let a = penalty_cutoff.powi(2);
        let filter_cutoff = 3;
        for arm_idx in 0..frames.len() {
            // let mut sum_max: f64 = 0.0;
            let mut active_candidates: Vec<(Option<CollisionObjectSlabHandle>, f64)> = Vec::new();
            for key in self.env_collision.active_pairs[arm_idx].keys() {
                let obstacle = self.env_collision.world.objects.get(*key).unwrap();
                // println!("Obstacle: {:?}", obstacle.data());
                let mut sum: f64 = 0.0;
                let last_elem = frames[arm_idx].0.len() - 1;
                for j in 0..last_elem {
                    let start_pt = Point3::from(frames[arm_idx].0[j]);
                    let end_pt = Point3::from(frames[arm_idx].0[j + 1]);
                    let segment = Segment::new(start_pt, end_pt);
                    let segment_pos = nalgebra::one();
                    let dis = distance(obstacle.position(), obstacle.shape().deref(), &segment_pos, &segment) - link_radius;
                    // println!("VARS -> {:?}, Link{}, Distance: {:?}", obstacle.data(), j, dis);
                    if dis > 0.0 {
                        sum += a / (dis + link_radius).powi(2);
                    } else if self.environment_mode != EnvironmentMode::None {
                        return true;
                    } else {
                        break;
                    }
                }
                active_candidates.push((Some(*key), sum));
            }

            // println!("Number of active obstacles: {}", active_obstacles.len());
            if self.environment_mode != EnvironmentMode::None {
                if active_candidates.len() > filter_cutoff {
                    active_candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
                    let active_obstacles = active_candidates[0..filter_cutoff].iter().cloned().collect();
                    self.env_collision.active_obstacles[arm_idx] = active_obstacles;
                } else {
                    self.env_collision.active_obstacles[arm_idx] = active_candidates;
                }
            }
        }

        return false;
    }

    pub fn print_active_pairs(&self) {
        let frames = self.robot.get_frames(&self.xopt);
        for i in 0..frames.len() {
            for (key, values) in self.env_collision.active_pairs[i].iter() {
                let collider = self.env_collision.world.objects.get(*key).unwrap();
                for v in values {
                    let link = self.env_collision.world.objects.get(*v).unwrap();
                    println!("Arm {}, Active pair {:?} and {:?}", i, collider.data().name, link.data().name);
                }
            }
        }
    }
}
