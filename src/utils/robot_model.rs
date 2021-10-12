use pyo3::prelude::*;
use nalgebra::{vector, Vector3};
use nalgebra::geometry::{Isometry3, Translation3, UnitQuaternion};
use urdf_rs::{Robot, read_from_string};
use k::{Chain, JointType};
use std::collections::HashMap;
// use std::ops::Deref;
use crate::utils::state::*;
use crate::utils::shapes::*;
use crate::utils::info::*;
use crate::utils::geometry::{quaternion_exp,quaternion_log};
use crate::utils::collision_manager::CollisionManager;

#[pyclass]
#[derive(Clone,Debug)]
pub struct RobotModel {
    pub description: Robot,
    pub chain: Chain<f64>,
    pub collision_manager: CollisionManager,
    #[pyo3(get)]
    pub child_map: HashMap<String, String>,
    #[pyo3(get)]
    pub joint_names: Vec<String>,
    #[pyo3(get)]
    pub current_state: State,
    #[pyo3(get)]
    pub joint_converters: Vec<(f64, f64, usize, String)>, // Multipler, Offset, Index, JointName\
    #[pyo3(get)]
    pub dims: usize,
    #[pyo3(get)]
    pub links: Vec<LinkInfo>,
    #[pyo3(get)]
    pub joints: Vec<JointInfo>
}

impl RobotModel {
    
    pub fn new(urdf: String, collision_objects: Vec<CollisionObject>) -> Self {

        let description: Robot = read_from_string(&urdf.as_str()).unwrap();
        let chain: Chain<f64> = Chain::from(description.clone());

        let mut collision_manager: CollisionManager = CollisionManager::new(collision_objects.clone());
        
        let mut joints: Vec<JointInfo> = Vec::new();
        let mut links: Vec<LinkInfo> = Vec::new();

        for joint in chain.iter_joints() {
            let type_string: String;
            let axis_vec: [f64;3];
            match joint.joint_type {
                JointType::Fixed => {
                    type_string = String::from("fixed");
                    axis_vec = [0.0,0.0,0.0]
                },
                JointType::Rotational{axis} => {
                    type_string = String::from("rotational");
                    axis_vec = [axis.x,axis.y,axis.z]
                },
                JointType::Linear{axis} => {
                    type_string = String::from("linear");
                    axis_vec = [axis.x,axis.y,axis.z]
                }
            };
            let lower_bound: f64;
            let upper_bound: f64;
            let mut max_velocity: f64 = 0.0;
            let mut mimic: Option<MimicInfo> = None;
            match joint.limits {
                    Some(range) => {
                        lower_bound = range.min;
                        upper_bound = range.max;
                    },
                    None => {
                        lower_bound = 0.0;
                        upper_bound = 0.0;
                    }
            }
            for joint_description in description.joints.clone() {
                    if joint_description.name == joint.name {
                        max_velocity = joint_description.limit.velocity;
                        match joint_description.mimic {
                            Some(mimic_info) => {
                                mimic = Some(MimicInfo::from(mimic_info));
                            },
                            None => {
                                mimic = None;
                            }
                        }
                    }
            };
            let joint_info = JointInfo{
                name:joint.name.clone(), 
                joint_type: type_string, 
                lower_bound: lower_bound, 
                upper_bound: upper_bound,
                max_velocity: max_velocity,
                axis: axis_vec,
                mimic: mimic.clone()
                };
            joints.push(joint_info)
        }

        for link in chain.iter_links() {
            links.push(LinkInfo::new(link.name.clone()))
        }

        let mut child_map: HashMap<String, String> = HashMap::new();
        let mut joint_values: HashMap<String, f64> = HashMap::new();
        let mut link_transforms: HashMap<String, Isometry3<f64>> = HashMap::new();
        let mut joint_names: Vec<String> = Vec::new();
        let mut joint_converters: Vec<(f64, f64, usize, String)> = Vec::new();

        // Set up the state
        let origin: Isometry3<f64> = Isometry3::identity();
        let world = String::from("world");
        link_transforms.insert(world,Isometry3::identity());

        let joint_positions = chain.joint_positions();


        for joint in description.joints.clone() {
            child_map.insert(joint.name.clone(),joint.child.link.clone());
        }

        let mut i: usize = 0;
        let mut dims: usize = 6;
        for joint in chain.iter_joints() {
            // Push the joint name to the names vec
            joint_names.push(joint.name.clone());
            // Add the joint position
            joint_values.insert(joint.name.clone(),joint_positions[i]);

            // Get the transform
            let transform = joint.world_transform().unwrap_or(Isometry3::identity());
            link_transforms.insert(child_map.get(&joint.name.clone()).unwrap().to_string(), transform);

            // Increment for the joint_positions
            i += 1;
        }

        i = 0;
        // Now that the joint_names are defined, re-iterate on stuff to set up the converters
        for joint in chain.iter_joints() {
            let joint_data = joints.iter().find(|j| *j.name == joint.name).unwrap();
            match &joint_data.mimic {
                Some(mimic_info) => {
                    let other_joint_index = joint_names.iter().position(|j| *j == mimic_info.joint).unwrap() + 6;
                    joint_converters.push((mimic_info.multiplier,mimic_info.offset,other_joint_index,joints[i].name.clone()));
                },
                None => {
                    let joint_index = i + 6;
                    joint_converters.push((1.0,0.0,joint_index,joints[i].name.clone()));
                    dims += 1;
                }
            }
            i += 1;
        }

        collision_manager.set_robot_frames(link_transforms);
        let proximity = collision_manager.get_proximity();

        let current_state = State::new(origin,joint_values.clone(),link_transforms.clone(),proximity.clone());

        Self { description, chain, collision_manager, child_map, joint_names, joints, links, joint_converters, dims, current_state }
    }

    pub fn get_state(&mut self, x: Vec<f64>) -> State {
        let translation: Translation3<f64> = Translation3::new(x[0],x[1],x[2]);
        let rotation: UnitQuaternion<f64> = quaternion_exp(vector![x[3],x[4],x[5]]);
        self.current_state.origin = Isometry3::from_parts(translation,rotation);
        
        // Create a new joint_positions set
        let mut joint_positions: Vec<f64> = Vec::new();

        // Use the converters to handle mimic joints
        for (multiplier,offset,index,name) in &self.joint_converters {
            let v = multiplier * x[*index] + offset;
            joint_positions.push(v);
            self.current_state.joints.insert(name.to_string(),v);
        }

        self.chain.set_origin(self.current_state.origin);
        self.chain.set_joint_positions_unchecked(&joint_positions);
        
        // Force-update the transforms of the joints and links
        self.chain.update_link_transforms();
        
        // Update the stored joint transforms
        for joint in self.chain.iter_joints() {
            let transform = joint.world_transform().unwrap_or(Isometry3::identity());
            self.current_state.frames.insert(self.child_map.get(&joint.name).unwrap().to_string(), transform);
        };

        self.collision_manager.set_robot_frames(self.current_state.frames);
        self.current_state.proximity = self.collision_manager.get_proximity();
        
        // Return the current state.
        return self.current_state.clone()
    }
    
    pub fn set_state(&mut self, state: State) {
        /*
        Accepts a state containing origin information and joint information and sets the robot's current state.
        Ignores frame information from the input state, and supports partial state updates.
        */
        self.current_state.origin = state.origin;
        self.chain.set_origin(state.origin);
        
        // Create a new joint_positions set
        let mut joint_positions: Vec<f64> = Vec::new();

        // Copy over any new joint positions
        for name in &self.joint_names {
            let v = state.joints.get(name).unwrap_or(self.current_state.joints.get(name).unwrap_or(&0.0));
            joint_positions.push(*v);
            self.current_state.joints.insert(name.to_string(),*v);
        }

        self.chain.set_joint_positions_unchecked(&joint_positions);
        
        // Force-update the transforms of the joints and links
        self.chain.update_link_transforms();
        
        // Update the stored joint transforms
        for joint in self.chain.iter_joints() {
            let transform = joint.world_transform().unwrap_or(Isometry3::identity());
            self.current_state.frames.insert(self.child_map.get(&joint.name).unwrap().to_string(), transform);
        };

        self.collision_manager.set_robot_frames(self.current_state.frames);
        self.current_state.proximity = self.collision_manager.get_proximity();
    }

    pub fn get_x(&self) -> Vec<f64> {
        let origin_translation: Vector3<f64> = self.current_state.origin.translation.vector;
        let origin_rotation: Vector3<f64> = quaternion_log(self.current_state.origin.rotation);
        let mut x: Vec<f64> = vec![
            origin_translation[0],
            origin_translation[1],
            origin_translation[2],
            origin_rotation[0],
            origin_rotation[1],
            origin_rotation[2]
        ];
        for joint in self.joints {
            match &joint.mimic {
                None => {
                    x.push(self.current_state.get_joint_position(&joint.name))
                },
                _ => {}
            }
        }
        return x;
    }
}