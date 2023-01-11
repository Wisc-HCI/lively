use nalgebra::{Vector3};
use nalgebra::geometry::{Isometry3, Translation3, UnitQuaternion};
use urdf_rs::{Robot, read_from_string};
use k::{Chain, JointType, center_of_mass};
use std::collections::HashMap;
use std::sync::Mutex;
// use std::ops::Deref;
use crate::utils::state::*;
use crate::utils::shapes::*;
use crate::utils::info::*;
use crate::utils::collision_manager::CollisionManager;

#[derive(Debug)]
pub struct RobotModel {
    pub description: Robot,
    pub chain: Chain<f64>,
    pub collision_manager: Mutex<CollisionManager>,
    pub child_map: HashMap<String, String>,
    pub joint_names: Vec<String>,
    pub joint_converters: Vec<(f64, f64, usize, String)>, // Multipler, Offset, Index, JointName\
    pub dims: usize,
    pub origin_link: String,
    pub links: Vec<LinkInfo>,
    pub joints: Vec<JointInfo>,
    pub collision_objects : Vec<Shape>,
    pub start_vec: Vec<f64>,
}

impl RobotModel {
    
    pub fn new(urdf: String, collision_objects: Vec<Shape>, collision_settings: &Option<CollisionSettingInfo>, displacement_bounds: Vec<ScalarRange>) -> Self {
        
        let description: Robot = read_from_string(&urdf.as_str()).unwrap();
        let chain: Chain<f64> = Chain::from(description.clone());
        println!("{:?}",description);

        let mut joints: Vec<JointInfo> = Vec::new();
        let mut links: Vec<LinkInfo> = Vec::new();

        let mut origin_link: String = String::from("base_link");

        let mut non_mimic_count: usize = 0;
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
            let mut parent_link: String = "".into();
            let mut child_link: String = "".into();
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
            for idx in 0..description.joints.len() {
                    if description.joints[idx].name == joint.name {
                        parent_link = description.joints[idx].parent.link.clone();
                        child_link = description.joints[idx].child.link.clone();
                        max_velocity = description.joints[idx].limit.velocity;
                        match &description.joints[idx].mimic {
                            Some(mimic_info) => {
                                mimic = Some(MimicInfo::from(mimic_info));
                            },
                            None => {
                                mimic = None;
                            }
                        }
                        break;
                    }
            };

            let joint_info = JointInfo {
                name:joint.name.clone(), 
                joint_type: type_string, 
                lower_bound, 
                upper_bound,
                max_velocity,
                axis: axis_vec,
                mimic: mimic.clone(),
                idx: 6 + non_mimic_count,
                parent_link,
                child_link
            };
            joints.push(joint_info);

            if mimic.is_none() {
                non_mimic_count += 1;
            }
        }

        for link in &description.links {
            let mut parent_joint: String = String::from("world");
            for joint in &description.joints {
                if joint.child.link == link.name {
                    parent_joint = joint.name.clone();
                }
            }
            let mut link_info = LinkInfo::from(link.clone());
            link_info.parent_joint = parent_joint.clone();
            links.push(link_info);
            if parent_joint.as_str() == "world" {
                origin_link = link.name.clone();
            } 
        }
        let collision_manager: Mutex<CollisionManager> = 
            Mutex::new(CollisionManager::new(links.clone(),collision_objects.clone(),collision_settings));

        let mut child_map: HashMap<String, String> = HashMap::new();
        let mut joint_names: Vec<String> = Vec::new();
        let mut joint_converters: Vec<(f64, f64, usize, String)> = Vec::new();

        for link in &links {
            child_map.insert(link.parent_joint.clone(),link.name.clone());
        }

        // println!("Child map {:?}",child_map);

        for joint in chain.iter_joints() {
            // Push the joint name to the names vec
            joint_names.push(joint.name.clone());
        }

        let mut dims: usize = 6;
        // Now that the joint_names are defined, re-iterate on stuff to set up the converters
        for joint_data in &joints {
            match &joint_data.mimic {
                Some(mimic_info) => {
                    let mut other_joint_index: usize = 0;
                    for joint in &joints {
                        if joint.name == mimic_info.joint {
                            other_joint_index = joint.idx;
                        }
                    }
                    joint_converters.push((mimic_info.multiplier,mimic_info.offset,other_joint_index,joint_data.name.clone()));
                },
                None => {
                    joint_converters.push((1.0,0.0,joint_data.idx,joint_data.name.clone()));
                    dims += 1;
                }
            }
        }

        let mut start_vec = vec![];
        for bound in displacement_bounds {
            start_vec.push(bound.value)
        }

        Self { description, chain, collision_manager, child_map, joint_names, joints, links, joint_converters, dims, origin_link,collision_objects,start_vec }
    }

    pub fn get_environmental_objects(&self) -> Vec<Shape>{
        return self.collision_objects.clone();
    }
    pub fn get_state(&self, x: &Vec<f64>,include_proximity: bool,timestamp: f64) -> State {
        let translation: Translation3<f64> = Translation3::new(x[0],x[1],x[2]);
        let rotation: UnitQuaternion<f64> = UnitQuaternion::from_euler_angles(x[3],x[4],x[5]);
        let origin = Isometry3::from_parts(translation,rotation);
        let mut joints: HashMap<String,f64> = HashMap::new();
        let mut frames: HashMap<String,TransformInfo> = HashMap::new();
        
        // Create a new joint_positions set
        let mut joint_positions: Vec<f64> = Vec::new();

        // Use the converters to handle mimic joints
        for (multiplier,offset,index,name) in &self.joint_converters {
            let v = multiplier * x[*index] + offset;
            joint_positions.push(v);
            joints.insert(name.to_string(),v);
        }

        self.chain.set_origin(origin);
        self.chain.set_joint_positions_unchecked(&joint_positions);
        self.chain.update_transforms();
        
        // // Force-update the transforms of the joints and links
        // self.chain.update_link_transforms();
        
        // Update the stored joint transforms
        // println!("Getting state!")

        frames.insert(self.origin_link.clone(),TransformInfo::new(origin,origin));
        for node in self.chain.iter() {
            let joint = node.joint();
            // println!("Joint Name {:?}",joint.name);
            let world_transform = joint.world_transform().unwrap_or(Isometry3::identity());
            let local_transform = joint.local_transform();
            match self.child_map.get(&joint.name) {
                Some(child_rel) => {
                    frames.insert(child_rel.to_string(), TransformInfo::new(world_transform,local_transform));
                },
                None => {}
            };
            // frames.insert(self.child_map.get(&joint.name).unwrap().to_string(), transform);
        };

        let proximity: Vec<ProximityInfo>;
        if include_proximity {
            proximity = self.collision_manager.lock().unwrap().get_proximity(&frames)
        } else {
            proximity = vec![]
        }
        
        let center_of_mass_vec = center_of_mass(&self.chain);

        // Return the current state.
        return State::new(origin, joints, frames, proximity, center_of_mass_vec, timestamp)
    }

    pub fn get_default_state(&self) -> State {

        let mut x = self.start_vec.clone();

        for joint in &self.joints {
            match &joint.mimic {
                None => {
                    x.push((joint.lower_bound + joint.upper_bound)/2.0)
                },
                _ => {}
            }
        }

        return self.get_state(&x,true,0.0)
    }
    
    pub fn get_filled_state(&self, state: &State) -> State {
        /*
        Accepts a state containing origin information and joint information and sets the robot's current state.
        Ignores frame information from the input state, and supports partial state updates.
        */

        // Turn the state into a vector and then get the state from it.
        return self.get_state(&self.get_x(state),true,0.0)
    }

    pub fn get_x(&self, state: &State) -> Vec<f64> {
        let origin_translation: Vector3<f64> = state.origin.translation.vector;
        let origin_rotation = state.origin.rotation.euler_angles();
        let mut x: Vec<f64> = vec![
            origin_translation[0],
            origin_translation[1],
            origin_translation[2],
            origin_rotation.0,
            origin_rotation.1,
            origin_rotation.2
        ];
        for joint in &self.joints {
            match &joint.mimic {
                None => {
                    x.push(state.get_joint_position(&joint.name))
                },
                _ => {}
            }
        }
        return x;
    }
}