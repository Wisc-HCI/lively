use crate::utils::info::{LinkInfo, ProximityInfo, ShapeUpdate};
use crate::utils::shapes;
use grid::Grid;
use array2d::Array2D;
use nalgebra::base::Vector3;
use nalgebra::geometry::Isometry3;
use nalgebra::vector;
use nalgebra::Point3;
use parry3d_f64::query::closest_points::*;
use parry3d_f64::shape::*;
use parry3d_f64::bounding_volume::AABB;
// use rapier3d_f64::dynamics::*;
// use rapier3d_f64::geometry::*;
// use rapier3d_f64::math::*;
// use rapier3d_f64::pipeline::*;
// use rapier3d_f64::prelude::SharedShape;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::fmt;
use profiling::scope;

const IGNORE_DISTANCE: f64 = 5.0;

// use log::info;

#[derive(Clone)]
pub struct CollisionManager {
    scene_compound_shapes_list : Vec<(String, Compound)>,
    scene_transient_shapes_list : Vec<(String ,SharedShape)>


}

impl fmt::Debug for CollisionManager {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CollisionManager").finish()
    }
}

impl CollisionManager {
    #[profiling::function]
    pub fn new(links: Vec<LinkInfo>, persistent_shapes: Vec<shapes::Shape>) -> Self {
        // info!("Creating CollisionManager");
        
      
        let mut scene_compound_shapes_list : Vec<(String, Compound)> = vec![];
        let mut scene_transient_shapes_list : Vec<(String, SharedShape)> = vec![];
        //let mut world_shapes_list : Vec<(Isometry3<f64>, SharedShape)> = vec![];
      
        //info!("length for link is {:?}" , links);
        let index: i32 = 0;
        for link in &links {
            let mut robot_shapes_list: Vec<(Isometry3<f64>, SharedShape)> = Vec::new();
            let frame_name = &link.name;
            for collision in &link.collisions {

                match collision {
                    shapes::Shape::Cylinder(cylinder_object) => {
                        let new_length = cylinder_object.length / 2.0;
                        let cylinder_shape =
                            SharedShape::cylinder(new_length, cylinder_object.radius);
                        let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                        robot_shapes_list.push((
                            cylinder_object.local_transform * transform_offset,
                            cylinder_shape,
                        ));
                    }
                    shapes::Shape::Sphere(sphere_object) => {
                        let sphere_shape = SharedShape::ball(sphere_object.radius);
                        robot_shapes_list.push((sphere_object.local_transform, sphere_shape));
                    }
                    shapes::Shape::Box(box_object) => {
                        let box_shape =
                            SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                        robot_shapes_list.push((box_object.local_transform, box_shape));
                    }
                    shapes::Shape::Capsule(capsule_object) => {
                        let length = capsule_object.length;
                        let point_a = Point3::new(0.0, 0.0, length);
                        let point_b = Point3::new(0.0, 0.0, -length);
                        let capsule_shape =
                            SharedShape::capsule(point_a, point_b, capsule_object.radius);
                        robot_shapes_list.push((capsule_object.local_transform, capsule_shape));
                    }
                    shapes::Shape::Mesh(_mesh_object) => {}
                    _ => {}
                }
            }
        
            for shape in &persistent_shapes {
                match shape {
                    shapes::Shape::Box(box_object) => {
                        let box_shape = SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                        if &box_object.frame == frame_name {
                            robot_shapes_list.push((box_object.local_transform, box_shape));
                        } else if box_object.frame == "world" {                         
                            let temp_list : Vec<(Isometry3<f64>, SharedShape)> = vec![(box_object.local_transform, box_shape)];
                            let temp_compound = Compound::new(temp_list);
                            scene_compound_shapes_list.push(("world".to_string(), temp_compound));
                            
                        }
                    }
                    shapes::Shape::Cylinder(cylinder_object) => {
                        let new_length = cylinder_object.length / 2.0;
                        let cylinder_shape = SharedShape::cylinder(new_length, cylinder_object.radius);
                        let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                        if &cylinder_object.frame == frame_name { 
                            robot_shapes_list.push((
                                cylinder_object.local_transform * transform_offset,
                                cylinder_shape,
                            ));
                        } else if cylinder_object.frame == "world" {
                            let temp_list : Vec<(Isometry3<f64>, SharedShape)> = vec![(cylinder_object.local_transform, cylinder_shape)];
                            let temp_compound = Compound::new(temp_list);
                            scene_compound_shapes_list.push(("world".to_string(), temp_compound));
                        }
                    }
                    shapes::Shape::Sphere(sphere_object) => {
                        let sphere_shape = SharedShape::ball(sphere_object.radius);
                        if &sphere_object.frame == frame_name {
                            robot_shapes_list.push((sphere_object.local_transform, sphere_shape));
                        } else if sphere_object.name == "world" {
                            let temp_list : Vec<(Isometry3<f64>, SharedShape)> = vec![(sphere_object.local_transform, sphere_shape)];
                            let temp_compound = Compound::new(temp_list);
                            scene_compound_shapes_list.push(("world".to_string(), temp_compound));
                        }
                    }
                    shapes::Shape::Capsule(capsule_object) => {
                        let point_a = Point3::new(
                            capsule_object.length * vector![0.0, 1.0, 0.0][0],
                            capsule_object.length * vector![0.0, 1.0, 0.0][1],
                            capsule_object.length * vector![0.0, 1.0, 0.0][2],
                        );
                        let point_b = Point3::new(
                            capsule_object.length * vector![0.0, -1.0, 0.0][0],
                            capsule_object.length * vector![0.0, -1.0, 0.0][1],
                            capsule_object.length * vector![0.0, -1.0, 0.0][2],
                        );
                        let capsule_shape = SharedShape::capsule(point_a, point_b, capsule_object.radius);

                        if &capsule_object.frame == frame_name {      
                            robot_shapes_list.push((capsule_object.local_transform, capsule_shape));
                        } else if capsule_object.name == "world" {
                            let temp_list : Vec<(Isometry3<f64>, SharedShape)> = vec![(capsule_object.local_transform, capsule_shape)];
                            let temp_compound = Compound::new(temp_list);
                            scene_compound_shapes_list.push(("world".to_string(), temp_compound));
                           
                        }
                    }
                    shapes::Shape::Hull(hull_object) => {
                        let hull_points: Vec<Point3<f64>> = hull_object
                        .points
                        .iter()
                        .map(|p| Point3::new(p.x, p.y, p.z))
                        .collect();
                        let hull_shape = SharedShape::convex_hull(hull_points.as_slice());
                        if &hull_object.frame == frame_name {
                            match hull_shape {
                                Some(valid_hull_shape) => {
                                    robot_shapes_list.push((hull_object.local_transform, valid_hull_shape));
                                }
                                None => {
                                    println!("the given points cannot form a valid hull shape");
                                }
                            }
                        } else if hull_object.frame == "world" {
                            match hull_shape {
                                Some(valid_hull_shape) => {
                                    let temp_list : Vec<(Isometry3<f64>, SharedShape)> = vec![(hull_object.local_transform, valid_hull_shape)];
                                    let temp_compound = Compound::new(temp_list);
                                    scene_compound_shapes_list.push(("world".to_string(), temp_compound));
                                    }
                                    None => {
                                        println!("the given points cannot form a valid hull shape");
                                    }
                                }
                            }
                        }
                    shapes::Shape::Mesh(_mesh_object) => {
                        /*
                        Ignore Mesh Objects
                        */
                    }
                }
            }

            if robot_shapes_list.len() != 0 {
                let robot_compound_shapes= Compound::new(robot_shapes_list);           
                scene_compound_shapes_list.push((frame_name.to_string(),robot_compound_shapes));
            }
            
        }

        Self {
            scene_compound_shapes_list,
            scene_transient_shapes_list
        }
}
    

    // pub fn set_robot_frames(&mut self, _frames: &HashMap<String, Isometry3<f64>>) {

    // }
    #[profiling::function]
    pub fn perform_updates(&mut self, shape_updates: &Vec<ShapeUpdate>) {
        //scope!("perform updates");
        for update in shape_updates {
            match update {
                ShapeUpdate::Add { id, shape } => {
                    match shape {
                        shapes::Shape::Box(box_object) => {
                            let box_collider = SharedShape::cuboid(
                                box_object.y,
                                box_object.x,
                                box_object.z,
                            );
                            
                            self.scene_transient_shapes_list.push((id.to_string(), box_collider));
                        }
                        shapes::Shape::Cylinder(cylinder_object) => {
                            //let physical = if cylinder_object.physical { 2 } else { 1 };

                            let new_length = cylinder_object.length / 2.0;
                            let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                            let cylinder_collider = SharedShape::cylinder(
                                new_length,
                                cylinder_object.radius,
                            );
                            self.scene_transient_shapes_list.push((id.to_string(), cylinder_collider));
                        }
                        shapes::Shape::Sphere(sphere_object) => {
                            //let physical = if sphere_object.physical { 2 } else { 1 };

                            let sphere_collider = SharedShape::ball(sphere_object.radius);
                            self.scene_transient_shapes_list.push((id.to_string(), sphere_collider));
                        }
                        shapes::Shape::Capsule(capsule_object) => {
                            //let physical = if capsule_object.physical { 2 } else { 1 };

                            let point_a = Point3::new(
                                capsule_object.length * vector![0.0, 1.0, 0.0][0],
                                capsule_object.length * vector![0.0, 1.0, 0.0][1],
                                capsule_object.length * vector![0.0, 1.0, 0.0][2],
                            );
                            let point_b = Point3::new(
                                capsule_object.length * vector![0.0, -1.0, 0.0][0],
                                capsule_object.length * vector![0.0, -1.0, 0.0][1],
                                capsule_object.length * vector![0.0, -1.0, 0.0][2],
                            );
                            let capsule_collider = SharedShape::capsule(
                                point_a,
                                point_b,
                                capsule_object.radius,
                            );

                            self.scene_transient_shapes_list.push((id.to_string(), capsule_collider));
                            
                           
                        }
                        shapes::Shape::Hull(hull_object) => {
                            //let physical = if hull_object.physical { 2 } else { 1 };

                            let hull_points: Vec<Point3<f64>> = hull_object
                                .points
                                .iter()
                                .map(|p| Point3::new(p.x, p.y, p.z))
                                .collect();

                            let hull_shape = SharedShape::convex_hull(hull_points.as_slice());
                            match hull_shape {
                                Some(valid_hull_shape) => {
                                    self.scene_transient_shapes_list.push((id.to_string(), valid_hull_shape));
                                }
                                None => {
                                    println!("The given points of hull_object cannot be used to form a hull object");
                                }
                            }
                        }
                        shapes::Shape::Mesh(_mesh_object) => {
                            /*
                            Ignore Mesh Objects
                            */
                        }
                    }
                }
                ShapeUpdate::Move { id, pose } => {
                    // for (transient_id, transient_handle) in &mut self.transient_group {
                    //     if transient_id == id {
                    //         let transient_collider =
                    //             self.link_collider_set.get_mut(*transient_handle);
                    //         match transient_collider {
                    //             Some(matched_transient_collider) => {
                    //                 matched_transient_collider.set_position(*pose);
                    //                 self.collider_changed.push(*transient_handle);
                    //                 // println!("value assigned");
                    //             }
                    //             None => {} //println!("Could not find the collider")
                    //         }
                    //         break;
                    //     }
                    // }
                }
                ShapeUpdate::Delete(id) => {
                    let mut counter = 0;
                    for (transient_id, transient_handle) in &mut self.scene_transient_shapes_list {
                        if transient_id == id {
                    
                            break;
                        }
                        counter += 1;
                    }

                    self.scene_transient_shapes_list.remove(counter);
                }
            }
        }
    }
    #[profiling::function]
    pub fn clear_all_transient_shapes(&mut self) {
        self.scene_transient_shapes_list.clear();
    }

   

  

    #[profiling::function]
        pub fn create_compound_shapes_grid(&self) -> Array2D<(String,Compound)>{
           // let mut result_grid : Array2D<(String,Compound)> = Array2D
           //let mut scene_compound_shapes_list : Vec<(String, Compound)> = vec![];
           let size = self.scene_compound_shapes_list.len();
           let mut temp_vec : Vec<Vec<(String,Compound)>> = vec![];
           for n in 1..=size{
               temp_vec.push(self.scene_compound_shapes_list.clone());
           }
           let result_grid = Array2D::from_rows(&temp_vec);
           return result_grid;
    
        }

    #[profiling::function]
    pub fn get_proximity(&self, frames: &HashMap<String, Isometry3<f64>>) -> Vec<ProximityInfo> {
        
        let mut result_vector: Vec<ProximityInfo> = vec![];
        let compound_shapes_grid = self.clone().clear_all_transient_shapes();

        

        return result_vector;
    }
}

