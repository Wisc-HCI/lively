use crate::utils::info::{LinkInfo, ProximityInfo, ShapeUpdate};
use crate::utils::shapes;

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
    scene_transient_shapes_list : Vec<(String ,Compound)>


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
        let mut scene_transient_shapes_list : Vec<(String, Compound)> = vec![];
        let mut world_shapes_list : Vec<(Isometry3<f64>, SharedShape)> = vec![];
      
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
                        let box_shape =
                        SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                        if &box_object.frame == frame_name {
                            robot_shapes_list.push((box_object.local_transform, box_shape));
                        } else if box_object.frame == "world" {
                            world_shapes_list.push((box_object.local_transform, box_shape));
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
                            world_shapes_list.push((cylinder_object.local_transform,cylinder_shape));       
                        }
                    }
                    shapes::Shape::Sphere(sphere_object) => {
                        let sphere_shape = SharedShape::ball(sphere_object.radius);
                        if &sphere_object.frame == frame_name {
                            robot_shapes_list.push((sphere_object.local_transform, sphere_shape));
                        } else if sphere_object.name == "world" {
                            world_shapes_list.push((sphere_object.local_transform,sphere_shape));
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
                            world_shapes_list.push((capsule_object.local_transform,capsule_shape));
                           
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
                                      world_shapes_list.push((hull_object.local_transform, valid_hull_shape));
                                    }
                                    None => {
                                        println!("the given points cannot form a valid hull shape");
                                    }
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
                let world_compound_shapes = Compound::new(world_shapes_list);
                scene_compound_shapes_list.push((frame_name,robot_compound_shapes));
                scene_transient_shapes_list.push((frame_name,world_compound_shapes));
                //scene_compound_shapes_aabb_list.push(("world",robot_compound_shapes));
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
                            self.scene_transient_shapes_aabb_list.push((id.to_string(), box_collider.compute_local_aabb()));
                        }
                        shapes::Shape::Cylinder(cylinder_object) => {
                            //let physical = if cylinder_object.physical { 2 } else { 1 };

                            let new_length = cylinder_object.length / 2.0;
                            let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                            let cylinder_collider = SharedShape::cylinder(
                                new_length,
                                cylinder_object.radius,
                            )
                            self.scene_transient_shapes_aabb_list.push((id.to_string(), cylinder_collider.compute_local_aabb()));
                        }
                        shapes::Shape::Sphere(sphere_object) => {
                            //let physical = if sphere_object.physical { 2 } else { 1 };

                            let sphere_collider = SharedShape::ball(sphere_object.radius);
                            self.scene_transient_shapes_aabb_list.push((id.to_string(), sphere_collider.compute_local_aabb()));
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
                            )

                            self.scene_transient_shapes_aabb_list.push((id.to_string(), capsule_collider.compute_local_aabb()));
                            
                           
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
                                    self.scene_transient_shapes_aabb_list.push((id.to_string(), valid_hull_shape.compute_local_aabb()));
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
                    for (transient_id, transient_handle) in &mut self.transient_group {
                        if transient_id == id {
                            let transient_collider =
                                self.link_collider_set.get_mut(*transient_handle);
                            match transient_collider {
                                Some(matched_transient_collider) => {
                                    matched_transient_collider.set_position(*pose);
                                    self.collider_changed.push(*transient_handle);
                                    // println!("value assigned");
                                }
                                None => {} //println!("Could not find the collider")
                            }
                            break;
                        }
                    }
                }
                ShapeUpdate::Delete(id) => {
                    let mut counter = 0;
                    for (transient_id, transient_handle) in &mut self.transient_group {
                        if transient_id == id {
                            self.link_collider_set.remove(
                                *transient_handle,
                                &mut self.island_manager,
                                &mut self.robot_rigid_body_set,
                                false,
                            );
                            // match removed {
                            //     Some (removed_collider) => {
                            //         println!("successfully removed");
                            //     },
                            //     None => {
                            //         println!("remove process failed");
                            //     },
                            // }
                            break;
                        }
                        counter += 1;
                    }

                    self.transient_group.remove(counter);
                }
            }
        }
    }
    #[profiling::function]
    pub fn clear_all_transient_shapes(&mut self) {
        for (_, transient_handle) in &mut self.transient_group {
            self.link_collider_set.remove(
                *transient_handle,
                &mut self.island_manager,
                &mut self.robot_rigid_body_set,
                false,
            );
            // match(removed){
            //     Some(removed_handle) =>{
            //         println!("clear_all_transient_shapes : successfully removed");
            //     },
            //     None => {
            //         println!("clear_all_transient_shapes : remove process failed");
            //     },
            // }
        }
        self.transient_group.clear();
    }

    // pub fn collision_pipeline_step(mut new_broad_phase : BroadPhase, mut new_narrow_phase : NarrowPhase, mut new_robot_rigid_body_set :  RigidBodySet,mut new_link_collider_set : ColliderSet) -> NarrowPhase{
    //     let mut collision_pipeline = CollisionPipeline::new();
    //     let physics_hooks = ();
    //     let event_handler = ();

    //     collision_pipeline.step(
    //         1.0,
    //         &mut new_broad_phase,
    //         &mut new_narrow_phase,
    //         &mut new_robot_rigid_body_set,
    //         &mut new_link_collider_set,
    //         &physics_hooks,
    //         &event_handler,
    //     );

    //     println!("number of contact_pairs: {:?}" , new_narrow_phase.contact_pairs().count()  );
    //     return new_narrow_phase;
    // }

    

    #[profiling::function]
    pub fn get_proximity(&self, frames: &HashMap<String, Isometry3<f64>>) -> Vec<ProximityInfo> {

    #[profiling::function]
    pub fn iterate_contact_pairs( shape_name_look_up : HashMap<ColliderHandle, String> , mut new_broad_phase : BroadPhase, mut new_narrow_phase : NarrowPhase, mut new_robot_rigid_body_set :  RigidBodySet, mut new_link_collider_set : ColliderSet) -> Vec<ProximityInfo> {
        let mut result_vector: Vec<ProximityInfo> = vec![];
       for pairs in new_narrow_phase.contact_pairs() {
            // info!("colliding pairs detected");
            let handle1 = pairs.collider1;
            let handle2 = pairs.collider2;
            let collider1 = new_link_collider_set.get(handle1);
            let collider2 = new_link_collider_set.get(handle2);

            match (collider1, collider2) {
                (Some(first), Some(second)) => {
                    if first.user_data == 0 && second.user_data == 0 {
                        //println!("contacts found between compound shapes");
                        let shape1 = first.shape(); //Shape
                        let shape_collider_1_pos = *first.position();
                        //println!("{:?}" ,shape_collider_1_pos);
                        let shape2 = second.shape();
                        let shape_collider_2_pos = *second.position();
                        match parry3d_f64::query::closest_points(
                            &shape_collider_1_pos,
                            shape1,
                            &shape_collider_2_pos,
                            shape2,
                            IGNORE_DISTANCE,
                        ) {
                            Ok(valid_closest_points) => {
                                match valid_closest_points {
                                   
                                    ClosestPoints::Intersecting => {
                                        let shape_name1 =
                                            shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            shape_name_look_up.get(&handle2).unwrap();
                                        result_vector.push(ProximityInfo::new(
                                            shape_name1.to_string(),
                                            shape_name2.to_string(),
                                            0.0,
                                            None,
                                            true,
                                        ));

                                        // // Distance is zero and there are no points included
                                        // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(0.0),None,is_physical))
                                    }
                                    ClosestPoints::WithinMargin(point1, point2) => {
                                        let shape_name1 =
                                            shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            shape_name_look_up.get(&handle2).unwrap();
                                        let dist = ((point1.x - point2.x) * (point1.x - point2.x)
                                            + (point1.y - point2.y) * (point1.y - point2.y)
                                            + (point1.z - point2.z) * (point1.z - point2.z) as f64)
                                            .sqrt();
                                        result_vector.push(ProximityInfo::new(
                                            shape_name1.to_string(),
                                            shape_name2.to_string(),
                                            dist,
                                            Some((point1, point2)),
                                            true,
                                        ));
                                        // let dist = distance(point1,point2);
                                        // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(dist),Some((point1,point2)),is_physical))
                                    }
                                    ClosestPoints::Disjoint => {
                                        //println!("disjoint is ignored");
                                    }
                                    

                                }
                            },
                            Err(_) => {println!("disjoint is ignored")}
                        }
                    } else if first.user_data == 0 && second.user_data == 1 {
                        // println!("contacts found between compound shape and (non-physical transient shapes or world frame
                        //             persistent shapes)");
                        let shape1 = first.shape(); //Shape
                        
                        
                        let shape_collider_1_pos = *first.position();
                       
                        let shape2 = second.shape();
                        let shape_collider_2_pos = *second.position();

                        //  println!("shape 1 is {:?}" ,shape_collider_1_pos);
                        //  println!("shape 2 is {:?}" ,shape_collider_2_pos);

                        match parry3d_f64::query::closest_points(
                            &shape_collider_1_pos,
                            shape1,
                            &shape_collider_2_pos,
                            shape2,
                            IGNORE_DISTANCE,
                        ) {
                            Ok(valid_closest_points) => {
                                match valid_closest_points {
                                    ClosestPoints::Intersecting => {
                                        let shape_name1 =
                                            shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            shape_name_look_up.get(&handle2).unwrap();
                                        result_vector.push(ProximityInfo::new(
                                            shape_name1.to_string(),
                                            shape_name2.to_string(),
                                            0.0,
                                            None,
                                            false,
                                        ));

                                        // // Distance is zero and there are no points included
                                        // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(0.0),None,is_physical))
                                    }
                                    ClosestPoints::WithinMargin(point1, point2) => {
                                        let shape_name1 =
                                            shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            shape_name_look_up.get(&handle2).unwrap();
                                        let dist = ((point1.x - point2.x) * (point1.x - point2.x)
                                            + (point1.y - point2.y) * (point1.y - point2.y)
                                            + (point1.z - point2.z) * (point1.z - point2.z) as f64)
                                            .sqrt();
                                        result_vector.push(ProximityInfo::new(
                                            shape_name1.to_string(),
                                            shape_name2.to_string(),
                                            dist,
                                            Some((point1, point2)),
                                            false,
                                        ));
                                        // let dist = distance(point1,point2);
                                        // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(dist),Some((point1,point2)),is_physical))
                                    }
                                    ClosestPoints::Disjoint => {
                                        // println!("disjoint is ignored");
                                    }
                                }
                            },
                            Err(_) => {println!("disjoint is ignored")}
                        }
                    } else if first.user_data == 0 && second.user_data == 2 {
                        // println!("contact found between compound shape and physical persistent shapes");
                        let shape1 = first.shape(); //Shape
                        let shape_collider_1_pos = *first.position();
                        //println!("{:?}" ,shape_collider_1_pos);
                        let shape2 = second.shape();
                        let shape_collider_2_pos = *second.position();
                        match parry3d_f64::query::closest_points(
                            &shape_collider_1_pos,
                            shape1,
                            &shape_collider_2_pos,
                            shape2,
                            IGNORE_DISTANCE,
                        ) {
                            Ok(valid_closest_points) => {
                                match valid_closest_points {
                                    ClosestPoints::Intersecting => {
                                        let shape_name1 =
                                            shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            shape_name_look_up.get(&handle2).unwrap();
                                        result_vector.push(ProximityInfo::new(
                                            shape_name1.to_string(),
                                            shape_name2.to_string(),
                                            0.0,
                                            None,
                                            true,
                                        ));

                                        // // Distance is zero and there are no points included
                                        // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(0.0),None,is_physical))
                                    }
                                    ClosestPoints::WithinMargin(point1, point2) => {
                                        let shape_name1 =
                                           shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            shape_name_look_up.get(&handle2).unwrap();
                                        let dist = ((point1.x - point2.x) * (point1.x - point2.x)
                                            + (point1.y - point2.y) * (point1.y - point2.y)
                                            + (point1.z - point2.z) * (point1.z - point2.z) as f64)
                                            .sqrt();
                                        result_vector.push(ProximityInfo::new(
                                            shape_name1.to_string(),
                                            shape_name2.to_string(),
                                            dist,
                                            Some((point1, point2)),
                                            true,
                                        ));
                                        // let dist = distance(point1,point2);
                                        // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(dist),Some((point1,point2)),is_physical))
                                    }
                                    ClosestPoints::Disjoint => {
                                        // println!("disjoint is ignored");
                                    }
                                }
                            },
                            Err(_) => {println!("disjoint is ignored")}
                        }
                    } else {
                        // println!("contact found between other shapes are ignored");
                    }
                }
                _ => {}
            }
        }

        return result_vector;
    }

        #[profiling::function]
        pub fn collision_pipeline_step(mut new_broad_phase : BroadPhase, mut new_narrow_phase : NarrowPhase, mut new_robot_rigid_body_set :  RigidBodySet, mut new_link_collider_set : ColliderSet) -> (NarrowPhase,ColliderSet,BroadPhase,RigidBodySet){
            let mut collision_pipeline = CollisionPipeline::new();
            let physics_hooks = ();
            let event_handler = ();
    
            collision_pipeline.step(
                1.0,
                &mut new_broad_phase,
                &mut new_narrow_phase,
                &mut new_robot_rigid_body_set,
                &mut new_link_collider_set,
                &physics_hooks,
                &event_handler,
            );
    
            println!("number of contact_pairs: {:?}" , new_narrow_phase.contact_pairs().count()  );
            return (new_narrow_phase,new_link_collider_set,new_broad_phase,new_robot_rigid_body_set);
        }

       

        
       // scope!("get");
        //----------------------------------------------------setframes()
        let mut new_link_group = self.link_group.clone();
        let mut new_transient_group = self.transient_group.clone();
        let mut new_link_collider_set = self.link_collider_set.clone();
        let mut new_robot_rigid_body_set = self.robot_rigid_body_set.clone();
        let mut new_broad_phase = self.broad_phase.clone();
        let mut new_narrow_phase = self.narrow_phase.clone();
        // let mut new_collider_vector_look_up = self.collider_vector_look_up.clone();
        // let mut new_collider_handle_look_up = self.collider_handle_look_up.clone();
        // let mut new_island_manager = self.island_manager.clone();
        let mut new_collider_changed = self.collider_changed.clone();
        let mut result_vector: Vec<ProximityInfo> = vec![];
        // let mut self.shape_name_look_up = self.shape_name_look_up.clone();

        for (name, group_handle) in &mut new_link_group {
            // println!("{}" , name);
            let link_collider = new_link_collider_set.get_mut(*group_handle);
            match link_collider {
                Some(matched_link_collider) => {
                    let transform = frames.get(name);
                    match transform {
                        Some(matched_transform) => {
                            // println!("{:?}", matched_transform);
                            matched_link_collider.set_position(*matched_transform);
                            new_collider_changed.push(*group_handle);
                            // println!("value assigned");
                        }
                        None => {} //println!("Could not find the value"),
                    }
                }
                None => {
                    //println!("{}{}" , " persistent shape updated for frame " , name);
                }
            }
        }

        for (name, group_handle) in &mut new_transient_group {
            let transient_collider = new_link_collider_set.get_mut(*group_handle);
            match transient_collider {
                Some(matched_transient_collider) => {
                    let transform = frames.get(name);
                    match transform {
                        Some(matched_transform) => {
                            //println!("{:?}", matched_transform);
                            matched_transient_collider.set_position(*matched_transform);
                            new_collider_changed.push(*group_handle);
                            //println!("value assigned");
                        }
                        None => {} //println!("Could not find the value"),
                    }
                }
                None => {} //println!("Could not find the handle"),
            }
        }

        // let mut pair_event_vec = Vec::<BroadPhasePairEvent>::new();
        // new_broad_phase.update(
        //     1.0,
        //     &mut new_link_collider_set,
        //     &new_collider_changed[0..new_collider_changed.len() - 1],
        //     &[],
        //     &mut pair_event_vec,
        // );
        // let gravity = vector![0.0, 0.0, 0.0];
        // let integration_parameters = IntegrationParameters::default();
        // let mut collision_pipeline = CollisionPipeline::new();
        // // let mut joint_set = JointSet::new();
        // // let mut ccd_solver = CCDSolver::new();
        // //let mut rigid_body_set = RigidBodySet::new();
        // let physics_hooks = ();
        // let event_handler = ();

        // collision_pipeline.step(
        //     1.0,
        //     &mut new_broad_phase,
        //     &mut new_narrow_phase,
        //     &mut new_robot_rigid_body_set,
        //     &mut new_link_collider_set,
        //     &physics_hooks,
        //     &event_handler,
        // );

        // println!("number of contact_pairs: {:?}" , new_narrow_phase.contact_pairs().count()  );collision_pipeline_step
        let (new_narrow_phase,new_link_collider_set,new_broad_phase,new_robot_rigid_body_set) = collision_pipeline_step(new_broad_phase, new_narrow_phase,  new_robot_rigid_body_set, new_link_collider_set);
        result_vector = iterate_contact_pairs(self.shape_name_look_up.clone(),new_broad_phase, new_narrow_phase,  new_robot_rigid_body_set, new_link_collider_set);

        return result_vector;

        // for pairs in new_narrow_phase.contact_pairs() {
        //     // info!("colliding pairs detected");
        //     let handle1 = pairs.collider1;
        //     let handle2 = pairs.collider2;
        //     let collider1 = new_link_collider_set.get(handle1);
        //     let collider2 = new_link_collider_set.get(handle2);

        //     match (collider1, collider2) {
        //         (Some(first), Some(second)) => {
        //             if first.user_data == 0 && second.user_data == 0 {
        //                 //println!("contacts found between compound shapes");
        //                 let shape1 = first.shape(); //Shape
        //                 let shape_collider_1_pos = *first.position();
        //                 //println!("{:?}" ,shape_collider_1_pos);
        //                 let shape2 = second.shape();
        //                 let shape_collider_2_pos = *second.position();
        //                 match parry3d_f64::query::closest_points(
        //                     &shape_collider_1_pos,
        //                     shape1,
        //                     &shape_collider_2_pos,
        //                     shape2,
        //                     IGNORE_DISTANCE,
        //                 ) {
        //                     Ok(valid_closest_points) => {
        //                         match valid_closest_points {
                                   
        //                             ClosestPoints::Intersecting => {
        //                                 let shape_name1 =
        //                                     self.shape_name_look_up.get(&handle1).unwrap();
        //                                 let shape_name2 =
        //                                     self.shape_name_look_up.get(&handle2).unwrap();
        //                                 result_vector.push(ProximityInfo::new(
        //                                     shape_name1.to_string(),
        //                                     shape_name2.to_string(),
        //                                     0.0,
        //                                     None,
        //                                     true,
        //                                 ));

        //                                 // // Distance is zero and there are no points included
        //                                 // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(0.0),None,is_physical))
        //                             }
        //                             ClosestPoints::WithinMargin(point1, point2) => {
        //                                 let shape_name1 =
        //                                     self.shape_name_look_up.get(&handle1).unwrap();
        //                                 let shape_name2 =
        //                                     self.shape_name_look_up.get(&handle2).unwrap();
        //                                 let dist = ((point1.x - point2.x) * (point1.x - point2.x)
        //                                     + (point1.y - point2.y) * (point1.y - point2.y)
        //                                     + (point1.z - point2.z) * (point1.z - point2.z) as f64)
        //                                     .sqrt();
        //                                 result_vector.push(ProximityInfo::new(
        //                                     shape_name1.to_string(),
        //                                     shape_name2.to_string(),
        //                                     dist,
        //                                     Some((point1, point2)),
        //                                     true,
        //                                 ));
        //                                 // let dist = distance(point1,point2);
        //                                 // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(dist),Some((point1,point2)),is_physical))
        //                             }
        //                             ClosestPoints::Disjoint => {
        //                                 //println!("disjoint is ignored");
        //                             }
                                    

        //                         }
        //                     },
        //                     Err(_) => {println!("disjoint is ignored")}
        //                 }
        //             } else if first.user_data == 0 && second.user_data == 1 {
        //                 // println!("contacts found between compound shape and (non-physical transient shapes or world frame
        //                 //             persistent shapes)");
        //                 let shape1 = first.shape(); //Shape
                        
                        
        //                 let shape_collider_1_pos = *first.position();
                       
        //                 let shape2 = second.shape();
        //                 let shape_collider_2_pos = *second.position();

        //                 //  println!("shape 1 is {:?}" ,shape_collider_1_pos);
        //                 //  println!("shape 2 is {:?}" ,shape_collider_2_pos);

        //                 match parry3d_f64::query::closest_points(
        //                     &shape_collider_1_pos,
        //                     shape1,
        //                     &shape_collider_2_pos,
        //                     shape2,
        //                     IGNORE_DISTANCE,
        //                 ) {
        //                     Ok(valid_closest_points) => {
        //                         match valid_closest_points {
        //                             ClosestPoints::Intersecting => {
        //                                 let shape_name1 =
        //                                     self.shape_name_look_up.get(&handle1).unwrap();
        //                                 let shape_name2 =
        //                                     self.shape_name_look_up.get(&handle2).unwrap();
        //                                 result_vector.push(ProximityInfo::new(
        //                                     shape_name1.to_string(),
        //                                     shape_name2.to_string(),
        //                                     0.0,
        //                                     None,
        //                                     false,
        //                                 ));

        //                                 // // Distance is zero and there are no points included
        //                                 // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(0.0),None,is_physical))
        //                             }
        //                             ClosestPoints::WithinMargin(point1, point2) => {
        //                                 let shape_name1 =
        //                                     self.shape_name_look_up.get(&handle1).unwrap();
        //                                 let shape_name2 =
        //                                     self.shape_name_look_up.get(&handle2).unwrap();
        //                                 let dist = ((point1.x - point2.x) * (point1.x - point2.x)
        //                                     + (point1.y - point2.y) * (point1.y - point2.y)
        //                                     + (point1.z - point2.z) * (point1.z - point2.z) as f64)
        //                                     .sqrt();
        //                                 result_vector.push(ProximityInfo::new(
        //                                     shape_name1.to_string(),
        //                                     shape_name2.to_string(),
        //                                     dist,
        //                                     Some((point1, point2)),
        //                                     false,
        //                                 ));
        //                                 // let dist = distance(point1,point2);
        //                                 // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(dist),Some((point1,point2)),is_physical))
        //                             }
        //                             ClosestPoints::Disjoint => {
        //                                 // println!("disjoint is ignored");
        //                             }
        //                         }
        //                     },
        //                     Err(_) => {println!("disjoint is ignored")}
        //                 }
        //             } else if first.user_data == 0 && second.user_data == 2 {
        //                 // println!("contact found between compound shape and physical persistent shapes");
        //                 let shape1 = first.shape(); //Shape
        //                 let shape_collider_1_pos = *first.position();
        //                 //println!("{:?}" ,shape_collider_1_pos);
        //                 let shape2 = second.shape();
        //                 let shape_collider_2_pos = *second.position();
        //                 match parry3d_f64::query::closest_points(
        //                     &shape_collider_1_pos,
        //                     shape1,
        //                     &shape_collider_2_pos,
        //                     shape2,
        //                     IGNORE_DISTANCE,
        //                 ) {
        //                     Ok(valid_closest_points) => {
        //                         match valid_closest_points {
        //                             ClosestPoints::Intersecting => {
        //                                 let shape_name1 =
        //                                     self.shape_name_look_up.get(&handle1).unwrap();
        //                                 let shape_name2 =
        //                                     self.shape_name_look_up.get(&handle2).unwrap();
        //                                 result_vector.push(ProximityInfo::new(
        //                                     shape_name1.to_string(),
        //                                     shape_name2.to_string(),
        //                                     0.0,
        //                                     None,
        //                                     true,
        //                                 ));

        //                                 // // Distance is zero and there are no points included
        //                                 // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(0.0),None,is_physical))
        //                             }
        //                             ClosestPoints::WithinMargin(point1, point2) => {
        //                                 let shape_name1 =
        //                                     self.shape_name_look_up.get(&handle1).unwrap();
        //                                 let shape_name2 =
        //                                     self.shape_name_look_up.get(&handle2).unwrap();
        //                                 let dist = ((point1.x - point2.x) * (point1.x - point2.x)
        //                                     + (point1.y - point2.y) * (point1.y - point2.y)
        //                                     + (point1.z - point2.z) * (point1.z - point2.z) as f64)
        //                                     .sqrt();
        //                                 result_vector.push(ProximityInfo::new(
        //                                     shape_name1.to_string(),
        //                                     shape_name2.to_string(),
        //                                     dist,
        //                                     Some((point1, point2)),
        //                                     true,
        //                                 ));
        //                                 // let dist = distance(point1,point2);
        //                                 // infos.push(ProximityInfo::new(shape1_name,shape2_name,Some(dist),Some((point1,point2)),is_physical))
        //                             }
        //                             ClosestPoints::Disjoint => {
        //                                 // println!("disjoint is ignored");
        //                             }
        //                         }
        //                     },
        //                     Err(_) => {println!("disjoint is ignored")}
        //                 }
        //             } else {
        //                 // println!("contact found between other shapes are ignored");
        //             }
        //         }
        //         _ => {}
        //     }
        // }

        // return result_vector;
    }
}
