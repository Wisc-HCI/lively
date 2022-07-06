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
    compound_shapes_list : Vec<(i32, AABB)>,
    position_sharedshape_list: Vec<(Isometry3<f32>, SharedShape)>,


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
      
        //info!("length for link is {:?}" , links);
        let index: i32 = 0;
        for link in &links {
            let mut collider_vec: Vec<(Isometry3<f64>, SharedShape)> = Vec::new();
            let frame_name = &link.name;
            for collision in &link.collisions {

                match collision {
                    shapes::Shape::Cylinder(cylinder_object) => {
                        let new_length = cylinder_object.length / 2.0;
                        let cylinder_shape =
                            SharedShape::cylinder(new_length, cylinder_object.radius);
                        let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                        collider_vec.push((
                            cylinder_object.local_transform * transform_offset,
                            cylinder_shape,
                        ));
                    }
                    shapes::Shape::Sphere(sphere_object) => {
                        let sphere_shape = SharedShape::ball(sphere_object.radius);
                        collider_vec.push((sphere_object.local_transform, sphere_shape));
                    }
                    shapes::Shape::Box(box_object) => {
                        let box_shape =
                            SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                        collider_vec.push((box_object.local_transform, box_shape));
                    }
                    shapes::Shape::Capsule(capsule_object) => {
                        let length = capsule_object.length;
                        let point_a = Point3::new(0.0, 0.0, length);
                        let point_b = Point3::new(0.0, 0.0, -length);
                        let capsule_shape =
                            SharedShape::capsule(point_a, point_b, capsule_object.radius);
                        collider_vec.push((capsule_object.local_transform, capsule_shape));
                    }
                    shapes::Shape::Mesh(_mesh_object) => {}
                    _ => {}
                }
            }
        
            for shape in &persistent_shapes {
                match shape {
                    shapes::Shape::Box(box_object) => {
                        if &box_object.frame == frame_name {
                            let box_shape =
                                SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                            collider_vec.push((box_object.local_transform, box_shape));
                        } else if box_object.frame == "world" {
                            let box_shape =
                                SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                            let box_compound_collider_aabb = Compound::new(collider_vec).compute_local_aabb();
                            index += 1;
                            self.compound_shapes_list.insert((index,box_compound_collider_aabb));
                        }
                    }
                    shapes::Shape::Cylinder(cylinder_object) => {
                        if &cylinder_object.frame == frame_name {
                            let new_length = cylinder_object.length / 2.0;
                            let cylinder_shape =
                                SharedShape::cylinder(new_length, cylinder_object.radius);
                            let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                            collider_vec.push((
                                cylinder_object.local_transform * transform_offset,
                                cylinder_shape,
                            ));
                        } else if cylinder_object.frame == "world" {
                            let new_length = cylinder_object.length / 2.0;
                            let cylinder_shape =
                                SharedShape::cylinder(new_length, cylinder_object.radius);
                            let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                            let cylinder_collider = ColliderBuilder::new(cylinder_shape)
                                .position(cylinder_object.local_transform * transform_offset)
                                .active_events(ActiveEvents::COLLISION_EVENTS)
                                .user_data(1)
                                .build();
                            let collider_handle = link_collider_set.insert(cylinder_collider);
                            shape_name_look_up
                                .insert(collider_handle, cylinder_object.name.to_string());
                            //println! {"persistent shape(cylinder) added to the world frame"}
                        }
                    }
                    shapes::Shape::Sphere(sphere_object) => {
                        if &sphere_object.frame == frame_name {
                            let sphere_shape = SharedShape::ball(sphere_object.radius);
                            collider_vec.push((sphere_object.local_transform, sphere_shape));
                        } else if sphere_object.name == "world" {
                            let sphere_shape = SharedShape::ball(sphere_object.radius);
                            let sphere_collider = ColliderBuilder::new(sphere_shape)
                                .position(sphere_object.local_transform)
                                .active_events(ActiveEvents::COLLISION_EVENTS)
                                .user_data(1)
                                .build();
                            let collider_handle = link_collider_set.insert(sphere_collider);
                            shape_name_look_up
                                .insert(collider_handle, sphere_object.name.to_string());
                            //println! {"persistent shape(sphere) added to the world frame"}
                        }
                    }
                    shapes::Shape::Capsule(capsule_object) => {
                        if &capsule_object.frame == frame_name {
                            let point_a = Point::new(
                                capsule_object.length * vector![0.0, 1.0, 0.0][0],
                                capsule_object.length * vector![0.0, 1.0, 0.0][1],
                                capsule_object.length * vector![0.0, 1.0, 0.0][2],
                            );
                            let point_b = Point::new(
                                capsule_object.length * vector![0.0, -1.0, 0.0][0],
                                capsule_object.length * vector![0.0, -1.0, 0.0][1],
                                capsule_object.length * vector![0.0, -1.0, 0.0][2],
                            );
                            let capsule_shape =
                                SharedShape::capsule(point_a, point_b, capsule_object.radius);
                            collider_vec.push((capsule_object.local_transform, capsule_shape));
                        } else if capsule_object.name == "world" {
                            let point_a = Point::new(
                                capsule_object.length * vector![0.0, 1.0, 0.0][0],
                                capsule_object.length * vector![0.0, 1.0, 0.0][1],
                                capsule_object.length * vector![0.0, 1.0, 0.0][2],
                            );
                            let point_b = Point::new(
                                capsule_object.length * vector![0.0, -1.0, 0.0][0],
                                capsule_object.length * vector![0.0, -1.0, 0.0][1],
                                capsule_object.length * vector![0.0, -1.0, 0.0][2],
                            );
                            let capsule_shape =
                                SharedShape::capsule(point_a, point_b, capsule_object.radius);
                            let capsule_collider = ColliderBuilder::new(capsule_shape)
                                .position(capsule_object.local_transform)
                                .active_events(ActiveEvents::COLLISION_EVENTS)
                                .user_data(1)
                                .build();
                            link_collider_set.insert(capsule_collider);
                            //("persistent shape(capsule) added to the world frame");
                        }
                    }
                    shapes::Shape::Hull(hull_object) => {
                        if &hull_object.frame == frame_name {
                            let hull_points: Vec<Point3<f64>> = hull_object
                                .points
                                .iter()
                                .map(|p| Point3::new(p.x, p.y, p.z))
                                .collect();
                            let hull_shape = SharedShape::convex_hull(hull_points.as_slice());
                            match hull_shape {
                                Some(valid_hull_shape) => {
                                    collider_vec
                                        .push((hull_object.local_transform, valid_hull_shape));
                                }
                                None => {
                                    println!("the given points cannot form a valid hull shape");
                                }
                            }
                        } else if hull_object.frame == "world" {
                            if &hull_object.frame == frame_name {
                                let hull_points: Vec<Point3<f64>> = hull_object
                                    .points
                                    .iter()
                                    .map(|p| Point3::new(p.x, p.y, p.z))
                                    .collect();
                                let hull_shape = SharedShape::convex_hull(hull_points.as_slice());
                                match hull_shape {
                                    Some(valid_hull_shape) => {
                                        let hull_collider = ColliderBuilder::new(valid_hull_shape)
                                            .position(hull_object.local_transform)
                                            .active_events(ActiveEvents::COLLISION_EVENTS)
                                            .user_data(1)
                                            .build();
                                        let collider_handle =
                                            link_collider_set.insert(hull_collider);
                                        shape_name_look_up
                                            .insert(collider_handle, hull_object.name.to_string());
                                       // println!("persistent shape(hull) added to the world frame");
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

            if collider_vec.len() != 0 {
                let link_collider = ColliderBuilder::compound(collider_vec)
                    .active_events(ActiveEvents::COLLISION_EVENTS)
                    .user_data(0)
                    .build();
                let collider_handle = link_collider_set.insert(link_collider);
                shape_name_look_up.insert(collider_handle, frame_name.to_string());
                link_group.push((frame_name.to_string(), collider_handle));
            }
        }

        Self {
            broad_phase,
            narrow_phase,
            link_group,
            link_collider_set,
            robot_rigid_body_set,
            transient_group,
            island_manager,
            collider_changed,
            shape_name_look_up,
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
                            let physical = if box_object.physical { 2 } else { 1 };

                            let box_collider = ColliderBuilder::new(SharedShape::cuboid(
                                box_object.y,
                                box_object.x,
                                box_object.z,
                            ))
                            .position(box_object.local_transform)
                            .active_events(ActiveEvents::COLLISION_EVENTS)
                            .user_data(physical)
                            .build();
                            let handle = self.link_collider_set.insert(box_collider);
                            self.transient_group.push((id.to_string(), handle));
                        }
                        shapes::Shape::Cylinder(cylinder_object) => {
                            let physical = if cylinder_object.physical { 2 } else { 1 };

                            let new_length = cylinder_object.length / 2.0;
                            let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                            let cylinder_collider = ColliderBuilder::new(SharedShape::cylinder(
                                new_length,
                                cylinder_object.radius,
                            ))
                            .position(cylinder_object.local_transform * transform_offset)
                            .active_events(ActiveEvents::COLLISION_EVENTS)
                            .user_data(physical)
                            .build();
                            let handle = self.link_collider_set.insert(cylinder_collider);
                            self.transient_group.push((id.to_string(), handle));
                        }
                        shapes::Shape::Sphere(sphere_object) => {
                            let physical = if sphere_object.physical { 2 } else { 1 };

                            let sphere_collider =
                                ColliderBuilder::new(SharedShape::ball(sphere_object.radius))
                                    .position(sphere_object.local_transform)
                                    .active_events(ActiveEvents::COLLISION_EVENTS)
                                    .user_data(physical)
                                    .build();
                            let handle = self.link_collider_set.insert(sphere_collider);
                            self.transient_group.push((id.to_string(), handle));
                        }
                        shapes::Shape::Capsule(capsule_object) => {
                            let physical = if capsule_object.physical { 2 } else { 1 };

                            let point_a = Point::new(
                                capsule_object.length * vector![0.0, 1.0, 0.0][0],
                                capsule_object.length * vector![0.0, 1.0, 0.0][1],
                                capsule_object.length * vector![0.0, 1.0, 0.0][2],
                            );
                            let point_b = Point::new(
                                capsule_object.length * vector![0.0, -1.0, 0.0][0],
                                capsule_object.length * vector![0.0, -1.0, 0.0][1],
                                capsule_object.length * vector![0.0, -1.0, 0.0][2],
                            );
                            let capsule_collider = ColliderBuilder::new(SharedShape::capsule(
                                point_a,
                                point_b,
                                capsule_object.radius,
                            ))
                            .position(capsule_object.local_transform)
                            .active_events(ActiveEvents::COLLISION_EVENTS)
                            .user_data(physical)
                            .build();
                            let handle = self.link_collider_set.insert(capsule_collider);
                            self.transient_group.push((id.to_string(), handle));
                        }
                        shapes::Shape::Hull(hull_object) => {
                            let physical = if hull_object.physical { 2 } else { 1 };

                            let hull_points: Vec<Point3<f64>> = hull_object
                                .points
                                .iter()
                                .map(|p| Point3::new(p.x, p.y, p.z))
                                .collect();

                            let hull_shape = SharedShape::convex_hull(hull_points.as_slice());
                            match hull_shape {
                                Some(valid_hull_shape) => {
                                    let hull_collider = ColliderBuilder::new(valid_hull_shape)
                                        .position(hull_object.local_transform)
                                        .active_events(ActiveEvents::COLLISION_EVENTS)
                                        .user_data(physical)
                                        .build();
                                    let handle = self.link_collider_set.insert(hull_collider);
                                    self.transient_group.push((id.to_string(), handle));
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
