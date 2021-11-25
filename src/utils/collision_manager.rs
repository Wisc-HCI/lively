use crate::utils::info::{ProximityInfo, ShapeUpdate};
use crate::utils::shapes;
use k::Chain;
use nalgebra::geometry::Isometry3;
use nalgebra::vector;
use nalgebra::Point3;
use parry3d_f64::query::closest_points::*;
use rapier3d_f64::dynamics::*;
use rapier3d_f64::geometry::*;
use rapier3d_f64::math::*;
use rapier3d_f64::pipeline::*;
use rapier3d_f64::prelude::SharedShape;
use std::collections::HashMap;
use std::fmt;

// pub struct ColliderHandleInfo {
//     pub collider_vector : Vec<(Isometry<Real>,SharedShape)>,
//     pub collider_handle : ColliderHandle,
//     pub shape_frame : String,
//     pub shape_id : Option<String>,
//     pub shape_name : String,
// }

#[derive(Clone)]
pub struct CollisionManager {
    link_group: Vec<(String, ColliderHandle)>,
    transient_group: Vec<(String, ColliderHandle)>,
    link_collider_set: ColliderSet,
    robot_rigid_body_set: RigidBodySet,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    collider_vector_look_up: HashMap<String, Vec<(Isometry<Real>, SharedShape)>>,
    collider_handle_look_up: HashMap<String, ColliderHandle>, // a look up table that stores the frame, ColliderHandle pair
    island_manager: IslandManager,
    collider_changed: Vec<ColliderHandle>,
    shape_name_look_up: HashMap<ColliderHandle, String>,
    // string, struct ?
}

impl fmt::Debug for CollisionManager {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CollisionManager").finish()
    }
}

impl CollisionManager {
    pub fn new(chain: Chain<f64>, persistent_shapes: Vec<shapes::Shape>) -> Self {
        let mut island_manager = IslandManager::new();
        let broad_phase = BroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let mut link_collider_set = ColliderSet::new();
        let mut robot_rigid_body_set = RigidBodySet::new();
        let mut link_group: Vec<(String, ColliderHandle)> = vec![];
        // let mut persistent_group : Vec<(String,ColliderHandle)> = vec![];
        // let mut pair_event_vec = Vec::<BroadPhasePairEvent>::new();
        let mut collider_handle_look_up = HashMap::new();
        let mut collider_vector_look_up = HashMap::new();
        let collider_changed: Vec<ColliderHandle> = vec![];
        let transient_group: Vec<(String, ColliderHandle)> = vec![];
        let mut shape_name_look_up = HashMap::new();

        //let robot_shapes: Vec<Shape> = vec![];
        // enumerate through the robot links and convert the collision objects into rapier stuff.
        for link in chain.iter_links() {
            let mut collider_vec = Vec::new();
            let name = &link.name;
            for collision in &link.collisions {
                let geom_element = &collision.geometry;
                let origin_element = collision.origin();
                match geom_element {
                    //parsing each link shapes.
                    k::link::Geometry::Cylinder { radius, length } => {
                        let new_length = *length / 2.0;
                        let cylinder_shape = SharedShape::cylinder(new_length, *radius);
                        collider_vec.push((*origin_element, cylinder_shape));
                    }
                    k::link::Geometry::Sphere { radius } => {
                        let sphere_shape = SharedShape::ball(*radius);
                        collider_vec.push((*origin_element, sphere_shape));
                    }
                    k::link::Geometry::Box {
                        depth,
                        width,
                        height,
                    } => {
                        let box_shape = SharedShape::cuboid(*width, *depth, *height);
                        collider_vec.push((*origin_element, box_shape));
                    }
                    k::link::Geometry::Capsule { radius, length } => {
                        let point_a = Point::new(
                            *length * vector![0.0, 1.0, 0.0][0],
                            *length * vector![0.0, 1.0, 0.0][1],
                            *length * vector![0.0, 1.0, 0.0][2],
                        );
                        let point_b = Point::new(
                            *length * vector![0.0, -1.0, 0.0][0],
                            *length * vector![0.0, -1.0, 0.0][1],
                            *length * vector![0.0, -1.0, 0.0][2],
                        );
                        let capsule_shape = SharedShape::capsule(point_a, point_b, *radius);
                        collider_vec.push((*origin_element, capsule_shape));
                    }
                    _ => {}
                }
            }
            if collider_vec.len() != 0 {
                collider_vector_look_up.insert(name.to_string(), collider_vec.clone());
                let link_collider = ColliderBuilder::compound(collider_vec)
                    .active_events(ActiveEvents::CONTACT_EVENTS)
                    .user_data(0)
                    .build();
                let link_collider_handle = link_collider_set.insert(link_collider);
                collider_handle_look_up.insert(name.to_string(), link_collider_handle);
                shape_name_look_up.insert(link_collider_handle, name.to_string());
                link_group.push((name.to_string(), link_collider_handle.clone()));
            }
        }

        for shape in &persistent_shapes {
            match shape {
                shapes::Shape::Box(box_object) => {
                    let box_shape = SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                    let frame_existing_collider_handle =
                        collider_handle_look_up.get_mut(&box_object.frame);
                    match frame_existing_collider_handle {
                        Some(collider_handle) => {
                            match collider_vector_look_up.get_mut(&box_object.frame) {
                                Some(collider_vector) => {
                                    collider_vector
                                        .push((box_object.local_transform, box_shape.clone()));
                                    let new_collider =
                                        ColliderBuilder::compound(collider_vector.to_vec())
                                            .active_events(ActiveEvents::CONTACT_EVENTS)
                                            .user_data(0)
                                            .build();
                                    match link_collider_set.remove(
                                        *collider_handle,
                                        &mut island_manager,
                                        &mut robot_rigid_body_set,
                                        false,
                                    ) {
                                        Some(_collider) => {
                                            let new_handle = link_collider_set.insert(new_collider);
                                            link_group.push((
                                                box_object.frame.to_string(),
                                                new_handle.clone(),
                                            ));
                                            shape_name_look_up.insert(
                                                new_handle.clone(),
                                                box_object.frame.to_string(),
                                            );
                                            println!("added to the compund shape");
                                        }
                                        None => {
                                            println!("could not remove. please check !!!!");
                                        }
                                    }
                                }
                                None => {
                                    println!("could not get collider vector");
                                }
                            }
                        }
                        None => {
                            let box_collider = ColliderBuilder::new(box_shape)
                                .position(box_object.local_transform)
                                .active_events(ActiveEvents::CONTACT_EVENTS)
                                .user_data(1)
                                .build();
                            link_collider_set.insert(box_collider);
                            println! {"persistent shape added to the world frame"}
                        }
                    }
                }

                shapes::Shape::Cylinder(cylinder_object) => {
                    let new_length = cylinder_object.length / 2.0;
                    let cylinder_shape = SharedShape::cylinder(new_length, cylinder_object.radius);
                    let frame_existing_collider_handle =
                        collider_handle_look_up.get_mut(&cylinder_object.frame);
                    match frame_existing_collider_handle {
                        Some(collider_handle) => {
                            match collider_vector_look_up.get_mut(&cylinder_object.frame) {
                                Some(collider_vector) => {
                                    collider_vector.push((
                                        cylinder_object.local_transform,
                                        cylinder_shape.clone(),
                                    ));
                                    let new_collider =
                                        ColliderBuilder::compound(collider_vector.to_vec())
                                            .active_events(ActiveEvents::CONTACT_EVENTS)
                                            .user_data(0)
                                            .build();
                                    match link_collider_set.remove(
                                        *collider_handle,
                                        &mut island_manager,
                                        &mut robot_rigid_body_set,
                                        false,
                                    ) {
                                        Some(_collider) => {
                                            let new_handle = link_collider_set.insert(new_collider);
                                            link_group.push((
                                                cylinder_object.frame.to_string(),
                                                new_handle.clone(),
                                            ));
                                            shape_name_look_up.insert(
                                                new_handle.clone(),
                                                cylinder_object.frame.to_string(),
                                            );

                                            println! {"added to the compund shape"}
                                        }
                                        None => {
                                            println!("could not remove please check !!!!");
                                        }
                                    }
                                }
                                None => {
                                    println!("could not get collider vector");
                                }
                            }
                        }
                        None => {
                            let cylinder_collider = ColliderBuilder::new(cylinder_shape)
                                .position(cylinder_object.local_transform)
                                .active_events(ActiveEvents::CONTACT_EVENTS)
                                .user_data(1)
                                .build();
                            link_collider_set.insert(cylinder_collider);
                            println! {"persistent shape added to the world frame"}
                        }
                    }
                }

                shapes::Shape::Sphere(sphere_object) => {
                    let sphere_shape = SharedShape::ball(sphere_object.radius);
                    let frame_existing_collider_handle =
                        collider_handle_look_up.get_mut(&sphere_object.frame);
                    match frame_existing_collider_handle {
                        Some(collider_handle) => {
                            match collider_vector_look_up.get_mut(&sphere_object.frame) {
                                Some(collider_vector) => {
                                    collider_vector.push((
                                        sphere_object.local_transform,
                                        sphere_shape.clone(),
                                    ));
                                    let new_collider =
                                        ColliderBuilder::compound(collider_vector.to_vec())
                                            .active_events(ActiveEvents::CONTACT_EVENTS)
                                            .user_data(0)
                                            .build();
                                    match link_collider_set.remove(
                                        *collider_handle,
                                        &mut island_manager,
                                        &mut robot_rigid_body_set,
                                        false,
                                    ) {
                                        Some(_collider) => {
                                            let new_handle = link_collider_set.insert(new_collider);
                                            link_group.push((
                                                sphere_object.frame.to_string(),
                                                new_handle.clone(),
                                            ));
                                            shape_name_look_up.insert(
                                                new_handle.clone(),
                                                sphere_object.frame.to_string(),
                                            );
                                            println! {"added to the compund shape"}
                                        }
                                        None => {
                                            println!("could not remove please check !!!!");
                                        }
                                    }
                                }
                                None => {
                                    println!("could not get collider vector");
                                }
                            }
                        }
                        None => {
                            let sphere_collider = ColliderBuilder::new(sphere_shape)
                                .position(sphere_object.local_transform)
                                .active_events(ActiveEvents::CONTACT_EVENTS)
                                .user_data(1)
                                .build();
                            link_collider_set.insert(sphere_collider);
                            println! {"persistent shape added to the world frame"}
                        }
                    }
                }

                shapes::Shape::Capsule(capsule_object) => {
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
                    let frame_existing_collider_handle =
                        collider_handle_look_up.get_mut(&capsule_object.frame);
                    match frame_existing_collider_handle {
                        Some(collider_handle) => {
                            match collider_vector_look_up.get_mut(&capsule_object.frame) {
                                Some(collider_vector) => {
                                    collider_vector.push((
                                        capsule_object.local_transform,
                                        capsule_shape.clone(),
                                    ));
                                    let new_collider =
                                        ColliderBuilder::compound(collider_vector.to_vec())
                                            .active_events(ActiveEvents::CONTACT_EVENTS)
                                            .user_data(0)
                                            .build();
                                    match link_collider_set.remove(
                                        *collider_handle,
                                        &mut island_manager,
                                        &mut robot_rigid_body_set,
                                        false,
                                    ) {
                                        Some(_collider) => {
                                            let new_handle = link_collider_set.insert(new_collider);
                                            link_group.push((
                                                capsule_object.frame.to_string(),
                                                new_handle.clone(),
                                            ));
                                            shape_name_look_up.insert(
                                                new_handle.clone(),
                                                capsule_object.frame.to_string(),
                                            );

                                            println!("persistent shape added to the compund shape");
                                        }
                                        None => {
                                            println!("could not remove please check !!!!");
                                        }
                                    }
                                }
                                None => {
                                    println!("could not get collider vector");
                                }
                            }
                        }
                        None => {
                            let capsule_collider = ColliderBuilder::new(capsule_shape)
                                .position(capsule_object.local_transform)
                                .active_events(ActiveEvents::CONTACT_EVENTS)
                                .user_data(1)
                                .build();
                            link_collider_set.insert(capsule_collider);
                            println!("persistent shape added to the world frame");
                        }
                    }
                }

                shapes::Shape::Hull(hull_object) => {
                    let hull_points: Vec<Point3<f64>> = hull_object
                        .points
                        .iter()
                        .map(|p| Point3::new(p.x, p.y, p.z))
                        .collect();

                    let hull_shape = SharedShape::convex_hull(hull_points.as_slice());
                    match hull_shape {
                        Some(valid_hull_shape) => {
                            let frame_existing_collider_handle =
                                collider_handle_look_up.get_mut(&hull_object.frame);
                            match frame_existing_collider_handle {
                                Some(collider_handle) => {
                                    match collider_vector_look_up.get_mut(&hull_object.frame) {
                                        Some(collider_vector) => {
                                            collider_vector.push((
                                                hull_object.local_transform,
                                                valid_hull_shape.clone(),
                                            ));
                                            let new_collider =
                                                ColliderBuilder::compound(collider_vector.to_vec())
                                                    .active_events(ActiveEvents::CONTACT_EVENTS)
                                                    .user_data(0)
                                                    .build();
                                            match link_collider_set.remove(
                                                *collider_handle,
                                                &mut island_manager,
                                                &mut robot_rigid_body_set,
                                                false,
                                            ) {
                                                Some(_collider) => {
                                                    let new_handle =
                                                        link_collider_set.insert(new_collider);
                                                    link_group.push((
                                                        hull_object.frame.to_string(),
                                                        new_handle.clone(),
                                                    ));
                                                    shape_name_look_up.insert(
                                                        new_handle.clone(),
                                                        hull_object.frame.to_string(),
                                                    );
                                                    println!("persistent shape added to the compund shape")
                                                }
                                                None => {
                                                    println!("could not remove please check !!!!");
                                                }
                                            }
                                        }
                                        None => {
                                            println!("could not get the collider vector");
                                        }
                                    }
                                }
                                None => {
                                    let hull_collider = ColliderBuilder::new(valid_hull_shape)
                                        .position(hull_object.local_transform)
                                        .active_events(ActiveEvents::CONTACT_EVENTS)
                                        .user_data(1)
                                        .build();
                                    link_collider_set.insert(hull_collider);
                                    println!("persistent shape added to the world frame");
                                }
                            }
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

        Self {
            broad_phase,
            narrow_phase,
            link_group,
            link_collider_set,
            robot_rigid_body_set,
            transient_group,
            collider_vector_look_up,
            island_manager,
            collider_handle_look_up,
            collider_changed,
            shape_name_look_up,
        }
    }

    // pub fn set_robot_frames(&mut self, _frames: &HashMap<String, Isometry3<f64>>) {

    // }

    pub fn perform_updates(&mut self, shape_updates: &Vec<ShapeUpdate>) {
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
                            .active_events(ActiveEvents::CONTACT_EVENTS)
                            .user_data(physical)
                            .build();
                            let handle = self.link_collider_set.insert(box_collider);
                            self.transient_group.push((id.to_string(), handle));
                        }
                        shapes::Shape::Cylinder(cylinder_object) => {
                            let physical = if cylinder_object.physical { 2 } else { 1 };

                            let new_length = cylinder_object.length / 2.0;
                            let cylinder_collider = ColliderBuilder::new(SharedShape::cylinder(
                                new_length,
                                cylinder_object.radius,
                            ))
                            .position(cylinder_object.local_transform)
                            .active_events(ActiveEvents::CONTACT_EVENTS)
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
                                    .active_events(ActiveEvents::CONTACT_EVENTS)
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
                            .active_events(ActiveEvents::CONTACT_EVENTS)
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
                                        .active_events(ActiveEvents::CONTACT_EVENTS)
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

    pub fn get_proximity(&self, frames: &HashMap<String, Isometry3<f64>>) -> Vec<ProximityInfo> {
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

        let mut pair_event_vec = Vec::<BroadPhasePairEvent>::new();
        new_broad_phase.update(
            1.0,
            &mut new_link_collider_set,
            &new_collider_changed[0..new_collider_changed.len() - 1],
            &[],
            &mut pair_event_vec,
        );
        // let gravity = vector![0.0, 0.0, 0.0];
        // let integration_parameters = IntegrationParameters::default();
        let mut collision_pipeline = CollisionPipeline::new();
        // let mut joint_set = JointSet::new();
        // let mut ccd_solver = CCDSolver::new();
        //let mut rigid_body_set = RigidBodySet::new();
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

        for pairs in new_narrow_phase.contact_pairs() {
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
                            1.0,
                        ) {
                            Ok(valid_closest_points) => {
                                match valid_closest_points {
                                    ClosestPoints::Intersecting => {
                                        let shape_name1 =
                                            self.shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            self.shape_name_look_up.get(&handle2).unwrap();
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
                                            self.shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            self.shape_name_look_up.get(&handle2).unwrap();
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
                            }
                            Err(_) => {}
                        }
                    } else if first.user_data == 0 && second.user_data == 1 {
                        // println!("contacts found between compound shape and (non-physical transient shapes or world frame
                        //             persistent shapes)");
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
                            1.0,
                        ) {
                            Ok(valid_closest_points) => {
                                match valid_closest_points {
                                    ClosestPoints::Intersecting => {
                                        let shape_name1 =
                                            self.shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            self.shape_name_look_up.get(&handle2).unwrap();
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
                                            self.shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            self.shape_name_look_up.get(&handle2).unwrap();
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
                            }
                            Err(_) => {}
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
                            1.0,
                        ) {
                            Ok(valid_closest_points) => {
                                match valid_closest_points {
                                    ClosestPoints::Intersecting => {
                                        let shape_name1 =
                                            self.shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            self.shape_name_look_up.get(&handle2).unwrap();
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
                                            self.shape_name_look_up.get(&handle1).unwrap();
                                        let shape_name2 =
                                            self.shape_name_look_up.get(&handle2).unwrap();
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
                            }
                            Err(_) => {}
                        }
                    } else {
                        // println!("contact found between other shapes are ignored");
                    }
                }
                _ => {}
            }
        }

        return vec![];
    }
}
