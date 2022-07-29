use crate::utils::info::{LinkInfo, ProximityInfo, ShapeUpdate};
use crate::utils::shapes;
use crate::utils::state::State;
use array2d::Array2D;
use nalgebra::base::Vector3;
use nalgebra::geometry::Isometry3;
use nalgebra::vector;
use nalgebra::Point3;
use parry3d_f64::query::closest_points::*;
use parry3d_f64::query::contact::Contact;
use parry3d_f64::shape::*;
use profiling::scope;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::fmt;
use std::time::{Duration, Instant};

const D_MAX: f64 = 1.0;
const R: f64 = 0.0;
const A_MAX: f64 = 0.5;
const TIME_BUDGET: Duration = Duration::from_micros(100);
const ACCURACY_BUDGET: f64 = 0.1;
const TIMED: bool = true;

// use log::info;

// use log::info;

#[derive(Clone)]
pub struct CollisionManager {
    scene_compound_shapes_list: Vec<(String, Compound)>,
    scene_transient_shapes_look_up: HashMap<String, usize>,
    scene_group_truth_distance_grid:
        Array2D<Option<(ProximityInfo, Isometry3<f64>, Isometry3<f64>)>>,
    scene_a_table: Array2D<f64>,
}

impl fmt::Debug for CollisionManager {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CollisionManager").finish()
    }
}

impl CollisionManager {
    #[profiling::function]
    pub fn new(links: Vec<LinkInfo>, persistent_shapes: Vec<shapes::Shape>) -> Self {
        let mut scene_compound_shapes_list: Vec<(String, Compound)> = vec![];
        let scene_transient_shapes_look_up = HashMap::new();
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

            if robot_shapes_list.len() != 0 {
                let robot_compound_shapes = Compound::new(robot_shapes_list);
                scene_compound_shapes_list.push((frame_name.to_string(), robot_compound_shapes));
            }
        }

        for shape in &persistent_shapes {
            match shape {
                shapes::Shape::Box(box_object) => {
                    let box_shape = SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                    if box_object.frame == "world" {
                        let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                            vec![(box_object.local_transform, box_shape)];
                        let temp_compound = Compound::new(temp_list);
                        scene_compound_shapes_list.push(("world".to_string(), temp_compound));
                    } else {
                        let index_element = scene_compound_shapes_list
                            .iter()
                            .position(|x| x.0 == box_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_compound_shapes_list.clone();
                                let (frame_name, compound_shape) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape = compound_shape.shapes().to_vec();
                                new_compound_shape.push((box_object.local_transform, box_shape));
                                scene_compound_shapes_list.remove(valid_index);
                                scene_compound_shapes_list.push((
                                    frame_name.to_string(),
                                    Compound::new(new_compound_shape),
                                ));
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(box_object.local_transform, box_shape)];
                                let temp_compound = Compound::new(temp_list);
                                scene_compound_shapes_list
                                    .push((box_object.frame.to_string(), temp_compound));
                            }
                        }
                    }
                }
                shapes::Shape::Cylinder(cylinder_object) => {
                    let new_length = cylinder_object.length / 2.0;
                    let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                    let cylinder_shape = SharedShape::cylinder(new_length, cylinder_object.radius);
                    if cylinder_object.frame == "world" {
                        let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                            vec![(cylinder_object.local_transform, cylinder_shape)];
                        let temp_compound = Compound::new(temp_list);
                        scene_compound_shapes_list.push(("world".to_string(), temp_compound));
                    } else {
                        let index_element = scene_compound_shapes_list
                            .iter()
                            .position(|x| x.0 == cylinder_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_compound_shapes_list.clone();
                                let (frame_name, compound_shape) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape = compound_shape.shapes().to_vec();
                                new_compound_shape.push((
                                    cylinder_object.local_transform * transform_offset,
                                    cylinder_shape,
                                ));
                                scene_compound_shapes_list.remove(valid_index);
                                scene_compound_shapes_list.push((
                                    frame_name.to_string(),
                                    Compound::new(new_compound_shape),
                                ));
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> = vec![(
                                    cylinder_object.local_transform * transform_offset,
                                    cylinder_shape,
                                )];
                                let temp_compound = Compound::new(temp_list);
                                scene_compound_shapes_list
                                    .push((cylinder_object.frame.to_string(), temp_compound));
                            }
                        }
                    }
                }
                shapes::Shape::Sphere(sphere_object) => {
                    let sphere_shape = SharedShape::ball(sphere_object.radius);
                    if sphere_object.frame == "world" {
                        let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                            vec![(sphere_object.local_transform, sphere_shape)];
                        let temp_compound = Compound::new(temp_list);
                        scene_compound_shapes_list.push(("world".to_string(), temp_compound));
                    } else {
                        let index_element = scene_compound_shapes_list
                            .iter()
                            .position(|x| x.0 == sphere_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_compound_shapes_list.clone();
                                let (frame_name, compound_shape) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape = compound_shape.shapes().to_vec();
                                new_compound_shape
                                    .push((sphere_object.local_transform, sphere_shape));
                                scene_compound_shapes_list.remove(valid_index);
                                scene_compound_shapes_list.push((
                                    frame_name.to_string(),
                                    Compound::new(new_compound_shape),
                                ));
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(sphere_object.local_transform, sphere_shape)];
                                let temp_compound = Compound::new(temp_list);
                                scene_compound_shapes_list
                                    .push((sphere_object.frame.to_string(), temp_compound));
                            }
                        }
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
                    let capsule_shape =
                        SharedShape::capsule(point_a, point_b, capsule_object.radius);
                    if capsule_object.frame == "world" {
                        let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                            vec![(capsule_object.local_transform, capsule_shape)];
                        let temp_compound = Compound::new(temp_list);
                        scene_compound_shapes_list.push(("world".to_string(), temp_compound));
                    } else {
                        let index_element = scene_compound_shapes_list
                            .iter()
                            .position(|x| x.0 == capsule_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_compound_shapes_list.clone();
                                let (frame_name, compound_shape) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape = compound_shape.shapes().to_vec();
                                new_compound_shape
                                    .push((capsule_object.local_transform, capsule_shape));
                                scene_compound_shapes_list.remove(valid_index);
                                scene_compound_shapes_list.push((
                                    frame_name.to_string(),
                                    Compound::new(new_compound_shape),
                                ));
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(capsule_object.local_transform, capsule_shape)];
                                let temp_compound = Compound::new(temp_list);
                                scene_compound_shapes_list
                                    .push((capsule_object.frame.to_string(), temp_compound));
                            }
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
                    if hull_object.frame == "world" {
                        match hull_shape {
                            Some(valid_hull_shape) => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(hull_object.local_transform, valid_hull_shape)];
                                let temp_compound = Compound::new(temp_list);
                                scene_compound_shapes_list
                                    .push(("world".to_string(), temp_compound));
                            }
                            None => {
                                println!("the given points cannot form a valid hull shape");
                            }
                        }
                    } else {
                        match hull_shape {
                            Some(valid_hull_shape) => {
                                let index_element = scene_compound_shapes_list
                                    .iter()
                                    .position(|x| x.0 == hull_object.frame);
                                match index_element {
                                    Some(valid_index) => {
                                        let temp_scene_compound_shapes_list =
                                            scene_compound_shapes_list.clone();
                                        let (frame_name, compound_shape) =
                                            temp_scene_compound_shapes_list
                                                .get(valid_index)
                                                .unwrap();
                                        let mut new_compound_shape =
                                            compound_shape.shapes().to_vec();
                                        new_compound_shape
                                            .push((hull_object.local_transform, valid_hull_shape));
                                        scene_compound_shapes_list.remove(valid_index);
                                        scene_compound_shapes_list.push((
                                            frame_name.to_string(),
                                            Compound::new(new_compound_shape),
                                        ));
                                    }
                                    None => {
                                        let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(hull_object.local_transform, valid_hull_shape)];
                                        let temp_compound = Compound::new(temp_list);
                                        scene_compound_shapes_list
                                            .push((hull_object.frame.to_string(), temp_compound));
                                    }
                                }
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

        let scene_compound_shapes_list_size = scene_compound_shapes_list.len();
        let scene_a_table = Array2D::filled_with(
            1.0,
            scene_compound_shapes_list_size,
            scene_compound_shapes_list_size,
        );

        let scene_group_truth_distance_grid = Array2D::fill_with(
            None,
            scene_compound_shapes_list_size,
            scene_compound_shapes_list_size,
        );

        Self {
            scene_compound_shapes_list,
            scene_transient_shapes_look_up,
            scene_group_truth_distance_grid,
            scene_a_table,
        }
    }

    #[profiling::function]
    pub fn compute_ground_truth_distance_grid(
        &mut self,
        initial_frames: &HashMap<String, Isometry3<f64>>,
    ) {
        let size = self.scene_compound_shapes_list.len();
        for i in 0..=size - 1 {
            for j in (i + 1)..=size - 1 {
                let (shape1_frame, shape1) = self.scene_compound_shapes_list.get(i).unwrap();
                let (shape2_frame, shape2) = self.scene_compound_shapes_list.get(j).unwrap();
                if shape1_frame == "world" && shape2_frame == "world" {
                    continue;
                } else {
                    let shape1_transform = initial_frames.get(shape1_frame);
                    match shape1_transform {
                        Some(shape1_transform) => {
                            let shape2_transform = initial_frames.get(shape2_frame);
                            match shape2_transform {
                                Some(shape2_transform) => {
                                    let contact = parry3d_f64::query::contact(
                                        shape1_transform,
                                        shape1,
                                        shape2_transform,
                                        shape2,
                                        D_MAX,
                                    );
                                    match contact {
                                        Ok(contact) => match contact {
                                            Some(valid_contact) => {
                                                let proximity = ProximityInfo::new(
                                                    shape1_frame.to_string(),
                                                    shape2_frame.to_string(),
                                                    valid_contact.dist,
                                                    Some((
                                                        valid_contact.point1,
                                                        valid_contact.point2,
                                                    )),
                                                    true,
                                                );
                                                self.scene_group_truth_distance_grid
                                                    .set(
                                                        i,
                                                        j,
                                                        Some((
                                                            proximity,
                                                            *shape1_transform,
                                                            *shape2_transform,
                                                        )),
                                                    )
                                                    .unwrap();
                                            }
                                            None => {}
                                        },
                                        Err(_) => {}
                                    }
                                }
                                None => {}
                            }
                        }
                        None => {}
                    }
                }
            }
        }
    }

    #[profiling::function]
    pub fn update_ground_truth_table(&mut self, current_state: &mut State) {}

    // pub fn set_robot_frames(&mut self, _frames: &HashMap<String, Isometry3<f64>>) {

    // }
    #[profiling::function]
    pub fn perform_updates(&mut self, shape_updates: &Vec<ShapeUpdate>) {
        //scope!("perform updates");
        for update in shape_updates {
            let mut update_shapes_list: Vec<(Isometry3<f64>, SharedShape)> = Vec::new();
            match update {
                ShapeUpdate::Add { id, shape } => {
                    match shape {
                        shapes::Shape::Box(box_object) => {
                            let box_collider =
                                SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                            update_shapes_list.push((box_object.local_transform, box_collider));
                            self.scene_compound_shapes_list.push((
                                box_object.frame.clone(),
                                Compound::new(update_shapes_list),
                            ));
                            let last_index = self.scene_compound_shapes_list.len() - 1;
                            self.scene_transient_shapes_look_up
                                .insert(id.to_string(), last_index);
                            //self.scene_transient_shapes_list.push((id.to_string(), box_collider));
                        }
                        shapes::Shape::Cylinder(cylinder_object) => {
                            //let physical = if cylinder_object.physical { 2 } else { 1 };

                            let new_length = cylinder_object.length / 2.0;
                            let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                            let cylinder_collider =
                                SharedShape::cylinder(new_length, cylinder_object.radius);
                            update_shapes_list.push((
                                cylinder_object.local_transform * transform_offset,
                                cylinder_collider,
                            ));
                            self.scene_compound_shapes_list.push((
                                cylinder_object.frame.clone(),
                                Compound::new(update_shapes_list),
                            ));
                            let last_index = self.scene_compound_shapes_list.len() - 1;
                            self.scene_transient_shapes_look_up
                                .insert(id.to_string(), last_index);
                            //self.scene_transient_shapes_list.push((id.to_string(), cylinder_collider));
                        }
                        shapes::Shape::Sphere(sphere_object) => {
                            //let physical = if sphere_object.physical { 2 } else { 1 };

                            let sphere_collider = SharedShape::ball(sphere_object.radius);
                            // self.scene_transient_shapes_list.push((id.to_string(), sphere_collider));
                            update_shapes_list
                                .push((sphere_object.local_transform, sphere_collider));
                            self.scene_compound_shapes_list.push((
                                sphere_object.frame.clone(),
                                Compound::new(update_shapes_list),
                            ));
                            let last_index = self.scene_compound_shapes_list.len() - 1;
                            self.scene_transient_shapes_look_up
                                .insert(id.to_string(), last_index);
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
                            let capsule_collider =
                                SharedShape::capsule(point_a, point_b, capsule_object.radius);

                            update_shapes_list
                                .push((capsule_object.local_transform, capsule_collider));
                            self.scene_compound_shapes_list.push((
                                capsule_object.frame.clone(),
                                Compound::new(update_shapes_list),
                            ));
                            let last_index = self.scene_compound_shapes_list.len() - 1;
                            self.scene_transient_shapes_look_up
                                .insert(id.to_string(), last_index);

                            //self.scene_transient_shapes_list.push((id.to_string(), capsule_collider));
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
                                    //self.scene_transient_shapes_list.push((id.to_string(), valid_hull_shape));
                                    update_shapes_list
                                        .push((hull_object.local_transform, valid_hull_shape));
                                    self.scene_compound_shapes_list.push((
                                        hull_object.frame.clone(),
                                        Compound::new(update_shapes_list),
                                    ));
                                    let last_index = self.scene_compound_shapes_list.len() - 1;
                                    self.scene_transient_shapes_look_up
                                        .insert(id.to_string(), last_index);
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
                    // let mut counter = 0;
                    // for (transient_id, transient_handle) in &mut self.scene_transient_shapes_list {
                    //     if transient_id == id {
                    //         break;
                    //     }
                    //     counter += 1;
                    // }

                    // self.scene_transient_shapes_list.remove(counter);
                }
            }
        }
    }
    #[profiling::function]
    pub fn clear_all_transient_shapes(&mut self) {
        //  self.scene_transient_shapes_list.clear();
    }

    #[profiling::function]
    pub fn compute_relative_change_in_transform(
        &self,
        shape1_current_frame: &String,
        shape2_current_frame: &String,
        current_frame: &HashMap<String, Isometry3<f64>>,
        shape1_j_transform: &Isometry3<f64>,
        shape2_j_transform: &Isometry3<f64>,
    ) -> Option<(f64, f64)> {
        let DEFAULT_FRAME_TRANSFORM: Isometry3<f64> = Isometry3::<f64>::identity();
        let shape1_j_translation = shape1_j_transform.translation;
        let shape1_j_rotation = shape1_j_transform.rotation;

        let shape1_k_translation = current_frame
            .get(shape1_current_frame)
            .unwrap_or(&DEFAULT_FRAME_TRANSFORM)
            .translation;
        let shape1_current_rotation = current_frame
            .get(shape1_current_frame)
            .unwrap_or(&DEFAULT_FRAME_TRANSFORM)
            .rotation;

        let shape2_j_translation = shape2_j_transform.translation;
        let shape2_j_rotation = shape2_j_transform.rotation;
        let shape2_current_translation = current_frame
            .get(shape2_current_frame)
            .unwrap_or(&DEFAULT_FRAME_TRANSFORM)
            .translation;
        let shape2_current_rotation = current_frame
            .get(shape2_current_frame)
            .unwrap_or(&DEFAULT_FRAME_TRANSFORM)
            .rotation;

        let change_in_relative_translation =
            ((shape1_j_translation.vector - shape2_j_translation.vector).norm()
                - (shape1_k_translation.vector - shape2_current_translation.vector).norm())
            .abs();
        let change_in_relative_rotation = (shape1_j_rotation
            .rotation_to(&shape2_j_rotation)
            .rotation_to(&shape1_current_rotation.rotation_to(&shape2_current_rotation)))
        .angle();

        return Some((change_in_relative_translation, change_in_relative_rotation));
    }
    #[profiling::function]
    pub fn compute_bounding_spheres(&self, compound_shape: &Compound) -> f64 {
        return compound_shape.local_bounding_sphere().radius;
    }

    #[profiling::function]
    pub fn compute_translation_of_a_point_induced_by_rotation(
        &self,
        distance: f64,
        rotation: f64,
    ) -> f64 {
        let result = f64::sqrt(2.0 * (distance * distance) * (1.0 - rotation.cos()));
        return result;
    }

    #[profiling::function]
    pub fn compute_lower_signed_distance_bound(
        &self,
        shape1: &Compound,
        shape1_frame: &String,
        shape2: &Compound,
        shape2_frame: &String,
        current_frame: &HashMap<String, Isometry3<f64>>,
        j_state: &(ProximityInfo, Isometry3<f64>, Isometry3<f64>),
    ) -> f64 {
        // let distance_between_shapes = self.
        let mut lower_bound = 0.0;
        let valid_distance = j_state.0.distance;
        match self.compute_relative_change_in_transform(
            shape1_frame,
            shape2_frame,
            current_frame,
            &j_state.1,
            &j_state.2,
        ) {
            Some((change_in_translation, change_in_rotation)) => {
                let shape1_bounding_sphere_radius = self.compute_bounding_spheres(shape1);
                let shape2_bounding_sphere_radius = self.compute_bounding_spheres(shape2);
                let mut max_bounding_sphere = 0.0;
                if shape1_bounding_sphere_radius > shape2_bounding_sphere_radius {
                    max_bounding_sphere = shape1_bounding_sphere_radius;
                } else if shape1_bounding_sphere_radius < shape2_bounding_sphere_radius {
                    max_bounding_sphere = shape2_bounding_sphere_radius;
                } else {
                    max_bounding_sphere = shape1_bounding_sphere_radius;
                }

                lower_bound = valid_distance
                    - change_in_translation
                    - self.compute_translation_of_a_point_induced_by_rotation(
                        max_bounding_sphere,
                        change_in_rotation,
                    );
            }
            None => {}
        }
        return lower_bound;
    }

    #[profiling::function]
    pub fn compute_upper_signed_distance_bound(
        &self,
        shape1_frame: &String,
        shape2_frame: &String,
        current_state: &HashMap<String, Isometry3<f64>>,
        j_state: &(ProximityInfo, Isometry3<f64>, Isometry3<f64>),
    ) -> f64 {
        let DEFAULT_FRAME_TRANSFORM: Isometry3<f64> = Isometry3::<f64>::identity();
        let shape1_current_state_rotation = current_state
            .get(shape1_frame)
            .unwrap_or(&DEFAULT_FRAME_TRANSFORM)
            .rotation;
        let shape1_past_state_rotation = j_state.1.rotation;
        let shape2_current_state_rotation = current_state
            .get(shape2_frame)
            .unwrap_or(&DEFAULT_FRAME_TRANSFORM)
            .rotation;
        let shape2_past_state_rotation = j_state.2.rotation;

        let shape1_current_state_translation = current_state
            .get(shape1_frame)
            .unwrap_or(&DEFAULT_FRAME_TRANSFORM)
            .translation;
        let shape1_past_state_translation = j_state.1.translation;
        let shape2_current_state_translation = current_state
            .get(shape2_frame)
            .unwrap_or(&DEFAULT_FRAME_TRANSFORM)
            .translation;
        let shape2_past_state_translation = j_state.2.translation;

        let mut final_result = 0.0;

        match j_state.0.points {
            Some((point1, point2)) => {
                let result1 = shape1_current_state_rotation
                    * ((shape1_past_state_rotation.inverse()
                        * (point1.coords - shape1_past_state_translation.vector))
                        + shape1_current_state_translation.vector);
                let result2 = shape2_current_state_rotation
                    * ((shape2_past_state_rotation.inverse()
                        * (point2.coords - shape2_past_state_translation.vector))
                        + shape2_current_state_translation.vector);
                final_result = (result1 - result2).norm();
            }
            None => {}
        }

        return final_result;
    }

    #[profiling::function]
    pub fn compute_loss_function(&self, x: &f64) -> f64 {
        let mut result = 0.0;
        if *x <= 0.0 {
            result = -*x + 1.0;
        } else {
            let c = 0.2 * A_MAX;

            result = (-(*x * *x) / 2.0 * c * c).exp();
        }
        return result;
    }

    //compute loss with cutoff
    #[profiling::function]
    pub fn compute_loss_with_cutoff(&self, x: &f64, a_value: &f64) -> f64 {
        let mut result = 0.0;

        if *x >= D_MAX || *x / *a_value >= A_MAX {
            result = 0.0;
        } else {
            result = self.compute_loss_function(x);
        }
        return result;
    }

    #[profiling::function]
    pub fn compute_maximum_loss_functions_error(
        &self,
        shape1: &Compound,
        shape1_frame: &String,
        shape2: &Compound,
        shape2_frame: &String,
        current_frame: &HashMap<String, Isometry3<f64>>,
        j_state: &(ProximityInfo, Isometry3<f64>, Isometry3<f64>),
    ) -> f64 {
        let mut result = 0.0;

        let lower_bound = self.compute_lower_signed_distance_bound(
            shape1,
            shape1_frame,
            shape2,
            shape2_frame,
            current_frame,
            j_state,
        );
        let upper_bound = self.compute_upper_signed_distance_bound(
            shape1_frame,
            shape2_frame,
            current_frame,
            j_state,
        );

        let estimated_distance = (1.0 - R) * lower_bound + R * upper_bound;
        let i = self
            .scene_compound_shapes_list
            .iter()
            .position(|x| x.0 == *shape1_frame)
            .unwrap();
        let j = self
            .scene_compound_shapes_list
            .iter()
            .position(|x| x.0 == *shape2_frame)
            .unwrap();
        let a_value = self.scene_a_table[(i, j)];
        let loss_value_distance = self.compute_loss_with_cutoff(&estimated_distance, &a_value);
        let loss_value_lower_bound = self.compute_loss_with_cutoff(&lower_bound, &a_value);
        let loss_value_upper_bound = self.compute_loss_with_cutoff(&upper_bound, &a_value);
        if (loss_value_lower_bound - loss_value_distance)
            == (loss_value_distance - loss_value_upper_bound)
        {
            result = loss_value_lower_bound.clone() - loss_value_distance.clone();
        } else if (loss_value_distance - loss_value_upper_bound)
            >= (loss_value_distance - loss_value_upper_bound)
        {
            result = loss_value_lower_bound.clone() - loss_value_distance.clone();
        } else {
            result = loss_value_distance.clone() - loss_value_upper_bound.clone();
        }

        return result;
    }

    #[profiling::function]
    pub fn ranking_maximum_loss_functions_error(
        &self,
        current_frame: &HashMap<String, Isometry3<f64>>,
    ) -> Vec<(String, Compound, String, Compound, f64, usize, usize)> {
        let mut loss_functions_error_vec: Vec<(
            String,
            Compound,
            String,
            Compound,
            f64,
            usize,
            usize,
        )> = vec![];
        let size = self.scene_compound_shapes_list.len();
        for i in 0..=size - 1 {
            for j in (i + 1)..=size - 1 {
                let (shape1_frame, shape1) = self.scene_compound_shapes_list.get(i).unwrap();
                let (shape2_frame, shape2) = self.scene_compound_shapes_list.get(j).unwrap();
                if shape1_frame == "world" && shape2_frame == "world" {
                    continue;
                } else {
                    match self.scene_group_truth_distance_grid.get(i, j) {
                        Some(element) => match element {
                            Some(valid_element) => {
                                let current_loss_function_error = self
                                    .compute_maximum_loss_functions_error(
                                        shape1,
                                        shape1_frame,
                                        shape2,
                                        shape2_frame,
                                        current_frame,
                                        valid_element,
                                    );

                                loss_functions_error_vec.push((
                                    shape1_frame.to_string(),
                                    shape1.clone(),
                                    shape2_frame.to_string(),
                                    shape2.clone(),
                                    current_loss_function_error,
                                    i,
                                    j,
                                ));
                            }
                            None => {}
                        },
                        None => {}
                    }
                }
            }
        }
        loss_functions_error_vec.sort_by(|a, b| a.4.partial_cmp(&b.4).unwrap());

        return loss_functions_error_vec;
    }

    #[profiling::function]
    pub fn get_proximity(
        &mut self,
        frames: &HashMap<String, Isometry3<f64>>,
    ) -> Vec<ProximityInfo> {
        let DEFAULT_FRAME_TRANSFORM: Isometry3<f64> = Isometry3::<f64>::identity();
        // let shape1_current_state_rotation =
        //     current_state.get(shape1_frame).unwrap_or(&DEFAULT_FRAME_TRANSFORM).rotation;
        let size = self.scene_compound_shapes_list.len();
        let mut result_vector: Vec<ProximityInfo> = vec![];

        let ranking_vector: Vec<(String, Compound, String, Compound, f64, usize, usize)> =
            self.ranking_maximum_loss_functions_error(frames);

        if TIMED {
            let timed_timer = Instant::now();
            for (shape1_frame, shape1, shape2_frame, shape2, _, i, j) in ranking_vector {
                if timed_timer.elapsed().as_micros() < TIME_BUDGET.as_micros() {
                    let shape1_transform = frames
                        .get(&shape1_frame)
                        .unwrap_or(&DEFAULT_FRAME_TRANSFORM);
                    let shape2_transform = frames
                        .get(&shape2_frame)
                        .unwrap_or(&DEFAULT_FRAME_TRANSFORM);
                    let contact = parry3d_f64::query::contact(
                        shape1_transform,
                        &shape1,
                        shape2_transform,
                        &shape2,
                        D_MAX,
                    );
                    match contact {
                        Ok(contact) => match contact {
                            Some(valid_contact) => {
                                let proximity = ProximityInfo::new(
                                    shape1_frame,
                                    shape2_frame,
                                    valid_contact.dist,
                                    Some((valid_contact.point1, valid_contact.point2)),
                                    true,
                                );

                                self.scene_group_truth_distance_grid
                                    .set(
                                        i,
                                        j,
                                        Some((proximity, *shape1_transform, *shape2_transform)),
                                    )
                                    .unwrap();
                            }
                            None => {}
                        },
                        Err(_) => {}
                    }
                }
            }
        } else {
            let mut remaining_error_summation = 0.0;
            let mut index = 0;

            for (_, _, _, _, loss_value, i, j) in ranking_vector.clone() {
                remaining_error_summation += loss_value;
            }

            while remaining_error_summation > ACCURACY_BUDGET
                || index < ranking_vector.clone().len() - 1
            {
                let (
                    current_shape1_frame,
                    current_shape1,
                    current_shape2_frame,
                    current_shape2,
                    current_loss_value,
                    i,
                    j,
                ) = ranking_vector.get(index).unwrap();

                let shape1_transform = frames.get(current_shape1_frame).unwrap_or(&DEFAULT_FRAME_TRANSFORM);
                let shape2_transform = frames.get(current_shape2_frame).unwrap_or(&DEFAULT_FRAME_TRANSFORM);
                let contact = parry3d_f64::query::contact(
                    shape1_transform,
                    current_shape1,
                    shape2_transform,
                    current_shape2,
                    D_MAX,
                );
                match contact {
                    Ok(contact) => match contact {
                        Some(valid_contact) => {
                            let proximity = ProximityInfo::new(
                                current_shape1_frame.clone(),
                                current_shape2_frame.clone(),
                                valid_contact.dist,
                                Some((valid_contact.point1, valid_contact.point2)),
                                true,
                            );

                            self.scene_group_truth_distance_grid
                                .set(
                                    *i,
                                    *j,
                                    Some((proximity, *shape1_transform, *shape2_transform)),
                                )
                                .unwrap();
                        }
                        None => {}
                    },
                    Err(_) => {}
                }
                remaining_error_summation -= current_loss_value;
                index += 1;
            }
        }

        return result_vector;
    }
}
