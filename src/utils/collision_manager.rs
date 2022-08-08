use crate::utils::info::{LinkInfo, ProximityInfo, ShapeUpdate};
use crate::utils::shapes;
use array2d::Array2D;
use nalgebra::base::Vector3;
use nalgebra::geometry::Isometry3;
use nalgebra::vector;
use nalgebra::Point3;
use parry3d_f64::query::closest_points::ClosestPoints;
use parry3d_f64::shape::*;
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
const A_VALUE: f64 = 1.0;
const OPTIMA_NUMBER: usize = 500;

// use log::info;

// use log::info;

#[derive(Clone)]
pub struct CollisionManager {
    scene_compound_shapes_list: Vec<(String, Compound, f64)>,
    scene_transient_shapes_look_up: HashMap<String, (usize, usize)>,
    scene_group_truth_distance_grid:
        Array2D<Option<(ProximityInfo, Isometry3<f64>, Isometry3<f64>)>>,
    optima_version: bool, //scene_a_table: Array2D<f64>,
}

impl fmt::Debug for CollisionManager {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CollisionManager").finish()
    }
}

impl CollisionManager {
    #[profiling::function]
    pub fn new(links: Vec<LinkInfo>, persistent_shapes: Vec<shapes::Shape>) -> Self {
        let mut optima_version = false;
        let mut scene_compound_shapes_list: Vec<(String, Compound, f64)> = vec![];
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
                let bounding_sphere_radius = robot_compound_shapes.local_bounding_sphere().radius;
                scene_compound_shapes_list.push((
                    frame_name.to_string(),
                    robot_compound_shapes,
                    bounding_sphere_radius,
                ));
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
                        let bounding_sphere_radius = temp_compound.local_bounding_sphere().radius;
                        scene_compound_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            bounding_sphere_radius,
                        ));
                    } else {
                        let index_element = scene_compound_shapes_list
                            .iter()
                            .position(|x| x.0 == box_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_compound_shapes_list.clone();
                                let (frame_name, compound_shape, _) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape_vec = compound_shape.shapes().to_vec();
                                new_compound_shape_vec
                                    .push((box_object.local_transform, box_shape));
                                let new_compound_shape = Compound::new(new_compound_shape_vec);
                                let bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                scene_compound_shapes_list.remove(valid_index);
                                scene_compound_shapes_list.insert(valid_index,(
                                    frame_name.to_string(),
                                    new_compound_shape,
                                    bounding_sphere_radius,
                                ));
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(box_object.local_transform, box_shape)];
                                let temp_compound = Compound::new(temp_list);
                                let bounding_sphere_radius =
                                    temp_compound.local_bounding_sphere().radius;
                                scene_compound_shapes_list.push((
                                    box_object.frame.to_string(),
                                    temp_compound,
                                    bounding_sphere_radius,
                                ));
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
                        let bounding_sphere_radius = temp_compound.local_bounding_sphere().radius;
                        scene_compound_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            bounding_sphere_radius,
                        ));
                    } else {
                        let index_element = scene_compound_shapes_list
                            .iter()
                            .position(|x| x.0 == cylinder_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_compound_shapes_list.clone();
                                let (frame_name, compound_shape, _) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape_vec = compound_shape.shapes().to_vec();
                                new_compound_shape_vec.push((
                                    cylinder_object.local_transform * transform_offset,
                                    cylinder_shape,
                                ));
                                let new_compound_shape = Compound::new(new_compound_shape_vec);
                                let bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;

                                scene_compound_shapes_list.remove(valid_index);
                                scene_compound_shapes_list.insert(valid_index,(
                                    frame_name.to_string(),
                                    new_compound_shape,
                                    bounding_sphere_radius,
                                ));
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> = vec![(
                                    cylinder_object.local_transform * transform_offset,
                                    cylinder_shape,
                                )];
                                let temp_compound = Compound::new(temp_list);
                                let bounding_sphere_radius =
                                    temp_compound.local_bounding_sphere().radius;
                                scene_compound_shapes_list.push((
                                    cylinder_object.frame.to_string(),
                                    temp_compound,
                                    bounding_sphere_radius,
                                ));
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
                        let bounding_sphere_radius = temp_compound.local_bounding_sphere().radius;
                        scene_compound_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            bounding_sphere_radius,
                        ));
                    } else {
                        let index_element = scene_compound_shapes_list
                            .iter()
                            .position(|x| x.0 == sphere_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_compound_shapes_list.clone();
                                let (frame_name, compound_shape, _) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape_vec = compound_shape.shapes().to_vec();
                                new_compound_shape_vec
                                    .push((sphere_object.local_transform, sphere_shape));
                                let new_compound_shape = Compound::new(new_compound_shape_vec);
                                let bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                scene_compound_shapes_list.remove(valid_index);
                                scene_compound_shapes_list.insert(valid_index,(
                                    frame_name.to_string(),
                                    new_compound_shape,
                                    bounding_sphere_radius,
                                ));
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(sphere_object.local_transform, sphere_shape)];
                                let temp_compound = Compound::new(temp_list);
                                let bounding_sphere_radius =
                                    temp_compound.local_bounding_sphere().radius();
                                scene_compound_shapes_list.push((
                                    sphere_object.frame.to_string(),
                                    temp_compound,
                                    bounding_sphere_radius,
                                ));
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
                        let temp_compound_radius = temp_compound.local_bounding_sphere().radius;
                        scene_compound_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            temp_compound_radius,
                        ));
                    } else {
                        let index_element = scene_compound_shapes_list
                            .iter()
                            .position(|x| x.0 == capsule_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_compound_shapes_list.clone();
                                let (frame_name, compound_shape, _) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape_vec = compound_shape.shapes().to_vec();
                                new_compound_shape_vec
                                    .push((capsule_object.local_transform, capsule_shape));
                                let new_compound_shape = Compound::new(new_compound_shape_vec);
                                let bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                scene_compound_shapes_list.remove(valid_index);
                                scene_compound_shapes_list.insert(valid_index,(
                                    frame_name.to_string(),
                                    new_compound_shape,
                                    bounding_sphere_radius,
                                ));
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(capsule_object.local_transform, capsule_shape)];
                                let temp_compound = Compound::new(temp_list);
                                let temp_compound_radius =
                                    temp_compound.local_bounding_sphere().radius;
                                scene_compound_shapes_list.push((
                                    capsule_object.frame.to_string(),
                                    temp_compound,
                                    temp_compound_radius,
                                ));
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
                                let bounding_sphere_radius =
                                    temp_compound.local_bounding_sphere().radius;
                                scene_compound_shapes_list.push((
                                    "world".to_string(),
                                    temp_compound,
                                    bounding_sphere_radius,
                                ));
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
                                        let (frame_name, compound_shape, _) =
                                            temp_scene_compound_shapes_list
                                                .get(valid_index)
                                                .unwrap();
                                        let mut new_compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        new_compound_shape_vec
                                            .push((hull_object.local_transform, valid_hull_shape));
                                        let new_compound_shape =
                                            Compound::new(new_compound_shape_vec);
                                        let bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        scene_compound_shapes_list.remove(valid_index);
                                        scene_compound_shapes_list.insert(valid_index,(
                                            frame_name.to_string(),
                                            new_compound_shape,
                                            bounding_sphere_radius,
                                        ));
                                    }
                                    None => {
                                        let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(hull_object.local_transform, valid_hull_shape)];
                                        let temp_compound = Compound::new(temp_list);
                                        let temp_compound_bounding_sphere_radius =
                                            temp_compound.local_bounding_sphere().radius;
                                        scene_compound_shapes_list.push((
                                            hull_object.frame.to_string(),
                                            temp_compound,
                                            temp_compound_bounding_sphere_radius,
                                        ));
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

        if scene_compound_shapes_list_size >= OPTIMA_NUMBER {
            optima_version = true;
        }

        let scene_group_truth_distance_grid = Array2D::filled_with(
            None,
            scene_compound_shapes_list_size,
            scene_compound_shapes_list_size,
        );

        Self {
            scene_compound_shapes_list,
            scene_transient_shapes_look_up,
            scene_group_truth_distance_grid,
            optima_version,
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
                let (shape1_frame, shape1, _) = self.scene_compound_shapes_list.get(i).unwrap();
                let (shape2_frame, shape2, _) = self.scene_compound_shapes_list.get(j).unwrap();
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
    pub fn update_ground_truth_distance_grid(&mut self) {}

    #[profiling::function]
    pub fn perform_updates(&mut self, shape_updates: &Vec<ShapeUpdate>) {
        if self.optima_version {
            let mut add_or_delete = "";
            let mut index_modified = 0;
            for update in shape_updates {
                match update {
                    ShapeUpdate::Add { id, shape } => {
                        match shape {
                            shapes::Shape::Box(box_object) => {
                                let box_collider =
                                    SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                                if box_object.frame == "world" {
                                    let already_exist_id =
                                        self.scene_transient_shapes_look_up.contains_key(id);
                                    if already_exist_id {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (box_object.local_transform, box_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list[*frame_index] = (
                                            frame_name.to_string(),
                                            new_compound_shape,
                                            new_bounding_sphere_radius,
                                        );
                                        add_or_delete = "robot_add";
                                    } else {
                                        //Vec<(String, Compound, f64)>,
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(box_object.local_transform, box_collider)];
                                        let compound_shape = Compound::new(shape_vec);
                                        let new_bounding_sphere_radius =
                                            compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list.push((
                                            box_object.frame.to_string(),
                                            compound_shape.clone(),
                                            new_bounding_sphere_radius,
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_compound_shapes_list.len() - 1, 0),
                                        );

                                        let mut compound_shape_as_rows =
                                            self.scene_group_truth_distance_grid.as_rows();
                                        //self.scene_group_truth_distance_grid = Array2D::from_rows(&compound_shape_as_rows);
                                        for i in 0..=self.scene_compound_shapes_list.len() - 1 {
                                            if self.scene_compound_shapes_list.get(i).unwrap().0
                                                == "world"
                                            {
                                                break;
                                            } else {
                                                let (shape1_frame, shape1, _) =
                                                    self.scene_compound_shapes_list.get(i).unwrap();
                                                let shape2_frame = &box_object.frame;
                                                let shape2_transform = box_object.local_transform;
                                                match self
                                                    .scene_group_truth_distance_grid
                                                    .get(
                                                        i,
                                                        self.scene_compound_shapes_list.len() - 2,
                                                    )
                                                    .unwrap()
                                                {
                                                    Some((_, shape1_transform, _)) => {
                                                        let contact = parry3d_f64::query::contact(
                                                            shape1_transform,
                                                            shape1,
                                                            &shape2_transform,
                                                            &compound_shape,
                                                            D_MAX,
                                                        );
                                                        match contact {
                                                            Ok(contact) => match contact {
                                                                Some(valid_contact) => {
                                                                    let proximity =
                                                                        ProximityInfo::new(
                                                                            shape1_frame
                                                                                .to_string(),
                                                                            shape2_frame
                                                                                .to_string(),
                                                                            valid_contact.dist,
                                                                            Some((
                                                                                valid_contact
                                                                                    .point1,
                                                                                valid_contact
                                                                                    .point2,
                                                                            )),
                                                                            true,
                                                                        );
                                                                    let temp =
                                                                        compound_shape_as_rows
                                                                            .get_mut(i)
                                                                            .unwrap();
                                                                    temp.push(Some((
                                                                        proximity,
                                                                        *shape1_transform,
                                                                        shape2_transform,
                                                                    )));
                                                                }

                                                                None => {}
                                                            },

                                                            Err(_) => {}
                                                        }
                                                    }
                                                    None => {}
                                                }
                                            }
                                        }

                                        let mut temp_vec: Vec<
                                            Option<(ProximityInfo, Isometry3<f64>, Isometry3<f64>)>,
                                        > = Vec::new();
                                        for _j in 0..=self.scene_compound_shapes_list.len() - 1 {
                                            temp_vec.push(None);
                                        }
                                        self.scene_group_truth_distance_grid
                                            .as_rows()
                                            .push(temp_vec);
                                    }
                                } else {
                                    let already_exist_id =
                                        self.scene_transient_shapes_look_up.contains_key(id);
                                    if already_exist_id {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (box_object.local_transform, box_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list[*frame_index] = (
                                            frame_name.to_string(),
                                            new_compound_shape,
                                            new_bounding_sphere_radius,
                                        );
                                        add_or_delete = "robot_add";

                                        //
                                    } else {
                                        let index = self
                                            .scene_compound_shapes_list
                                            .iter()
                                            .position(|x| x.0 == box_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                index_modified = valid_index;
                                                let (_, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(valid_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec.push((
                                                    box_object.local_transform,
                                                    box_collider,
                                                ));
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec.clone());
                                                let new_bounding_sphere_radius = new_compound_shape
                                                    .local_bounding_sphere()
                                                    .radius;
                                                self.scene_compound_shapes_list[valid_index] = (
                                                    box_object.frame.to_string(),
                                                    new_compound_shape,
                                                    new_bounding_sphere_radius,
                                                );

                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (
                                                        valid_index.clone(),
                                                        compound_shape_vec.len() - 1,
                                                    ),
                                                );
                                                add_or_delete = "robot_add";
                                            }
                                            None => {
                                                println!(
                                                "The frame does not exist for this transient shape"
                                            );
                                            }
                                        }
                                    }
                                }
                            }
                            shapes::Shape::Cylinder(cylinder_object) => {
                                let new_length = cylinder_object.length / 2.0;
                                let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                                let cylinder_collider =
                                    SharedShape::cylinder(new_length, cylinder_object.radius);
                                if cylinder_object.frame == "world" {
                                    let already_exist_id =
                                        self.scene_transient_shapes_look_up.contains_key(id);
                                    if already_exist_id {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] = (
                                            cylinder_object.local_transform * transform_offset,
                                            cylinder_collider,
                                        );
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list[*frame_index] = (
                                            frame_name.to_string(),
                                            new_compound_shape,
                                            new_bounding_sphere_radius,
                                        );
                                        add_or_delete = "robot_add";
                                    } else {
                                        //Vec<(String, Compound, f64)>,
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> = vec![(
                                            cylinder_object.local_transform,
                                            cylinder_collider,
                                        )];
                                        let compound_shape = Compound::new(shape_vec);
                                        let new_bounding_sphere_radius =
                                            compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list.push((
                                            cylinder_object.frame.to_string(),
                                            compound_shape.clone(),
                                            new_bounding_sphere_radius,
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_compound_shapes_list.len() - 1, 0),
                                        );

                                        let mut compound_shape_as_rows =
                                            self.scene_group_truth_distance_grid.as_rows();
                                        //self.scene_group_truth_distance_grid = Array2D::from_rows(&compound_shape_as_rows);
                                        for i in 0..=self.scene_compound_shapes_list.len() - 1 {
                                            if self.scene_compound_shapes_list.get(i).unwrap().0
                                                == "world"
                                            {
                                                break;
                                            } else {
                                                let (shape1_frame, shape1, _) =
                                                    self.scene_compound_shapes_list.get(i).unwrap();
                                                let shape2_frame = &cylinder_object.frame;
                                                let shape2_transform =
                                                    cylinder_object.local_transform;
                                                match self
                                                    .scene_group_truth_distance_grid
                                                    .get(
                                                        i,
                                                        self.scene_compound_shapes_list.len() - 2,
                                                    )
                                                    .unwrap()
                                                {
                                                    Some((_, shape1_transform, _)) => {
                                                        let contact = parry3d_f64::query::contact(
                                                            shape1_transform,
                                                            shape1,
                                                            &shape2_transform,
                                                            &compound_shape,
                                                            D_MAX,
                                                        );
                                                        match contact {
                                                            Ok(contact) => match contact {
                                                                Some(valid_contact) => {
                                                                    let proximity =
                                                                        ProximityInfo::new(
                                                                            shape1_frame
                                                                                .to_string(),
                                                                            shape2_frame
                                                                                .to_string(),
                                                                            valid_contact.dist,
                                                                            Some((
                                                                                valid_contact
                                                                                    .point1,
                                                                                valid_contact
                                                                                    .point2,
                                                                            )),
                                                                            true,
                                                                        );
                                                                    let temp =
                                                                        compound_shape_as_rows
                                                                            .get_mut(i)
                                                                            .unwrap();
                                                                    temp.push(Some((
                                                                        proximity,
                                                                        *shape1_transform,
                                                                        shape2_transform,
                                                                    )));
                                                                }

                                                                None => {}
                                                            },

                                                            Err(_) => {}
                                                        }
                                                    }
                                                    None => {}
                                                }
                                            }
                                        }

                                        let mut temp_vec: Vec<
                                            Option<(ProximityInfo, Isometry3<f64>, Isometry3<f64>)>,
                                        > = Vec::new();
                                        for _j in 0..=self.scene_compound_shapes_list.len() - 1 {
                                            temp_vec.push(None);
                                        }
                                        self.scene_group_truth_distance_grid
                                            .as_rows()
                                            .push(temp_vec);
                                    }
                                } else {
                                    let already_exist_id =
                                        self.scene_transient_shapes_look_up.contains_key(id);
                                    if already_exist_id {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (cylinder_object.local_transform, cylinder_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list[*frame_index] = (
                                            frame_name.to_string(),
                                            new_compound_shape,
                                            new_bounding_sphere_radius,
                                        );
                                        add_or_delete = "robot_add";

                                        //
                                    } else {
                                        let index = self
                                            .scene_compound_shapes_list
                                            .iter()
                                            .position(|x| x.0 == cylinder_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                index_modified = valid_index;
                                                let (_, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(valid_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec.push((
                                                    cylinder_object.local_transform,
                                                    cylinder_collider,
                                                ));
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec.clone());
                                                let new_bounding_sphere_radius = new_compound_shape
                                                    .local_bounding_sphere()
                                                    .radius;
                                                self.scene_compound_shapes_list[valid_index] = (
                                                    cylinder_object.frame.to_string(),
                                                    new_compound_shape,
                                                    new_bounding_sphere_radius,
                                                );
                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (
                                                        valid_index.clone(),
                                                        compound_shape_vec.len() - 1,
                                                    ),
                                                );
                                                add_or_delete = "robot_add";
                                            }
                                            None => {
                                                println!(
                                                "The frame does not exist for this transient shape"
                                            );
                                            }
                                        }
                                    }
                                }
                            }
                            shapes::Shape::Sphere(sphere_object) => {
                                let sphere_collider = SharedShape::ball(sphere_object.radius);
                                if sphere_object.frame == "world" {
                                    let already_exist_id =
                                        self.scene_transient_shapes_look_up.contains_key(id);
                                    if already_exist_id {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (sphere_object.local_transform, sphere_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list[*frame_index] = (
                                            frame_name.to_string(),
                                            new_compound_shape,
                                            new_bounding_sphere_radius,
                                        );
                                        add_or_delete = "robot_add";
                                    } else {
                                        //Vec<(String, Compound, f64)>,
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(sphere_object.local_transform, sphere_collider)];
                                        let compound_shape = Compound::new(shape_vec);
                                        let new_bounding_sphere_radius =
                                            compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list.push((
                                            sphere_object.frame.to_string(),
                                            compound_shape.clone(),
                                            new_bounding_sphere_radius,
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_compound_shapes_list.len() - 1, 0),
                                        );

                                        let mut compound_shape_as_rows =
                                            self.scene_group_truth_distance_grid.as_rows();
                                        //self.scene_group_truth_distance_grid = Array2D::from_rows(&compound_shape_as_rows);
                                        for i in 0..=self.scene_compound_shapes_list.len() - 1 {
                                            if self.scene_compound_shapes_list.get(i).unwrap().0
                                                == "world"
                                            {
                                                break;
                                            } else {
                                                let (shape1_frame, shape1, _) =
                                                    self.scene_compound_shapes_list.get(i).unwrap();
                                                let shape2_frame = &sphere_object.frame;
                                                let shape2_transform =
                                                    sphere_object.local_transform;
                                                match self
                                                    .scene_group_truth_distance_grid
                                                    .get(
                                                        i,
                                                        self.scene_compound_shapes_list.len() - 2,
                                                    )
                                                    .unwrap()
                                                {
                                                    Some((_, shape1_transform, _)) => {
                                                        let contact = parry3d_f64::query::contact(
                                                            shape1_transform,
                                                            shape1,
                                                            &shape2_transform,
                                                            &compound_shape,
                                                            D_MAX,
                                                        );
                                                        match contact {
                                                            Ok(contact) => match contact {
                                                                Some(valid_contact) => {
                                                                    let proximity =
                                                                        ProximityInfo::new(
                                                                            shape1_frame
                                                                                .to_string(),
                                                                            shape2_frame
                                                                                .to_string(),
                                                                            valid_contact.dist,
                                                                            Some((
                                                                                valid_contact
                                                                                    .point1,
                                                                                valid_contact
                                                                                    .point2,
                                                                            )),
                                                                            true,
                                                                        );
                                                                    let temp =
                                                                        compound_shape_as_rows
                                                                            .get_mut(i)
                                                                            .unwrap();
                                                                    temp.push(Some((
                                                                        proximity,
                                                                        *shape1_transform,
                                                                        shape2_transform,
                                                                    )));
                                                                }

                                                                None => {}
                                                            },

                                                            Err(_) => {}
                                                        }
                                                    }
                                                    None => {}
                                                }
                                            }
                                        }

                                        let mut temp_vec: Vec<
                                            Option<(ProximityInfo, Isometry3<f64>, Isometry3<f64>)>,
                                        > = Vec::new();
                                        for _j in 0..=self.scene_compound_shapes_list.len() - 1 {
                                            temp_vec.push(None);
                                        }
                                        self.scene_group_truth_distance_grid
                                            .as_rows()
                                            .push(temp_vec);
                                    }
                                } else {
                                    let already_exist_id =
                                        self.scene_transient_shapes_look_up.contains_key(id);
                                    if already_exist_id {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (sphere_object.local_transform, sphere_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list[*frame_index] = (
                                            frame_name.to_string(),
                                            new_compound_shape,
                                            new_bounding_sphere_radius,
                                        );
                                        add_or_delete = "robot_add";

                                        //
                                    } else {
                                        let index = self
                                            .scene_compound_shapes_list
                                            .iter()
                                            .position(|x| x.0 == sphere_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                index_modified = valid_index;
                                                let (_, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(valid_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec.push((
                                                    sphere_object.local_transform,
                                                    sphere_collider,
                                                ));
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec.clone());
                                                let new_bounding_sphere_radius = new_compound_shape
                                                    .local_bounding_sphere()
                                                    .radius;
                                                self.scene_compound_shapes_list[valid_index] = (
                                                    sphere_object.frame.to_string(),
                                                    new_compound_shape,
                                                    new_bounding_sphere_radius,
                                                );
                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (
                                                        valid_index.clone(),
                                                        compound_shape_vec.len() - 1,
                                                    ),
                                                );
                                                add_or_delete = "robot_add";
                                            }
                                            None => {
                                                println!(
                                                "The frame does not exist for this transient shape"
                                            );
                                            }
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

                                let capsule_collider =
                                    SharedShape::capsule(point_a, point_b, capsule_object.radius);
                                if capsule_object.frame == "world" {
                                    let already_exist_id =
                                        self.scene_transient_shapes_look_up.contains_key(id);
                                    if already_exist_id {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (capsule_object.local_transform, capsule_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list[*frame_index] = (
                                            frame_name.to_string(),
                                            new_compound_shape,
                                            new_bounding_sphere_radius,
                                        );
                                        add_or_delete = "robot_add";
                                    } else {
                                        //Vec<(String, Compound, f64)>,
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> = vec![(
                                            capsule_object.local_transform,
                                            capsule_collider,
                                        )];
                                        let compound_shape = Compound::new(shape_vec);
                                        let new_bounding_sphere_radius =
                                            compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list.push((
                                            capsule_object.frame.to_string(),
                                            compound_shape.clone(),
                                            new_bounding_sphere_radius,
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_compound_shapes_list.len() - 1, 0),
                                        );

                                        let mut compound_shape_as_rows =
                                            self.scene_group_truth_distance_grid.as_rows();
                                        //self.scene_group_truth_distance_grid = Array2D::from_rows(&compound_shape_as_rows);
                                        for i in 0..=self.scene_compound_shapes_list.len() - 1 {
                                            if self.scene_compound_shapes_list.get(i).unwrap().0
                                                == "world"
                                            {
                                                break;
                                            } else {
                                                let (shape1_frame, shape1, _) =
                                                    self.scene_compound_shapes_list.get(i).unwrap();
                                                let shape2_frame = &capsule_object.frame;
                                                let shape2_transform =
                                                    capsule_object.local_transform;
                                                match self
                                                    .scene_group_truth_distance_grid
                                                    .get(
                                                        i,
                                                        self.scene_compound_shapes_list.len() - 2,
                                                    )
                                                    .unwrap()
                                                {
                                                    Some((_, shape1_transform, _)) => {
                                                        let contact = parry3d_f64::query::contact(
                                                            shape1_transform,
                                                            shape1,
                                                            &shape2_transform,
                                                            &compound_shape,
                                                            D_MAX,
                                                        );
                                                        match contact {
                                                            Ok(contact) => match contact {
                                                                Some(valid_contact) => {
                                                                    let proximity =
                                                                        ProximityInfo::new(
                                                                            shape1_frame
                                                                                .to_string(),
                                                                            shape2_frame
                                                                                .to_string(),
                                                                            valid_contact.dist,
                                                                            Some((
                                                                                valid_contact
                                                                                    .point1,
                                                                                valid_contact
                                                                                    .point2,
                                                                            )),
                                                                            true,
                                                                        );
                                                                    let temp =
                                                                        compound_shape_as_rows
                                                                            .get_mut(i)
                                                                            .unwrap();
                                                                    temp.push(Some((
                                                                        proximity,
                                                                        *shape1_transform,
                                                                        shape2_transform,
                                                                    )));
                                                                }

                                                                None => {}
                                                            },

                                                            Err(_) => {}
                                                        }
                                                    }
                                                    None => {}
                                                }
                                            }
                                        }

                                        let mut temp_vec: Vec<
                                            Option<(ProximityInfo, Isometry3<f64>, Isometry3<f64>)>,
                                        > = Vec::new();
                                        for _j in 0..=self.scene_compound_shapes_list.len() - 1 {
                                            temp_vec.push(None);
                                        }
                                        self.scene_group_truth_distance_grid
                                            .as_rows()
                                            .push(temp_vec);
                                    }
                                } else {
                                    let already_exist_id =
                                        self.scene_transient_shapes_look_up.contains_key(id);
                                    if already_exist_id {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (capsule_object.local_transform, capsule_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        self.scene_compound_shapes_list[*frame_index] = (
                                            frame_name.to_string(),
                                            new_compound_shape,
                                            new_bounding_sphere_radius,
                                        );
                                        add_or_delete = "robot_add";

                                        //
                                    } else {
                                        let index = self
                                            .scene_compound_shapes_list
                                            .iter()
                                            .position(|x| x.0 == capsule_object.frame);

                                        match index {
                                            Some(valid_index) => {
                                                index_modified = valid_index;
                                                let (_, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(valid_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec.push((
                                                    capsule_object.local_transform,
                                                    capsule_collider,
                                                ));
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec.clone());
                                                let new_bounding_sphere_radius = new_compound_shape
                                                    .local_bounding_sphere()
                                                    .radius;
                                                self.scene_compound_shapes_list[valid_index] = (
                                                    capsule_object.frame.to_string(),
                                                    new_compound_shape,
                                                    new_bounding_sphere_radius,
                                                );
                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (
                                                        valid_index.clone(),
                                                        compound_shape_vec.len() - 1,
                                                    ),
                                                );
                                                add_or_delete = "robot_add";
                                            }
                                            None => {
                                                println!(
                                                "The frame does not exist for this transient shape"
                                            );
                                            }
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

                                let hull_collider =
                                    SharedShape::convex_hull(hull_points.as_slice());
                                match hull_collider {
                                    Some(valid_hull_collider) => {
                                        if hull_object.frame == "world" {
                                            let already_exist_id = self
                                                .scene_transient_shapes_look_up
                                                .contains_key(id);
                                            if already_exist_id {
                                                println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                                let (frame_index, vec_index) = self
                                                    .scene_transient_shapes_look_up
                                                    .get(id)
                                                    .unwrap();
                                                let (frame_name, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(*frame_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec[*vec_index] = (
                                                    hull_object.local_transform,
                                                    valid_hull_collider,
                                                );
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec);
                                                let new_bounding_sphere_radius = new_compound_shape
                                                    .local_bounding_sphere()
                                                    .radius;
                                                self.scene_compound_shapes_list[*frame_index] = (
                                                    frame_name.to_string(),
                                                    new_compound_shape,
                                                    new_bounding_sphere_radius,
                                                );
                                                add_or_delete = "robot_add";
                                            } else {
                                                //Vec<(String, Compound, f64)>,
                                                let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                                    vec![(
                                                        hull_object.local_transform,
                                                        valid_hull_collider,
                                                    )];
                                                let compound_shape = Compound::new(shape_vec);
                                                let new_bounding_sphere_radius =
                                                    compound_shape.local_bounding_sphere().radius;
                                                self.scene_compound_shapes_list.push((
                                                    hull_object.frame.to_string(),
                                                    compound_shape.clone(),
                                                    new_bounding_sphere_radius,
                                                ));
                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (self.scene_compound_shapes_list.len() - 1, 0),
                                                );

                                                let mut compound_shape_as_rows =
                                                    self.scene_group_truth_distance_grid.as_rows();
                                                //self.scene_group_truth_distance_grid = Array2D::from_rows(&compound_shape_as_rows);
                                                for i in
                                                    0..=self.scene_compound_shapes_list.len() - 1
                                                {
                                                    if self
                                                        .scene_compound_shapes_list
                                                        .get(i)
                                                        .unwrap()
                                                        .0
                                                        == "world"
                                                    {
                                                        break;
                                                    } else {
                                                        let (shape1_frame, shape1, _) = self
                                                            .scene_compound_shapes_list
                                                            .get(i)
                                                            .unwrap();
                                                        let shape2_frame = &hull_object.frame;
                                                        let shape2_transform =
                                                            hull_object.local_transform;
                                                        match self
                                                            .scene_group_truth_distance_grid
                                                            .get(
                                                                i,
                                                                self.scene_compound_shapes_list
                                                                    .len()
                                                                    - 2,
                                                            )
                                                            .unwrap()
                                                        {
                                                            Some((_, shape1_transform, _)) => {
                                                                let contact =
                                                                    parry3d_f64::query::contact(
                                                                        shape1_transform,
                                                                        shape1,
                                                                        &shape2_transform,
                                                                        &compound_shape,
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
                                                                            let temp = compound_shape_as_rows
                                                                    .get_mut(i)
                                                                    .unwrap();
                                                                            temp.push(Some((
                                                                                proximity,
                                                                                *shape1_transform,
                                                                                shape2_transform,
                                                                            )));
                                                                        }

                                                                        None => {}
                                                                    },

                                                                    Err(_) => {}
                                                                }
                                                            }
                                                            None => {}
                                                        }
                                                    }
                                                }

                                                let mut temp_vec: Vec<
                                                    Option<(
                                                        ProximityInfo,
                                                        Isometry3<f64>,
                                                        Isometry3<f64>,
                                                    )>,
                                                > = Vec::new();
                                                for _j in
                                                    0..=self.scene_compound_shapes_list.len() - 1
                                                {
                                                    temp_vec.push(None);
                                                }
                                                self.scene_group_truth_distance_grid
                                                    .as_rows()
                                                    .push(temp_vec);
                                            }
                                        } else {
                                            let already_exist_id = self
                                                .scene_transient_shapes_look_up
                                                .contains_key(id);
                                            if already_exist_id {
                                                println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                                let (frame_index, vec_index) = self
                                                    .scene_transient_shapes_look_up
                                                    .get(id)
                                                    .unwrap();
                                                let (frame_name, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(*frame_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec[*vec_index] = (
                                                    hull_object.local_transform,
                                                    valid_hull_collider,
                                                );
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec);
                                                let new_bounding_sphere_radius = new_compound_shape
                                                    .local_bounding_sphere()
                                                    .radius;
                                                self.scene_compound_shapes_list[*frame_index] = (
                                                    frame_name.to_string(),
                                                    new_compound_shape,
                                                    new_bounding_sphere_radius,
                                                );
                                                add_or_delete = "robot_add";

                                                //
                                            } else {
                                                let index = self
                                                    .scene_compound_shapes_list
                                                    .iter()
                                                    .position(|x| x.0 == hull_object.frame);

                                                match index {
                                                    Some(valid_index) => {
                                                        index_modified = valid_index;
                                                        let (_, compound_shape, _) = self
                                                            .scene_compound_shapes_list
                                                            .get(valid_index)
                                                            .unwrap();
                                                        let mut compound_shape_vec =
                                                            compound_shape.shapes().to_vec();
                                                        compound_shape_vec.push((
                                                            hull_object.local_transform,
                                                            valid_hull_collider,
                                                        ));
                                                        let new_compound_shape = Compound::new(
                                                            compound_shape_vec.clone(),
                                                        );
                                                        let new_bounding_sphere_radius =
                                                            new_compound_shape
                                                                .local_bounding_sphere()
                                                                .radius;
                                                        self.scene_compound_shapes_list
                                                            [valid_index] = (
                                                            hull_object.frame.to_string(),
                                                            new_compound_shape,
                                                            new_bounding_sphere_radius,
                                                        );
                                                        self.scene_transient_shapes_look_up.insert(
                                                            id.to_string(),
                                                            (
                                                                valid_index.clone(),
                                                                compound_shape_vec.len() - 1,
                                                            ),
                                                        );
                                                        add_or_delete = "robot_add";
                                                    }
                                                    None => {
                                                        println!("The frame does not exist for this transient shape");
                                                    }
                                                }
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
                    ShapeUpdate::Move { id, pose } => {
                        match self.scene_transient_shapes_look_up.get(id) {
                            Some((valid_move_item_frame_idx, valid_move_item_vec_idx)) => {
                                let (frame_name, compound_shape, _) = self
                                    .scene_compound_shapes_list
                                    .get(*valid_move_item_frame_idx)
                                    .unwrap();
                                let mut compound_shape_vec = compound_shape.shapes().to_vec();
                                let (_, primitive_shape) =
                                    compound_shape_vec.get(*valid_move_item_vec_idx).unwrap();
                                compound_shape_vec[*valid_move_item_vec_idx] =
                                    (*pose, primitive_shape.clone());
                                let new_compound_shape = Compound::new(compound_shape_vec);
                                let new_bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                self.scene_compound_shapes_list[*valid_move_item_frame_idx] = (
                                    frame_name.to_string(),
                                    new_compound_shape,
                                    new_bounding_sphere_radius,
                                );
                                add_or_delete = "robot_move";
                            }
                            None => {
                                println!("this id does not exist");
                            }
                        }
                    }
                    ShapeUpdate::Delete(id) => {
                        let delete_item_hashmap = self.scene_transient_shapes_look_up.clone();
                        let delete_item = delete_item_hashmap.get(id);
                        match delete_item {
                            Some((valid_delete_item_frame_index, valid_delete_item_index)) => {
                                let (frame_name, compound_shape, _) = self
                                    .scene_compound_shapes_list
                                    .get(*valid_delete_item_frame_index)
                                    .unwrap();
                                let mut compound_shape_vec = compound_shape.shapes().to_vec();
                                compound_shape_vec.remove(*valid_delete_item_index);
                                let new_compound_shape = Compound::new(compound_shape_vec);
                                let new_bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                self.scene_transient_shapes_look_up.remove_entry(id);
                                self.scene_compound_shapes_list[*valid_delete_item_frame_index] = (
                                    frame_name.to_string(),
                                    new_compound_shape,
                                    new_bounding_sphere_radius,
                                );
                                add_or_delete = "robot_delete"
                            }
                            None => {
                                println!("this id does not exist");
                            }
                        }
                    }
                }

                if add_or_delete == "robot_add" || add_or_delete == "robot_move" {
                    let (shape_modified_frame, shape_modified, _) =
                        self.scene_compound_shapes_list.get(index_modified).unwrap();
                    for i in 0..=index_modified {
                        // Array2D<Option<(ProximityInfo, Isometry3<f64>, Isometry3<f64>)>>,
                        let temp_scene_group_truth_distance_grid = self.scene_group_truth_distance_grid.clone();
                        let temp_element = temp_scene_group_truth_distance_grid 
                            .get(i, index_modified)
                            .unwrap();
                        match temp_element {
                            Some((_, shape1_transform, shape2_transform)) => {
                                let (other_shape_frame, other_shape, _) =
                                    self.scene_compound_shapes_list.get(i).unwrap();
                                let contact = parry3d_f64::query::contact(
                                    shape1_transform,
                                    shape_modified,
                                    shape2_transform,
                                    other_shape,
                                    D_MAX,
                                );
                                match contact {
                                    Ok(contact) => match contact {
                                        Some(valid_contact) => {
                                            let new_proximity_info = ProximityInfo::new(
                                                shape_modified_frame.to_string(),
                                                other_shape_frame.to_string(),
                                                valid_contact.dist,
                                                Some((valid_contact.point1, valid_contact.point2)),
                                                true,
                                            );
                                            self.scene_group_truth_distance_grid.set(
                                                i,
                                                index_modified,
                                                Some((
                                                    new_proximity_info,
                                                    *shape1_transform,
                                                    *shape2_transform,
                                                )),
                                            ).unwrap();
                                        }
                                        None => {}
                                    },
                                    Err(_) => {}
                                }
                            }
                            None => {}
                        }
                    }

                    for j in (index_modified + 1)..=self.scene_compound_shapes_list.len() {
                        let temp_scene_group_truth_distance_grid = self.scene_group_truth_distance_grid.clone();
                        let temp_element =
                             temp_scene_group_truth_distance_grid
                            .get(index_modified, j)
                            .unwrap();
                        match temp_element {
                            Some((_, shape1_transform, shape2_transform)) => {
                                let (other_shape_frame, other_shape, _) =
                                    self.scene_compound_shapes_list.get(j).unwrap();
                                let contact = parry3d_f64::query::contact(
                                    shape1_transform,
                                    shape_modified,
                                    shape2_transform,
                                    other_shape,
                                    D_MAX,
                                );
                                match contact {
                                    Ok(contact) => match contact {
                                        Some(valid_contact) => {
                                            let new_proximity_info = ProximityInfo::new(
                                                shape_modified_frame.to_string(),
                                                other_shape_frame.to_string(),
                                                valid_contact.dist,
                                                Some((valid_contact.point1, valid_contact.point2)),
                                                true,
                                            );
                                            self.scene_group_truth_distance_grid.set(
                                                index_modified,
                                                j,
                                                Some((
                                                    new_proximity_info,
                                                    *shape1_transform,
                                                    *shape2_transform,
                                                )),
                                            ).unwrap();
                                        }
                                        None => {}
                                    },
                                    Err(_) => {}
                                }
                            }
                            None => {}
                        }
                    }
                }
            }
        } else {
            for update in shape_updates {
                match update {
                    ShapeUpdate::Add { id, shape } => {
                        match shape {
                            shapes::Shape::Box(box_object) => {
                                let box_collider =
                                    SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                                if box_object.frame == "world" {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (box_object.local_transform, box_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_compound_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0);
                                    } else {
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(box_object.local_transform, box_collider)];
                                        let compound_shape = Compound::new(shape_vec);
                                        self.scene_compound_shapes_list.push((
                                            box_object.frame.to_string(),
                                            compound_shape,
                                            0.0,
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_compound_shapes_list.len() - 1, 0),
                                        );
                                    }
                                } else {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (box_object.local_transform, box_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_compound_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0);
                                    } else {
                                        let index = self
                                            .scene_compound_shapes_list
                                            .iter()
                                            .position(|x| x.0 == box_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                let (_, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(valid_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec.push((
                                                    box_object.local_transform,
                                                    box_collider,
                                                ));
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec.clone());
                                                self.scene_compound_shapes_list[valid_index] = (
                                                    box_object.frame.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                );

                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (
                                                        valid_index.clone(),
                                                        compound_shape_vec.len() - 1,
                                                    ),
                                                );
                                            }
                                            None => {
                                                println!(
                                                "The frame does not exist for this transient shape"
                                            );
                                            }
                                        }
                                    }
                                }
                            }
                            shapes::Shape::Cylinder(cylinder_object) => {
                                let new_length = cylinder_object.length / 2.0;
                                let transform_offset = Isometry3::rotation(Vector3::x() * 0.5 * PI);
                                let cylinder_collider =
                                    SharedShape::cylinder(new_length, cylinder_object.radius);
                                if cylinder_object.frame == "world" {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] = (
                                            cylinder_object.local_transform * transform_offset,
                                            cylinder_collider,
                                        );
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_compound_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0);
                                    } else {
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> = vec![(
                                            cylinder_object.local_transform * transform_offset,
                                            cylinder_collider,
                                        )];
                                        let compound_shape = Compound::new(shape_vec);
                                        self.scene_compound_shapes_list.push((
                                            cylinder_object.frame.to_string(),
                                            compound_shape,
                                            0.0,
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_compound_shapes_list.len() - 1, 0),
                                        );
                                    }
                                } else {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] = (
                                            cylinder_object.local_transform * transform_offset,
                                            cylinder_collider,
                                        );
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_compound_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0);
                                    } else {
                                        let index = self
                                            .scene_compound_shapes_list
                                            .iter()
                                            .position(|x| x.0 == cylinder_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                let (_, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(valid_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec.push((
                                                    cylinder_object.local_transform
                                                        * transform_offset,
                                                    cylinder_collider,
                                                ));
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec.clone());
                                                self.scene_compound_shapes_list[valid_index] = (
                                                    cylinder_object.frame.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                );

                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (
                                                        valid_index.clone(),
                                                        compound_shape_vec.len() - 1,
                                                    ),
                                                );
                                            }
                                            None => {
                                                println!(
                                                "The frame does not exist for this transient shape"
                                            );
                                            }
                                        }
                                    }
                                }
                            }
                            shapes::Shape::Sphere(sphere_object) => {
                                let sphere_collider = SharedShape::ball(sphere_object.radius);
                                if sphere_object.frame == "world" {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (sphere_object.local_transform, sphere_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_compound_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0);
                                    } else {
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(sphere_object.local_transform, sphere_collider)];
                                        let compound_shape = Compound::new(shape_vec);
                                        self.scene_compound_shapes_list.push((
                                            sphere_object.frame.to_string(),
                                            compound_shape,
                                            0.0,
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_compound_shapes_list.len() - 1, 0),
                                        );
                                    }
                                } else {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (sphere_object.local_transform, sphere_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_compound_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0);
                                    } else {
                                        let index = self
                                            .scene_compound_shapes_list
                                            .iter()
                                            .position(|x| x.0 == sphere_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                let (_, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(valid_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec.push((
                                                    sphere_object.local_transform,
                                                    sphere_collider,
                                                ));
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec.clone());
                                                self.scene_compound_shapes_list[valid_index] = (
                                                    sphere_object.frame.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                );

                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (
                                                        valid_index.clone(),
                                                        compound_shape_vec.len() - 1,
                                                    ),
                                                );
                                            }
                                            None => {
                                                println!(
                                                "The frame does not exist for this transient shape"
                                            );
                                            }
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
                                let capsule_collider =
                                    SharedShape::capsule(point_a, point_b, capsule_object.radius);
                                if capsule_object.frame == "world" {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (capsule_object.local_transform, capsule_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_compound_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0);
                                    } else {
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> = vec![(
                                            capsule_object.local_transform,
                                            capsule_collider,
                                        )];
                                        let compound_shape = Compound::new(shape_vec);
                                        self.scene_compound_shapes_list.push((
                                            capsule_object.frame.to_string(),
                                            compound_shape,
                                            0.0,
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_compound_shapes_list.len() - 1, 0),
                                        );
                                    }
                                } else {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _) = self
                                            .scene_compound_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (capsule_object.local_transform, capsule_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_compound_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0);
                                    } else {
                                        let index = self
                                            .scene_compound_shapes_list
                                            .iter()
                                            .position(|x| x.0 == capsule_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                let (_, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(valid_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec.push((
                                                    capsule_object.local_transform,
                                                    capsule_collider,
                                                ));
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec.clone());
                                                self.scene_compound_shapes_list[valid_index] = (
                                                    capsule_object.frame.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                );

                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (
                                                        valid_index.clone(),
                                                        compound_shape_vec.len() - 1,
                                                    ),
                                                );
                                            }
                                            None => {
                                                println!(
                                                    "The frame does not exist for this transient shape"
                                                );
                                            }
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

                                let hull_collider =
                                    SharedShape::convex_hull(hull_points.as_slice());
                                match hull_collider {
                                    Some(valid_hull_collider) => {
                                        if hull_object.frame == "world" {
                                            if self.scene_transient_shapes_look_up.contains_key(id)
                                            {
                                                println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                                let (frame_index, vec_index) = self
                                                    .scene_transient_shapes_look_up
                                                    .get(id)
                                                    .unwrap();
                                                let (frame_name, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(*frame_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec[*vec_index] = (
                                                    hull_object.local_transform,
                                                    valid_hull_collider,
                                                );
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec);
                                                self.scene_compound_shapes_list[*frame_index] = (
                                                    frame_name.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                );
                                            } else {
                                                let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                                    vec![(
                                                        hull_object.local_transform,
                                                        valid_hull_collider,
                                                    )];
                                                let compound_shape = Compound::new(shape_vec);
                                                self.scene_compound_shapes_list.push((
                                                    hull_object.frame.to_string(),
                                                    compound_shape,
                                                    0.0,
                                                ));
                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (self.scene_compound_shapes_list.len() - 1, 0),
                                                );
                                            }
                                        } else {
                                            if self.scene_transient_shapes_look_up.contains_key(id)
                                            {
                                                println!("WARNING: overwring the shape because another transient shape with the same id already exist in the scene");
                                                let (frame_index, vec_index) = self
                                                    .scene_transient_shapes_look_up
                                                    .get(id)
                                                    .unwrap();
                                                let (frame_name, compound_shape, _) = self
                                                    .scene_compound_shapes_list
                                                    .get(*frame_index)
                                                    .unwrap();
                                                let mut compound_shape_vec =
                                                    compound_shape.shapes().to_vec();
                                                compound_shape_vec[*vec_index] = (
                                                    hull_object.local_transform,
                                                    valid_hull_collider,
                                                );
                                                let new_compound_shape =
                                                    Compound::new(compound_shape_vec);
                                                self.scene_compound_shapes_list[*frame_index] = (
                                                    frame_name.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                );
                                            } else {
                                                let index = self
                                                    .scene_compound_shapes_list
                                                    .iter()
                                                    .position(|x| x.0 == hull_object.frame);
                                                match index {
                                                    Some(valid_index) => {
                                                        let (_, compound_shape, _) = self
                                                            .scene_compound_shapes_list
                                                            .get(valid_index)
                                                            .unwrap();
                                                        let mut compound_shape_vec =
                                                            compound_shape.shapes().to_vec();
                                                        compound_shape_vec.push((
                                                            hull_object.local_transform,
                                                            valid_hull_collider,
                                                        ));
                                                        let new_compound_shape = Compound::new(
                                                            compound_shape_vec.clone(),
                                                        );
                                                        self.scene_compound_shapes_list
                                                            [valid_index] = (
                                                            hull_object.frame.to_string(),
                                                            new_compound_shape,
                                                            0.0,
                                                        );

                                                        self.scene_transient_shapes_look_up.insert(
                                                            id.to_string(),
                                                            (
                                                                valid_index.clone(),
                                                                compound_shape_vec.len() - 1,
                                                            ),
                                                        );
                                                    }
                                                    None => {
                                                        println!(
                                                    "The frame does not exist for this transient shape"
                                                );
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    None => {}
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
                        match self.scene_transient_shapes_look_up.get(id) {
                            Some((valid_move_item_frame_idx, valid_move_item_vec_idx)) => {
                                let (frame_name, compound_shape, _) = self
                                    .scene_compound_shapes_list
                                    .get(*valid_move_item_frame_idx)
                                    .unwrap();
                                let mut compound_shape_vec = compound_shape.shapes().to_vec();
                                let (_, primitive_shape) =
                                    compound_shape_vec.get(*valid_move_item_vec_idx).unwrap();
                                compound_shape_vec[*valid_move_item_vec_idx] =
                                    (*pose, primitive_shape.clone());
                                let new_compound_shape = Compound::new(compound_shape_vec);
                                let new_bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                self.scene_compound_shapes_list[*valid_move_item_frame_idx] = (
                                    frame_name.to_string(),
                                    new_compound_shape,
                                    new_bounding_sphere_radius,
                                );
                            }
                            None => {
                                println!("this id does not exist");
                            }
                        }
                    }
                    ShapeUpdate::Delete(id) => {
                        let delete_item_hashmap = self.scene_transient_shapes_look_up.clone();
                        let delete_item = delete_item_hashmap.get(id);
                        match delete_item {
                            Some((valid_delete_item_frame_index, valid_delete_item_index)) => {
                                let (frame_name, compound_shape, _) = self
                                    .scene_compound_shapes_list
                                    .get(*valid_delete_item_frame_index)
                                    .unwrap();
                                let mut compound_shape_vec = compound_shape.shapes().to_vec();
                                compound_shape_vec.remove(*valid_delete_item_index);
                                let new_compound_shape = Compound::new(compound_shape_vec);
                                let new_bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                self.scene_transient_shapes_look_up.remove_entry(id);
                                self.scene_compound_shapes_list[*valid_delete_item_frame_index] = (
                                    frame_name.to_string(),
                                    new_compound_shape,
                                    new_bounding_sphere_radius,
                                );
                            }
                            None => {
                                println!("this id does not exist");
                            }
                        }
                    }
                }
            }
        }
    }

    #[profiling::function]
    pub fn clear_all_transient_shapes(&mut self) {
        if self.optima_version {
            for (_, (frame_idx, vec_idx)) in self.scene_transient_shapes_look_up.iter() {
                let temp_scene_compound_shapes_list = self.scene_compound_shapes_list.clone();
                let (shape_modified_frame, shape_modified, _) =
                    temp_scene_compound_shapes_list.get(*frame_idx).unwrap();
                    let mut compound_shape_vec = shape_modified.shapes().to_vec();
                    compound_shape_vec.remove(*vec_idx);
                    let new_compound_shape = Compound::new(compound_shape_vec);
                    let new_bounding_sphere_radius = new_compound_shape.local_bounding_sphere().radius;
                    self.scene_compound_shapes_list[*frame_idx] = (
                        shape_modified_frame.to_string(),
                        new_compound_shape,
                        new_bounding_sphere_radius,
                    );

               
                for i in 0..=*frame_idx {
                    let temp_scene_group_truth_distance_grid = self.scene_group_truth_distance_grid.clone();
                    let temp_element = temp_scene_group_truth_distance_grid
                        .get(i, *frame_idx)
                        .unwrap();
                    match temp_element {
                        Some((_, shape1_transform, shape2_transform)) => {
                            let (other_shape_frame, other_shape, _) =
                                self.scene_compound_shapes_list.get(i).unwrap();
                            let contact = parry3d_f64::query::contact(
                                shape1_transform,
                                shape_modified,
                                shape2_transform,
                                other_shape,
                                D_MAX,
                            );
                            match contact {
                                Ok(contact) => match contact {
                                    Some(valid_contact) => {
                                        let new_proximity_info = ProximityInfo::new(
                                            shape_modified_frame.to_string(),
                                            other_shape_frame.to_string(),
                                            valid_contact.dist,
                                            Some((valid_contact.point1, valid_contact.point2)),
                                            true,
                                        );
                                        self.scene_group_truth_distance_grid.set(
                                            i,
                                            *frame_idx,
                                            Some((
                                                new_proximity_info,
                                                *shape1_transform,
                                                *shape2_transform,
                                            )),
                                        ).unwrap();
                                       
                                    }
                                    None => {}
                                },
                                Err(_) => {}
                            }
                        }
                        None => {}
                    }
                }

                for j in (*frame_idx + 1)..=self.scene_compound_shapes_list.len() {
                    let temp_scene_group_truth_distance_grid = self.scene_group_truth_distance_grid.clone();
                    let temp_element = temp_scene_group_truth_distance_grid
                        .get(*frame_idx, j)
                        .unwrap();
                    match temp_element {
                        Some((_, shape1_transform, shape2_transform)) => {
                            let (other_shape_frame, other_shape, _) =
                                self.scene_compound_shapes_list.get(j).unwrap();
                            let contact = parry3d_f64::query::contact(
                                shape1_transform,
                                shape_modified,
                                shape2_transform,
                                other_shape,
                                D_MAX,
                            );
                            match contact {
                                Ok(contact) => match contact {
                                    Some(valid_contact) => {
                                        let new_proximity_info = ProximityInfo::new(
                                            shape_modified_frame.to_string(),
                                            other_shape_frame.to_string(),
                                            valid_contact.dist,
                                            Some((valid_contact.point1, valid_contact.point2)),
                                            true,
                                        );
                                        self.scene_group_truth_distance_grid.set(
                                            *frame_idx,
                                            j,
                                            Some((
                                                new_proximity_info,
                                                *shape1_transform,
                                                *shape2_transform,
                                            )),
                                        ).unwrap();
                                    }
                                    None => {}
                                },
                                Err(_) => {}
                            }
                        }
                        None => {}
                    }
                }
            }
        } else {
            for (_, (frame_idx, vec_idx)) in self.scene_transient_shapes_look_up.iter() {
                let (frame_name, compound_shape, _) =
                    self.scene_compound_shapes_list.get(*frame_idx).unwrap();
                let mut compound_shape_vec = compound_shape.shapes().to_vec();
                compound_shape_vec.remove(*vec_idx);
                let new_compound_shape = Compound::new(compound_shape_vec);
               

                self.scene_compound_shapes_list[*frame_idx] = (
                    frame_name.to_string(),
                    new_compound_shape,
                    0.0,
                );
            }
            self.scene_transient_shapes_look_up.clear();
        }
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
        let default_frame_transform: Isometry3<f64> = Isometry3::<f64>::identity();
        let shape1_j_translation = shape1_j_transform.translation;
        let shape1_j_rotation = shape1_j_transform.rotation;

        let shape1_k_translation = current_frame
            .get(shape1_current_frame)
            .unwrap_or(&default_frame_transform)
            .translation;
        let shape1_current_rotation = current_frame
            .get(shape1_current_frame)
            .unwrap_or(&default_frame_transform)
            .rotation;

        let shape2_j_translation = shape2_j_transform.translation;
        let shape2_j_rotation = shape2_j_transform.rotation;
        let shape2_current_translation = current_frame
            .get(shape2_current_frame)
            .unwrap_or(&default_frame_transform)
            .translation;
        let shape2_current_rotation = current_frame
            .get(shape2_current_frame)
            .unwrap_or(&default_frame_transform)
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
                let max_bounding_sphere;
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
        let default_frame_transform: Isometry3<f64> = Isometry3::<f64>::identity();
        let shape1_current_state_rotation = current_state
            .get(shape1_frame)
            .unwrap_or(&default_frame_transform)
            .rotation;
        let shape1_past_state_rotation = j_state.1.rotation;
        let shape2_current_state_rotation = current_state
            .get(shape2_frame)
            .unwrap_or(&default_frame_transform)
            .rotation;
        let shape2_past_state_rotation = j_state.2.rotation;

        let shape1_current_state_translation = current_state
            .get(shape1_frame)
            .unwrap_or(&default_frame_transform)
            .translation;
        let shape1_past_state_translation = j_state.1.translation;
        let shape2_current_state_translation = current_state
            .get(shape2_frame)
            .unwrap_or(&default_frame_transform)
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
        let result;
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
        let result;

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
        let result;

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
        let a_value = A_VALUE;
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
        let timed_timer = Instant::now();
        for i in 0..=size - 1 {
            for j in (i + 1)..=size - 1 {
                if timed_timer.elapsed().as_micros() < TIME_BUDGET.as_micros() {
                    let (shape1_frame, shape1, _) = self.scene_compound_shapes_list.get(i).unwrap();
                    let (shape2_frame, shape2, _) = self.scene_compound_shapes_list.get(j).unwrap();
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
                }else{break;}
                }
               
        }

        loss_functions_error_vec.sort_by(|a, b| a.4.partial_cmp(&b.4).unwrap());

        return loss_functions_error_vec;
    }

    #[profiling::function]
    pub fn get_proximity(
        &mut self,
        frames: &HashMap<String, Isometry3<f64>>, // <String, (Iso, Iso)>// frame, world, local
    ) -> Vec<ProximityInfo> {
        if self.optima_version {
            let default_frame_transform: Isometry3<f64> = Isometry3::<f64>::identity();
            let mut result_vector: Vec<ProximityInfo> = vec![];
            let ranking_vector: Vec<(String, Compound, String, Compound, f64, usize, usize)> =
                self.ranking_maximum_loss_functions_error(frames);
            if TIMED {
                let timed_timer = Instant::now();
                for (shape1_frame, shape1, shape2_frame, shape2, _, i, j) in ranking_vector {
                    if timed_timer.elapsed().as_micros() < TIME_BUDGET.as_micros() {
                        let shape1_transform = frames
                            .get(&shape1_frame)
                            .unwrap_or(&default_frame_transform);
                        let shape2_transform = frames
                            .get(&shape2_frame)
                            .unwrap_or(&default_frame_transform);
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
                                    result_vector.push(proximity.clone());
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
                    }else{
                        break;
                    }
                }
            } else {
                let mut remaining_error_summation = 0.0;
                let mut index = 0;

                for (_, _, _, _, loss_value, _, _) in ranking_vector.clone() {
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
                    let shape1_transform = frames
                        .get(current_shape1_frame)
                        .unwrap_or(&default_frame_transform);
                    let shape2_transform = frames
                        .get(current_shape2_frame)
                        .unwrap_or(&default_frame_transform);
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
                                result_vector.push(proximity.clone());
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
        } else {
            let size = self.scene_compound_shapes_list.len();
            //println!("The size is {:?}" , size);
            let mut result_vector: Vec<ProximityInfo> = vec![];
            //let compound_shapes_grid = self.clone().create_compound_shapes_grid();
            for i in 0..=size - 1 {
                for j in (i + 1)..=size - 1 {
                    let (shape1_frame, shape1, _) = self.scene_compound_shapes_list.get(i).unwrap();
                    let (shape2_frame, shape2, _) = self.scene_compound_shapes_list.get(j).unwrap();
                    if shape1_frame == "world" && shape2_frame == "world" {
                        continue;
                    }
                    let shape1_transform = frames.get(shape1_frame);
                    match shape1_transform {
                        Some(shape1_transform) => {
                            let shape2_transform = frames.get(shape2_frame);
                            match shape2_transform {
                                Some(shape2_transform) => {
                                    match parry3d_f64::query::closest_points(
                                        &shape1_transform,
                                        shape1,
                                        &shape2_transform,
                                        shape2,
                                        D_MAX,
                                    ) {
                                        Ok(valid_closest_points) => match valid_closest_points {
                                            ClosestPoints::Intersecting => {
                                                result_vector.push(ProximityInfo::new(
                                                    shape1_frame.to_string(),
                                                    shape2_frame.to_string(),
                                                    0.0,
                                                    None,
                                                    true,
                                                ));
                                            }
                                            ClosestPoints::WithinMargin(point1, point2) => {
                                                let dist = ((point1.x - point2.x)
                                                    * (point1.x - point2.x)
                                                    + (point1.y - point2.y)
                                                        * (point1.y - point2.y)
                                                    + (point1.z - point2.z)
                                                        * (point1.z - point2.z) as f64)
                                                    .sqrt();
                                                result_vector.push(ProximityInfo::new(
                                                    shape1_frame.to_string(),
                                                    shape2_frame.to_string(),
                                                    dist,
                                                    Some((point1, point2)),
                                                    true,
                                                ));
                                            }
                                            ClosestPoints::Disjoint => {}
                                        },
                                        Err(_) => {
                                            println!()
                                        }
                                    }
                                }
                                None => {}
                            }
                        }
                        None => {}
                    }
                }
            }

            return result_vector;
        }
    }
}
