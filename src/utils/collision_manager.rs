use crate::utils::info::{
    CollisionSettingInfo, LinkInfo, ProximityInfo, ShapeUpdate, TransformInfo,
};
use crate::utils::shapes;
use indexmap::IndexMap;
use nalgebra::base::Vector3;
use nalgebra::geometry::Isometry3;
use nalgebra::vector;
use nalgebra::Point3;
use parry3d_f64::shape::*;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::fmt;

//use std::time::{Duration, Instant};
//use instant::{now};
use instant::{Duration, Instant};

const ACCURACY_BUDGET: f64 = 0.1;
#[derive(Clone)]
pub struct CollisionManager {
    scene_collision_shapes_list: Vec<(String, Compound, f64, Isometry3<f64>, String, bool)>,
    scene_optima_transient_shapes_look_up:
        IndexMap<String, (String, String, Option<usize>, SharedShape, bool)>,
    scene_group_truth_distance_hashmap: IndexMap<
        String,
        IndexMap<
            String,
            (
                ProximityInfo,
                Compound,
                Compound,
                f64,
                f64,
                Isometry3<f64>,
                Isometry3<f64>,
                String,
                String,
            ),
        >,
    >,
    d_max: f64,
    r: f64,
    a_max: f64,
    time_budget: u64,
    timed: bool,
}

impl fmt::Debug for CollisionManager {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CollisionManager").finish()
    }
}

impl CollisionManager {
    // #[profiling::function]
    pub fn new(
        links: Vec<LinkInfo>,
        persistent_shapes: Vec<shapes::Shape>,
        collision_settings: &Option<CollisionSettingInfo>,
    ) -> Self {
        let d_max: f64;
        let r: f64;
        let a_max: f64;
        let time_budget: u64;
        let timed: bool;

        match collision_settings {
            Some(collision_settings) => {
                d_max = collision_settings.d_max;
                r = collision_settings.r;
                a_max = collision_settings.a_max;
                time_budget = collision_settings.time_budget;
                timed = collision_settings.timed;
            }
            None => {
                let default_collision_settings = CollisionSettingInfo::default();
                d_max = default_collision_settings.d_max;
                r = default_collision_settings.r;
                a_max = default_collision_settings.a_max;
                time_budget = default_collision_settings.time_budget;
                timed = default_collision_settings.timed;
            }
        }

        let mut scene_collision_shapes_list: Vec<(
            String,
            Compound,
            f64,
            Isometry3<f64>,
            String,
            bool,
        )> = vec![];

        let scene_group_truth_distance_hashmap = IndexMap::new();
        let scene_optima_transient_shapes_look_up = IndexMap::new();

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
                    shapes::Shape::Box(object) => {
                        let box_shape =
                            SharedShape::cuboid(object.x / 2.0, object.y / 2.0, object.z / 2.0);
                        robot_shapes_list.push((object.local_transform, box_shape));
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
                scene_collision_shapes_list.push((
                    frame_name.to_string(),
                    robot_compound_shapes,
                    bounding_sphere_radius,
                    Isometry3::identity(),
                    frame_name.to_string(),
                    true,
                ));
            }
        }

        for shape in &persistent_shapes {
            match shape {
                shapes::Shape::Box(object) => {
                    let box_shape =
                        SharedShape::cuboid(object.x / 2.0, object.y / 2.0, object.z / 2.0);
                    if object.frame == "world" {
                        let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                            vec![(object.local_transform, box_shape)];
                        let temp_compound = Compound::new(temp_list);
                        let bounding_sphere_radius = temp_compound.local_bounding_sphere().radius;
                        scene_collision_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            bounding_sphere_radius,
                            Isometry3::identity(),
                            object.name.to_string(),
                            object.physical,
                        ));
                    } else {
                        let index_element = scene_collision_shapes_list
                            .iter()
                            .position(|x| x.0 == object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_collision_shapes_list.clone();
                                let (
                                    frame_name,
                                    compound_shape,
                                    _,
                                    transform,
                                    robot_link_name,
                                    physical,
                                ) = temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape_vec = compound_shape.shapes().to_vec();
                                new_compound_shape_vec.push((object.local_transform, box_shape));
                                let new_compound_shape = Compound::new(new_compound_shape_vec);
                                let bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                scene_collision_shapes_list.remove(valid_index);
                                scene_collision_shapes_list.insert(
                                    valid_index,
                                    (
                                        frame_name.to_string(),
                                        new_compound_shape,
                                        bounding_sphere_radius,
                                        *transform,
                                        robot_link_name.to_string(),
                                        *physical,
                                    ),
                                );
                            }
                            None => {}
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
                        scene_collision_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            bounding_sphere_radius,
                            Isometry3::identity(),
                            cylinder_object.name.to_string(),
                            cylinder_object.physical,
                        ));
                    } else {
                        let index_element = scene_collision_shapes_list
                            .iter()
                            .position(|x| x.0 == cylinder_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_collision_shapes_list.clone();
                                let (
                                    frame_name,
                                    compound_shape,
                                    _,
                                    transform,
                                    robot_link_name,
                                    physical,
                                ) = temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape_vec = compound_shape.shapes().to_vec();
                                new_compound_shape_vec.push((
                                    cylinder_object.local_transform * transform_offset,
                                    cylinder_shape,
                                ));
                                let new_compound_shape = Compound::new(new_compound_shape_vec);
                                let bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;

                                scene_collision_shapes_list.remove(valid_index);
                                scene_collision_shapes_list.insert(
                                    valid_index,
                                    (
                                        frame_name.to_string(),
                                        new_compound_shape,
                                        bounding_sphere_radius,
                                        *transform,
                                        robot_link_name.to_string(),
                                        *physical,
                                    ),
                                );
                            }
                            None => {}
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
                        scene_collision_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            bounding_sphere_radius,
                            Isometry3::default(),
                            sphere_object.name.to_string(),
                            sphere_object.physical,
                        ));
                    } else {
                        let index_element = scene_collision_shapes_list
                            .iter()
                            .position(|x| x.0 == sphere_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_collision_shapes_list.clone();
                                let (
                                    frame_name,
                                    compound_shape,
                                    _,
                                    transform,
                                    robot_link_name,
                                    physical,
                                ) = temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape_vec = compound_shape.shapes().to_vec();
                                new_compound_shape_vec
                                    .push((sphere_object.local_transform, sphere_shape));
                                let new_compound_shape = Compound::new(new_compound_shape_vec);
                                let bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                scene_collision_shapes_list.remove(valid_index);
                                scene_collision_shapes_list.insert(
                                    valid_index,
                                    (
                                        frame_name.to_string(),
                                        new_compound_shape,
                                        bounding_sphere_radius,
                                        *transform,
                                        robot_link_name.to_string(),
                                        *physical,
                                    ),
                                );
                            }
                            None => {}
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
                        scene_collision_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            temp_compound_radius,
                            Isometry3::default(),
                            capsule_object.name.to_string(),
                            capsule_object.physical,
                        ));
                    } else {
                        let index_element = scene_collision_shapes_list
                            .iter()
                            .position(|x| x.0 == capsule_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_collision_shapes_list.clone();
                                let (
                                    frame_name,
                                    compound_shape,
                                    _,
                                    transform,
                                    robot_link_name,
                                    physical,
                                ) = temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape_vec = compound_shape.shapes().to_vec();
                                new_compound_shape_vec
                                    .push((capsule_object.local_transform, capsule_shape));
                                let new_compound_shape = Compound::new(new_compound_shape_vec);
                                let bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                scene_collision_shapes_list.remove(valid_index);
                                scene_collision_shapes_list.insert(
                                    valid_index,
                                    (
                                        frame_name.to_string(),
                                        new_compound_shape,
                                        bounding_sphere_radius,
                                        *transform,
                                        robot_link_name.to_string(),
                                        *physical,
                                    ),
                                );
                            }
                            None => {}
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
                                scene_collision_shapes_list.push((
                                    "world".to_string(),
                                    temp_compound,
                                    bounding_sphere_radius,
                                    Isometry3::default(),
                                    hull_object.name.to_string(),
                                    hull_object.physical,
                                ));
                            }
                            None => {
                                // println!("the given points cannot form a valid hull shape");
                            }
                        }
                    } else {
                        match hull_shape {
                            Some(valid_hull_shape) => {
                                let index_element = scene_collision_shapes_list
                                    .iter()
                                    .position(|x| x.0 == hull_object.frame);
                                match index_element {
                                    Some(valid_index) => {
                                        let temp_scene_compound_shapes_list =
                                            scene_collision_shapes_list.clone();
                                        let (
                                            frame_name,
                                            compound_shape,
                                            _,
                                            transform,
                                            robot_link_name,
                                            physical,
                                        ) = temp_scene_compound_shapes_list
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
                                        scene_collision_shapes_list.remove(valid_index);
                                        scene_collision_shapes_list.insert(
                                            valid_index,
                                            (
                                                frame_name.to_string(),
                                                new_compound_shape,
                                                bounding_sphere_radius,
                                                *transform,
                                                robot_link_name.to_string(),
                                                *physical,
                                            ),
                                        );
                                    }
                                    None => {}
                                }
                            }
                            None => {
                                //println!("the given points cannot form a valid hull shape");
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

        Self {
            scene_collision_shapes_list,
            scene_group_truth_distance_hashmap,
            scene_optima_transient_shapes_look_up,
            d_max,
            r,
            a_max,
            time_budget,
            timed,
        }
    }

    pub fn compute_ground_truth_distance_hashmap(
        &mut self,
        initial_frames: &HashMap<String, TransformInfo>,
        proximity_info: &Vec<ProximityInfo>,
    ) {
        let default_transform = TransformInfo::default();
        let size = self.scene_collision_shapes_list.len();
        if size != 0 {
            for i in 0..=size - 1 {
                if self.scene_collision_shapes_list.get(i).unwrap().0 == "world" {
                    break;
                } else {
                    let (shape1_frame, shape1, rad1, _, shape1_name, shape1_physical) =
                        self.scene_collision_shapes_list.get(i).unwrap();
                    let mut proximity: ProximityInfo;

                    let mut value_hashmap: IndexMap<
                        String,
                        (
                            ProximityInfo,
                            Compound,
                            Compound,
                            f64,
                            f64,
                            Isometry3<f64>,
                            Isometry3<f64>,
                            String,
                            String,
                        ),
                    > = IndexMap::new();

                    for j in (i + 1)..=size - 1 {
                        let (shape2_frame, shape2, rad2, _, shape2_name, shape2_physical) =
                            self.scene_collision_shapes_list.get(j).unwrap();

                        if shape1_frame == "world" && shape2_frame == "world" {
                            continue;
                        } else {
                            let shape1_transform = initial_frames
                                .get(shape1_name)
                                .unwrap_or(&default_transform);
                            let shape2_transform = initial_frames
                                .get(shape2_name)
                                .unwrap_or(&default_transform);

                            match parry3d_f64::query::contact(
                                &shape1_transform.world,
                                shape1,
                                &shape2_transform.world,
                                &shape2.clone(),
                                self.d_max,
                            ) {
                                Ok(contact) => match contact {
                                    Some(valid_contact) => {
                                        if proximity_info.len() != 0 {
                                            'f: for item in proximity_info {
                                                if shape1_name.to_string() == item.shape1
                                                    && shape2_name.to_string() == item.shape2
                                                {
                                                    proximity = ProximityInfo::new(
                                                        shape1_name.to_string(),
                                                        shape2_name.to_string(),
                                                        valid_contact.dist,
                                                        Some((
                                                            valid_contact.point1,
                                                            valid_contact.point2,
                                                        )),
                                                        *shape1_physical && *shape2_physical,
                                                        self.compute_loss_with_cutoff(
                                                            &valid_contact.dist,
                                                            &item.average_distance.unwrap_or(1.0),
                                                        ),
                                                        item.average_distance,
                                                    );
                                                    value_hashmap.insert(
                                                        shape2_name.to_string(),
                                                        (
                                                            proximity,
                                                            shape1.clone(),
                                                            shape2.clone(),
                                                            *rad1,
                                                            *rad2,
                                                            shape1_transform.world,
                                                            shape2_transform.world,
                                                            shape1_frame.to_string(),
                                                            shape2_frame.to_string(),
                                                        ),
                                                    );
                                                    break 'f;
                                                }
                                            }
                                        } else {
                                            proximity = ProximityInfo::new(
                                                shape1_name.to_string(),
                                                shape2_name.to_string(),
                                                valid_contact.dist,
                                                Some((valid_contact.point1, valid_contact.point2)),
                                                *shape1_physical && *shape2_physical,
                                                self.compute_loss_with_cutoff(
                                                    &valid_contact.dist,
                                                    &1.0,
                                                ),
                                                Some(1.0),
                                            );
                                            value_hashmap.insert(
                                                shape2_name.to_string(),
                                                (
                                                    proximity,
                                                    shape1.clone(),
                                                    shape2.clone(),
                                                    *rad1,
                                                    *rad2,
                                                    shape1_transform.world,
                                                    shape2_transform.world,
                                                    shape1_frame.to_string(),
                                                    shape2_frame.to_string(),
                                                ),
                                            );
                                        }
                                    }
                                    None => {
                                        if proximity_info.len() != 0 {
                                            'f: for item in proximity_info {
                                                if shape1_name.to_string() == item.shape1
                                                    && shape2_name.to_string() == item.shape2
                                                {
                                                    proximity = ProximityInfo::new(
                                                        shape1_name.to_string(),
                                                        shape2_name.to_string(),
                                                        self.d_max,
                                                        None,
                                                        *shape1_physical && *shape2_physical,
                                                        self.compute_loss_with_cutoff(
                                                            &self.d_max,
                                                            &item.average_distance.unwrap_or(1.0),
                                                        ),
                                                        item.average_distance,
                                                    );
                                                    value_hashmap.insert(
                                                        shape2_name.to_string(),
                                                        (
                                                            proximity,
                                                            shape1.clone(),
                                                            shape2.clone(),
                                                            *rad1,
                                                            *rad2,
                                                            shape1_transform.world,
                                                            shape2_transform.world,
                                                            shape1_frame.to_string(),
                                                            shape2_frame.to_string(),
                                                        ),
                                                    );
                                                    break 'f;
                                                }
                                            }
                                        } else {
                                            proximity = ProximityInfo::new(
                                                shape1_name.to_string(),
                                                shape2_name.to_string(),
                                                self.d_max,
                                                None,
                                                *shape1_physical && *shape2_physical,
                                                self.compute_loss_with_cutoff(&self.d_max, &1.0),
                                                Some(1.0),
                                            );
                                            value_hashmap.insert(
                                                shape2_name.to_string(),
                                                (
                                                    proximity,
                                                    shape1.clone(),
                                                    shape2.clone(),
                                                    *rad1,
                                                    *rad2,
                                                    shape1_transform.world,
                                                    shape2_transform.world,
                                                    shape1_frame.to_string(),
                                                    shape2_frame.to_string(),
                                                ),
                                            );
                                        }
                                    }
                                },
                                Err(_) => {}
                            }
                        }
                    }

                    self.scene_group_truth_distance_hashmap
                        .insert(shape1_name.to_string(), value_hashmap);
                }
            }
        }
    }
    //  for (key, vec) in &self.scene_group_truth_distance_hashmap {
    //     for item in vec {
    //         println!("the key is {:?} and the proximityInfo in vec is {:?}" , key, item.0);
    //     }

    //  }

    pub fn compute_a_table(
        &mut self,
        frames: &Vec<HashMap<String, TransformInfo>>,
    ) -> Vec<ProximityInfo> {
        let mut result_vector: Vec<ProximityInfo> = vec![];
        let mut proximity_look_up: HashMap<(String, String), Vec<f64>> = HashMap::new();
        let size = self.scene_collision_shapes_list.len();
        let default_transform = TransformInfo::default();

        if size != 0 {
            for frame in frames {
                for i in 0..=size - 1 {
                    let (shape1_frame, shape1, _, _, shape1_name, _) =
                        self.scene_collision_shapes_list.get(i).unwrap();
                    if shape1_frame == "world" {
                        break;
                    }
                    for j in (i + 1)..=size - 1 {
                        let (shape2_frame, shape2, _, _, shape2_name, _) =
                            self.scene_collision_shapes_list.get(j).unwrap();
                        if shape1_frame == "world" && shape2_frame == "world" {
                            continue;
                        }
                        let shape1_transform = frame.get(shape1_name).unwrap_or(&default_transform);
                        let shape2_transform = frame.get(shape2_name).unwrap_or(&default_transform);
                        match parry3d_f64::query::contact(
                            &shape1_transform.world,
                            shape1,
                            &shape2_transform.world,
                            shape2,
                            self.d_max,
                        ) {
                            Ok(contact) => match contact {
                                Some(valid_contact) => {
                                    match proximity_look_up.get_mut(&(
                                        shape1_name.to_string(),
                                        shape2_name.to_string(),
                                    )) {
                                        Some(valid_vec) => {
                                            valid_vec.push(valid_contact.dist);
                                        }
                                        None => {
                                            proximity_look_up.insert(
                                                (shape1_name.to_string(), shape2_name.to_string()),
                                                vec![valid_contact.dist],
                                            );
                                        }
                                    }
                                }
                                None => {
                                    match proximity_look_up.get_mut(&(
                                        shape1_name.to_string(),
                                        shape2_name.to_string(),
                                    )) {
                                        Some(valid_vec) => {
                                            valid_vec.push(self.d_max);
                                        }
                                        None => {
                                            proximity_look_up.insert(
                                                (shape1_name.to_string(), shape2_name.to_string()),
                                                vec![self.d_max],
                                            );
                                        }
                                    }
                                }
                            },
                            Err(_) => {}
                        }
                    }
                }
            }

            // println!("lookup: {:?}",proximity_look_up);
            for ((shape1_name, shape2_name), a_vec) in proximity_look_up {
                let mut total = 0.0;
                let mut index = 0.0;
                for value in a_vec.clone() {
                    total += value;
                    index += 1.0;
                }
                let a_value = Some(total / index);

                let proximity = ProximityInfo::new(
                    shape1_name.clone(),
                    shape2_name.clone(),
                    0.0,
                    None,
                    true,
                    0.0,
                    a_value.clone(),
                );
                result_vector.push(proximity.clone());

                for (name, hashmap) in &self.scene_group_truth_distance_hashmap.clone() {
                    for (name2, item) in hashmap {
                        if item.0.shape1 == shape1_name && item.0.shape2 == shape2_name {
                            match self.scene_group_truth_distance_hashmap.get_mut(name) {
                                Some(valid_hashmap) => {
                                    valid_hashmap.get_mut(name2).unwrap().0.average_distance =
                                        a_value;

                                    break;
                                }
                                None => {
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        // println!("--------------------------------------------------------------------------------");
        // for (key, vec) in &self.scene_group_truth_distance_hashmap {
        //         for item in vec {
        //             println!("the key is {:?} and the proximityInfo in vec is {:?} ___ {:?} ___{:?} __ {:?} __ {:?}" , key, item.0, item.3, item.4, item.7, item.8,
        //                         );
        //         }

        //      }
        // println!("--------------------------------------------------------------------------------");

        return result_vector;
    }

    pub fn add_robot_link_transient_shape(
        &mut self,
        id: &String,
        name: String,
        frame: String,
        local_transform: Isometry3<f64>,
        collider: SharedShape,
    ) {
        match self.scene_group_truth_distance_hashmap.clone().get(&frame) {
            Some(shapes_hashmap) => {
                let (_, item) = shapes_hashmap.first().unwrap();
                let shape1_transform = &item.5.clone();
                let old_shape1 = &item.1;
                let mut old_shape1_vec = old_shape1.shapes().to_vec();
                old_shape1_vec.push((local_transform, collider.clone()));
                let index = old_shape1_vec.len() - 1;
                let new_shape1 = Compound::new(old_shape1_vec);

                let new_shape1_radius = new_shape1.local_bounding_sphere().radius;

                for (shape2_name, item) in shapes_hashmap {
                    match parry3d_f64::query::contact(
                        &shape1_transform,
                        &new_shape1,
                        &item.6,
                        &item.2,
                        self.d_max,
                    ) {
                        Ok(contact) => match contact {
                            Some(valid_contact) => {
                                let new_proximity = ProximityInfo::new(
                                    item.0.shape1.to_string(),
                                    item.0.shape2.to_string(),
                                    valid_contact.dist,
                                    Some((valid_contact.point1, valid_contact.point2)),
                                    item.0.physical,
                                    self.compute_loss_with_cutoff(
                                        &valid_contact.dist,
                                        &item.0.average_distance.unwrap_or(1.0),
                                    ),
                                    item.0.average_distance,
                                );

                                let mut_shapes_hashmap = self
                                    .scene_group_truth_distance_hashmap
                                    .get_mut(&frame)
                                    .unwrap();
                                let mut_shapes_pair =
                                    mut_shapes_hashmap.get_mut(shape2_name).unwrap();
                                mut_shapes_pair.0 = new_proximity.clone();
                                mut_shapes_pair.1 = new_shape1.clone();
                                mut_shapes_pair.3 = new_shape1_radius;
                            }
                            None => {
                                let new_proximity = ProximityInfo::new(
                                    item.0.shape1.to_string(),
                                    item.0.shape2.to_string(),
                                    self.d_max,
                                    None,
                                    item.0.physical,
                                    self.compute_loss_with_cutoff(
                                        &self.d_max,
                                        &item.0.average_distance.unwrap_or(1.0),
                                    ),
                                    item.0.average_distance,
                                );

                                let mut_shapes_hashmap = self
                                    .scene_group_truth_distance_hashmap
                                    .get_mut(&frame)
                                    .unwrap();
                                let mut_shapes_pair =
                                    mut_shapes_hashmap.get_mut(shape2_name).unwrap();
                                mut_shapes_pair.0 = new_proximity.clone();
                                mut_shapes_pair.1 = new_shape1.clone();
                                mut_shapes_pair.3 = new_shape1_radius;
                            }
                        },
                        Err(_) => {
                            continue;
                        }
                    }
                }
                self.scene_optima_transient_shapes_look_up.insert(
                    id.to_string(),
                    (frame, name.to_string(), Some(index), collider.clone(), true),
                );
                // println!(
                //     "id : {:?} , robot_link transient shape for {:?} is added",
                //     id,
                //     name.to_string()
                // );
            }
            None => {
                // println!(
                //     "the frame you provided for [{:?}] transient shape is not valid",
                //     name.to_string()
                // );
            }
        }
    }

    pub fn add_world_transient_shape(
        &mut self,
        id: &String,
        name: String,
        local_transform: Isometry3<f64>,
        collider: SharedShape,
        physical: bool,
    ) {
        let shape2_transform = TransformInfo::default().world;
        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
            vec![(local_transform, collider.clone())];
        let shape2 = Compound::new(shape_vec);
        let shape2_radius = shape2.local_bounding_sphere().radius;
        let mut added = false;
        for (shape1_name, shapes_hashmap) in self.scene_group_truth_distance_hashmap.clone() {
            if shapes_hashmap.len() == 0 {
                break;
            }

            let (_, item) = shapes_hashmap.first().unwrap();
            let shape1_transform = &item.5.clone();
            let shape1_radius = &item.3;
            let shape1 = &item.1;
            let shape1_frame = &item.7;

            match parry3d_f64::query::contact(
                shape1_transform,
                shape1,
                &shape2_transform,
                &shape2.clone(),
                self.d_max,
            ) {
                Ok(contact) => match contact {
                    Some(valid_contact) => {
                        added = true;
                        let proximity = ProximityInfo::new(
                            shape1_name.to_string(),
                            name.to_string(),
                            valid_contact.dist,
                            Some((valid_contact.point1, valid_contact.point2)),
                            true && physical,
                            self.compute_loss_with_cutoff(&valid_contact.dist, &1.0),
                            None,
                        );
                        let mut_hashmap = self
                            .scene_group_truth_distance_hashmap
                            .get_mut(&shape1_name.clone())
                            .unwrap();

                        mut_hashmap.insert(
                            name.to_string(),
                            (
                                proximity,
                                shape1.clone(),
                                shape2.clone(),
                                *shape1_radius,
                                shape2_radius,
                                *shape1_transform,
                                shape2_transform,
                                shape1_frame.to_string(),
                                "world".to_string(),
                            ),
                        );
                    }
                    None => {
                        added = true;
                        let proximity = ProximityInfo::new(
                            shape1_name.to_string(),
                            name.to_string(),
                            self.d_max,
                            None,
                            true && physical,
                            self.compute_loss_with_cutoff(&self.d_max, &1.0),
                            None,
                        );
                        let mut_hashmap = self
                            .scene_group_truth_distance_hashmap
                            .get_mut(&shape1_name.clone())
                            .unwrap();

                        mut_hashmap.insert(
                            name.to_string(),
                            (
                                proximity,
                                shape1.clone(),
                                shape2.clone(),
                                *shape1_radius,
                                shape2_radius,
                                *shape1_transform,
                                shape2_transform,
                                shape1_frame.to_string(),
                                "world".to_string(),
                            ),
                        );
                    }
                },
                Err(_) => {}
            }
        }
        if added == true {
            self.scene_optima_transient_shapes_look_up.insert(
                id.to_string(),
                (
                    "world".to_string(),
                    name.to_string(),
                    None,
                    collider,
                    physical,
                ),
            );
            // println!(
            //     "id : {:?} , world transient shape for {:?} is added",
            //     id, name
            // );
        }
    }

    pub fn remove_transient_shape(&mut self, id: &String) {
        match self.scene_optima_transient_shapes_look_up.clone().get(id) {
            Some((frame, delete_name, index, _, _)) => {
                if frame == "world" {
                    for (shape1_frame, _) in self.scene_group_truth_distance_hashmap.clone() {
                        let mut_shapes_hashmap = self
                            .scene_group_truth_distance_hashmap
                            .get_mut(&shape1_frame)
                            .unwrap();
                        mut_shapes_hashmap.remove_entry(delete_name);
                    }
                    self.scene_optima_transient_shapes_look_up.remove_entry(id);
                    // println!("removal for transient shape of id {:?} is successful", id);
                } else {
                    match self.scene_group_truth_distance_hashmap.clone().get(frame) {
                        Some(shapes_hashmap) => {
                            let (_, pair_info) = shapes_hashmap.first().unwrap();
                            let mut old_shape1_vec = pair_info.1.shapes().to_vec();

                            match index {
                                Some(index) => {
                                    if *index < old_shape1_vec.len() {
                                        let mut replace_hashmap: IndexMap<
                                            String,
                                            (
                                                ProximityInfo,
                                                Compound,
                                                Compound,
                                                f64,
                                                f64,
                                                Isometry3<f64>,
                                                Isometry3<f64>,
                                                String,
                                                String,
                                            ),
                                        > = IndexMap::new();
                                        old_shape1_vec.remove(*index);
                                        let shape1 = Compound::new(old_shape1_vec.clone());
                                        let shape1_rad = shape1.local_bounding_sphere().radius;
                                        let shape1_transform = pair_info.5;

                                        for (shape2_frame, pair_info) in shapes_hashmap {
                                            let shape2_transform = pair_info.6;
                                            let shape2 = &pair_info.2;
                                            match parry3d_f64::query::contact(
                                                &shape1_transform,
                                                &shape1.clone(),
                                                &shape2_transform,
                                                &shape2.clone(),
                                                self.d_max,
                                            ) {
                                                Ok(contact) => match contact {
                                                    Some(valid_contact) => {
                                                        let proximity = ProximityInfo::new(
                                                            pair_info.0.shape1.to_string(),
                                                            pair_info.0.shape2.to_string(),
                                                            valid_contact.dist,
                                                            Some((
                                                                valid_contact.point1,
                                                                valid_contact.point2,
                                                            )),
                                                            pair_info.0.physical,
                                                            self.compute_loss_with_cutoff(
                                                                &valid_contact.dist,
                                                                &pair_info
                                                                    .0
                                                                    .average_distance
                                                                    .unwrap_or(1.0),
                                                            ),
                                                            pair_info.0.average_distance,
                                                        );

                                                        replace_hashmap.insert(
                                                            shape2_frame.to_string(),
                                                            (
                                                                proximity,
                                                                shape1.clone(),
                                                                shape2.clone(),
                                                                shape1_rad,
                                                                pair_info.4,
                                                                shape1_transform,
                                                                shape2_transform,
                                                                frame.to_string(),
                                                                shape2_frame.to_string(),
                                                            ),
                                                        );
                                                        self.scene_optima_transient_shapes_look_up
                                                            .remove_entry(&id.to_string());
                                                    }

                                                    None => {
                                                        let proximity = ProximityInfo::new(
                                                            pair_info.0.shape1.to_string(),
                                                            pair_info.0.shape2.to_string(),
                                                            self.d_max,
                                                            None,
                                                            pair_info.0.physical,
                                                            self.compute_loss_with_cutoff(
                                                                &self.d_max,
                                                                &pair_info
                                                                    .0
                                                                    .average_distance
                                                                    .unwrap_or(1.0),
                                                            ),
                                                            pair_info.0.average_distance,
                                                        );

                                                        replace_hashmap.insert(
                                                            shape2_frame.to_string(),
                                                            (
                                                                proximity,
                                                                shape1.clone(),
                                                                shape2.clone(),
                                                                shape1_rad,
                                                                pair_info.4,
                                                                shape1_transform,
                                                                shape2_transform,
                                                                frame.to_string(),
                                                                shape2_frame.to_string(),
                                                            ),
                                                        );
                                                        self.scene_optima_transient_shapes_look_up
                                                            .remove_entry(&id.to_string());
                                                    }
                                                },
                                                Err(_) => {}
                                            }
                                        }

                                        self.scene_group_truth_distance_hashmap
                                            .clone()
                                            .remove(frame)
                                            .unwrap();
                                        self.scene_group_truth_distance_hashmap
                                            .insert(frame.to_string(), replace_hashmap);
                                        // println!(
                                        //     "removal for transient shape of id {:?} is successful",
                                        //     id
                                        // );
                                    }
                                }
                                None => {}
                            }

                            //let
                        }
                        None => {}
                    }
                }
            }
            None => {
                // println!("invalid id for removal: [{:?}]", id);
            }
        }
    }

    pub fn move_transient_shape(&mut self, id: &String, pose: Isometry3<f64>) {
        match self.scene_optima_transient_shapes_look_up.clone().get(id) {
            Some((frame, name, _, collider, physical)) => {
                self.remove_transient_shape(&id);
                if frame == "world" {
                    self.add_world_transient_shape(
                        id,
                        name.to_string(),
                        pose,
                        collider.clone(),
                        *physical,
                    );
                    //println!("transient shape moved for id of {:?}", id);
                } else {
                    self.add_robot_link_transient_shape(
                        id,
                        name.to_string(),
                        frame.to_string(),
                        pose,
                        collider.clone(),
                    );
                    // println!("transient shape moved for id of {:?}", id);
                }
            }
            None => {
                //println!("invalid id for move: [{:?}]", id);
            }
        }
    }

    pub fn perform_updates(&mut self, shape_updates: &Vec<ShapeUpdate>) {
        for update in shape_updates {
            match update {
                ShapeUpdate::Add(add_shape) => match &add_shape.shape {
                    shapes::Shape::Box(object) => {
                        let box_collider = SharedShape::cuboid(object.y, object.x, object.z);

                        if self
                            .scene_optima_transient_shapes_look_up
                            .clone()
                            .contains_key(&add_shape.id)
                        {
                            println!("WARNING: overwriting the shape [{:?}] because another transient shape with the same id already exist in the scene" , add_shape.id);
                            self.remove_transient_shape(&add_shape.id);
                            if object.frame == "world" {
                                // if replace shape is world frame
                                self.add_world_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.local_transform,
                                    box_collider,
                                    object.physical,
                                );
                            } else {
                                // if replace shape is robot link frame
                                self.add_robot_link_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.frame.to_string(),
                                    object.local_transform,
                                    box_collider,
                                )
                            }
                        } else {
                            if object.frame == "world" {
                                // adding a brand new world transient shape
                                self.add_world_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.local_transform,
                                    box_collider,
                                    object.physical,
                                );
                            } else {
                                // addinng a brand new robot link shape
                                self.add_robot_link_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.frame.to_string(),
                                    object.local_transform,
                                    box_collider,
                                )
                            }
                        }
                    }

                    shapes::Shape::Cylinder(object) => {
                        let new_length = object.length / 2.0;
                        let collider = SharedShape::cylinder(new_length, object.radius);
                        if self
                            .scene_optima_transient_shapes_look_up
                            .clone()
                            .contains_key(&add_shape.id)
                        {
                            println!("WARNING: overwriting the shape [{:?}] because another transient shape with the same id already exist in the scene" , add_shape.id);
                            self.remove_transient_shape(&add_shape.id);
                            if object.frame == "world" {
                                // if replace shape is world frame
                                self.add_world_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.local_transform,
                                    collider,
                                    object.physical,
                                );
                            } else {
                                // if replace shape is robot link frame
                                self.add_robot_link_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.frame.to_string(),
                                    object.local_transform,
                                    collider,
                                )
                            }
                        } else {
                            if object.frame == "world" {
                                // adding a brand new world transient shape
                                self.add_world_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.local_transform,
                                    collider,
                                    object.physical,
                                );
                            } else {
                                // addinng a brand new robot link shape
                                self.add_robot_link_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.frame.to_string(),
                                    object.local_transform,
                                    collider,
                                )
                            }
                        }
                    }
                    shapes::Shape::Capsule(object) => {
                        let point_a = Point3::new(
                            object.length * vector![0.0, 1.0, 0.0][0],
                            object.length * vector![0.0, 1.0, 0.0][1],
                            object.length * vector![0.0, 1.0, 0.0][2],
                        );
                        let point_b = Point3::new(
                            object.length * vector![0.0, -1.0, 0.0][0],
                            object.length * vector![0.0, -1.0, 0.0][1],
                            object.length * vector![0.0, -1.0, 0.0][2],
                        );
                        let collider = SharedShape::capsule(point_a, point_b, object.radius);

                        if self
                            .scene_optima_transient_shapes_look_up
                            .clone()
                            .contains_key(&add_shape.id)
                        {
                            println!("WARNING: overwriting the shape [{:?}] because another transient shape with the same id already exist in the scene" , add_shape.id);
                            self.remove_transient_shape(&add_shape.id);
                            if object.frame == "world" {
                                // if replace shape is world frame
                                self.add_world_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.local_transform,
                                    collider,
                                    object.physical,
                                );
                            } else {
                                // if replace shape is robot link frame
                                self.add_robot_link_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.frame.to_string(),
                                    object.local_transform,
                                    collider,
                                )
                            }
                        } else {
                            if object.frame == "world" {
                                // adding a brand new world transient shape
                                self.add_world_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.local_transform,
                                    collider,
                                    object.physical,
                                );
                            } else {
                                // addinng a brand new robot link shape
                                self.add_robot_link_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.frame.to_string(),
                                    object.local_transform,
                                    collider,
                                )
                            }
                        }
                    }
                    shapes::Shape::Sphere(object) => {
                        let collider = SharedShape::ball(object.radius);
                        if self
                            .scene_optima_transient_shapes_look_up
                            .clone()
                            .contains_key(&add_shape.id)
                        {
                            println!("WARNING: overwriting the shape [{:?}] because another transient shape with the same id already exist in the scene" , add_shape.id);
                            self.remove_transient_shape(&add_shape.id);
                            if object.frame == "world" {
                                // if replace shape is world frame
                                self.add_world_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.local_transform,
                                    collider,
                                    object.physical,
                                );
                            } else {
                                // if replace shape is robot link frame
                                self.add_robot_link_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.frame.to_string(),
                                    object.local_transform,
                                    collider,
                                )
                            }
                        } else {
                            if object.frame == "world" {
                                // adding a brand new world transient shape
                                self.add_world_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.local_transform,
                                    collider,
                                    object.physical,
                                );
                            } else {
                                // addinng a brand new robot link shape
                                self.add_robot_link_transient_shape(
                                    &add_shape.id,
                                    object.name.to_string(),
                                    object.frame.to_string(),
                                    object.local_transform,
                                    collider,
                                )
                            }
                        }
                    }
                    shapes::Shape::Hull(object) => {
                        let hull_points: Vec<Point3<f64>> = object
                            .points
                            .iter()
                            .map(|p| Point3::new(p.x, p.y, p.z))
                            .collect();

                        match SharedShape::convex_hull(hull_points.as_slice()) {
                            Some(collider) => {
                                if self
                                    .scene_optima_transient_shapes_look_up
                                    .clone()
                                    .contains_key(&add_shape.id)
                                {
                                    println!("WARNING: overwriting the shape [{:?}] because another transient shape with the same id already exist in the scene" , add_shape.id);
                                    self.remove_transient_shape(&add_shape.id);
                                    if object.frame == "world" {
                                        // if replace shape is world frame
                                        self.add_world_transient_shape(
                                            &add_shape.id,
                                            object.name.to_string(),
                                            object.local_transform,
                                            collider,
                                            object.physical,
                                        );
                                    } else {
                                        // if replace shape is robot link frame
                                        self.add_robot_link_transient_shape(
                                            &add_shape.id,
                                            object.name.to_string(),
                                            object.frame.to_string(),
                                            object.local_transform,
                                            collider,
                                        )
                                    }
                                } else {
                                    if object.frame == "world" {
                                        // adding a brand new world transient shape
                                        self.add_world_transient_shape(
                                            &add_shape.id,
                                            object.name.to_string(),
                                            object.local_transform,
                                            collider,
                                            object.physical,
                                        );
                                    } else {
                                        // addinng a brand new robot link shape
                                        self.add_robot_link_transient_shape(
                                            &add_shape.id,
                                            object.name.to_string(),
                                            object.frame.to_string(),
                                            object.local_transform,
                                            collider,
                                        )
                                    }
                                }
                            }
                            None => {
                                // println!("cannot form a valid hull");
                            }
                        }
                    }
                    shapes::Shape::Mesh(_) => {}
                },
                ShapeUpdate::Move(move_shape) => {
                    self.move_transient_shape(&move_shape.id, move_shape.transform);
                }
                ShapeUpdate::Delete(id) => {
                    self.remove_transient_shape(id);
                }
            }
        }
    }
    pub fn clear_all_transient_shapes(&mut self) {
        for (id, _) in self.scene_optima_transient_shapes_look_up.clone() {
            self.remove_transient_shape(&id);
        }
        //println!("ALL transient shapes removed");
    }

    pub fn compute_relative_change_in_transform(
        &self,
        shape1_current_frame: &String,
        shape2_current_frame: &String,
        current_frame: &HashMap<String, TransformInfo>,
        shape1_j_transform: &Isometry3<f64>,
        shape2_j_transform: &Isometry3<f64>,
    ) -> Option<(f64, f64)> {
        let default_frame_transform: TransformInfo = TransformInfo::default();
        let shape1_j_translation = shape1_j_transform.translation;
        let shape1_j_rotation = shape1_j_transform.rotation;

        let shape1_current_translation = current_frame
            .get(shape1_current_frame)
            .unwrap_or(&default_frame_transform)
            .world
            .translation;
        let shape1_current_rotation = current_frame
            .get(shape1_current_frame)
            .unwrap_or(&default_frame_transform)
            .world
            .rotation;

        let shape2_j_translation = shape2_j_transform.translation;
        let shape2_j_rotation = shape2_j_transform.rotation;
        let shape2_current_translation = current_frame
            .get(shape2_current_frame)
            .unwrap_or(&default_frame_transform)
            .world
            .translation;
        let shape2_current_rotation = current_frame
            .get(shape2_current_frame)
            .unwrap_or(&default_frame_transform)
            .world
            .rotation;

        let change_in_relative_translation =
            ((shape1_j_translation.vector - shape2_j_translation.vector).norm()
                - (shape1_current_translation.vector - shape2_current_translation.vector).norm())
            .abs();
        let change_in_relative_rotation = (shape1_j_rotation
            .rotation_to(&shape2_j_rotation)
            .rotation_to(&shape1_current_rotation.rotation_to(&shape2_current_rotation)))
        .angle();

        return Some((change_in_relative_translation, change_in_relative_rotation));
    }
    // #[profiling::function]
    pub fn compute_bounding_spheres(&self, compound_shape: &Compound) -> f64 {
        return compound_shape.local_bounding_sphere().radius;
    }

    //#[profiling::function]
    pub fn compute_translation_of_a_point_induced_by_rotation(
        &self,
        distance: f64,
        rotation: f64,
    ) -> f64 {
        let result = f64::sqrt(2.0 * (distance * distance) * (1.0 - rotation.cos()));
        return result;
    }

    // #[profiling::function]
    pub fn compute_lower_signed_distance_bound(
        &self,
        shape1_frame: &String,
        shape2_frame: &String,
        current_frame: &HashMap<String, TransformInfo>,
        j_state: &(ProximityInfo, Isometry3<f64>, Isometry3<f64>),
        rad1: f64,
        rad2: f64,
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
                let shape1_bounding_sphere_radius = rad1;
                let shape2_bounding_sphere_radius = rad2;
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

    //#[profiling::function]
    pub fn compute_upper_signed_distance_bound(
        &self,
        shape1_frame: &String,
        shape2_frame: &String,
        current_state: &HashMap<String, TransformInfo>,
        j_state: &(ProximityInfo, Isometry3<f64>, Isometry3<f64>),
    ) -> f64 {
        let default_frame_transform: TransformInfo = TransformInfo::default();
        let shape1_current_state_rotation = current_state
            .get(shape1_frame)
            .unwrap_or(&default_frame_transform)
            .world
            .rotation;

        let shape1_past_state_rotation = j_state.1.rotation;

        let shape2_current_state_rotation = current_state
            .get(shape2_frame)
            .unwrap_or(&default_frame_transform)
            .world
            .rotation;

        let shape2_past_state_rotation = j_state.2.rotation;

        let shape1_current_state_translation = current_state
            .get(shape1_frame)
            .unwrap_or(&default_frame_transform)
            .world
            .translation;

        let shape1_past_state_translation = j_state.1.translation;

        let shape2_current_state_translation = current_state
            .get(shape2_frame)
            .unwrap_or(&default_frame_transform)
            .world
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

    //#[profiling::function]
    pub fn compute_loss_function(&self, x: &f64) -> f64 {
        let result;
        if *x <= 0.0 {
            result = -*x + 1.0;
        } else {
            let c = 0.2 * self.a_max;

            result = (-(*x * *x) / (0.2 * c * c)).exp();
        }
        //println!("entered compute_loss_function with {:?}" , result);
        return result;
    }

    //#[profiling::function]
    pub fn compute_loss_with_cutoff(&self, x: &f64, a_value: &f64) -> f64 {
        // if *a_std == 0.0 {
        //     return 0.0;
        // }

        // if *x >= self.d_max || *x >= self.a_max * *a_std + *a_value {
        //     return 0.0;
        // } else {

        //     return self.compute_loss_function(& ((*x - *a_value) / *a_std));
        // }

        // if *a_std == 0.0 {
        //     return 0.0;
        // }

        if *x >= self.d_max || *x / *a_value >= self.a_max {
            return 0.0;
        } else {
            return self.compute_loss_function(&(*x / *a_value));
        }
    }

    // #[profiling::function]
    pub fn compute_maximum_loss_functions_error(
        &self,
        shape1_frame: &String,
        shape2_frame: &String,
        current_frame: &HashMap<String, TransformInfo>,
        j_state: &(ProximityInfo, Isometry3<f64>, Isometry3<f64>),
        rad1: f64,
        rad2: f64,
    ) -> f64 {
        let result;

        let lower_bound = self.compute_lower_signed_distance_bound(
            shape1_frame,
            shape2_frame,
            current_frame,
            j_state,
            rad1,
            rad2,
        );

        let upper_bound = self.compute_upper_signed_distance_bound(
            shape1_frame,
            shape2_frame,
            current_frame,
            j_state,
        );

        let estimated_distance = (1.0 - self.r) * lower_bound + self.r * upper_bound;

        let a_value = j_state.0.average_distance.unwrap_or(1.0);

        let loss_value_distance = self.compute_loss_with_cutoff(&estimated_distance, &a_value);
        let loss_value_lower_bound = self.compute_loss_with_cutoff(&lower_bound, &a_value);
        let loss_value_upper_bound = self.compute_loss_with_cutoff(&upper_bound, &a_value);

        result = (loss_value_lower_bound - loss_value_distance)
            .max(loss_value_distance - loss_value_upper_bound);

        // println!("first shape is {:?}, second shape is {:?}, lower bound is {:?} , upper bound is {:?}, estimated distance is {:?}, a_value is {:?},loss_value_distance is {:?} ,
        //          loss_value_lower_bound is {:?} , loss_value_upper_bound is {:?}, result is {:?}" , shape1_frame, shape2_frame, lower_bound, upper_bound, estimated_distance, a_value,loss_value_distance,
        //         loss_value_lower_bound, loss_value_upper_bound, result);

        return result;
    }

    //#[profiling::function]
    pub fn ranking_maximum_loss_functions_error(
        &self,
        current_frame: &HashMap<String, TransformInfo>,
    ) -> Vec<(
        String,
        Compound,
        String,
        Compound,
        f64,
        Isometry3<f64>,
        Isometry3<f64>,
        ProximityInfo,
    )> {
        let mut loss_functions_error_vec: Vec<(
            String,
            Compound,
            String,
            Compound,
            f64,
            Isometry3<f64>,
            Isometry3<f64>,
            ProximityInfo,
        )> = vec![];
        let timed_timer = Instant::now();
        let default_frame_transform: TransformInfo = TransformInfo::default();
        // for (name, transform) in current_frame {
        //     println!("{:?} , {:?}" , name, transform.world);
        // }

        // println!("--------------------------------------------------------------------------------");
        // for (key, vec) in &self.scene_group_truth_distance_hashmap {
        //         for item in vec {
        //             println!("in ranking_maximum_loss_functions_error, the key is {:?} and the proximityInfo in vec is {:?} " , key, item.0
        //                         );
        //         }

        //      }
        // println!("--------------------------------------------------------------------------------");

        for (_, valid_hashmap) in self.scene_group_truth_distance_hashmap.clone() {
            for (
                _name2,
                (
                    proximity,
                    shape1_compound,
                    shape2_compound,
                    rad1,
                    rad2,
                    pos1,
                    pos2,
                    shape1_frame,
                    shape2_frame,
                ),
            ) in valid_hashmap
            {
                // println!("the shapes in rankingMax is {:?}, {:?} " , shape1_frame,shape2_frame);
                let new_shape1_transform = current_frame
                    .get(&shape1_frame)
                    .unwrap_or(&default_frame_transform)
                    .world;
                let new_shape2_transform = current_frame
                    .get(&shape2_frame)
                    .unwrap_or(&default_frame_transform)
                    .world;
                if timed_timer.elapsed().as_micros()
                    < Duration::from_micros(self.time_budget).as_micros()
                {
                    let current_loss_function_error = self.compute_maximum_loss_functions_error(
                        &shape1_frame,
                        &shape2_frame,
                        current_frame,
                        &(proximity.clone(), pos1, pos2),
                        rad1,
                        rad2,
                    );

                    //println!("the current_loss_function_error for {:?} , {:?} is : {:?}" , proximity.shape1, proximity.shape2, current_loss_function_error);
                    loss_functions_error_vec.push((
                        shape1_frame,
                        shape1_compound.clone(),
                        shape2_frame,
                        shape2_compound.clone(),
                        current_loss_function_error,
                        new_shape1_transform,
                        new_shape2_transform,
                        proximity,
                    ));
                } else {
                    loss_functions_error_vec.push((
                        shape1_frame,
                        shape1_compound.clone(),
                        shape2_frame,
                        shape2_compound.clone(),
                        proximity.loss,
                        new_shape1_transform,
                        new_shape2_transform,
                        proximity,
                    ));
                }
                //}
            }
        }
        // println!("-------------------------------------------------");
        // for item in loss_functions_error_vec.clone() {

        //     println!("the loss value with error for {:?} , {:?} is : {:?}" , item.7.shape1, item.7.shape2, item.4);
        // }
        // println!("-------------------------------------------------");
        if loss_functions_error_vec.len() > 1 {
            loss_functions_error_vec
                .sort_unstable_by(|a, b| b.4.partial_cmp(&a.4).unwrap_or(Ordering::Equal));
        }

        return loss_functions_error_vec;
    }
    //#[profiling::function]
    pub fn get_proximity(&mut self, frames: &HashMap<String, TransformInfo>) -> Vec<ProximityInfo> {
        let mut result_vector: Vec<ProximityInfo> = vec![];
        let ranking_vector: Vec<(
            String,
            Compound,
            String,
            Compound,
            f64,
            Isometry3<f64>,
            Isometry3<f64>,
            ProximityInfo,
        )> = self.ranking_maximum_loss_functions_error(frames);
        if self.timed {
            //let timed_timer = Instant::now();

            for (shape1_frame, shape1, _, shape2, _, pos1, pos2, proximity_info) in ranking_vector {
                //if timed_timer.elapsed().as_micros() < Duration::from_micros(500).as_micros() {

                let contact =
                    parry3d_f64::query::contact(&pos1, &shape1, &pos2, &shape2, self.d_max);
                match contact {
                    Ok(contact) => match contact {
                        Some(valid_contact) => {
                            let dist = valid_contact.dist;
                            let loss = self.compute_loss_with_cutoff(
                                &dist,
                                &proximity_info.average_distance.unwrap_or(1.0),
                            );

                            let proximity = ProximityInfo::new(
                                proximity_info.clone().shape1,
                                proximity_info.clone().shape2,
                                dist,
                                Some((valid_contact.point1, valid_contact.point2)),
                                proximity_info.physical,
                                loss,
                                proximity_info.average_distance,
                            );
                            let mut_hashmap = self
                                .scene_group_truth_distance_hashmap
                                .get_mut(&shape1_frame)
                                .unwrap();
                            for (_, item) in mut_hashmap {
                                if item.0.shape1 == proximity_info.shape1
                                    && item.0.shape2 == proximity_info.shape2
                                {
                                    item.0 = proximity.clone();
                                    item.5 = pos1;
                                    item.6 = pos2;

                                    break;
                                }
                            }
                            result_vector.push(proximity.clone());
                        }
                        None => {
                            result_vector.push(proximity_info);
                        }
                    },

                    Err(_) => {}
                }

                // }else {
                // result_vector.push(proximity_info);

                // }
            }

            // println!("-----------------------------------------------------------------------------------");
            // for item in result_vector.clone() {
            //        //if item.shape2 == "sphere" {
            //             println!("the info is : {:?}", item);
            //         //}

            //         //println!("{:?} and {:?},and the average distance is {:?}" , item.shape1, item.shape2, item.average_distance.unwrap_or(1.0));

            // }

            // println!("-----------------------------------------------------------------------------------");

            return result_vector;
        } else {
            let mut remaining_error_summation = 0.0;
            let mut index = 0;
            let size = ranking_vector.len();

            for (_, _, _, _, loss_value, _, _, _) in ranking_vector.clone() {
                remaining_error_summation += loss_value;
            }
            // println!("-----------------------------------------------------------------------------------");
            // // // for item in result_vector.clone() {
            // // //     println!("the info is : {:?}" , item );
            // // // }
            //  println!("the length is : {:?}" , remaining_error_summation.clone() );
            //  println!("-----------------------------------------------------------------------------------");
            while remaining_error_summation > ACCURACY_BUDGET
                && remaining_error_summation != 0.0
                && index < size - 1
            {
                // println!("{:?}" , remaining_error_summation);
                // println!("{:?}" , index);
                match ranking_vector.get(index) {
                    Some((
                        current_shape1_frame,
                        current_shape1,
                        current_shape2_frame,
                        current_shape2,
                        current_loss_value,
                        _,
                        pos2,
                        proximity_info,
                    )) => {
                        let shape1_transform = frames.get(current_shape1_frame);
                        match shape1_transform {
                            Some(shape1_transform) => {
                                let shape2_transform = pos2;
                                let contact = parry3d_f64::query::contact(
                                    &shape1_transform.world,
                                    current_shape1,
                                    &shape2_transform,
                                    current_shape2,
                                    self.d_max,
                                );
                                match contact {
                                    Ok(contact) => match contact {
                                        Some(valid_contact) => {
                                            let proximity = ProximityInfo::new(
                                                current_shape1_frame.clone(),
                                                current_shape2_frame.clone(),
                                                valid_contact.dist,
                                                Some((valid_contact.point1, valid_contact.point2)),
                                                proximity_info.physical,
                                                self.compute_loss_with_cutoff(
                                                    &valid_contact.dist,
                                                    &proximity_info.average_distance.unwrap(),
                                                ),
                                                proximity_info.average_distance,
                                            );
                                            result_vector.push(proximity.clone());
                                            remaining_error_summation -= current_loss_value;
                                            index += 1;
                                        }
                                        None => {}
                                    },
                                    Err(_) => {
                                        remaining_error_summation -= current_loss_value;
                                        index += 1;
                                    }
                                }
                                remaining_error_summation -= current_loss_value;
                                index += 1;
                            }
                            None => {
                                remaining_error_summation -= current_loss_value;
                                index += 1;
                            }
                        }
                    }
                    None => {
                        break;
                    }
                }
            }

            // println!("-----------------------------------------------------------------------------------");
            // // // for item in result_vector.clone() {
            // // //     println!("the info is : {:?}" , item );
            // // // }
            //  println!("the length is : {:?}" , result_vector.clone().len() );
            //  println!("-----------------------------------------------------------------------------------");
            return result_vector;
        }
        //println!("================================================================================================");
    }
}
