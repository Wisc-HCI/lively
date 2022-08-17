use crate::utils::info::{
    CollisionSettingInfo, LinkInfo, ProximityInfo, ShapeUpdate, TransformInfo,
};
use crate::utils::shapes;
use nalgebra::base::Vector3;
use nalgebra::geometry::Isometry3;
use nalgebra::vector;
use nalgebra::Point3;
use parry3d_f64::shape::*;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::fmt;
use std::time::{Duration, Instant};



const OPTIMA_NUMBER: usize = 10;
const ACCURACY_BUDGET: f64 = 0.1;
#[derive(Clone)]
pub struct CollisionManager {
    scene_collision_shapes_list: Vec<(String, Compound, f64, Isometry3<f64> )>,
    scene_optima_collision_shapes_look_up: HashMap<String, (Compound, f64, Isometry3<f64>)>,
    scene_transient_shapes_look_up: HashMap<String, (usize, usize)>,
    scene_optima_transient_shapes_look_up: HashMap<String, (String, Vec<Option<usize>>)>,
    scene_group_truth_distance_hashmap: HashMap<
        String,
        Vec<(
            ProximityInfo,
            Compound,
            Compound,
            f64,
            f64,
            Isometry3<f64>,
            Isometry3<f64>,
        )>,
    >,
    optima_version: bool,
    d_max: f64,
    r: f64,
    a_max: f64,
    time_budget: Duration,
    timed: bool,
    a_value: f64,
}

impl fmt::Debug for CollisionManager {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CollisionManager").finish()
    }
}


impl CollisionManager {
    #[profiling::function]
    pub fn new(
        links: Vec<LinkInfo>,
        persistent_shapes: Vec<shapes::Shape>,
        collision_settings: &Option<CollisionSettingInfo>,
    ) -> Self {
        let d_max : f64;
        let r : f64;
        let a_max : f64;
        let time_budget : Duration;
        let timed : bool;
        let a_value : f64;
        match collision_settings {
            Some (collision_settings) => {
                d_max = collision_settings.d_max;
                r = collision_settings.r;
                a_max = collision_settings.a_max;
                time_budget = collision_settings.time_budget;
                timed= collision_settings.timed;
                a_value = collision_settings.a_value;
            }
            None => {
                d_max = CollisionSettingInfo::default().d_max;
                r = CollisionSettingInfo::default().r;
                a_max = CollisionSettingInfo::default().a_max;
                time_budget = CollisionSettingInfo::default().time_budget;
                timed= CollisionSettingInfo::default().timed;
                a_value = CollisionSettingInfo::default().a_value;

            }
        }
        let mut optima_version = false;
        let mut scene_collision_shapes_list: Vec<(String, Compound, f64,Isometry3<f64>)> = vec![];
        let scene_optima_collision_shapes_look_up: HashMap<
            String,
            (Compound, f64, Isometry3<f64>),
        > = HashMap::new();
        let scene_group_truth_distance_hashmap = HashMap::new();
        let scene_transient_shapes_look_up = HashMap::new();
        let scene_optima_transient_shapes_look_up = HashMap::new();
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
                scene_collision_shapes_list.push((
                    frame_name.to_string(),
                    robot_compound_shapes,
                    bounding_sphere_radius,
                    Isometry3::identity()
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
                        scene_collision_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            bounding_sphere_radius,
                            box_object.local_transform
                        ));
                    } else {
                        let index_element = scene_collision_shapes_list
                            .iter()
                            .position(|x| x.0 == box_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_collision_shapes_list.clone();
                                let (frame_name, compound_shape, _,_) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
                                let mut new_compound_shape_vec = compound_shape.shapes().to_vec();
                                new_compound_shape_vec
                                    .push((box_object.local_transform, box_shape));
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
                                        box_object.local_transform
                                    ),
                                );
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(box_object.local_transform, box_shape)];
                                let temp_compound = Compound::new(temp_list);
                                let bounding_sphere_radius =
                                    temp_compound.local_bounding_sphere().radius;
                                scene_collision_shapes_list.push((
                                    box_object.frame.to_string(),
                                    temp_compound,
                                    bounding_sphere_radius,
                                    box_object.local_transform,
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
                        scene_collision_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            bounding_sphere_radius,
                            cylinder_object.local_transform
                        ));
                    } else {
                        let index_element = scene_collision_shapes_list
                            .iter()
                            .position(|x| x.0 == cylinder_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_collision_shapes_list.clone();
                                let (frame_name, compound_shape,_, _) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
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
                                        cylinder_object.local_transform
                                    ),
                                );
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> = vec![(
                                    cylinder_object.local_transform * transform_offset,
                                    cylinder_shape,
                                )];
                                let temp_compound = Compound::new(temp_list);
                                let bounding_sphere_radius =
                                    temp_compound.local_bounding_sphere().radius;
                                scene_collision_shapes_list.push((
                                    cylinder_object.frame.to_string(),
                                    temp_compound,
                                    bounding_sphere_radius,
                                    cylinder_object.local_transform
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
                        scene_collision_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            bounding_sphere_radius,
                            sphere_object.local_transform
                        ));
                    } else {
                        let index_element = scene_collision_shapes_list
                            .iter()
                            .position(|x| x.0 == sphere_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_collision_shapes_list.clone();
                                let (frame_name, compound_shape, _,_) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
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
                                        sphere_object.local_transform
                                    ),
                                );
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(sphere_object.local_transform, sphere_shape)];
                                let temp_compound = Compound::new(temp_list);
                                let bounding_sphere_radius =
                                    temp_compound.local_bounding_sphere().radius();
                                scene_collision_shapes_list.push((
                                    sphere_object.frame.to_string(),
                                    temp_compound,
                                    bounding_sphere_radius,
                                    sphere_object.local_transform
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
                        scene_collision_shapes_list.push((
                            "world".to_string(),
                            temp_compound,
                            temp_compound_radius,
                            capsule_object.local_transform
                        ));
                    } else {
                        let index_element = scene_collision_shapes_list
                            .iter()
                            .position(|x| x.0 == capsule_object.frame);
                        match index_element {
                            Some(valid_index) => {
                                let temp_scene_compound_shapes_list =
                                    scene_collision_shapes_list.clone();
                                let (frame_name, compound_shape,_, _) =
                                    temp_scene_compound_shapes_list.get(valid_index).unwrap();
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
                                        capsule_object.local_transform
                                    ),
                                );
                            }
                            None => {
                                let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(capsule_object.local_transform, capsule_shape)];
                                let temp_compound = Compound::new(temp_list);
                                let temp_compound_radius =
                                    temp_compound.local_bounding_sphere().radius;
                                scene_collision_shapes_list.push((
                                    capsule_object.frame.to_string(),
                                    temp_compound,
                                    temp_compound_radius,
                                    capsule_object.local_transform
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
                                scene_collision_shapes_list.push((
                                    "world".to_string(),
                                    temp_compound,
                                    bounding_sphere_radius,
                                    hull_object.local_transform

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
                                        let (frame_name, compound_shape, _,_) =
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
                                        scene_collision_shapes_list.remove(valid_index);
                                        scene_collision_shapes_list.insert(
                                            valid_index,
                                            (
                                                frame_name.to_string(),
                                                new_compound_shape,
                                                bounding_sphere_radius,
                                                hull_object.local_transform
                                            ),
                                        );
                                    }
                                    None => {
                                        let temp_list: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(hull_object.local_transform, valid_hull_shape)];
                                        let temp_compound = Compound::new(temp_list);
                                        let temp_compound_bounding_sphere_radius =
                                            temp_compound.local_bounding_sphere().radius;
                                        scene_collision_shapes_list.push((
                                            hull_object.frame.to_string(),
                                            temp_compound,
                                            temp_compound_bounding_sphere_radius,
                                            hull_object.local_transform
                                        ));
                                    }
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

        let scene_compound_shapes_list_size = scene_collision_shapes_list.len();

        if scene_compound_shapes_list_size >= OPTIMA_NUMBER {
            optima_version = true;
        }

        Self {
            scene_collision_shapes_list,
            scene_optima_collision_shapes_look_up,
            scene_transient_shapes_look_up,
            scene_group_truth_distance_hashmap,
            optima_version,
            scene_optima_transient_shapes_look_up,
            d_max,
            r,
            a_max,
            time_budget,
            timed,
            a_value,
        }
    }

  
    pub fn compute_ground_truth_distance_hashmap(
        &mut self,
        initial_frames: &HashMap<String, TransformInfo>,
    ) {
       if self.optima_version {
        let size = self.scene_collision_shapes_list.len();
        for i in 0..=size - 1 {
            if self.scene_collision_shapes_list.get(i).unwrap().0 == "world" {
                break;
            } else {
                let (shape1_frame, shape1, rad1,_) = self.scene_collision_shapes_list.get(i).unwrap();
                let mut proximity: ProximityInfo;
                let mut shape2_frame: String;
                let mut shape2: Compound;
                let mut rad2: f64;
                let mut valid_shape1_transform;
                let mut valid_shape2_transform;
                let mut value_vec: Vec<(
                    ProximityInfo,
                    Compound,
                    Compound,
                    f64,
                    f64,
                    Isometry3<f64>,
                    Isometry3<f64>,
                )> = vec![];

                for j in (i + 1)..=size - 1 {
                    // i : shape1 ; j : shape2
                    shape2_frame = self
                        .scene_collision_shapes_list
                        .get(j)
                        .unwrap()
                        .0
                        .to_string();
                    shape2 = self.scene_collision_shapes_list.get(j).unwrap().1.clone();
                    rad2 = self.scene_collision_shapes_list.get(j).unwrap().2;
                    
                    if shape1_frame == "world" && shape2_frame == "world" {
                        continue;
                    } else {
                        let shape1_transform = initial_frames.get(shape1_frame);
                        match shape1_transform {
                            Some(shape1_transform) => {
                                valid_shape1_transform = shape1_transform.world;
                                        valid_shape2_transform =  self.scene_collision_shapes_list.get(j).unwrap().3;
                                        match parry3d_f64::query::contact(
                                            &shape1_transform.world,
                                            shape1,
                                            &valid_shape2_transform,
                                            &shape2,
                                            self.d_max,
                                        ) {
                                            Ok(contact) => match contact {
                                                Some(valid_contact) => {
                                                    proximity = ProximityInfo::new(
                                                        shape1_frame.to_string(),
                                                        shape2_frame.to_string(),
                                                        valid_contact.dist,
                                                        Some((
                                                            valid_contact.point1,
                                                            valid_contact.point2,
                                                        )),
                                                        true,
                                                    );
                                                    value_vec.push((
                                                        proximity,
                                                        shape1.clone(),
                                                        shape2,
                                                        *rad1,
                                                        rad2,
                                                        valid_shape1_transform,
                                                        valid_shape2_transform,
                                                    ));
                                                }
                                                None => {
                                                    self.scene_optima_collision_shapes_look_up
                                                        .insert(
                                                            shape1_frame.to_string(),
                                                            (
                                                                shape1.clone(),
                                                                *rad1,
                                                                shape1_transform.world,
                                                            ),
                                                        );
                                                }
                                            },
                                            Err(_) => {
                                                self.scene_optima_collision_shapes_look_up.insert(
                                                    shape1_frame.to_string(),
                                                    (shape1.clone(), *rad1, shape1_transform.world),
                                                );
                                            }
                                        }
                                    }
                                   
                                
                            
                            None => {}
                        }
                    }
                }

                self.scene_group_truth_distance_hashmap
                    .insert(shape1_frame.to_string(), value_vec);
            
            }
        }
       
       }
       
    }
    

  
    pub fn perform_updates(&mut self, shape_updates: &Vec<ShapeUpdate>) {
        if self.optima_version {
            for update in shape_updates {
                match update {
                    ShapeUpdate::Add { id, shape } => match shape {
                        shapes::Shape::Box(box_object) => {
                            let box_collider =
                                SharedShape::cuboid(box_object.y, box_object.x, box_object.z);
                                
                            if self.scene_optima_transient_shapes_look_up.clone().contains_key(id) {
                               
                                println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                              
                                let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(box_object.local_transform.clone(), box_collider)];
                                let new_compound_shape = Compound::new(shape_vec);
                                let new_bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                match self.scene_optima_transient_shapes_look_up.get(id) {
                                    Some((frame_name, index_vec)) => {
                                        if frame_name == "world" {
                                            let mut index = 0;
                                            for (frame_name, temp_vec) in
                                                self.scene_group_truth_distance_hashmap.clone()
                                            {
                                                if temp_vec.len() == 0 {
                                                    index += 1;
                                                    continue;
                                                }
                                                match index_vec.get(index) {
                                                    Some(index_move) => match index_move {
                                                        Some(index_move) => {
                                                            match self
                                                                .scene_group_truth_distance_hashmap
                                                                .get_mut(&frame_name)
                                                                .unwrap()
                                                                .get_mut(*index_move)
                                                            {
                                                                Some(_) => {
                                                                    self.scene_group_truth_distance_hashmap
                                                                                    .get_mut(&frame_name)
                                                                                    .unwrap()
                                                                                    .get_mut(*index_move)
                                                                                    .unwrap()
                                                                                    .1 = new_compound_shape.clone();
                                                                    self.scene_group_truth_distance_hashmap
                                                                                    .get_mut(&frame_name)
                                                                                    .unwrap()
                                                                                    .get_mut(*index_move)
                                                                                    .unwrap()
                                                                                    .3 = new_bounding_sphere_radius;
                                                                    self.scene_group_truth_distance_hashmap
                                                                                    .get_mut(&frame_name)
                                                                                    .unwrap()
                                                                                    .get_mut(*index_move)
                                                                                    .unwrap()
                                                                                    .5 = box_object.local_transform;

                                                                    match parry3d_f64::query::contact(
                                                                                &temp_vec
                                                                                    .get(*index_move)
                                                                                    .unwrap()
                                                                                    .5,
                                                                                &temp_vec
                                                                                    .get(*index_move)
                                                                                    .unwrap()
                                                                                    .1,
                                                                                &temp_vec
                                                                                    .get(*index_move)
                                                                                    .unwrap()
                                                                                    .6,
                                                                                &temp_vec
                                                                                    .get(*index_move)
                                                                                    .unwrap()
                                                                                    .2,
                                                                                self.d_max,
                                                                            ) {
                                                                                Ok(contact) => match contact {
                                                                                    Some(valid_contact) => {
                                                                                        self.scene_group_truth_distance_hashmap
                                                                                        .get_mut(&frame_name)
                                                                                        .unwrap()
                                                                                        .get_mut(*index_move)
                                                                                        .unwrap()
                                                                                        .0
                                                                                        .distance = valid_contact.dist;
                                                                                        self.scene_group_truth_distance_hashmap
                                                                                        .get_mut(&frame_name)
                                                                                        .unwrap()
                                                                                        .get_mut(*index_move)
                                                                                        .unwrap()
                                                                                        .0
                                                                                        .points = Some((
                                                                                        valid_contact.point1,
                                                                                        valid_contact.point2,
                                                                                    ));
                                                                                    }
                                                                                    None => {}
                                                                                },
                                                                                Err(_) => {}
                                                                            }

                                                                    index += 1;
                                                                }
                                                                None => {
                                                                    index += 1;
                                                                    continue;
                                                                }
                                                            }
                                                        }
                                                        None => {
                                                            index += 1;
                                                            continue;
                                                        }
                                                    },
                                                    None => {
                                                        index += 1;
                                                        continue;
                                                    }
                                                }
                                            }
                                        } else {
                                            let temp_vec = self
                                                .scene_group_truth_distance_hashmap
                                                .get(frame_name)
                                                .unwrap()
                                                .clone();

                                            match self
                                                .scene_optima_transient_shapes_look_up
                                                .get(id)
                                                .unwrap()
                                                .1
                                                .get(0)
                                                .unwrap()
                                            {
                                                Some(index_change) => {
                                                    for _ in temp_vec {
                                                        self.scene_group_truth_distance_hashmap
                                                            .get_mut(frame_name)
                                                            .unwrap()
                                                            .get_mut(*index_change)
                                                            .unwrap()
                                                            .1 = new_compound_shape.clone();
                                                        self.scene_group_truth_distance_hashmap
                                                            .get_mut(frame_name)
                                                            .unwrap()
                                                            .get_mut(*index_change)
                                                            .unwrap()
                                                            .3 = new_bounding_sphere_radius;
                                                        self.scene_group_truth_distance_hashmap
                                                            .get_mut(frame_name)
                                                            .unwrap()
                                                            .get_mut(*index_change)
                                                            .unwrap()
                                                            .5 = box_object.local_transform;

                                                        match parry3d_f64::query::contact(
                                                            &self
                                                                .scene_group_truth_distance_hashmap
                                                                .get(frame_name)
                                                                .unwrap()
                                                                .get(*index_change)
                                                                .unwrap()
                                                                .5,
                                                            &new_compound_shape,
                                                            &self
                                                                .scene_group_truth_distance_hashmap
                                                                .get(frame_name)
                                                                .unwrap()
                                                                .get(*index_change)
                                                                .unwrap()
                                                                .6,
                                                            &self
                                                                .scene_group_truth_distance_hashmap
                                                                .get(frame_name)
                                                                .unwrap()
                                                                .get(*index_change)
                                                                .unwrap()
                                                                .2,
                                                            self.d_max,
                                                        ) {
                                                            Ok(contact) => match contact {
                                                                Some(valid_contact) => {
                                                                    self.scene_group_truth_distance_hashmap
                                                                        .get_mut(frame_name)
                                                                        .unwrap()
                                                                        .get_mut(*index_change)
                                                                        .unwrap()
                                                                        .0
                                                                        .distance = valid_contact.dist;
                                                                    self.scene_group_truth_distance_hashmap
                                                                        .get_mut(frame_name)
                                                                        .unwrap()
                                                                        .get_mut(*index_change)
                                                                        .unwrap()
                                                                        .0
                                                                        .points = Some((
                                                                        valid_contact.point1,
                                                                        valid_contact.point2,
                                                                    ));
                                                                }
                                                                None => {}
                                                            },
                                                            Err(_) => {}
                                                        }
                                                    }
                                                }
                                                None => {}
                                            }
                                        }
                                    }
                                    None => {
                                        //println!("the id you have provided is not valid");
                                    }
                                }
                            } else {
                                if box_object.frame == "world" {
                                    let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                        vec![(box_object.local_transform.clone(), box_collider)];
                                    let new_compound_shape = Compound::new(shape_vec);
                                    let new_bounding_sphere_radius =
                                        new_compound_shape.local_bounding_sphere().radius;
                                    let mut index_vec: Vec<Option<usize>> = vec![];
                                    for (shape_frame, shapes_vec) in
                                        self.scene_group_truth_distance_hashmap.clone()
                                    {
                                       
                                        if shapes_vec.len() == 0 {
                                            let temp_tuple = self
                                                .scene_optima_collision_shapes_look_up
                                                .get(&shape_frame);
                                            match temp_tuple {
                                                Some((shape1, rad1, trans1)) => {
                                                    match parry3d_f64::query::contact(
                                                        &trans1,
                                                        shape1,
                                                        &box_object.local_transform,
                                                        &new_compound_shape,
                                                        self.d_max,
                                                    ) {
                                                        Ok(contact) => match contact {
                                                            Some(valid_contact) => {
                                                                let proximity = ProximityInfo::new(
                                                                    shape_frame.to_string(),
                                                                    box_object.frame.to_string(),
                                                                    valid_contact.dist,
                                                                    Some((
                                                                        valid_contact.point1,
                                                                        valid_contact.point2,
                                                                    )),
                                                                    true,
                                                                );
                                                                self.scene_group_truth_distance_hashmap
                                                                    .get_mut(&shape_frame)
                                                                    .unwrap()
                                                                    .push((
                                                                        proximity,
                                                                        shape1.clone(),
                                                                        new_compound_shape.clone(),
                                                                        *rad1,
                                                                        new_bounding_sphere_radius,
                                                                        *trans1,
                                                                        box_object.local_transform,
                                                                    ));
                                                                index_vec.push(Some(
                                                                    self.scene_group_truth_distance_hashmap
                                                                        .get_mut(&shape_frame)
                                                                        .unwrap()
                                                                        .len()
                                                                        - 1)
                                                                );
                                                            }
                                                            None => {
                                                                index_vec.push(None);
                                                            }
                                                        },
                                                        Err(_) => {
                                                            index_vec.push(None);
                                                        }
                                                    }
                                                }
                                                None => {
                                                    continue;
                                                }
                                            }
                                        } else {
                                            let temp_tuple = shapes_vec.get(0).unwrap();
                                            match parry3d_f64::query::contact(
                                                &temp_tuple.5,
                                                &temp_tuple.1,
                                                &box_object.local_transform,
                                                &new_compound_shape,
                                                self.d_max,
                                            ) {
                                                Ok(contact) => match contact {
                                                    Some(valid_contact) => {
                                                        let proximity = ProximityInfo::new(
                                                            shape_frame.to_string(),
                                                            box_object.frame.to_string(),
                                                            valid_contact.dist,
                                                            Some((
                                                                valid_contact.point1,
                                                                valid_contact.point2,
                                                            )),
                                                            true,
                                                        );
                                                        self.scene_group_truth_distance_hashmap
                                                            .get_mut(&shape_frame)
                                                            .unwrap()
                                                            .push((
                                                                proximity,
                                                                temp_tuple.1.clone(),
                                                                new_compound_shape.clone(),
                                                                temp_tuple.3,
                                                                new_bounding_sphere_radius,
                                                                temp_tuple.5,
                                                                box_object.local_transform,
                                                            ));
                                                        index_vec.push(Some(
                                                            self.scene_group_truth_distance_hashmap
                                                                .get_mut(&shape_frame)
                                                                .unwrap()
                                                                .len()
                                                                - 1,
                                                        ));
                                                    }
                                                    None => {
                                                        index_vec.push(None);
                                                    }
                                                },
                                                Err(_) => {
                                                    index_vec.push(None);
                                                }
                                            }
                                        }
                                    }

                                    self.scene_optima_transient_shapes_look_up.insert(
                                        id.to_string(),
                                        (box_object.frame.to_string(), index_vec.clone()),
                                    );

                                } else {
                                    match self
                                        .scene_group_truth_distance_hashmap
                                        .get_mut(&box_object.frame)
                                    {
                                        Some(valid_vec) => {
                                            let mut shape1_vec =
                                                valid_vec.get_mut(0).unwrap().1.shapes().to_vec();
                                            shape1_vec
                                                .push((box_object.local_transform, box_collider));
                                            let new_compound_shape =
                                                Compound::new(shape1_vec.clone());
                                            let new_bounding_sphere_radius =
                                                new_compound_shape.local_bounding_sphere().radius;
                                            let mut index_vec: Vec<Option<usize>> = vec![];
                                            for i in 0..valid_vec.len() {
                                                let tuple = valid_vec.get_mut(i).unwrap();
                                                match parry3d_f64::query::contact(
                                                    &tuple.5,
                                                    &new_compound_shape,
                                                    &tuple.6,
                                                    &tuple.2,
                                                    self.d_max,
                                                ) {
                                                    Ok(contact) => match contact {
                                                        Some(valid_contact) => {
                                                            tuple.0.distance = valid_contact.dist;
                                                            tuple.0.points = Some((
                                                                valid_contact.point1,
                                                                valid_contact.point2,
                                                            ));
                                                            tuple.1 = new_compound_shape.clone();
                                                            tuple.3 = new_bounding_sphere_radius;
                                                            index_vec
                                                                .push(Some(shape1_vec.len() - 1));
                                                        }

                                                        None => {
                                                            index_vec.push(None);
                                                        }
                                                    },
                                                    Err(_) => {
                                                        index_vec.push(None);
                                                    }
                                                }
                                            }
                                            self.scene_optima_transient_shapes_look_up.insert(
                                                id.to_string(),
                                                (box_object.frame.to_string(), index_vec.clone()),
                                            );
                                        }
                                        None => {}
                                    }
                                }
                            }
                        }
                        shapes::Shape::Cylinder(cylinder_object) => {
                            let new_length = cylinder_object.length / 2.0;
                            let cylinder_collider =
                                SharedShape::cylinder(new_length, cylinder_object.radius);
                              
                                if self.scene_optima_transient_shapes_look_up.contains_key(id) {
                                   
                                    println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                   
                                    let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                        vec![(cylinder_object.local_transform.clone(), cylinder_collider)];
                                    let new_compound_shape = Compound::new(shape_vec);
                                    let new_bounding_sphere_radius =
                                        new_compound_shape.local_bounding_sphere().radius;
                                    match self.scene_optima_transient_shapes_look_up.get(id) {
                                        Some((frame_name, index_vec)) => {
                                            if frame_name == "world" {
                                                let mut index = 0;
                                                for (frame_name, temp_vec) in
                                                    self.scene_group_truth_distance_hashmap.clone()
                                                {
                                                    if temp_vec.len() == 0 {
                                                        index += 1;
                                                        continue;
                                                    }
                                                    match index_vec.get(index) {
                                                        Some(index_move) => match index_move {
                                                            Some(index_move) => {
                                                                match self
                                                                    .scene_group_truth_distance_hashmap
                                                                    .get_mut(&frame_name)
                                                                    .unwrap()
                                                                    .get_mut(*index_move)
                                                                {
                                                                    Some(_) => {
                                                                        self.scene_group_truth_distance_hashmap
                                                                                        .get_mut(&frame_name)
                                                                                        .unwrap()
                                                                                        .get_mut(*index_move)
                                                                                        .unwrap()
                                                                                        .1 = new_compound_shape.clone();
                                                                        self.scene_group_truth_distance_hashmap
                                                                                        .get_mut(&frame_name)
                                                                                        .unwrap()
                                                                                        .get_mut(*index_move)
                                                                                        .unwrap()
                                                                                        .3 = new_bounding_sphere_radius;
                                                                        self.scene_group_truth_distance_hashmap
                                                                                        .get_mut(&frame_name)
                                                                                        .unwrap()
                                                                                        .get_mut(*index_move)
                                                                                        .unwrap()
                                                                                        .5 = cylinder_object.local_transform;
    
                                                                        match parry3d_f64::query::contact(
                                                                                    &temp_vec
                                                                                        .get(*index_move)
                                                                                        .unwrap()
                                                                                        .5,
                                                                                    &temp_vec
                                                                                        .get(*index_move)
                                                                                        .unwrap()
                                                                                        .1,
                                                                                    &temp_vec
                                                                                        .get(*index_move)
                                                                                        .unwrap()
                                                                                        .6,
                                                                                    &temp_vec
                                                                                        .get(*index_move)
                                                                                        .unwrap()
                                                                                        .2,
                                                                                    self.d_max,
                                                                                ) {
                                                                                    Ok(contact) => match contact {
                                                                                        Some(valid_contact) => {
                                                                                            self.scene_group_truth_distance_hashmap
                                                                                            .get_mut(&frame_name)
                                                                                            .unwrap()
                                                                                            .get_mut(*index_move)
                                                                                            .unwrap()
                                                                                            .0
                                                                                            .distance = valid_contact.dist;
                                                                                            self.scene_group_truth_distance_hashmap
                                                                                            .get_mut(&frame_name)
                                                                                            .unwrap()
                                                                                            .get_mut(*index_move)
                                                                                            .unwrap()
                                                                                            .0
                                                                                            .points = Some((
                                                                                            valid_contact.point1,
                                                                                            valid_contact.point2,
                                                                                        ));
                                                                                        }
                                                                                        None => {}
                                                                                    },
                                                                                    Err(_) => {}
                                                                                }
    
                                                                        index += 1;
                                                                    }
                                                                    None => {
                                                                        index += 1;
                                                                        continue;
                                                                    }
                                                                }
                                                            }
                                                            None => {
                                                                index += 1;
                                                                continue;
                                                            }
                                                        },
                                                        None => {
                                                            index += 1;
                                                            continue;
                                                        }
                                                    }
                                                }
                                            } else {
                                                let temp_vec = self
                                                    .scene_group_truth_distance_hashmap
                                                    .get(frame_name)
                                                    .unwrap()
                                                    .clone();
    
                                                match self
                                                    .scene_optima_transient_shapes_look_up
                                                    .get(id)
                                                    .unwrap()
                                                    .1
                                                    .get(0)
                                                    .unwrap()
                                                {
                                                    Some(index_change) => {
                                                        for _ in temp_vec {
                                                            self.scene_group_truth_distance_hashmap
                                                                .get_mut(frame_name)
                                                                .unwrap()
                                                                .get_mut(*index_change)
                                                                .unwrap()
                                                                .1 = new_compound_shape.clone();
                                                            self.scene_group_truth_distance_hashmap
                                                                .get_mut(frame_name)
                                                                .unwrap()
                                                                .get_mut(*index_change)
                                                                .unwrap()
                                                                .3 = new_bounding_sphere_radius;
                                                            self.scene_group_truth_distance_hashmap
                                                                .get_mut(frame_name)
                                                                .unwrap()
                                                                .get_mut(*index_change)
                                                                .unwrap()
                                                                .5 = cylinder_object.local_transform;
    
                                                            match parry3d_f64::query::contact(
                                                                &self
                                                                    .scene_group_truth_distance_hashmap
                                                                    .get(frame_name)
                                                                    .unwrap()
                                                                    .get(*index_change)
                                                                    .unwrap()
                                                                    .5,
                                                                &new_compound_shape,
                                                                &self
                                                                    .scene_group_truth_distance_hashmap
                                                                    .get(frame_name)
                                                                    .unwrap()
                                                                    .get(*index_change)
                                                                    .unwrap()
                                                                    .6,
                                                                &self
                                                                    .scene_group_truth_distance_hashmap
                                                                    .get(frame_name)
                                                                    .unwrap()
                                                                    .get(*index_change)
                                                                    .unwrap()
                                                                    .2,
                                                                self.d_max,
                                                            ) {
                                                                Ok(contact) => match contact {
                                                                    Some(valid_contact) => {
                                                                        self.scene_group_truth_distance_hashmap
                                                                            .get_mut(frame_name)
                                                                            .unwrap()
                                                                            .get_mut(*index_change)
                                                                            .unwrap()
                                                                            .0
                                                                            .distance = valid_contact.dist;
                                                                        self.scene_group_truth_distance_hashmap
                                                                            .get_mut(frame_name)
                                                                            .unwrap()
                                                                            .get_mut(*index_change)
                                                                            .unwrap()
                                                                            .0
                                                                            .points = Some((
                                                                            valid_contact.point1,
                                                                            valid_contact.point2,
                                                                        ));
                                                                    }
                                                                    None => {}
                                                                },
                                                                Err(_) => {}
                                                            }
                                                        }
                                                    }
                                                    None => {}
                                                }
                                            }
                                        }
                                        None => {
                                            //println!("the id you have provided is not valid");
                                        }
                                    }
                                } else {
                                    if cylinder_object.frame == "world" {
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(cylinder_object.local_transform.clone(), cylinder_collider)];
                                        let new_compound_shape = Compound::new(shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        let mut index_vec: Vec<Option<usize>> = vec![];
                                        for (shape_frame, shapes_vec) in
                                            self.scene_group_truth_distance_hashmap.clone()
                                        {
                                            if shapes_vec.len() == 0 {
                                                let temp_tuple = self
                                                    .scene_optima_collision_shapes_look_up
                                                    .get(&shape_frame);
                                                match temp_tuple {
                                                    Some((shape1, rad1, trans1)) => {
                                                        match parry3d_f64::query::contact(
                                                            &trans1,
                                                            shape1,
                                                            &cylinder_object.local_transform,
                                                            &new_compound_shape,
                                                            self.d_max,
                                                        ) {
                                                            Ok(contact) => match contact {
                                                                Some(valid_contact) => {
                                                                    let proximity = ProximityInfo::new(
                                                                        shape_frame.to_string(),
                                                                        cylinder_object.frame.to_string(),
                                                                        valid_contact.dist,
                                                                        Some((
                                                                            valid_contact.point1,
                                                                            valid_contact.point2,
                                                                        )),
                                                                        true,
                                                                    );
                                                                    self.scene_group_truth_distance_hashmap
                                                                        .get_mut(&shape_frame)
                                                                        .unwrap()
                                                                        .push((
                                                                            proximity,
                                                                            shape1.clone(),
                                                                            new_compound_shape.clone(),
                                                                            *rad1,
                                                                            new_bounding_sphere_radius,
                                                                            *trans1,
                                                                            cylinder_object.local_transform,
                                                                        ));
                                                                    index_vec.push(Some(
                                                                        self.scene_group_truth_distance_hashmap
                                                                            .get_mut(&shape_frame)
                                                                            .unwrap()
                                                                            .len()
                                                                            - 1)
                                                                    );
                                                                }
                                                                None => {
                                                                    index_vec.push(None);
                                                                }
                                                            },
                                                            Err(_) => {
                                                                index_vec.push(None);
                                                            }
                                                        }
                                                    }
                                                    None => {
                                                        continue;
                                                    }
                                                }
                                            } else {
                                                let temp_tuple = shapes_vec.get(0).unwrap();
                                                match parry3d_f64::query::contact(
                                                    &temp_tuple.5,
                                                    &temp_tuple.1,
                                                    &cylinder_object.local_transform,
                                                    &new_compound_shape,
                                                    self.d_max,
                                                ) {
                                                    Ok(contact) => match contact {
                                                        Some(valid_contact) => {
                                                            let proximity = ProximityInfo::new(
                                                                shape_frame.to_string(),
                                                                cylinder_object.frame.to_string(),
                                                                valid_contact.dist,
                                                                Some((
                                                                    valid_contact.point1,
                                                                    valid_contact.point2,
                                                                )),
                                                                true,
                                                            );
                                                            self.scene_group_truth_distance_hashmap
                                                                .get_mut(&shape_frame)
                                                                .unwrap()
                                                                .push((
                                                                    proximity,
                                                                    temp_tuple.1.clone(),
                                                                    new_compound_shape.clone(),
                                                                    temp_tuple.3,
                                                                    new_bounding_sphere_radius,
                                                                    temp_tuple.5,
                                                                    cylinder_object.local_transform,
                                                                ));
                                                            index_vec.push(Some(
                                                                self.scene_group_truth_distance_hashmap
                                                                    .get_mut(&shape_frame)
                                                                    .unwrap()
                                                                    .len()
                                                                    - 1,
                                                            ));
                                                        }
                                                        None => {
                                                            index_vec.push(None);
                                                        }
                                                    },
                                                    Err(_) => {
                                                        index_vec.push(None);
                                                    }
                                                }
                                            }
                                           
                                        }
                                        self.scene_optima_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (cylinder_object.frame.to_string(), index_vec),
                                        );
                                      
                                    } else {
                                        match self
                                            .scene_group_truth_distance_hashmap
                                            .get_mut(&cylinder_object.frame)
                                        {
                                            Some(valid_vec) => {
                                                let mut shape1_vec =
                                                    valid_vec.get_mut(0).unwrap().1.shapes().to_vec();
                                                shape1_vec
                                                    .push((cylinder_object.local_transform, cylinder_collider));
                                                let new_compound_shape =
                                                    Compound::new(shape1_vec.clone());
                                                let new_bounding_sphere_radius =
                                                    new_compound_shape.local_bounding_sphere().radius;
                                                let mut index_vec: Vec<Option<usize>> = vec![];
                                                for i in 0..valid_vec.len() {
                                                    let tuple = valid_vec.get_mut(i).unwrap();
                                                    match parry3d_f64::query::contact(
                                                        &tuple.5,
                                                        &new_compound_shape,
                                                        &tuple.6,
                                                        &tuple.2,
                                                        self.d_max,
                                                    ) {
                                                        Ok(contact) => match contact {
                                                            Some(valid_contact) => {
                                                                tuple.0.distance = valid_contact.dist;
                                                                tuple.0.points = Some((
                                                                    valid_contact.point1,
                                                                    valid_contact.point2,
                                                                ));
                                                                tuple.1 = new_compound_shape.clone();
                                                                tuple.3 = new_bounding_sphere_radius;
                                                                index_vec
                                                                    .push(Some(shape1_vec.len() - 1));
                                                            }
    
                                                            None => {
                                                                index_vec.push(None);
                                                            }
                                                        },
                                                        Err(_) => {
                                                            index_vec.push(None);
                                                        }
                                                    }
                                                }
                                                self.scene_optima_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (cylinder_object.frame.to_string(), index_vec),
                                                );

                                               
                                            }
                                            None => {}
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
                              
                                if self.scene_optima_transient_shapes_look_up.contains_key(id) {
                                   
                                    println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                    
                                    let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                        vec![(capsule_object.local_transform.clone(), capsule_collider)];
                                    let new_compound_shape = Compound::new(shape_vec);
                                    let new_bounding_sphere_radius =
                                        new_compound_shape.local_bounding_sphere().radius;
                                    match self.scene_optima_transient_shapes_look_up.get(id) {
                                        Some((frame_name, index_vec)) => {
                                            if frame_name == "world" {
                                                let mut index = 0;
                                                for (frame_name, temp_vec) in
                                                    self.scene_group_truth_distance_hashmap.clone()
                                                {
                                                    if temp_vec.len() == 0 {
                                                        index += 1;
                                                        continue;
                                                    }
                                                    match index_vec.get(index) {
                                                        Some(index_move) => match index_move {
                                                            Some(index_move) => {
                                                                match self
                                                                    .scene_group_truth_distance_hashmap
                                                                    .get_mut(&frame_name)
                                                                    .unwrap()
                                                                    .get_mut(*index_move)
                                                                {
                                                                    Some(_) => {
                                                                        self.scene_group_truth_distance_hashmap
                                                                                        .get_mut(&frame_name)
                                                                                        .unwrap()
                                                                                        .get_mut(*index_move)
                                                                                        .unwrap()
                                                                                        .1 = new_compound_shape.clone();
                                                                        self.scene_group_truth_distance_hashmap
                                                                                        .get_mut(&frame_name)
                                                                                        .unwrap()
                                                                                        .get_mut(*index_move)
                                                                                        .unwrap()
                                                                                        .3 = new_bounding_sphere_radius;
                                                                        self.scene_group_truth_distance_hashmap
                                                                                        .get_mut(&frame_name)
                                                                                        .unwrap()
                                                                                        .get_mut(*index_move)
                                                                                        .unwrap()
                                                                                        .5 = capsule_object.local_transform;
    
                                                                        match parry3d_f64::query::contact(
                                                                                    &temp_vec
                                                                                        .get(*index_move)
                                                                                        .unwrap()
                                                                                        .5,
                                                                                    &temp_vec
                                                                                        .get(*index_move)
                                                                                        .unwrap()
                                                                                        .1,
                                                                                    &temp_vec
                                                                                        .get(*index_move)
                                                                                        .unwrap()
                                                                                        .6,
                                                                                    &temp_vec
                                                                                        .get(*index_move)
                                                                                        .unwrap()
                                                                                        .2,
                                                                                        self.d_max,
                                                                                ) {
                                                                                    Ok(contact) => match contact {
                                                                                        Some(valid_contact) => {
                                                                                            self.scene_group_truth_distance_hashmap
                                                                                            .get_mut(&frame_name)
                                                                                            .unwrap()
                                                                                            .get_mut(*index_move)
                                                                                            .unwrap()
                                                                                            .0
                                                                                            .distance = valid_contact.dist;
                                                                                            self.scene_group_truth_distance_hashmap
                                                                                            .get_mut(&frame_name)
                                                                                            .unwrap()
                                                                                            .get_mut(*index_move)
                                                                                            .unwrap()
                                                                                            .0
                                                                                            .points = Some((
                                                                                            valid_contact.point1,
                                                                                            valid_contact.point2,
                                                                                        ));
                                                                                        }
                                                                                        None => {}
                                                                                    },
                                                                                    Err(_) => {}
                                                                                }
    
                                                                        index += 1;
                                                                    }
                                                                    None => {
                                                                        index += 1;
                                                                        continue;
                                                                    }
                                                                }
                                                            }
                                                            None => {
                                                                index += 1;
                                                                continue;
                                                            }
                                                        },
                                                        None => {
                                                            index += 1;
                                                            continue;
                                                        }
                                                    }
                                                }
                                            } else {
                                                let temp_vec = self
                                                    .scene_group_truth_distance_hashmap
                                                    .get(frame_name)
                                                    .unwrap()
                                                    .clone();
    
                                                match self
                                                    .scene_optima_transient_shapes_look_up
                                                    .get(id)
                                                    .unwrap()
                                                    .1
                                                    .get(0)
                                                    .unwrap()
                                                {
                                                    Some(index_change) => {
                                                        for _ in temp_vec {
                                                            self.scene_group_truth_distance_hashmap
                                                                .get_mut(frame_name)
                                                                .unwrap()
                                                                .get_mut(*index_change)
                                                                .unwrap()
                                                                .1 = new_compound_shape.clone();
                                                            self.scene_group_truth_distance_hashmap
                                                                .get_mut(frame_name)
                                                                .unwrap()
                                                                .get_mut(*index_change)
                                                                .unwrap()
                                                                .3 = new_bounding_sphere_radius;
                                                            self.scene_group_truth_distance_hashmap
                                                                .get_mut(frame_name)
                                                                .unwrap()
                                                                .get_mut(*index_change)
                                                                .unwrap()
                                                                .5 = capsule_object.local_transform;
    
                                                            match parry3d_f64::query::contact(
                                                                &self
                                                                    .scene_group_truth_distance_hashmap
                                                                    .get(frame_name)
                                                                    .unwrap()
                                                                    .get(*index_change)
                                                                    .unwrap()
                                                                    .5,
                                                                &new_compound_shape,
                                                                &self
                                                                    .scene_group_truth_distance_hashmap
                                                                    .get(frame_name)
                                                                    .unwrap()
                                                                    .get(*index_change)
                                                                    .unwrap()
                                                                    .6,
                                                                &self
                                                                    .scene_group_truth_distance_hashmap
                                                                    .get(frame_name)
                                                                    .unwrap()
                                                                    .get(*index_change)
                                                                    .unwrap()
                                                                    .2,
                                                                    self.d_max,
                                                            ) {
                                                                Ok(contact) => match contact {
                                                                    Some(valid_contact) => {
                                                                        self.scene_group_truth_distance_hashmap
                                                                            .get_mut(frame_name)
                                                                            .unwrap()
                                                                            .get_mut(*index_change)
                                                                            .unwrap()
                                                                            .0
                                                                            .distance = valid_contact.dist;
                                                                        self.scene_group_truth_distance_hashmap
                                                                            .get_mut(frame_name)
                                                                            .unwrap()
                                                                            .get_mut(*index_change)
                                                                            .unwrap()
                                                                            .0
                                                                            .points = Some((
                                                                            valid_contact.point1,
                                                                            valid_contact.point2,
                                                                        ));
                                                                    }
                                                                    None => {}
                                                                },
                                                                Err(_) => {}
                                                            }
                                                        }
                                                    }
                                                    None => {}
                                                }
                                            }
                                        }
                                        None => {
                                            //println!("the id you have provided is not valid");
                                        }
                                    }
                                } else {
                                    if capsule_object.frame == "world" {
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(capsule_object.local_transform.clone(), capsule_collider)];
                                        let new_compound_shape = Compound::new(shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        let mut index_vec: Vec<Option<usize>> = vec![];
                                        for (shape_frame, shapes_vec) in
                                            self.scene_group_truth_distance_hashmap.clone()
                                        {
                                            if shapes_vec.len() == 0 {
                                                let temp_tuple = self
                                                    .scene_optima_collision_shapes_look_up
                                                    .get(&shape_frame);
                                                match temp_tuple {
                                                    Some((shape1, rad1, trans1)) => {
                                                        match parry3d_f64::query::contact(
                                                            &trans1,
                                                            shape1,
                                                            &capsule_object.local_transform,
                                                            &new_compound_shape,
                                                            self.d_max,
                                                        ) {
                                                            Ok(contact) => match contact {
                                                                Some(valid_contact) => {
                                                                    let proximity = ProximityInfo::new(
                                                                        shape_frame.to_string(),
                                                                        capsule_object.frame.to_string(),
                                                                        valid_contact.dist,
                                                                        Some((
                                                                            valid_contact.point1,
                                                                            valid_contact.point2,
                                                                        )),
                                                                        true,
                                                                    );
                                                                    self.scene_group_truth_distance_hashmap
                                                                        .get_mut(&shape_frame)
                                                                        .unwrap()
                                                                        .push((
                                                                            proximity,
                                                                            shape1.clone(),
                                                                            new_compound_shape.clone(),
                                                                            *rad1,
                                                                            new_bounding_sphere_radius,
                                                                            *trans1,
                                                                            capsule_object.local_transform,
                                                                        ));
                                                                    index_vec.push(Some(
                                                                        self.scene_group_truth_distance_hashmap
                                                                            .get_mut(&shape_frame)
                                                                            .unwrap()
                                                                            .len()
                                                                            - 1)
                                                                    );
                                                                }
                                                                None => {
                                                                    index_vec.push(None);
                                                                }
                                                            },
                                                            Err(_) => {
                                                                index_vec.push(None);
                                                            }
                                                        }
                                                    }
                                                    None => {
                                                        continue;
                                                    }
                                                }
                                            } else {
                                                let temp_tuple = shapes_vec.get(0).unwrap();
                                                match parry3d_f64::query::contact(
                                                    &temp_tuple.5,
                                                    &temp_tuple.1,
                                                    &capsule_object.local_transform,
                                                    &new_compound_shape,
                                                    self.d_max,
                                                ) {
                                                    Ok(contact) => match contact {
                                                        Some(valid_contact) => {
                                                            let proximity = ProximityInfo::new(
                                                                shape_frame.to_string(),
                                                                capsule_object.frame.to_string(),
                                                                valid_contact.dist,
                                                                Some((
                                                                    valid_contact.point1,
                                                                    valid_contact.point2,
                                                                )),
                                                                true,
                                                            );
                                                            self.scene_group_truth_distance_hashmap
                                                                .get_mut(&shape_frame)
                                                                .unwrap()
                                                                .push((
                                                                    proximity,
                                                                    temp_tuple.1.clone(),
                                                                    new_compound_shape.clone(),
                                                                    temp_tuple.3,
                                                                    new_bounding_sphere_radius,
                                                                    temp_tuple.5,
                                                                    capsule_object.local_transform,
                                                                ));
                                                            index_vec.push(Some(
                                                                self.scene_group_truth_distance_hashmap
                                                                    .get_mut(&shape_frame)
                                                                    .unwrap()
                                                                    .len()
                                                                    - 1,
                                                            ));
                                                        }
                                                        None => {
                                                            index_vec.push(None);
                                                        }
                                                    },
                                                    Err(_) => {
                                                        index_vec.push(None);
                                                    }
                                                }
                                            }
                                          
                                        }
                                        self.scene_optima_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (capsule_object.frame.to_string(), index_vec),
                                        );
                                       
                                    } else {
                                        match self
                                            .scene_group_truth_distance_hashmap
                                            .get_mut(&capsule_object.frame)
                                        {
                                            Some(valid_vec) => {
                                                let mut shape1_vec =
                                                    valid_vec.get_mut(0).unwrap().1.shapes().to_vec();
                                                shape1_vec
                                                    .push((capsule_object.local_transform, capsule_collider));
                                                let new_compound_shape =
                                                    Compound::new(shape1_vec.clone());
                                                let new_bounding_sphere_radius =
                                                    new_compound_shape.local_bounding_sphere().radius;
                                                let mut index_vec: Vec<Option<usize>> = vec![];
                                                for i in 0..valid_vec.len() {
                                                    let tuple = valid_vec.get_mut(i).unwrap();
                                                    match parry3d_f64::query::contact(
                                                        &tuple.5,
                                                        &new_compound_shape,
                                                        &tuple.6,
                                                        &tuple.2,
                                                        self.d_max,
                                                    ) {
                                                        Ok(contact) => match contact {
                                                            Some(valid_contact) => {
                                                                tuple.0.distance = valid_contact.dist;
                                                                tuple.0.points = Some((
                                                                    valid_contact.point1,
                                                                    valid_contact.point2,
                                                                ));
                                                                tuple.1 = new_compound_shape.clone();
                                                                tuple.3 = new_bounding_sphere_radius;
                                                                index_vec
                                                                    .push(Some(shape1_vec.len() - 1));
                                                            }
    
                                                            None => {
                                                                index_vec.push(None);
                                                            }
                                                        },
                                                        Err(_) => {
                                                            index_vec.push(None);
                                                        }
                                                    }
                                                }
                                                self.scene_optima_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (capsule_object.frame.to_string(), index_vec),
                                                );
                                              
                                            }
                                            None => {}
                                        }
                                    }
                                }
                        }
                        shapes::Shape::Sphere(sphere_object) => {
                            let sphere_collider = SharedShape::ball(sphere_object.radius);
                           
                            if self.scene_optima_transient_shapes_look_up.contains_key(id) {
                               
                                println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                
                                let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                    vec![(sphere_object.local_transform.clone(), sphere_collider)];
                                let new_compound_shape = Compound::new(shape_vec);
                                let new_bounding_sphere_radius =
                                    new_compound_shape.local_bounding_sphere().radius;
                                match self.scene_optima_transient_shapes_look_up.get(id) {
                                    Some((frame_name, index_vec)) => {
                                        if frame_name == "world" {
                                            let mut index = 0;
                                            for (frame_name, temp_vec) in
                                                self.scene_group_truth_distance_hashmap.clone()
                                            {
                                                if temp_vec.len() == 0 {
                                                    index += 1;
                                                    continue;
                                                }
                                                match index_vec.get(index) {
                                                    Some(index_move) => match index_move {
                                                        Some(index_move) => {
                                                            match self
                                                                .scene_group_truth_distance_hashmap
                                                                .get_mut(&frame_name)
                                                                .unwrap()
                                                                .get_mut(*index_move)
                                                            {
                                                                Some(_) => {
                                                                    self.scene_group_truth_distance_hashmap
                                                                                    .get_mut(&frame_name)
                                                                                    .unwrap()
                                                                                    .get_mut(*index_move)
                                                                                    .unwrap()
                                                                                    .1 = new_compound_shape.clone();
                                                                    self.scene_group_truth_distance_hashmap
                                                                                    .get_mut(&frame_name)
                                                                                    .unwrap()
                                                                                    .get_mut(*index_move)
                                                                                    .unwrap()
                                                                                    .3 = new_bounding_sphere_radius;
                                                                    self.scene_group_truth_distance_hashmap
                                                                                    .get_mut(&frame_name)
                                                                                    .unwrap()
                                                                                    .get_mut(*index_move)
                                                                                    .unwrap()
                                                                                    .5 = sphere_object.local_transform;

                                                                    match parry3d_f64::query::contact(
                                                                                &temp_vec
                                                                                    .get(*index_move)
                                                                                    .unwrap()
                                                                                    .5,
                                                                                &temp_vec
                                                                                    .get(*index_move)
                                                                                    .unwrap()
                                                                                    .1,
                                                                                &temp_vec
                                                                                    .get(*index_move)
                                                                                    .unwrap()
                                                                                    .6,
                                                                                &temp_vec
                                                                                    .get(*index_move)
                                                                                    .unwrap()
                                                                                    .2,
                                                                                    self.d_max,
                                                                            ) {
                                                                                Ok(contact) => match contact {
                                                                                    Some(valid_contact) => {
                                                                                        self.scene_group_truth_distance_hashmap
                                                                                        .get_mut(&frame_name)
                                                                                        .unwrap()
                                                                                        .get_mut(*index_move)
                                                                                        .unwrap()
                                                                                        .0
                                                                                        .distance = valid_contact.dist;
                                                                                        self.scene_group_truth_distance_hashmap
                                                                                        .get_mut(&frame_name)
                                                                                        .unwrap()
                                                                                        .get_mut(*index_move)
                                                                                        .unwrap()
                                                                                        .0
                                                                                        .points = Some((
                                                                                        valid_contact.point1,
                                                                                        valid_contact.point2,
                                                                                    ));
                                                                                    }
                                                                                    None => {}
                                                                                },
                                                                                Err(_) => {}
                                                                            }

                                                                    index += 1;
                                                                }
                                                                None => {
                                                                    index += 1;
                                                                    continue;
                                                                }
                                                            }
                                                        }
                                                        None => {
                                                            index += 1;
                                                            continue;
                                                        }
                                                    },
                                                    None => {
                                                        index += 1;
                                                        continue;
                                                    }
                                                }
                                            }
                                        } else {
                                            let temp_vec = self
                                                .scene_group_truth_distance_hashmap
                                                .get(frame_name)
                                                .unwrap()
                                                .clone();

                                            match self
                                                .scene_optima_transient_shapes_look_up
                                                .get(id)
                                                .unwrap()
                                                .1
                                                .get(0)
                                                .unwrap()
                                            {
                                                Some(index_change) => {
                                                    for _ in temp_vec {
                                                        self.scene_group_truth_distance_hashmap
                                                            .get_mut(frame_name)
                                                            .unwrap()
                                                            .get_mut(*index_change)
                                                            .unwrap()
                                                            .1 = new_compound_shape.clone();
                                                        self.scene_group_truth_distance_hashmap
                                                            .get_mut(frame_name)
                                                            .unwrap()
                                                            .get_mut(*index_change)
                                                            .unwrap()
                                                            .3 = new_bounding_sphere_radius;
                                                        self.scene_group_truth_distance_hashmap
                                                            .get_mut(frame_name)
                                                            .unwrap()
                                                            .get_mut(*index_change)
                                                            .unwrap()
                                                            .5 = sphere_object.local_transform;

                                                        match parry3d_f64::query::contact(
                                                            &self
                                                                .scene_group_truth_distance_hashmap
                                                                .get(frame_name)
                                                                .unwrap()
                                                                .get(*index_change)
                                                                .unwrap()
                                                                .5,
                                                            &new_compound_shape,
                                                            &self
                                                                .scene_group_truth_distance_hashmap
                                                                .get(frame_name)
                                                                .unwrap()
                                                                .get(*index_change)
                                                                .unwrap()
                                                                .6,
                                                            &self
                                                                .scene_group_truth_distance_hashmap
                                                                .get(frame_name)
                                                                .unwrap()
                                                                .get(*index_change)
                                                                .unwrap()
                                                                .2,
                                                                self.d_max,
                                                        ) {
                                                            Ok(contact) => match contact {
                                                                Some(valid_contact) => {
                                                                    self.scene_group_truth_distance_hashmap
                                                                        .get_mut(frame_name)
                                                                        .unwrap()
                                                                        .get_mut(*index_change)
                                                                        .unwrap()
                                                                        .0
                                                                        .distance = valid_contact.dist;
                                                                    self.scene_group_truth_distance_hashmap
                                                                        .get_mut(frame_name)
                                                                        .unwrap()
                                                                        .get_mut(*index_change)
                                                                        .unwrap()
                                                                        .0
                                                                        .points = Some((
                                                                        valid_contact.point1,
                                                                        valid_contact.point2,
                                                                    ));
                                                                }
                                                                None => {}
                                                            },
                                                            Err(_) => {}
                                                        }
                                                    }
                                                }
                                                None => {}
                                            }
                                        }
                                    }
                                    None => {
                                       
                                    }
                                }
                            } else {
                                if sphere_object.frame == "world" {
                                    let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                        vec![(sphere_object.local_transform.clone(), sphere_collider)];
                                    let new_compound_shape = Compound::new(shape_vec);
                                    let new_bounding_sphere_radius =
                                        new_compound_shape.local_bounding_sphere().radius;
                                    let mut index_vec: Vec<Option<usize>> = vec![];
                                    for (shape_frame, shapes_vec) in
                                        self.scene_group_truth_distance_hashmap.clone()
                                    {
                                        if shapes_vec.len() == 0 {
                                            let temp_tuple = self
                                                .scene_optima_collision_shapes_look_up
                                                .get(&shape_frame);
                                            match temp_tuple {
                                                Some((shape1, rad1, trans1)) => {
                                                    match parry3d_f64::query::contact(
                                                        &trans1,
                                                        shape1,
                                                        &sphere_object.local_transform,
                                                        &new_compound_shape,
                                                        self.d_max,
                                                    ) {
                                                        Ok(contact) => match contact {
                                                            Some(valid_contact) => {
                                                                let proximity = ProximityInfo::new(
                                                                    shape_frame.to_string(),
                                                                    sphere_object.frame.to_string(),
                                                                    valid_contact.dist,
                                                                    Some((
                                                                        valid_contact.point1,
                                                                        valid_contact.point2,
                                                                    )),
                                                                    true,
                                                                );
                                                                self.scene_group_truth_distance_hashmap
                                                                    .get_mut(&shape_frame)
                                                                    .unwrap()
                                                                    .push((
                                                                        proximity,
                                                                        shape1.clone(),
                                                                        new_compound_shape.clone(),
                                                                        *rad1,
                                                                        new_bounding_sphere_radius,
                                                                        *trans1,
                                                                        sphere_object.local_transform,
                                                                    ));
                                                                index_vec.push(Some(
                                                                    self.scene_group_truth_distance_hashmap
                                                                        .get_mut(&shape_frame)
                                                                        .unwrap()
                                                                        .len()
                                                                        - 1)
                                                                );
                                                            }
                                                            None => {
                                                                index_vec.push(None);
                                                            }
                                                        },
                                                        Err(_) => {
                                                            index_vec.push(None);
                                                        }
                                                    }
                                                }
                                                None => {
                                                    continue;
                                                }
                                            }
                                        } else {
                                            let temp_tuple = shapes_vec.get(0).unwrap();
                                            match parry3d_f64::query::contact(
                                                &temp_tuple.5,
                                                &temp_tuple.1,
                                                &sphere_object.local_transform,
                                                &new_compound_shape,
                                                self.d_max,
                                            ) {
                                                Ok(contact) => match contact {
                                                    Some(valid_contact) => {
                                                        let proximity = ProximityInfo::new(
                                                            shape_frame.to_string(),
                                                            sphere_object.frame.to_string(),
                                                            valid_contact.dist,
                                                            Some((
                                                                valid_contact.point1,
                                                                valid_contact.point2,
                                                            )),
                                                            true,
                                                        );
                                                        self.scene_group_truth_distance_hashmap
                                                            .get_mut(&shape_frame)
                                                            .unwrap()
                                                            .push((
                                                                proximity,
                                                                temp_tuple.1.clone(),
                                                                new_compound_shape.clone(),
                                                                temp_tuple.3,
                                                                new_bounding_sphere_radius,
                                                                temp_tuple.5,
                                                                sphere_object.local_transform,
                                                            ));
                                                        index_vec.push(Some(
                                                            self.scene_group_truth_distance_hashmap
                                                                .get_mut(&shape_frame)
                                                                .unwrap()
                                                                .len()
                                                                - 1,
                                                        ));
                                                    }
                                                    None => {
                                                        index_vec.push(None);
                                                    }
                                                },
                                                Err(_) => {
                                                    index_vec.push(None);
                                                }
                                            }
                                        }
                                     
                                    }
                                    self.scene_optima_transient_shapes_look_up.insert(
                                        id.to_string(),
                                        (sphere_object.frame.to_string(), index_vec),
                                    );
                                   
                                   
                                   
                                           
                                    
                                 
                                } else {
                                    match self
                                        .scene_group_truth_distance_hashmap
                                        .get_mut(&sphere_object.frame)
                                    {
                                        Some(valid_vec) => {
                                            let mut shape1_vec =
                                                valid_vec.get_mut(0).unwrap().1.shapes().to_vec();
                                            shape1_vec
                                                .push((sphere_object.local_transform, sphere_collider));
                                            let new_compound_shape =
                                                Compound::new(shape1_vec.clone());
                                            let new_bounding_sphere_radius =
                                                new_compound_shape.local_bounding_sphere().radius;
                                            let mut index_vec: Vec<Option<usize>> = vec![];
                                            for i in 0..valid_vec.len() {
                                                let tuple = valid_vec.get_mut(i).unwrap();
                                                match parry3d_f64::query::contact(
                                                    &tuple.5,
                                                    &new_compound_shape,
                                                    &tuple.6,
                                                    &tuple.2,
                                                    self.d_max,
                                                ) {
                                                    Ok(contact) => match contact {
                                                        Some(valid_contact) => {
                                                            tuple.0.distance = valid_contact.dist;
                                                            tuple.0.points = Some((
                                                                valid_contact.point1,
                                                                valid_contact.point2,
                                                            ));
                                                            tuple.1 = new_compound_shape.clone();
                                                            tuple.3 = new_bounding_sphere_radius;
                                                            index_vec
                                                                .push(Some(shape1_vec.len() - 1));
                                                        }

                                                        None => {
                                                            index_vec.push(None);
                                                        }
                                                    },
                                                    Err(_) => {
                                                        index_vec.push(None);
                                                    }
                                                }
                                            }
                                            self.scene_optima_transient_shapes_look_up.insert(
                                                id.to_string(),
                                                (sphere_object.frame.to_string(), index_vec),
                                            );
                                           
                                        }
                                        None => {}
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
                            match SharedShape::convex_hull(hull_points.as_slice()) {
                                Some(hull_collider) => {
                                    if self.scene_optima_transient_shapes_look_up.contains_key(id) {
                                      
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(hull_object.local_transform.clone(), hull_collider)];
                                        let new_compound_shape = Compound::new(shape_vec);
                                        let new_bounding_sphere_radius =
                                            new_compound_shape.local_bounding_sphere().radius;
                                        match self.scene_optima_transient_shapes_look_up.get(id) {
                                            Some((frame_name, index_vec)) => {
                                                if frame_name == "world" {
                                                    let mut index = 0;
                                                    for (frame_name, temp_vec) in
                                                        self.scene_group_truth_distance_hashmap.clone()
                                                    {
                                                        if temp_vec.len() == 0 {
                                                            index += 1;
                                                            continue;
                                                        }
                                                        match index_vec.get(index) {
                                                            Some(index_move) => match index_move {
                                                                Some(index_move) => {
                                                                    match self
                                                                        .scene_group_truth_distance_hashmap
                                                                        .get_mut(&frame_name)
                                                                        .unwrap()
                                                                        .get_mut(*index_move)
                                                                    {
                                                                        Some(_) => {
                                                                            self.scene_group_truth_distance_hashmap
                                                                                            .get_mut(&frame_name)
                                                                                            .unwrap()
                                                                                            .get_mut(*index_move)
                                                                                            .unwrap()
                                                                                            .1 = new_compound_shape.clone();
                                                                            self.scene_group_truth_distance_hashmap
                                                                                            .get_mut(&frame_name)
                                                                                            .unwrap()
                                                                                            .get_mut(*index_move)
                                                                                            .unwrap()
                                                                                            .3 = new_bounding_sphere_radius;
                                                                            self.scene_group_truth_distance_hashmap
                                                                                            .get_mut(&frame_name)
                                                                                            .unwrap()
                                                                                            .get_mut(*index_move)
                                                                                            .unwrap()
                                                                                            .5 = hull_object.local_transform;
        
                                                                            match parry3d_f64::query::contact(
                                                                                        &temp_vec
                                                                                            .get(*index_move)
                                                                                            .unwrap()
                                                                                            .5,
                                                                                        &temp_vec
                                                                                            .get(*index_move)
                                                                                            .unwrap()
                                                                                            .1,
                                                                                        &temp_vec
                                                                                            .get(*index_move)
                                                                                            .unwrap()
                                                                                            .6,
                                                                                        &temp_vec
                                                                                            .get(*index_move)
                                                                                            .unwrap()
                                                                                            .2,
                                                                                            self.d_max,
                                                                                    ) {
                                                                                        Ok(contact) => match contact {
                                                                                            Some(valid_contact) => {
                                                                                                self.scene_group_truth_distance_hashmap
                                                                                                .get_mut(&frame_name)
                                                                                                .unwrap()
                                                                                                .get_mut(*index_move)
                                                                                                .unwrap()
                                                                                                .0
                                                                                                .distance = valid_contact.dist;
                                                                                                self.scene_group_truth_distance_hashmap
                                                                                                .get_mut(&frame_name)
                                                                                                .unwrap()
                                                                                                .get_mut(*index_move)
                                                                                                .unwrap()
                                                                                                .0
                                                                                                .points = Some((
                                                                                                valid_contact.point1,
                                                                                                valid_contact.point2,
                                                                                            ));
                                                                                            }
                                                                                            None => {}
                                                                                        },
                                                                                        Err(_) => {}
                                                                                    }
        
                                                                            index += 1;
                                                                        }
                                                                        None => {
                                                                            index += 1;
                                                                            continue;
                                                                        }
                                                                    }
                                                                }
                                                                None => {
                                                                    index += 1;
                                                                    continue;
                                                                }
                                                            },
                                                            None => {
                                                                index += 1;
                                                                continue;
                                                            }
                                                        }
                                                    }
                                                } else {
                                                    let temp_vec = self
                                                        .scene_group_truth_distance_hashmap
                                                        .get(frame_name)
                                                        .unwrap()
                                                        .clone();
        
                                                    match self
                                                        .scene_optima_transient_shapes_look_up
                                                        .get(id)
                                                        .unwrap()
                                                        .1
                                                        .get(0)
                                                        .unwrap()
                                                    {
                                                        Some(index_change) => {
                                                            for _ in temp_vec {
                                                                self.scene_group_truth_distance_hashmap
                                                                    .get_mut(frame_name)
                                                                    .unwrap()
                                                                    .get_mut(*index_change)
                                                                    .unwrap()
                                                                    .1 = new_compound_shape.clone();
                                                                self.scene_group_truth_distance_hashmap
                                                                    .get_mut(frame_name)
                                                                    .unwrap()
                                                                    .get_mut(*index_change)
                                                                    .unwrap()
                                                                    .3 = new_bounding_sphere_radius;
                                                                self.scene_group_truth_distance_hashmap
                                                                    .get_mut(frame_name)
                                                                    .unwrap()
                                                                    .get_mut(*index_change)
                                                                    .unwrap()
                                                                    .5 = hull_object.local_transform;
        
                                                                match parry3d_f64::query::contact(
                                                                    &self
                                                                        .scene_group_truth_distance_hashmap
                                                                        .get(frame_name)
                                                                        .unwrap()
                                                                        .get(*index_change)
                                                                        .unwrap()
                                                                        .5,
                                                                    &new_compound_shape,
                                                                    &self
                                                                        .scene_group_truth_distance_hashmap
                                                                        .get(frame_name)
                                                                        .unwrap()
                                                                        .get(*index_change)
                                                                        .unwrap()
                                                                        .6,
                                                                    &self
                                                                        .scene_group_truth_distance_hashmap
                                                                        .get(frame_name)
                                                                        .unwrap()
                                                                        .get(*index_change)
                                                                        .unwrap()
                                                                        .2,
                                                                        self.d_max,
                                                                ) {
                                                                    Ok(contact) => match contact {
                                                                        Some(valid_contact) => {
                                                                            self.scene_group_truth_distance_hashmap
                                                                                .get_mut(frame_name)
                                                                                .unwrap()
                                                                                .get_mut(*index_change)
                                                                                .unwrap()
                                                                                .0
                                                                                .distance = valid_contact.dist;
                                                                            self.scene_group_truth_distance_hashmap
                                                                                .get_mut(frame_name)
                                                                                .unwrap()
                                                                                .get_mut(*index_change)
                                                                                .unwrap()
                                                                                .0
                                                                                .points = Some((
                                                                                valid_contact.point1,
                                                                                valid_contact.point2,
                                                                            ));
                                                                        }
                                                                        None => {}
                                                                    },
                                                                    Err(_) => {}
                                                                }
                                                            }
                                                        }
                                                        None => {}
                                                    }
                                                }
                                            }
                                            None => {
                                                //println!("the id you have provided is not valid");
                                            }
                                        }
                                    } else {
                                        if hull_object.frame == "world" {
                                            let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                                vec![(hull_object.local_transform.clone(), hull_collider)];
                                            let new_compound_shape = Compound::new(shape_vec);
                                            let new_bounding_sphere_radius =
                                                new_compound_shape.local_bounding_sphere().radius;
                                            let mut index_vec: Vec<Option<usize>> = vec![];
                                            for (shape_frame, shapes_vec) in
                                                self.scene_group_truth_distance_hashmap.clone()
                                            {
                                                if shapes_vec.len() == 0 {
                                                    let temp_tuple = self
                                                        .scene_optima_collision_shapes_look_up
                                                        .get(&shape_frame);
                                                    match temp_tuple {
                                                        Some((shape1, rad1, trans1)) => {
                                                            match parry3d_f64::query::contact(
                                                                &trans1,
                                                                shape1,
                                                                &hull_object.local_transform,
                                                                &new_compound_shape,
                                                                self.d_max,
                                                            ) {
                                                                Ok(contact) => match contact {
                                                                    Some(valid_contact) => {
                                                                        let proximity = ProximityInfo::new(
                                                                            shape_frame.to_string(),
                                                                            hull_object.frame.to_string(),
                                                                            valid_contact.dist,
                                                                            Some((
                                                                                valid_contact.point1,
                                                                                valid_contact.point2,
                                                                            )),
                                                                            true,
                                                                        );
                                                                        self.scene_group_truth_distance_hashmap
                                                                            .get_mut(&shape_frame)
                                                                            .unwrap()
                                                                            .push((
                                                                                proximity,
                                                                                shape1.clone(),
                                                                                new_compound_shape.clone(),
                                                                                *rad1,
                                                                                new_bounding_sphere_radius,
                                                                                *trans1,
                                                                                hull_object.local_transform,
                                                                            ));
                                                                        index_vec.push(Some(
                                                                            self.scene_group_truth_distance_hashmap
                                                                                .get_mut(&shape_frame)
                                                                                .unwrap()
                                                                                .len()
                                                                                - 1)
                                                                        );
                                                                    }
                                                                    None => {
                                                                        index_vec.push(None);
                                                                    }
                                                                },
                                                                Err(_) => {
                                                                    index_vec.push(None);
                                                                }
                                                            }
                                                        }
                                                        None => {
                                                            continue;
                                                        }
                                                    }
                                                } else {
                                                    let temp_tuple = shapes_vec.get(0).unwrap();
                                                    match parry3d_f64::query::contact(
                                                        &temp_tuple.5,
                                                        &temp_tuple.1,
                                                        &hull_object.local_transform,
                                                        &new_compound_shape,
                                                        self.d_max,
                                                    ) {
                                                        Ok(contact) => match contact {
                                                            Some(valid_contact) => {
                                                                let proximity = ProximityInfo::new(
                                                                    shape_frame.to_string(),
                                                                    hull_object.frame.to_string(),
                                                                    valid_contact.dist,
                                                                    Some((
                                                                        valid_contact.point1,
                                                                        valid_contact.point2,
                                                                    )),
                                                                    true,
                                                                );
                                                                self.scene_group_truth_distance_hashmap
                                                                    .get_mut(&shape_frame)
                                                                    .unwrap()
                                                                    .push((
                                                                        proximity,
                                                                        temp_tuple.1.clone(),
                                                                        new_compound_shape.clone(),
                                                                        temp_tuple.3,
                                                                        new_bounding_sphere_radius,
                                                                        temp_tuple.5,
                                                                        hull_object.local_transform,
                                                                    ));
                                                                index_vec.push(Some(
                                                                    self.scene_group_truth_distance_hashmap
                                                                        .get_mut(&shape_frame)
                                                                        .unwrap()
                                                                        .len()
                                                                        - 1,
                                                                ));
                                                            }
                                                            None => {
                                                                index_vec.push(None);
                                                            }
                                                        },
                                                        Err(_) => {
                                                            index_vec.push(None);
                                                        }
                                                    }
                                                }
                                            }
                                            self.scene_optima_transient_shapes_look_up.insert(
                                                id.to_string(),
                                                (hull_object.frame.to_string(), index_vec),
                                            );
                                        } else {
                                            match self
                                                .scene_group_truth_distance_hashmap
                                                .get_mut(&hull_object.frame)
                                            {
                                                Some(valid_vec) => {
                                                    let mut shape1_vec =
                                                        valid_vec.get_mut(0).unwrap().1.shapes().to_vec();
                                                    shape1_vec
                                                        .push((hull_object.local_transform, hull_collider));
                                                    let new_compound_shape =
                                                        Compound::new(shape1_vec.clone());
                                                    let new_bounding_sphere_radius =
                                                        new_compound_shape.local_bounding_sphere().radius;
                                                    let mut index_vec: Vec<Option<usize>> = vec![];
                                                    for i in 0..valid_vec.len() {
                                                        let tuple = valid_vec.get_mut(i).unwrap();
                                                        match parry3d_f64::query::contact(
                                                            &tuple.5,
                                                            &new_compound_shape,
                                                            &tuple.6,
                                                            &tuple.2,
                                                            self.d_max,
                                                        ) {
                                                            Ok(contact) => match contact {
                                                                Some(valid_contact) => {
                                                                    tuple.0.distance = valid_contact.dist;
                                                                    tuple.0.points = Some((
                                                                        valid_contact.point1,
                                                                        valid_contact.point2,
                                                                    ));
                                                                    tuple.1 = new_compound_shape.clone();
                                                                    tuple.3 = new_bounding_sphere_radius;
                                                                    index_vec
                                                                        .push(Some(shape1_vec.len() - 1));
                                                                }
        
                                                                None => {
                                                                    index_vec.push(None);
                                                                }
                                                            },
                                                            Err(_) => {
                                                                index_vec.push(None);
                                                            }
                                                        }
                                                    }
                                                    self.scene_optima_transient_shapes_look_up.insert(
                                                        id.to_string(),
                                                        (hull_object.frame.to_string(), index_vec),
                                                    );
                                                }
                                                None => {}
                                            }
                                        }
                                    }
                                }
                                None => {}
                            }
                        }
                        shapes::Shape::Mesh(_mesh_object) => {}
                    },
                    ShapeUpdate::Move { id, pose } => {
                        match self.scene_optima_transient_shapes_look_up.get(id) {
                            Some((frame_name, index_vec)) => {
                                if frame_name == "world" {
                                    let mut index = 0;
                                    for (frame_name, temp_vec) in
                                        self.scene_group_truth_distance_hashmap.clone()
                                    {
                                        if temp_vec.len() == 0 {
                                            index += 1;
                                            continue;
                                        }
                                        match index_vec.get(index) {
                                            Some(index_move) => match index_move {
                                                Some(index_move) => {
                                                    match self
                                                        .scene_group_truth_distance_hashmap
                                                        .get_mut(&frame_name)
                                                        .unwrap()
                                                        .get_mut(*index_move)
                                                    {
                                                        Some(_) => {
                                                            self.scene_group_truth_distance_hashmap
                                                                    .get_mut(&frame_name)
                                                                    .unwrap()
                                                                    .get_mut(*index_move)
                                                                    .unwrap()
                                                                    .6 = *pose;
                                                            match parry3d_f64::query::contact(
                                                                &temp_vec
                                                                    .get(*index_move)
                                                                    .unwrap()
                                                                    .5,
                                                                &temp_vec
                                                                    .get(*index_move)
                                                                    .unwrap()
                                                                    .1,
                                                                &temp_vec
                                                                    .get(*index_move)
                                                                    .unwrap()
                                                                    .6,
                                                                &temp_vec
                                                                    .get(*index_move)
                                                                    .unwrap()
                                                                    .2,
                                                                    self.d_max,
                                                            ) {
                                                                Ok(contact) => match contact {
                                                                    Some(valid_contact) => {
                                                                        self.scene_group_truth_distance_hashmap
                                                                        .get_mut(&frame_name)
                                                                        .unwrap()
                                                                        .get_mut(*index_move)
                                                                        .unwrap()
                                                                        .0
                                                                        .distance = valid_contact.dist;
                                                                        self.scene_group_truth_distance_hashmap
                                                                        .get_mut(&frame_name)
                                                                        .unwrap()
                                                                        .get_mut(*index_move)
                                                                        .unwrap()
                                                                        .0
                                                                        .points = Some((
                                                                        valid_contact.point1,
                                                                        valid_contact.point2,
                                                                    ));
                                                                    }
                                                                    None => {}
                                                                },
                                                                Err(_) => {}
                                                            }

                                                            index += 1;
                                                        }
                                                        None => {
                                                            index += 1;
                                                            continue;
                                                        }
                                                    }
                                                }
                                                None => {
                                                    index += 1;
                                                    continue;
                                                }
                                            },
                                            None => {
                                                index += 1;
                                                continue;
                                            }
                                        }
                                    }
                                } else {
                                    let temp_vec = self
                                        .scene_group_truth_distance_hashmap
                                        .get(frame_name)
                                        .unwrap()
                                        .clone();

                                    match self
                                        .scene_optima_transient_shapes_look_up
                                        .get(id)
                                        .unwrap()
                                        .1
                                        .get(0)
                                        .unwrap()
                                    {
                                        Some(index_change) => {
                                            for (_, shape1, _, _, _, _, _) in temp_vec {
                                                let mut shape_vec = shape1.shapes().to_vec();
                                                if shape_vec.len() == 0 {
                                                    continue;
                                                }
                                                
                                                match  &shape_vec.get(*index_change){
                                                    Some (_) => {
                                                        let primitive_shape =
                                                         &shape_vec.get(*index_change).unwrap().1;
                                                         shape_vec[*index_change] =
                                                    (*pose, primitive_shape.clone());
                                                let new_compound_shape = Compound::new(shape_vec);
                                                let new_radius = new_compound_shape
                                                    .local_bounding_sphere()
                                                    .radius();
                                                self.scene_group_truth_distance_hashmap
                                                    .get_mut(frame_name)
                                                    .unwrap()
                                                    .get_mut(*index_change)
                                                    .unwrap()
                                                    .1 = new_compound_shape.clone();
                                                self.scene_group_truth_distance_hashmap
                                                    .get_mut(frame_name)
                                                    .unwrap()
                                                    .get_mut(*index_change)
                                                    .unwrap()
                                                    .3 = new_radius;
                                                match parry3d_f64::query::contact(
                                                    &self
                                                        .scene_group_truth_distance_hashmap
                                                        .get(frame_name)
                                                        .unwrap()
                                                        .get(*index_change)
                                                        .unwrap()
                                                        .5,
                                                    &new_compound_shape,
                                                    &self
                                                        .scene_group_truth_distance_hashmap
                                                        .get(frame_name)
                                                        .unwrap()
                                                        .get(*index_change)
                                                        .unwrap()
                                                        .6,
                                                    &self
                                                        .scene_group_truth_distance_hashmap
                                                        .get(frame_name)
                                                        .unwrap()
                                                        .get(*index_change)
                                                        .unwrap()
                                                        .2,
                                                        self.d_max,
                                                ) {
                                                    Ok(contact) => match contact {
                                                        Some(valid_contact) => {
                                                            self.scene_group_truth_distance_hashmap
                                                        .get_mut(frame_name)
                                                        .unwrap()
                                                        .get_mut(*index_change)
                                                        .unwrap()
                                                        .0
                                                        .distance = valid_contact.dist;
                                                            self.scene_group_truth_distance_hashmap
                                                        .get_mut(frame_name)
                                                        .unwrap()
                                                        .get_mut(*index_change)
                                                        .unwrap()
                                                        .0
                                                        .points = Some((
                                                        valid_contact.point1,
                                                        valid_contact.point2,
                                                    ));
                                                        }
                                                        None => {}
                                                    },
                                                    Err(_) => {}
                                                }

                                                    }
                                                    None => {
                                                        continue;
                                                    }
                                                }
                                                
                                            }
                                        }
                                        None => {}
                                    }
                                }
                            }
                            None => {
                               // println!("the id you have provided is not valid");
                            }
                        }
                    }
                    ShapeUpdate::Delete(id) => {
                        match self.scene_optima_transient_shapes_look_up.get(id) {
                            Some((frame_name, index_vec)) => {
                                if frame_name == "world" {
                                    let mut index = 0;
                                    for (frame_name, ground_truth_element_vec) in
                                        self.scene_group_truth_distance_hashmap.clone()
                                    {
                                        
                                        if index > index_vec.len()-1{
                                            break;
                                        }else{
                                            match index_vec.get(index) {
                                                Some(index_remove) => match index_remove {
                                                    Some(index_remove) => {
                                                        if ground_truth_element_vec.len() == 0 || *index_remove > ground_truth_element_vec.len() -1 {
                                                            index += 1;
                                                            continue;
                                                        }
                                                        self.scene_group_truth_distance_hashmap
                                                            .get_mut(&frame_name)
                                                            .unwrap()
                                                            .remove(*index_remove);
                                                        index += 1;
                                                    }
                                                    None => {
                                                        index += 1;
                                                        continue;
                                                    }
                                                },
                                                None => {
                                                    index += 1;
                                                    continue;
                                                }
                                            }
                                        }
                                       
                                    }
                                    self.scene_optima_transient_shapes_look_up
                                        .remove_entry(id)
                                        .unwrap();
                                } else {
                                    let ground_truth_element_vec = self
                                        .scene_group_truth_distance_hashmap
                                        .get_mut(frame_name)
                                        .unwrap();
                                    let index = index_vec.get(0).unwrap().unwrap();

                                    for tuple in &mut *ground_truth_element_vec {
                                        let mut shape1_compound_shapes_vec = tuple.1.shapes().to_vec();
                                      
                                        shape1_compound_shapes_vec.remove(index);
                                        let new_compound_shape = Compound::new(shape1_compound_shapes_vec);
                                        tuple.1 = new_compound_shape.clone();
                                        tuple.3 = new_compound_shape.local_bounding_sphere().radius;
                                        match parry3d_f64::query::contact(
                                            &tuple.5,
                                            &new_compound_shape,
                                            &tuple.6,
                                            &tuple.2,
                                            self.d_max,
                                        ) {
                                            Ok(contact) => match contact {
                                                Some(valid_contact) => {
                                                    tuple.0.distance = valid_contact.dist;
                                                    tuple.0.points = Some((
                                                        valid_contact.point1,
                                                        valid_contact.point2,
                                                    ));
                                                }
                                                None => {
                                                    continue;
                                                }
                                            },
                                            Err(_) => {
                                                continue;
                                            }
                                        }
                                    }

                                    self.scene_optima_transient_shapes_look_up
                                        .remove_entry(id)
                                        .unwrap();
                                    
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
                                        println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _,_) = self
                                            .scene_collision_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (box_object.local_transform, box_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_collision_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0,box_object.local_transform);
                                    } else {
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(box_object.local_transform, box_collider)];
                                        let compound_shape = Compound::new(shape_vec);
                                        self.scene_collision_shapes_list.push((
                                            box_object.frame.to_string(),
                                            compound_shape,
                                            0.0,
                                            box_object.local_transform
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_collision_shapes_list.len() - 1, 0),
                                        );
                                    }
                                } else {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _,_) = self
                                            .scene_collision_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (box_object.local_transform, box_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_collision_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0,box_object.local_transform);
                                    } else {
                                        let index = self
                                            .scene_collision_shapes_list
                                            .iter()
                                            .position(|x| x.0 == box_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                let (_, compound_shape, _,_) = self
                                                    .scene_collision_shapes_list
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
                                                self.scene_collision_shapes_list[valid_index] = (
                                                    box_object.frame.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                    box_object.local_transform
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
                                            //     println!(
                                            //     "The frame does not exist for this transient shape"
                                            // );
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
                                        println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _,_) = self
                                            .scene_collision_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] = (
                                            cylinder_object.local_transform * transform_offset,
                                            cylinder_collider,
                                        );
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_collision_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0,cylinder_object.local_transform);
                                    } else {
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> = vec![(
                                            cylinder_object.local_transform * transform_offset,
                                            cylinder_collider,
                                        )];
                                        let compound_shape = Compound::new(shape_vec);
                                        self.scene_collision_shapes_list.push((
                                            cylinder_object.frame.to_string(),
                                            compound_shape,
                                            0.0,
                                            cylinder_object.local_transform
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_collision_shapes_list.len() - 1, 0),
                                        );
                                    }
                                } else {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape,_, _) = self
                                            .scene_collision_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] = (
                                            cylinder_object.local_transform * transform_offset,
                                            cylinder_collider,
                                        );
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_collision_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0,cylinder_object.local_transform);
                                    } else {
                                        let index = self
                                            .scene_collision_shapes_list
                                            .iter()
                                            .position(|x| x.0 == cylinder_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                let (_, compound_shape, _,_) = self
                                                    .scene_collision_shapes_list
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
                                                self.scene_collision_shapes_list[valid_index] = (
                                                    cylinder_object.frame.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                    cylinder_object.local_transform
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
                                            //     println!(
                                            //     "The frame does not exist for this transient shape"
                                            // );
                                            }
                                        }
                                    }
                                }
                            }
                            shapes::Shape::Sphere(sphere_object) => {
                                let sphere_collider = SharedShape::ball(sphere_object.radius);
                                if sphere_object.frame == "world" {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _,_) = self
                                            .scene_collision_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (sphere_object.local_transform, sphere_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_collision_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0,sphere_object.local_transform);
                                    } else {
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                            vec![(sphere_object.local_transform, sphere_collider)];
                                        let compound_shape = Compound::new(shape_vec);
                                        self.scene_collision_shapes_list.push((
                                            sphere_object.frame.to_string(),
                                            compound_shape,
                                            0.0,
                                            sphere_object.local_transform
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_collision_shapes_list.len() - 1, 0),
                                        );
                                    }
                                } else {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _,_) = self
                                            .scene_collision_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (sphere_object.local_transform, sphere_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_collision_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0,sphere_object.local_transform);
                                    } else {
                                        let index = self
                                            .scene_collision_shapes_list
                                            .iter()
                                            .position(|x| x.0 == sphere_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                let (_, compound_shape,_, _) = self
                                                    .scene_collision_shapes_list
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
                                                self.scene_collision_shapes_list[valid_index] = (
                                                    sphere_object.frame.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                    sphere_object.local_transform
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
                                            //     println!(
                                            //     "The frame does not exist for this transient shape"
                                            // );
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
                                        println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _,_) = self
                                            .scene_collision_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (capsule_object.local_transform, capsule_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_collision_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0,capsule_object.local_transform);
                                    } else {
                                        let shape_vec: Vec<(Isometry3<f64>, SharedShape)> = vec![(
                                            capsule_object.local_transform,
                                            capsule_collider,
                                        )];
                                        let compound_shape = Compound::new(shape_vec);
                                        self.scene_collision_shapes_list.push((
                                            capsule_object.frame.to_string(),
                                            compound_shape,
                                            0.0,
                                            capsule_object.local_transform
                                        ));
                                        self.scene_transient_shapes_look_up.insert(
                                            id.to_string(),
                                            (self.scene_collision_shapes_list.len() - 1, 0),
                                        );
                                    }
                                } else {
                                    if self.scene_transient_shapes_look_up.contains_key(id) {
                                        println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                        let (frame_index, vec_index) =
                                            self.scene_transient_shapes_look_up.get(id).unwrap();
                                        let (frame_name, compound_shape, _,_) = self
                                            .scene_collision_shapes_list
                                            .get(*frame_index)
                                            .unwrap();
                                        let mut compound_shape_vec =
                                            compound_shape.shapes().to_vec();
                                        compound_shape_vec[*vec_index] =
                                            (capsule_object.local_transform, capsule_collider);
                                        let new_compound_shape = Compound::new(compound_shape_vec);
                                        self.scene_collision_shapes_list[*frame_index] =
                                            (frame_name.to_string(), new_compound_shape, 0.0,capsule_object.local_transform);
                                    } else {
                                        let index = self
                                            .scene_collision_shapes_list
                                            .iter()
                                            .position(|x| x.0 == capsule_object.frame);
                                        match index {
                                            Some(valid_index) => {
                                                let (_, compound_shape, _,_) = self
                                                    .scene_collision_shapes_list
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
                                                self.scene_collision_shapes_list[valid_index] = (
                                                    capsule_object.frame.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                    capsule_object.local_transform
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
                                                // println!(
                                                //     "The frame does not exist for this transient shape"
                                                // );
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
                                                println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                                let (frame_index, vec_index) = self
                                                    .scene_transient_shapes_look_up
                                                    .get(id)
                                                    .unwrap();
                                                let (frame_name, compound_shape, _,_) = self
                                                    .scene_collision_shapes_list
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
                                                self.scene_collision_shapes_list[*frame_index] = (
                                                    frame_name.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                    hull_object.local_transform
                                                );
                                            } else {
                                                let shape_vec: Vec<(Isometry3<f64>, SharedShape)> =
                                                    vec![(
                                                        hull_object.local_transform,
                                                        valid_hull_collider,
                                                    )];
                                                let compound_shape = Compound::new(shape_vec);
                                                self.scene_collision_shapes_list.push((
                                                    hull_object.frame.to_string(),
                                                    compound_shape,
                                                    0.0,
                                                    hull_object.local_transform
                                                ));
                                                self.scene_transient_shapes_look_up.insert(
                                                    id.to_string(),
                                                    (self.scene_collision_shapes_list.len() - 1, 0),
                                                );
                                            }
                                        } else {
                                            if self.scene_transient_shapes_look_up.contains_key(id)
                                            {
                                                println!("WARNING: overwriting the shape because another transient shape with the same id already exist in the scene");
                                                let (frame_index, vec_index) = self
                                                    .scene_transient_shapes_look_up
                                                    .get(id)
                                                    .unwrap();
                                                let (frame_name, compound_shape, _,_) = self
                                                    .scene_collision_shapes_list
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
                                                self.scene_collision_shapes_list[*frame_index] = (
                                                    frame_name.to_string(),
                                                    new_compound_shape,
                                                    0.0,
                                                    hull_object.local_transform
                                                );
                                            } else {
                                                let index = self
                                                    .scene_collision_shapes_list
                                                    .iter()
                                                    .position(|x| x.0 == hull_object.frame);
                                                match index {
                                                    Some(valid_index) => {
                                                        let (_, compound_shape, _,_) = self
                                                            .scene_collision_shapes_list
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
                                                        self.scene_collision_shapes_list
                                                            [valid_index] = (
                                                            hull_object.frame.to_string(),
                                                            new_compound_shape,
                                                            0.0,
                                                            hull_object.local_transform
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
                                                //         println!(
                                                //     "The frame does not exist for this transient shape"
                                                // );
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
                                let (frame_name, compound_shape, _,_) = self
                                    .scene_collision_shapes_list
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
                                self.scene_collision_shapes_list[*valid_move_item_frame_idx] = (
                                    frame_name.to_string(),
                                    new_compound_shape,
                                    new_bounding_sphere_radius,
                                    *pose
                                );
                            }
                            None => {
                               // println!("this id does not exist");
                            }
                        }
                    }
                    ShapeUpdate::Delete(id) => {
                        let delete_item_hashmap = self.scene_transient_shapes_look_up.clone();
                        let delete_item = delete_item_hashmap.get(id);
                        match delete_item {
                            Some((valid_delete_item_frame_index, valid_delete_item_index)) => {
                                let (frame_name, compound_shape, _,trans) = self
                                    .scene_collision_shapes_list
                                    .get(*valid_delete_item_frame_index)
                                    .unwrap();
                                let mut compound_shape_vec = compound_shape.shapes().to_vec();
                                if compound_shape_vec.len() >= 2 {
                                    compound_shape_vec.remove(*valid_delete_item_index);
                                    let new_compound_shape = Compound::new(compound_shape_vec);
                                    self.scene_collision_shapes_list
                                        [*valid_delete_item_frame_index] =
                                        (frame_name.to_string(), new_compound_shape, 0.0, *trans);
                                }
                                self.scene_transient_shapes_look_up.remove_entry(id);
                            }
                            None => {
                               // println!("this id does not exist");
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn clear_all_transient_shapes(&mut self) {
        if self.optima_version {
            for (_, (frame_name, index_vec)) in self.scene_optima_transient_shapes_look_up.iter() {
                if frame_name == "world" {
                    let mut index = 0;
                    for (frame_name, ground_truth_element_vec) in
                        self.scene_group_truth_distance_hashmap.clone()
                    {
                        
                        if index > index_vec.len()-1{
                            break;
                        }else{
                            match index_vec.get(index) {
                                Some(index_remove) => match index_remove {
                                    Some(index_remove) => {
                                        if ground_truth_element_vec.len() == 0 || *index_remove > ground_truth_element_vec.len() -1 {
                                            index += 1;
                                            continue;
                                        }
                                        self.scene_group_truth_distance_hashmap
                                            .get_mut(&frame_name)
                                            .unwrap()
                                            .remove(*index_remove);
                                        index += 1;
                                    }
                                    None => {
                                        index += 1;
                                        continue;
                                    }
                                },
                                None => {
                                    index += 1;
                                    continue;
                                }
                            }
                        }
                       
                    }
                    
                } else {
                    let ground_truth_element_vec = self
                        .scene_group_truth_distance_hashmap
                        .get_mut(frame_name)
                        .unwrap();
                    let index = index_vec.get(0).unwrap().unwrap();

                    for tuple in &mut *ground_truth_element_vec {
                        let mut shape1_compound_shapes_vec = tuple.1.shapes().to_vec();
                        if index > shape1_compound_shapes_vec.len() -1 {
                            continue;
                        }
                        shape1_compound_shapes_vec.remove(index);
                        let new_compound_shape = Compound::new(shape1_compound_shapes_vec);
                        tuple.1 = new_compound_shape.clone();
                        tuple.3 = new_compound_shape.local_bounding_sphere().radius;
                        match parry3d_f64::query::contact(
                            &tuple.5,
                            &new_compound_shape,
                            &tuple.6,
                            &tuple.2,
                            self.d_max,
                        ) {
                            Ok(contact) => match contact {
                                Some(valid_contact) => {
                                    tuple.0.distance = valid_contact.dist;
                                    tuple.0.points = Some((
                                        valid_contact.point1,
                                        valid_contact.point2,
                                    ));
                                }
                                None => {
                                    continue;
                                }
                            },
                            Err(_) => {
                                continue;
                            }
                        }
                    }

                    
                    
                }
            }
            self.scene_transient_shapes_look_up.clear();
        } else {
            for (_, (frame_idx, vec_idx)) in self.scene_transient_shapes_look_up.iter() {
                let (frame_name, compound_shape, _,trans) =
                    self.scene_collision_shapes_list.get(*frame_idx).unwrap();
                let mut compound_shape_vec = compound_shape.shapes().to_vec();
                if compound_shape_vec.len() >= 2 {
                    compound_shape_vec.remove(*vec_idx);
                    let new_compound_shape = Compound::new(compound_shape_vec);
                    self.scene_collision_shapes_list[*frame_idx] =
                        (frame_name.to_string(), new_compound_shape, 0.0,*trans);
                }
            }
            self.scene_transient_shapes_look_up.clear();
        }
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

        let shape1_k_translation = current_frame
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
                - (shape1_k_translation.vector - shape2_current_translation.vector).norm())
            .abs();
        let change_in_relative_rotation = (shape1_j_rotation
            .rotation_to(&shape2_j_rotation)
            .rotation_to(&shape1_current_rotation.rotation_to(&shape2_current_rotation)))
        .angle();

        return Some((change_in_relative_translation, change_in_relative_rotation));
    }
    
    pub fn compute_bounding_spheres(&self, compound_shape: &Compound) -> f64 {
        return compound_shape.local_bounding_sphere().radius;
    }

    
    pub fn compute_translation_of_a_point_induced_by_rotation(
        &self,
        distance: f64,
        rotation: f64,
    ) -> f64 {
        let result = f64::sqrt(2.0 * (distance * distance) * (1.0 - rotation.cos()));
        return result;
    }

   
    pub fn compute_lower_signed_distance_bound(
        &self,
        shape1: &Compound,
        shape1_frame: &String,
        shape2: &Compound,
        shape2_frame: &String,
        current_frame: &HashMap<String, TransformInfo>,
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

   
    pub fn compute_loss_function(&self, x: &f64) -> f64 {
        let result;
        if *x <= 0.0 {
            result = -*x + 1.0;
        } else {
            let c = 0.2 * self.a_max;

            result = (-(*x * *x) / 2.0 * c * c).exp();
        }
        return result;
    }


    pub fn compute_loss_with_cutoff(&self, x: &f64, a_value: &f64) -> f64 {
        let result;

        if *x >= self.d_max || *x / *a_value >= self.a_max {
            result = 0.0;
        } else {
            result = self.compute_loss_function(x);
        }
        return result;
    }

   
    pub fn compute_maximum_loss_functions_error(
        &self,
        shape1: &Compound,
        shape1_frame: &String,
        shape2: &Compound,
        shape2_frame: &String,
        current_frame: &HashMap<String, TransformInfo>,
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

        let estimated_distance = (1.0 - self.r) * lower_bound + self.r * upper_bound;
        let a_value = self.a_value;
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

 
    pub fn ranking_maximum_loss_functions_error(
        &self,
        current_frame: &HashMap<String, TransformInfo>,
    ) -> Vec<(String, Compound, String, Compound, f64)> {
        let mut loss_functions_error_vec: Vec<(String, Compound, String, Compound, f64)> = vec![];
        let timed_timer = Instant::now();
        for (_, valid_vec) in self.scene_group_truth_distance_hashmap.clone() {
            for (proximity, shape1, shape2, _, _, pos1, pos2) in valid_vec {
                if timed_timer.elapsed().as_micros() < self.time_budget.as_micros() {
                    let current_loss_function_error = self.compute_maximum_loss_functions_error(
                        &shape1,
                        &proximity.shape1,
                        &shape2,
                        &proximity.shape2,
                        current_frame,
                        &(proximity.clone(), pos1, pos2),
                    );
                    loss_functions_error_vec.push((
                        proximity.shape1.to_string(),
                        shape1.clone(),
                        proximity.shape2.to_string(),
                        shape2.clone(),
                        current_loss_function_error,
                    ));
                }
            }
        }

        loss_functions_error_vec.sort_by(|a, b| a.4.partial_cmp(&b.4).unwrap());

        return loss_functions_error_vec;
    }

    #[profiling::function]
    pub fn get_proximity(&mut self, frames: &HashMap<String, TransformInfo>) -> Vec<ProximityInfo> {
        let mut result_vector: Vec<ProximityInfo> = vec![];
        let default_frame_transform: TransformInfo = TransformInfo::default();
        if self.optima_version {
            let ranking_vector: Vec<(String, Compound, String, Compound, f64)> =
                self.ranking_maximum_loss_functions_error(frames);
            if self.timed {
                let timed_timer = Instant::now();
                for (shape1_frame, shape1, shape2_frame, shape2, _) in ranking_vector {
                    if timed_timer.elapsed().as_micros() < self.time_budget.as_micros() {
                        let shape1_transform = frames
                            .get(&shape1_frame)
                            .unwrap_or(&default_frame_transform)
                            .world;
                        let shape2_transform = frames
                            .get(&shape2_frame)
                            .unwrap_or(&default_frame_transform)
                            .world;
                        let contact = parry3d_f64::query::contact(
                            &shape1_transform,
                            &shape1,
                            &shape2_transform,
                            &shape2,
                            self.d_max,
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
                                }
                                None => {}
                            },

                            Err(_) => {}
                        }
                    } else {
                        break;
                    }
                }
                return result_vector;
            } else {
                let mut remaining_error_summation = 0.0;
                let mut index = 0;
                for (_, _, _, _, loss_value) in ranking_vector.clone() {
                    remaining_error_summation += loss_value;
                }
                while remaining_error_summation > ACCURACY_BUDGET
                    && index < ranking_vector.clone().len() - 1
                {
                    match ranking_vector.get(index) {
                        Some((
                            current_shape1_frame,
                            current_shape1,
                            current_shape2_frame,
                            current_shape2,
                            current_loss_value,
                        )) => {
                            let shape1_transform = frames.get(current_shape1_frame).unwrap();
                            let shape2_transform = frames.get(current_shape2_frame).unwrap();
                            let contact = parry3d_f64::query::contact(
                                &shape1_transform.world,
                                current_shape1,
                                &shape2_transform.world,
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
                                            true,
                                        );
                                        result_vector.push(proximity.clone());
                                    }
                                    None => {}
                                },
                                Err(_) => {}
                            }
                            remaining_error_summation -= current_loss_value;
                            index += 1;
                        }
                        None => {
                            break;
                        }
                    }
                }
               
                return result_vector;
            }
        } else {
            
            let size = self.scene_collision_shapes_list.len();
            if size != 0 {
                for i in 0..=size - 1 {
                    let (shape1_frame, shape1, _, _) =
                    self.scene_collision_shapes_list.get(i).unwrap();
                    if shape1_frame == "world" {
                         break;
                    } 
                    for j in (i + 1)..=size - 1 {                                              
                         let (shape2_frame, shape2, _, env_trans) =
                            self.scene_collision_shapes_list.get(j).unwrap();
                        if shape1_frame == "world" && shape2_frame == "world" {
                            continue;
                        }                    
                        let shape1_transform = frames.get(shape1_frame);
                        match shape1_transform {
                            Some(shape1_transform) => {
                                let shape2_transform = env_trans;
                               
                                        match parry3d_f64::query::contact(
                                            &shape1_transform.world,
                                            shape1,
                                            &shape2_transform,
                                            shape2,
                                            self.d_max,
                                        ) {
                                            Ok(contact) => match contact {
                                                Some(valid_contact) => {
                                                    result_vector.push(ProximityInfo::new(
                                                        shape1_frame.to_string(),
                                                        shape2_frame.to_string(),
                                                        valid_contact.dist,
                                                        Some((
                                                            valid_contact.point1,
                                                            valid_contact.point2,
                                                        )),
                                                        true,
                                                    ));
                                                   
                                                }
                                                None => {
                                                   
                                                }
                                            },
                                            Err(_) => {
                                               
                                            }
                                        }
                                    
                                   
                                
                            }
                            None => {}
                        }
                    }
                }
            }
            
           
            return result_vector;
        }
    }
}
