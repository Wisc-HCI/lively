
use optimization_engine::{constraints::*, panoc::*, *};
#[cfg(feature = "jsbindings")]
use serde::{Serialize};
use crate::utils::info::{*};
use crate::utils::vars::{*};
use crate::utils::shapes::{*};
use crate::utils::robot_model::RobotModel;
use crate::utils::state::State;
use crate::utils::goals::Goal;
use crate::utils::objective_set::ObjectiveSet;
use crate::objectives::objective::Objective;
use rand::{thread_rng, Rng};
use rand::rngs::ThreadRng;
use std::collections::HashMap;
#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "jsbindings")]
use wasm_bindgen::prelude::*;
#[cfg(feature = "jsbindings")]
use serde_wasm_bindgen;
#[cfg(feature = "jsbindings")]
use nalgebra::geometry::Translation3;
#[cfg(feature = "jsbindings")]
use nalgebra::Isometry3;
#[cfg(feature = "jsbindings")]
use nalgebra::Quaternion;
#[cfg(feature = "jsbindings")]
use nalgebra::UnitQuaternion;

#[repr(C)]
#[cfg(not(feature = "jsbindings"))]
#[cfg_attr(feature = "pybindings", pyclass)]
pub struct Solver {
    pub robot_model: RobotModel,

    // Optimization utility
    pub vars: Vars,

    pub panoc_cache: PANOCCache,

    pub lower_bounds: Vec<f64>,

    pub upper_bounds: Vec<f64>,

    pub objective_set: ObjectiveSet,

    // Optimization values
    pub xopt: Vec<f64>,

    // Optimization Settings
    pub max_retries: usize,

    pub max_iterations: usize

}

#[repr(C)]
#[cfg(not(feature = "pybindings"))]
#[cfg(feature = "jsbindings")]
#[wasm_bindgen]
pub struct Solver {
    #[wasm_bindgen(skip)]
    pub robot_model: RobotModel,

    // Optimization utility
    #[wasm_bindgen(skip)]
    pub vars: Vars,

    #[wasm_bindgen(skip)]
    pub panoc_cache: PANOCCache,

    #[wasm_bindgen(skip)]
    pub lower_bounds: Vec<f64>,

    #[wasm_bindgen(skip)]
    pub upper_bounds: Vec<f64>,

    #[wasm_bindgen(skip)]
    pub objective_set: ObjectiveSet,

    // Optimization values
    #[wasm_bindgen(skip)]
    pub xopt: Vec<f64>,

    // Optimization Settings
    #[wasm_bindgen(skip)]
    pub max_retries: usize,

    #[wasm_bindgen(skip)]
    pub max_iterations: usize,

}

impl Solver {
    pub fn new(
        urdf: String, 
        objectives: HashMap<String,Objective>, 
        root_bounds: Option<Vec<ScalarRange>>,
        shapes: Option<Vec<Shape>>,
        initial_state: Option<State>,
        max_retries: Option<usize>,
        max_iterations: Option<usize>,
        collision_settings: Option<CollisionSettingInfo>
    ) -> Self {

        // Parse the bounds that were given

        let displacement_bounds: Vec<ScalarRange> = root_bounds.unwrap_or(vec![
            ScalarRange {value:0.0,delta:0.0},
            ScalarRange {value:0.0,delta:0.0},
            ScalarRange {value:0.0,delta:0.0},
            ScalarRange {value:0.0,delta:0.0},
            ScalarRange {value:0.0,delta:0.0},
            ScalarRange {value:0.0,delta:0.0},
        ]);
        let mut lower_bounds: Vec<f64> = vec![];
        let mut upper_bounds: Vec<f64> = vec![];

        for bound in &displacement_bounds {
            lower_bounds.push(bound.value - bound.delta);
            upper_bounds.push(bound.value + bound.delta);
        }

        // Define the robot model, which is used for kinematics and handling collisions
        let robot_model = RobotModel::new(
            urdf, 
            shapes.unwrap_or(vec![]),
            &collision_settings,
            displacement_bounds
        );

        let current_state: State;
        let average_distance: Vec<ProximityInfo>;
        match initial_state {
            Some(state) => {
                average_distance = state.clone().proximity;
                current_state = robot_model.get_filled_state(&state);
            },
            None => {
                average_distance = vec![];
                current_state = robot_model.get_default_state();
            }
        }

        robot_model.collision_manager.lock().unwrap().compute_ground_truth_distance_hashmap(&current_state.frames,&average_distance);
        // Vars contains the variables that are passed along to each objective each solve
        let vars = Vars::new(&current_state, robot_model.joints.clone(), robot_model.links.clone());

        // Panoc_Cache is a PANOCCache
        let panoc_cache = PANOCCache::new(robot_model.dims.clone(), 1e-14, 10);
        // Bounds is defined by the robot root bounds and the joint limits
        
        for joint in &robot_model.joints {
            // Only include non-mimic joints
            match joint.mimic {
                Some(_) => {},
                None => {
                    lower_bounds.push(joint.lower_bound.clone());
                    upper_bounds.push(joint.upper_bound.clone());
                }
            }
            
        }

        let initial_x = robot_model.get_x(&current_state);
        
        Self {
            robot_model,
            // Non-visible values
            vars,
            panoc_cache,
            objective_set: ObjectiveSet::new(&objectives),
            upper_bounds,
            lower_bounds,
            xopt: initial_x.clone(),
            max_retries: max_retries.unwrap_or(1),
            max_iterations: max_iterations.unwrap_or(150)
        }
    }

   
    pub fn reset(
        &mut self, 
        state: State,
        weights: HashMap<String,f64>
    ) -> () {
        // Hande the state updates
        // First update the robot model, and then use that for updating rest
        let current_state = self.robot_model.get_filled_state(&state);
        
        // Handle updating the values in vars
        self.vars.history.reset(&current_state);
        self.robot_model.collision_manager.lock().unwrap().clear_all_transient_shapes();

        // Handle updating weights
        for (key,weight) in &weights {
            match self.objective_set.objectives.get_mut(key) {
                Some(objective) => objective.set_weight(*weight),
                _ => {}
            }
        }
        ()
    }
    
    // #[profiling::function]
    pub fn solve(
        &mut self,
        goals: HashMap<String,Goal>,
        weights: HashMap<String,f64>,
        time: f64,
        shape_updates: Option<Vec<ShapeUpdate>>
    ) -> State {
        
        let xopt = self.xopt.clone();

        if self.objective_set.objectives.len() == 0 {
            return self.get_current_state();
        }

        let mut rng: ThreadRng = thread_rng();

        // Update Goals/Objectives/Time for objectives if provided updates
        for (key,objective) in &mut self.objective_set.objectives {
            match goals.get(key) {
                Some(goal) => objective.set_goal(goal),
                _ => {}
            }
            match weights.get(key) {
                Some(weight) => objective.set_weight(*weight),
                _ => {}
            }
            objective.update(time)
        }

        // Update the collision objects if provided
        match shape_updates {
            Some(updates) => self.robot_model.collision_manager.lock().unwrap().perform_updates(&updates),
            None => {}
        }

        // Solve with the given number of retries
        self.xopt = self.solve_with_retries(xopt,&mut rng, time);
        let state = self.robot_model.get_state(&self.xopt,true,time);
        // println!("State Core Frames {:?}",self.vars.state_core.frames);
        self.vars.history.update(&state);

        return state;
    }
    
    pub fn solve_with_retries(
        &mut self,
        x: Vec<f64>,
        rng: &mut ThreadRng,
        time: f64
    ) -> Vec<f64> {
        
        // Run until the max_retries is met, or the solution could be improved
        let mut best_cost = f64::INFINITY;
        let mut best_x = x.clone();
        let mut xopt = x.clone();
        let mut try_count = 0;

        while try_count < self.max_retries {

            if try_count > 0 {
                for i in 0..xopt.len() {
                    let upper = self.upper_bounds[i].min(50.0);
                    let lower = self.lower_bounds[i].max(-50.0);
                
                    if upper - lower > 0.0 {
                        xopt[i] = 0.9*best_x[i]+0.1*rng.gen_range(lower..upper);
                    }
                }
            }

            let try_cost = optimize(
                &mut xopt, 
                &self.objective_set, 
                &self.robot_model, 
                &self.vars, 
                &mut self.panoc_cache, 
                &self.lower_bounds, 
                &self.upper_bounds, 
                self.max_iterations,
                time
            );
            if try_cost < best_cost {
                best_x = xopt.clone();
                best_cost = try_cost;
            }
            try_count += 1;
        }
        // println!("Best {:?}",best_x);
        return best_x.to_vec();
    }

    pub fn get_objectives(&self) -> HashMap<String,Objective> {
        self.objective_set.objectives.clone()
    }

    pub fn set_objectives(&mut self, objectives: HashMap<String,Objective>) {
        self.objective_set.objectives = objectives;
    }

    pub fn get_goals(&self) -> HashMap<String,Option<Goal>> {
        let mut goals: HashMap<String,Option<Goal>> = HashMap::new();
        for (k,v) in self.objective_set.objectives.iter() {
            goals.insert(k.clone(),v.get_goal());
        }
        return goals;
    }

    pub fn get_links(&self) -> &Vec<LinkInfo> {
        return &self.robot_model.links
    }

    pub fn get_joints(&self) -> &Vec<JointInfo> {
        return &self.robot_model.joints
    }

    pub fn compute_average_distance_table(&mut self) -> Vec<ProximityInfo> {

        let mut sampled_states: Vec<HashMap<String,TransformInfo>> = vec![];
        let mut rng: ThreadRng = thread_rng();

        for _ in 0..1000 {
            let mut x = self.xopt.clone();
            for i in 0..x.len() {
                let upper = self.upper_bounds[i].min(50.0);
                let lower = self.lower_bounds[i].max(-50.0);
                
                if upper - lower > 0.0 {
                    x[i] = rng.gen_range(lower..upper);
                }
            }
           // println!("x: {:?}",x);
            let state = self.robot_model.get_state(&x, false, 0.0);
            sampled_states.push(state.frames);
        }
        
        return self.robot_model.collision_manager.lock().unwrap().compute_a_table(&sampled_states);
    }

    pub fn get_current_state(&self) -> State {
        return self.vars.history.prev1.clone()
    }
}

#[cfg(feature = "jsbindings")]
fn serialize<T>(obj: &T) -> Result<JsValue, serde_wasm_bindgen::Error>
where
    T: Serialize
{
    Ok(obj.serialize(&serde_wasm_bindgen::Serializer::json_compatible())?)
}


#[cfg(feature = "jsbindings")]
#[wasm_bindgen]
impl Solver {
    #[wasm_bindgen(constructor)]
    pub fn from_javascript(
        urdf: String, 
        objectives: JsValue, 
        root_bounds: JsValue,
        shapes: JsValue,
        initial_state: JsValue,
        max_retries: Option<usize>,
        max_iterations: Option<usize>,
        collision_settings: JsValue
    ) -> Self {
            console_error_panic_hook::set_once();
            let inner_objectives:HashMap<String,Objective> = serde_wasm_bindgen::from_value(objectives).unwrap();
            let inner_bounds:Option<Vec<ScalarRange>> = serde_wasm_bindgen::from_value(root_bounds).unwrap();
            let inner_shapes:Option<Vec<Shape>> = serde_wasm_bindgen::from_value(shapes).unwrap();
            let inner_state:Option<State> = serde_wasm_bindgen::from_value(initial_state).unwrap();
            let inner_collision_settings:Option<CollisionSettingInfo> = serde_wasm_bindgen::from_value(collision_settings).unwrap();
            Self::new(urdf, inner_objectives, inner_bounds, inner_shapes, inner_state, max_retries, max_iterations, inner_collision_settings)
    }

    #[wasm_bindgen(getter = objectives)]
    pub fn objectives_javascript(&self) -> Result<JsValue,serde_wasm_bindgen::Error> {
        return serialize(&self.objective_set.objectives)
    }

    #[wasm_bindgen(setter = objectives)]
    pub fn set_objectives_javascript(&mut self, objectives: JsValue) {
        let inner_objectives: HashMap<String,Objective> = serde_wasm_bindgen::from_value(objectives).unwrap();
        self.set_objectives(inner_objectives);
    }

    #[wasm_bindgen(getter = currentState)]
    pub fn current_state_javascript(&self) -> Result<JsValue,serde_wasm_bindgen::Error> {
        return serialize(&self.get_current_state())
    }

    #[wasm_bindgen(getter = currentGoals)]
    pub fn current_goals_javascript(&self) -> Result<JsValue,serde_wasm_bindgen::Error> {
        return serialize(&self.get_goals())
    }

    #[wasm_bindgen(getter = links)]
    pub fn links_javascript(&self) -> Result<JsValue,serde_wasm_bindgen::Error> {
        return serialize(self.get_links())
    }

    #[wasm_bindgen(getter = joints)]
    pub fn joints_javascript(&self) -> Result<JsValue,serde_wasm_bindgen::Error> {
        return serialize(self.get_joints())
    }

    #[wasm_bindgen(js_name = reset)]
    pub fn reset_javascript(
        &mut self, 
        state: JsValue,
        weights: JsValue,
    ) {
        let inner_state:State = serde_wasm_bindgen::from_value(state).unwrap();
        let inner_weights:HashMap<String,f64> = serde_wasm_bindgen::from_value(weights).unwrap();
        self.reset(inner_state,inner_weights);
    }

    #[wasm_bindgen(js_name = solve)]
    pub fn solve_javascript(
        &mut self,
        goals: JsValue,
        weights: JsValue,
        time: f64,
        shape_updates: JsValue
    ) -> Result<JsValue,serde_wasm_bindgen::Error> {
        let inner_goals: HashMap<String,Goal> = serde_wasm_bindgen::from_value(goals).unwrap();
        let inner_weights:HashMap<String,f64> = serde_wasm_bindgen::from_value(weights).unwrap();
        let inner_updates: Option<Vec<ShapeUpdate>> = serde_wasm_bindgen::from_value(shape_updates).unwrap();
        let state:State = self.solve(inner_goals,inner_weights,time,inner_updates);
        return serialize(&state);
    }

    #[wasm_bindgen(js_name = updates)]
    pub fn updates(
        &mut self,
    ) -> Result<JsValue,serde_wasm_bindgen::Error> {
        let iso_1 = Isometry3::from_parts(
            // defining transform from translation and rotation
            Translation3::new(
                1.7497281999999998,
                -0.24972819999999987,
                0.050000000000000044,
            ),
            UnitQuaternion::from_quaternion(Quaternion::new(
                0.0,
                0.0,
                -0.7069999677447771,
                0.7072135784958345,
            )),
        );
        //box_1 here is a static environmental shape. This means that box_1 can not be moved or deleted.
        let box_1 = Shape::Box(BoxShape::new(
            //can be 'CylinderShape', 'CapsuleShape', or 'SphereShape'
            "conveyorCollisionShapeBase".to_string(), // name can be arbitrary
            "world".to_string(),                      // frame name
            true,                                     // physical collision
            1.0,                                      // dimension of the box
            1.1,
            1.7,
            iso_1, // local_transform of the box
        ));
        
        let add_shape_update = AddShape { 
            id: "addBox".to_string(), //The id must be unique
            shape: box_1.clone(),
        };
        let shape_update: Vec<ShapeUpdate> = vec![ShapeUpdate::Add(add_shape_update)]; // shape_update
        return serialize(&shape_update);
    }


    #[wasm_bindgen(js_name = computeAverageDistanceTable)]
    pub fn compute_average_distance_table_javascript(&mut self) -> Result<JsValue,serde_wasm_bindgen::Error> {
        return serialize(&self.compute_average_distance_table())
    }
}

#[cfg(feature = "pybindings")]
#[pymethods]
impl Solver {
    #[new]
    fn from_python(
        urdf: String, 
        objectives: std::collections::HashMap<String,Objective>, 
        root_bounds: Option<Vec<ScalarRange>>,
        shapes: Option<Vec<Shape>>,
        initial_state: Option<State>,
        max_retries: Option<usize>,
        max_iterations: Option<usize>,
        collision_settings: Option<CollisionSettingInfo>
    ) -> Self {
            Solver::new(urdf, objectives, root_bounds, shapes, initial_state, max_retries, max_iterations, collision_settings)
    }

    #[getter(objectives)]
    pub fn get_objectives_python(&self) -> PyResult<HashMap<String,Objective>>{
        return Ok(self.objective_set.objectives.clone());
    }

    #[setter(objectives)]
    pub fn set_objectives_python(&mut self, objectives: HashMap<String,Objective>) {
        self.set_objectives(objectives);
    }

    #[getter(current_state)]
    pub fn get_current_state_python(&self) -> PyResult<State> {
        Ok(self.get_current_state())
    }

    #[getter(current_goals)]
    pub fn get_current_goals_python(&self) -> PyResult<HashMap<String,Option<Goal>>> {
        Ok(self.get_goals())
    }

    #[getter(links)]
    pub fn get_links_python(&self) -> PyResult<Vec<LinkInfo>> {
        Ok(self.get_links().to_vec())
    }

    #[getter(joints)]
    pub fn get_joints_python(&self) -> PyResult<Vec<JointInfo>> {
        Ok(self.get_joints().to_vec())
    }

    #[pyo3(name="reset")]
    fn reset_python(
        &mut self, 
        state: State,
        weights: HashMap<String,f64>
    ) -> PyResult<()> {
        Ok(self.reset(state,weights))
    }

    #[pyo3(name="solve")]
    fn solve_python(
        &mut self,
        goals:  HashMap<String,Goal>,
        weights: HashMap<String,f64>,
        time: f64,
        shape_updates: Option<Vec<ShapeUpdate>>
    ) -> PyResult<State> {
        // let mut inner_goals: std::collections::HashMap<String, Goal> = HashMap::new();
        // for (key,goal) in goals{
        //     inner_goals.insert(key,Goal::from(goal));
        // }
        // // let inner_goals = goals.map(|gs| gs.iter().map(|og| og.as_ref().map(|g| Goal::from(g.clone()))).collect());
        //  let inner_updates = shape_updates.map(|cs| cs.iter().map(|s| ShapeUpdate::from(s.clone())).collect());
        Ok(self.solve(
            goals,
            weights,
            time,
            shape_updates)
        )
        
    }

    #[pyo3(name="compute_average_distance_table")]
    fn compute_average_distance_table_python(&mut self) -> PyResult<Vec<ProximityInfo>> {
        Ok(self.compute_average_distance_table())
    }
}

pub fn optimize(
    x: &mut [f64],
    objective_set: &ObjectiveSet,
    robot_model: &RobotModel,
    vars: &Vars,
    cache: &mut PANOCCache,
    lower_bounds: &Vec<f64>,
    upper_bounds: &Vec<f64>,
    max_iter: usize,
    timestamp: f64
) -> f64 {

    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        let (_my_obj, my_grad) = objective_set.gradient(&robot_model, &vars, u, timestamp);
        for i in 0..my_grad.len() {
            grad[i] = my_grad[i];
        }
        Ok(())
    };

    let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
        *c = objective_set.call(&robot_model, &vars, u, timestamp);
        Ok(())
    };

    let panoc_bounds = Rectangle::new(
        Option::from(lower_bounds.as_slice()),
        Option::from(upper_bounds.as_slice()),
    );

    /* PROBLEM STATEMENT */
    let problem = Problem::new(&panoc_bounds, df, f);
    let mut panoc = PANOCOptimizer::new(problem, cache)
        .with_max_iter(max_iter)
        .with_tolerance(0.0005);

    // Invoke the solver
    let result = panoc.solve(x);
    // println!("Result Retrieved {:?}",result);


    match result {
        Err(_err) => return f64::INFINITY.clone(),
        _ => return result.unwrap().cost_value(),
    }
    
}