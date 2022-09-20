use optimization_engine::{constraints::*, panoc::*, *};
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
use std::f32::INFINITY;
use std::f64::consts::{PI};
use std::collections::HashMap;

#[repr(C)]
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
    pub xopt_core: Vec<f64>,

    // Optimization Settings
    pub only_core: bool,
    pub max_retries: usize,
    pub max_iterations: usize,
}

impl Solver {
    pub fn new(
        urdf: String, 
        objectives: HashMap<String,Objective>, 
        root_bounds: Option<Vec<(f64,f64)>>,
        shapes: Option<Vec<Shape>>,
        initial_state: Option<State>,
        only_core: Option<bool>,
        max_retries: Option<usize>,
        max_iterations: Option<usize>,
        collision_settings: Option<CollisionSettingInfo>
    ) -> Self {

        // Parse the bounds that were given

        let displacement_bounds: Vec<(f64,f64)> = root_bounds.unwrap_or(vec![
            (0.0,INFINITY.into()),
            (0.0,INFINITY.into()),
            (0.0,INFINITY.into()),
            (0.0,PI/-2.0),
            (0.0,PI/-2.0),
            (0.0,PI/-2.0),
        ]);
        let mut lower_bounds: Vec<f64> = vec![];
        let mut upper_bounds: Vec<f64> = vec![];

        for bound in &displacement_bounds {
            lower_bounds.push(bound.0 - bound.1);
            upper_bounds.push(bound.0 + bound.1);
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
            xopt_core: initial_x.clone(),
            only_core: only_core.unwrap_or(false),
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
        self.vars.history_core.reset(&current_state);
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
        let xopt_core = self.xopt_core.clone();

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

        // First, do xopt_core to develop a non-lively baseline
        self.xopt_core = self.solve_with_retries(xopt_core,true,self.only_core,&mut rng);
        self.vars.state_core = self.robot_model.get_state(&self.xopt_core,self.only_core);
        // println!("State Core Frames {:?}",self.vars.state_core.frames);
        self.vars.history_core.update(&self.vars.state_core);


        if self.only_core {
            self.vars.history.update(&self.vars.state_core);
            return self.vars.state_core.clone()
        } else {
            self.xopt = self.solve_with_retries(xopt,false,true,&mut rng);
            let state = self.robot_model.get_state(&self.xopt,true);
            // println!("Solve with retries, prox: {:?}",state.proximity);
            self.vars.history.update(&state);
            return state
        }
    }
    
    pub fn solve_with_retries(
        &mut self,
        x: Vec<f64>,
        is_core: bool,
        is_last: bool,
        rng: &mut ThreadRng
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
                is_core,
                is_last
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

    pub fn compute_average_distance_table(&mut self) -> Vec<ProximityInfo> {

        let mut sampled_states: Vec<HashMap<String,TransformInfo>> = vec![];
        let mut rng: ThreadRng = thread_rng();

        for _ in 0..1000 {
            let mut x = self.xopt_core.clone();
            for i in 0..x.len() {
                let upper = self.upper_bounds[i].min(50.0);
                let lower = self.lower_bounds[i].max(-50.0);
                
                if upper - lower > 0.0 {
                    x[i] = rng.gen_range(lower..upper);
                }
            }
           // println!("x: {:?}",x);
            let state = self.robot_model.get_state(&x, false);
            sampled_states.push(state.frames);
        }
        
        return self.robot_model.collision_manager.lock().unwrap().compute_a_table(&sampled_states);
    }

    pub fn get_current_state(&self) -> State {
        return self.vars.history.prev1.clone()
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
    is_core: bool,
    is_last: bool
) -> f64 {

    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        let (_my_obj, my_grad) = objective_set.gradient(&robot_model, &vars, u, is_core, is_last);
        for i in 0..my_grad.len() {
            grad[i] = my_grad[i];
        }
        Ok(())
    };

    let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
        *c = objective_set.call(&robot_model, &vars, u, is_core, is_last);
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