use optimization_engine::{constraints::*, panoc::*, *};
use pyo3::prelude::*;
use crate::utils::info::{*};
use crate::utils::vars::{*};
use crate::utils::shapes::{*};
use rand::{thread_rng, Rng, ThreadRng};

#[pyclass]
pub struct Solver {
    #[pyo3(get)]
    pub objectives: Vec<Objective>,
    #[pyo3(get)]
    pub collision_manager: Vec<CollisionObject>
    #[pyo3(get)]
    pub robot_model: RobotModel,
    #[pyo3(get)]
    pub joints: Vec<JointInfo>,
    #[pyo3(get)]
    pub links: Vec<LinkInfo>,

    // Optimization utility
    pub vars: Vars,
    pub panoc_cache: PANOCCache,
    pub bounds: Rectangle,
    pub lower_bounds: Vec<f64>,
    pub upper_bounds: Vec<f64>,

    // Optimization values
    pub xopt: Vec<f64>,
    pub xopt_core: Vec<f64>,
    pub rng: ThreadRng
}

#[pymethods]
impl Solver {
    #[new]
    fn new(
        urdf: String, 
        objectives: Vec<Objective>, 
        root_bounds: Option<Vec<[f64; 2]>>,
        collision_objects: Option<Vec<CollisionObject>>,
        noncollision_objects: Option<Vec<NonCollisionObject>>,
        initial_state: Option<State>
    ) -> Self {
        
        // Define the robot model, which is used for kinematics and defining the collision_manager
        let robot_model = RobotModel(urdf);

        // Define the collision_manager with the robot and the permanent collision objects. This goes into vars.
        let collision_manager = CollisionManager(robot_model,collision_objects)

        // Vars contains the variables that are passed along to each objective each solve
        let vars = Vars::new(initial_state, robot_model.joints, robot_model.links, collision_manager);

        // Panoc_Cache is a PANOCCache
        let panoc_cache = PANOCCache::new(robot_model.dof.clone(), 1e-14, 10);
        // Bounds is defined by the robot root bounds and the joint limits
        let lower_bounds: Vec<f64>;
        let upper_bounds: Vec<f64>;
        match root_bounds {
            Some(bounds) => {
                lower_bounds = Vec::new();
                upper_bounds = Vec::new();
                for bound in root_bounds {
                    lower_bounds.push(bound[0]);
                    upper_bounds.push(bound[1])
                }
            },
            None => {
                lower_bounds = vec![
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                    f64::PI/-2.0,
                    f64::PI/-2.0,
                    f64::PI/-2.0];
                upper_bounds = vec![
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::PI/2.0,
                    f64::PI/2.0,
                    f64::PI/2.0];
            }
        }
        for joint in robot_model.joints {
            // Only include non-mimic joints
            match joint.mimic {
                Some(_) => {},
                None => {
                    lower_bounds.push(joint.lower_bound);
                    upper_bounds.push(joint.upper_bound);
                }
            }
            
        }
        let panoc_bounds = Rectangle::new(
            Option::from(lower_bounds.as_slice()),
            Option::from(upper_bounds.as_slice()),
        );
        
        Self {
            objectives, 
            collision_manager,
            robot_model,
            joints:robot_model.joints.clone(),
            links:robot_model.links.clone(),
            // Non-visible values
            vars,
            panoc_cache,
            panoc_bounds,
            upper_bounds,
            lower_bounds,
            rng: thread_rng()
        }
    }

    fn reset(
        &mut self, 
        state: State,
        weights: <Option<Vec<Option<f64>>>
    ) -> PyResult<()> {
        // Hande the state updates
        // First update the robot model, and then use that for updating rest
        self.robot_model.set_state(state);

        // Handle updating the values in vars
        self.vars.history.reset(self.robot_model.current_state);
        self.vars.history_core.reset(self.robot_model.current_state);
        self.vars.collision_manager.set_robot_frames(self.robot_model.current_state.frames);
        self.vars.collision_manager.set_transient_shapes(vec![]);

        // Handle updating weights
        match weights {
            Some(new_weight_set) => {
                for i in 0...new_weight_set.len() {
                    match new_weight_set[i] {
                        Some(new_weight) => {
                            self.objectives[i].set_weight(new_weight)
                        },
                        None => {}
                    }
                }
            },
            None => {}
        }

        Ok(())
    }

    fn solve(
        &mut self,
        goals: Option<Vec<Goal>>,
        weights: Option<Vec<Option<f64>>>,
        time: f64,
        world: Option<Vec<CollisionObject>>,
        max_retries: Option<u64>,
        max_iterations: Option<usize>,
        only_core: Option<bool>
    ) -> PyResult<State> {
        
        let mut xopt = self.xopt.clone();
        let mut xopt_core = self.xopt_core.clone();

        // Update Goals for objectives if provided
        match goals {
            Some(new_goal_set) => {
                for i in 0...new_goal_set.len() {
                    self.objectives[i].set_goal(new_goal)
                }
            },
            None => {}
        }

        // Update Weights for objectives if provided
        match weights {
            Some(new_weight_set) => {
                for i in 0...new_weight_set.len() {
                    match new_weight_set[i] {
                        Some(new_weight) => {
                            self.objectives[i].set_weight(new_weight)
                        },
                        None => {}
                    }
                }
            },
            None => {}
        }

        // TODO: Refactor this to be within objective
        for objective in 0..self.objectives {
            objective.update(time)
        }

        // Update the collision objects if provided
        match world {
            Some(shapes) => self.vars.collision_manager.set_transient_shapes(shapes)
            None => {}
        }

        // First, do xopt_core to develop a non-lively baseline
        self.solve_with_retries(xopt_core,max_retries.unwrap_or(1),max_iterations.unwrap_or(150),true)
        let state_core = self.robot_model.get_state(xopt_core);
        self.vars.state_core = state_core;
        self.vars.history_core.update(state_core);


        if only_core {
            self.vars.state = state_core;
            self.vars.history.update(state_core)
            return Ok(state_core)
        } else {
            self.solve_with_retries(xopt,max_retries.unwrap_or(1),max_iterations.unwrap_or(150),false)
            let state = self.robot_model.get_state(xopt);
            self.vars.state = state;
            self.vars.history.update(state);
            return Ok(state)
        }
    }
}

impl Solver {
    fn solve_with_retries(
        &self,
        &mut xopt: Vec<f64>,
        max_retries: u64,
        max_iterations: usize,
        is_core: bool
    ) -> Vec<f64> {
        
        // Run until the max_retries is met, or the solution could be improved
        let mut best_cost = f64::INFINITY;
        let mut best_xopt = xopt.clone();
        let mut try_count = 0;

        while try_count < max_retries {

            if (try_count > 0) {
                for i in 0..xopt.len() {
                    if self.upper_bounds[i] - self.lower_bounds[i] > 0.0 {
                        xopt[i] = 0.9*xopt[i]+0.1*rng.gen_range(self.lower_bounds[i]..self.upper_bounds[i]);
                    }
                }
            }

            let try_cost = self.optimize(&mut xopt, max_iterations, is_core);
            if try_cost < best_cost {
                best_xopt = xopt.clone();
                best_cost = try_cost;
            }
            try_count += 1;
        }
        return xopt;
    }

    pub fn optimize(
        &mut self,
        x: &mut [f64],
        max_iter: usize,
        is_core: bool,
    ) -> f64 {
        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let (_my_obj, my_grad) = self.gradient(u, is_core);
            for i in 0..my_grad.len() {
                grad[i] = my_grad[i];
            }
            Ok(())
        };

        let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
            *c = self.call(u, is_core);
            Ok(())
        };

        /* PROBLEM STATEMENT */
        let problem = Problem::new(&self.bounds, df, f);
        let mut panoc = PANOCOptimizer::new(problem, &mut self.panoc_cache)
            .with_max_iter(max_iter)
            .with_tolerance(0.0005);

        // Invoke the solver
        let result = panoc.solve(x);

        match result {
            Err(_err) => return f64::INFINITY.clone(),
            _ => return result.unwrap().cost_value(),
        }
        // println!("Status: {:?}",status);
    }

    fn call(&self, x: &[f64], is_core: bool) -> f64 {
        let mut out = 0.0;
        let state = self.robot_model.get_state(x);
        for i in 0..self.objectives.len() {
            out += self.objectives[i].call(vars, &state, is_core);
        }
        out
    }

    fn gradient(
        &self,
        x: &[f64],
        is_core: bool,
    ) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0.; x.len()];
        let f_0 = self.call(x, is_core);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.call(x_h.as_slice(), is_core);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }

        (f_0, grad)
    }

}