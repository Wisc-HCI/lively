#[cfg(feature = "pybindings")]
use pyo3::prelude::*;
#[cfg(feature = "pybindings")]
use crate::lively_tk::Solver;
#[cfg(feature = "pybindings")]
use crate::objectives::objective::Objective;
#[cfg(feature = "pybindings")]
use crate::utils::shapes::*;
#[cfg(feature = "pybindings")]
use crate::utils::info::*;
#[cfg(feature = "pybindings")]
use crate::utils::state::State;
#[cfg(feature = "pybindings")]
use crate::utils::goals::Goal;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::objectives::*;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::shapes::*;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::goals::*;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::info::*;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::geometry::*;
#[cfg(feature = "pybindings")]
use crate::wrappers::python::state::*;

#[cfg(feature = "pybindings")]
#[pyclass(name="Solver")] 
pub struct PySolver(Solver);

#[cfg(feature = "pybindings")]
#[pymethods]
impl PySolver {
    #[new]
    fn new(
        urdf: String, 
        objectives: Vec<PyObjective>, 
        root_bounds: Option<Vec<ScalarRange>>,
        shapes: Option<Vec<PyShape>>,
        initial_state: Option<PyState>,
        only_core: Option<bool>,
        max_retries: Option<u64>,
        max_iterations: Option<usize>
    ) -> Self {
            let inner_objectives = objectives.iter().map(|o| Objective::from(o.clone())).collect();
            let inner_shapes = shapes.map(|cs| cs.iter().map(|s| Shape::from(s.clone())).collect());
            let inner_bounds = root_bounds.map(|bs| bs.iter().map(|b| (b.value,b.delta)).collect());
            let inner_state = initial_state.map(|s| State::from(s));
            PySolver(Solver::new(urdf, inner_objectives, inner_bounds, inner_shapes, inner_state, only_core, max_retries, max_iterations))
    }

    #[getter]
    pub fn get_objectives(&self) -> PyResult<Vec<PyObjective>>{
        Ok(self.0.objective_set.objectives.iter().map(|o| PyObjective::from(o.clone())).collect())
    }

    fn reset(
        &mut self, 
        state: PyState,
        weights: Option<Vec<Option<f64>>>
    ) -> PyResult<()> {
        Ok(self.0.reset(State::from(state),weights))
    }

    fn solve(
        &mut self,
        goals: Option<Vec<Option<PyGoal>>>,
        weights: Option<Vec<Option<f64>>>,
        time: f64,
        shape_updates: Option<Vec<PyShapeUpdate>>
    ) -> PyResult<PyState> {
        let inner_goals = goals.map(|gs| gs.iter().map(|og| og.as_ref().map(|g| Goal::from(g.clone()))).collect());
        let inner_updates = shape_updates.map(|cs| cs.iter().map(|s| ShapeUpdate::from(s.clone())).collect());
        Ok(PyState::from(self.0.solve(
            inner_goals,
            weights,
            time,
            inner_updates)))
    }
}