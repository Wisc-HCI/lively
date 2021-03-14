use nalgebra::geometry::{Quaternion, UnitQuaternion};
use nalgebra::Vector3;
use pyo3::prelude::*;

fn vec_to_vector(value: Vec<f64>) -> Vector3<f64> {
    return Vector3::new(value[0], value[1], value[2]);
}

fn vector_to_vec(value: Vector3<f64>) -> Vec<f64> {
    return vec![value[0], value[1], value[2]];
}

fn vec_to_quat(value: Vec<f64>) -> UnitQuaternion<f64> {
    return UnitQuaternion::from_quaternion(Quaternion::new(value[0], value[1], value[2], value[3]));
}

fn quat_to_vec(value: UnitQuaternion<f64>) -> Vec<f64> {
    return vec![value[0], value[1], value[2], value[3]];
}

#[derive(Clone, Copy, Debug)]
pub enum Goal {
    Scalar(f64),                     // input is a float
    Vector(Vector3<f64>),            // input is a 3-vector
    Quaternion(UnitQuaternion<f64>), // input is a quaternion
    None,
}

#[pyclass]
#[derive(Clone, Copy, Debug)]
pub struct GoalSpec {
    pub value: Goal,
}

#[pymethods]
impl GoalSpec {
    #[new]
    fn new(scalar: Option<f64>, vector: Option<Vec<f64>>, quaternion: Option<Vec<f64>>) -> Self {
        let mut value: Goal = Goal::None;
        match scalar {
            Some(s) => value = Goal::Scalar(s),
            None => {}
        };
        match vector {
            Some(v) => value = Goal::Vector(vec_to_vector(v)),
            None => {}
        };
        match quaternion {
            Some(q) => value = Goal::Quaternion(vec_to_quat(q)),
            None => {}
        };
        return Self { value };
    }

    #[getter(scalar)]
    fn get_scalar(&self) -> PyResult<Option<f64>> {
        let response: Option<f64>;
        match self.value {
            Goal::Scalar(s) => response = Option::Some(s),
            Goal::Vector(_v) => response = Option::None,
            Goal::Quaternion(_q) => response = Option::None,
            Goal::None => response = Option::None,
        }
        return Ok(response);
    }

    #[setter(scalar)]
    fn set_scalar(&mut self, scalar: f64) -> PyResult<()> {
        self.value = Goal::Scalar(scalar);
        return Ok(());
    }

    #[getter(vector)]
    fn get_vector(&self) -> PyResult<Option<Vec<f64>>> {
        let response: Option<Vec<f64>>;
        match self.value {
            Goal::Scalar(_s) => response = Option::None,
            Goal::Vector(v) => response = Option::Some(vector_to_vec(v)),
            Goal::Quaternion(_q) => response = Option::None,
            Goal::None => response = Option::None,
        }
        return Ok(response);
    }

    #[setter(vector)]
    fn set_vector(&mut self, vector: Vec<f64>) -> PyResult<()> {
        self.value = Goal::Vector(vec_to_vector(vector));
        return Ok(());
    }

    #[getter(quaternion)]
    fn get_quaternion(&self) -> PyResult<Option<Vec<f64>>> {
        let response: Option<Vec<f64>>;
        match self.value {
            Goal::Scalar(_s) => response = Option::None,
            Goal::Vector(_v) => response = Option::None,
            Goal::Quaternion(q) => response = Option::Some(quat_to_vec(q)),
            Goal::None => response = Option::None,
        }
        return Ok(response);
    }

    #[setter(quaternion)]
    fn set_quaternion(&mut self, quaternion: Vec<f64>) -> PyResult<()> {
        self.value = Goal::Quaternion(vec_to_quat(quaternion));
        return Ok(());
    }

    fn clear(&mut self) -> PyResult<()> {
        self.value = Goal::None;
        return Ok(());
    }

    fn __str__(&self) -> PyResult<String> {
        let mut response: String = String::from("<null>");
        match self.value {
            Goal::Scalar(s) => response = format!("<GoalSpec scalar: {:?}>", s),
            Goal::Vector(v) => {
                response = format!("<GoalSpec vector: {:?},{:?},{:?}>", v.x, v.y, v.z)
            }
            Goal::Quaternion(q) => {
                response = format!(
                    "<GoalSpec quaternion: {:?},{:?},{:?},{:?}>",
                    q[0], q[1], q[2], q[3]
                )
            }
            Goal::None => {}
        }
        return Ok(response);
    }

    fn __repr__(&self) -> PyResult<String> {
        return self.__str__();
    }
}

#[pyclass]
#[derive(Clone, Copy, Debug)]
pub struct ObjectiveInput {
    #[pyo3(get, set)]
    pub weight: f64,
    pub value: Goal,
}

#[pymethods]
impl ObjectiveInput {
    #[new]
    fn new(
        weight: f64,
        scalar: Option<f64>,
        vector: Option<Vec<f64>>,
        quaternion: Option<Vec<f64>>,
    ) -> Self {
        let mut value: Goal = Goal::None;
        match scalar {
            Some(s) => value = Goal::Scalar(s),
            None => {}
        };
        match vector {
            Some(v) => value = Goal::Vector(vec_to_vector(v)),
            None => {}
        };
        match quaternion {
            Some(q) => value = Goal::Quaternion(vec_to_quat(q)),
            None => {}
        };
        return Self { weight, value };
    }

    #[getter(scalar)]
    fn get_scalar(&self) -> PyResult<Option<f64>> {
        let response: Option<f64>;
        match self.value {
            Goal::Scalar(s) => response = Option::Some(s),
            Goal::Vector(_v) => response = Option::None,
            Goal::Quaternion(_q) => response = Option::None,
            Goal::None => response = Option::None,
        }
        return Ok(response);
    }

    #[setter(scalar)]
    fn set_scalar(&mut self, scalar: f64) -> PyResult<()> {
        self.value = Goal::Scalar(scalar);
        return Ok(());
    }

    #[getter(vector)]
    fn get_vector(&self) -> PyResult<Option<Vec<f64>>> {
        let response: Option<Vec<f64>>;
        match self.value {
            Goal::Scalar(_s) => response = Option::None,
            Goal::Vector(v) => response = Option::Some(vector_to_vec(v)),
            Goal::Quaternion(_q) => response = Option::None,
            Goal::None => response = Option::None,
        }
        return Ok(response);
    }

    #[setter(vector)]
    fn set_vector(&mut self, vector: Vec<f64>) -> PyResult<()> {
        self.value = Goal::Vector(vec_to_vector(vector));
        return Ok(());
    }

    #[getter(quaternion)]
    fn get_quaternion(&self) -> PyResult<Option<Vec<f64>>> {
        let response: Option<Vec<f64>>;
        match self.value {
            Goal::Scalar(_s) => response = Option::None,
            Goal::Vector(_v) => response = Option::None,
            Goal::Quaternion(q) => response = Option::Some(quat_to_vec(q)),
            Goal::None => response = Option::None,
        }
        return Ok(response);
    }

    #[setter(quaternion)]
    fn set_quaternion(&mut self, quaternion: Vec<f64>) -> PyResult<()> {
        self.value = Goal::Quaternion(vec_to_quat(quaternion));
        return Ok(());
    }

    fn clear(&mut self) -> PyResult<()> {
        self.value = Goal::None;
        return Ok(());
    }

    fn __str__(&self) -> PyResult<String> {
        let mut response: String = String::from("<null>");
        let weight: String = format!("{:?}", self.weight);
        match self.value {
            Goal::Scalar(s) => {
                response = format!("<ObjectiveInput weight: {:?}, scalar: {:?}>", weight, s)
            }
            Goal::Vector(v) => {
                response = format!(
                    "<ObjectiveInput weight: {:?}, vector: {:?},{:?},{:?}>",
                    weight, v.x, v.y, v.z
                )
            }
            Goal::Quaternion(q) => {
                response = format!(
                    "<ObjectiveInput weight: {:?}, quaternion: {:?},{:?},{:?},{:?}>",
                    weight, q[0], q[1], q[2], q[3]
                )
            }
            Goal::None => {}
        }
        return Ok(response);
    }

    fn __repr__(&self) -> PyResult<String> {
        return self.__str__();
    }
}
