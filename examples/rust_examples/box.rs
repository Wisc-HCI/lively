use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use core::fmt::Debug;

#[derive(Debug,Clone,Serialize,Deserialize)]
enum GoalEnum {
    X(f64),
    Y(String)
}

#[derive(Debug,Serialize,Deserialize)]
struct ObjectiveA {	
    pub value: f64
}
#[derive(Debug,Serialize,Deserialize)]
struct ObjectiveB {	
    pub value: String
}

trait Objective:  {
    // Update function. By default does nothing
    fn update(&self, _time: f64) {}

    // Goal setter function
    fn set_goal(&mut self, goal: &GoalEnum);

    // Define a method on the caller type which takes an
    // additional single parameter `T` and does nothing with it.
    fn call(&self, _state: bool) -> f64 {
        return 0.0
    }
}

impl Debug for dyn Objective {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self.call(true))
    }
}

impl Objective for ObjectiveA {
    fn set_goal(&mut self, goal: &GoalEnum) {
        match goal {
            GoalEnum::X(value) => self.value = *value,
            _ => {}
        };
    }

    fn call(&self, _state: bool) -> f64 {
        return self.value
    }
}

impl Objective for ObjectiveB {
    fn set_goal(&mut self, goal: &GoalEnum) {
        match goal {
            GoalEnum::Y(value) => self.value = value.into(),
            _ => {}
        };
    }

    fn call(&self, _state: bool) -> f64 {
        if self.value == "World".to_string() {
            return 0.0;
        } else {
            return 1.0;
        }
    }
}

fn main() {

    let mut obj_collection: HashMap<String,Box<dyn Objective>> = HashMap::new();
    obj_collection.insert("a".into(), Box::new(ObjectiveA{value:30.0}));
    obj_collection.insert("b".into(), Box::new(ObjectiveB{value:"Hello".into()}));

    let mut goal_collection: HashMap<String,GoalEnum> = HashMap::new();
    goal_collection.insert("a".into(),GoalEnum::X(20.0));
    goal_collection.insert("b".into(),GoalEnum::Y("World".into()));

    for (key,obj) in &mut obj_collection {
        match goal_collection.get(key) {
            Some(goal) => obj.set_goal(goal),
            _ => {}
        }
        println!("{:?}",obj);
    }

    // println!("{:?}",obj_collection);
    // let mut block_b = ObjectiveB{value:"Hello".into()};
    // block_a.set_goal(GoalEnum::X(20.0));
    // block_b.set_goal(GoalEnum::Y("World".into()));
    // // let mut block_a = Block::A(ObjectiveA{value:30.0});
    // // let mut block_b = Block::B(BlockB{value:"Hello".into()});
    // // block_a.common_fn(true,InputEnum::X(40.0));
    // // block_b.common_fn(true,InputEnum::Y("World".into()));
    // // println!("Block A {:?} {:?}",block_a,block_a.call(true));
    // println!("Block B {:?} {:?}",block_b,block_b.call(true));
    // // println!("New Block B {:?}",block_b);
}