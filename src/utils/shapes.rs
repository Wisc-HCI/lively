use nalgebra::geometry::{Isometry3};

#[derive(Clone,Debug,PartialEq)]
pub struct BoxShape {
    pub frame: String,
    pub name: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub local_transform: Isometry3<f64>
    // handle: Whatever Parry3D/Rapier handle is needed
}

impl BoxShape {
    pub fn new(name: String, frame:String, x: f64, y: f64, z: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, x, y, z, local_transform}
    }
}

#[derive(Clone,Debug,PartialEq)]
pub struct CylinderShape {
    pub frame: String,
    pub name: String,
    pub length: f64,
    pub radius: f64,
    pub local_transform: Isometry3<f64>,
    // handle: Whatever Parry3D/Rapier handle is needed 
}

impl CylinderShape {
    pub fn new(name: String, frame:String, length: f64, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, length, radius, local_transform}
    }
}

#[derive(Clone,Debug,PartialEq)]
pub struct CapsuleShape {
    pub frame: String,
    pub name: String,
    pub length: f64,
    pub radius: f64,
    pub local_transform: Isometry3<f64>,
    // handle: Whatever Parry3D/Rapier handle is needed 
}

impl CapsuleShape {
    pub fn new(name: String, frame:String, length: f64, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, length, radius, local_transform}
    }
}

#[derive(Clone,Debug,PartialEq)]
pub struct SphereShape {
    pub frame: String,
    pub name: String,
    pub radius: f64,
    pub local_transform: Isometry3<f64>
    // handle: Whatever Parry3D/Rapier handle is needed 
}

impl SphereShape {
    pub fn new(name: String, frame:String, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, radius, local_transform}
    }
}

#[derive(Clone,Debug,PartialEq)]
pub struct BoxZone {
    pub frame: String,
    pub name: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub local_transform: Isometry3<f64>
    // handle: Whatever Parry3D/Rapier handle is needed
}

impl BoxZone {
    pub fn new(name: String, frame:String, x: f64, y: f64, z: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, x, y, z, local_transform}
    }
}

#[derive(Clone,Debug,PartialEq)]
pub struct CylinderZone {
    pub frame: String,
    pub name: String,
    pub length: f64,
    pub radius: f64,
    pub local_transform: Isometry3<f64>,
    // handle: Whatever Parry3D/Rapier handle is needed 
}

impl CylinderZone {
    pub fn new(name: String, frame:String, length: f64, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, length, radius, local_transform}
    }
}

#[derive(Clone,Debug,PartialEq)]
pub struct CapsuleZone {
    pub frame: String,
    pub name: String,
    pub length: f64,
    pub radius: f64,
    pub local_transform: Isometry3<f64>,
    // handle: Whatever Parry3D/Rapier handle is needed 
}

impl CapsuleZone {
    pub fn new(name: String, frame:String, length: f64, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, length, radius, local_transform}
    }
}

#[derive(Clone,Debug,PartialEq)]
pub struct SphereZone {
    pub frame: String,
    pub name: String,
    pub radius: f64,
    pub local_transform: Isometry3<f64>
    // handle: Whatever Parry3D/Rapier handle is needed 
}

impl SphereZone {
    pub fn new(name: String, frame:String, radius: f64, local_transform: Isometry3<f64>) -> Self {
        // TODO: add/create shape and handle
        Self {name, frame, radius, local_transform}
    }
}

#[derive(Clone,Debug,PartialEq)]
pub enum Shape {
    Box(BoxShape),
    Cylinder(CylinderShape),
    Sphere(SphereShape),
    Capsule(CapsuleShape)
}



#[derive(Clone,Debug,PartialEq)]
pub enum Zone {
    Box(BoxZone),
    Cylinder(CylinderZone),
    Sphere(SphereZone),
    Capsule(CapsuleZone)
}