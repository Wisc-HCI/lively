
pub enum Transition {
    Timed(f64),
    Event(String)
}
pub struct Edge {
    pub parent: Node,
    pub child: Node
}