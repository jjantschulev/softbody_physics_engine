use nannou::glam::Vec2;

pub trait PointCollision {
    fn contains_point(&self, p: Vec2) -> bool;
    fn closest_point_on_edge(&self, p: Vec2) -> Option<Vec2>;
}
