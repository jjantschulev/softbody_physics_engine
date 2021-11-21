use nannou::glam::Vec2;

use super::mass_point::MassPoint;

pub struct Spring {
    length: f32,
    stiffness: f32,
    damping: f32,
}

impl Spring {
    pub fn new(length: f32, stiffness: f32, damping: f32) -> Self {
        Self {
            length,
            stiffness,
            damping,
        }
    }

    pub fn calculate_force(&self, a: &MassPoint, b: &MassPoint) -> Vec2 {
        let dist = a.pos.distance(b.pos);
        let force = self.stiffness * (dist - self.length);
        let dir = (b.pos - a.pos).normalize_or_zero();
        let movement_difference = b.vel - a.vel;
        let damping = self.damping * dir.dot(movement_difference);
        dir * (force + damping)
    }
}
