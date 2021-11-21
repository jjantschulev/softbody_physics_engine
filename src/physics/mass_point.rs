use nannou::glam::Vec2;

use super::{
    collision_detection::PointCollision, engine::PhysicsEngineConfig, fixed_body::FixedBody,
};

pub struct MassPoint {
    mass: f32,
    pub(crate) pos: Vec2,
    pub(crate) vel: Vec2,
    force: Vec2,
}

impl MassPoint {
    pub fn new(mass: f32, pos: Vec2) -> Self {
        Self {
            mass,
            pos,
            vel: Vec2::ZERO,
            force: Vec2::ZERO,
        }
    }

    pub(crate) fn update(&mut self, delta: f32, config: &PhysicsEngineConfig) {
        self.force += config.gravity * self.mass;
        self.force += -self.vel.normalize_or_zero()
            * self.vel.length()
            // * self.vel.length()
            * config.drag_coefficient;

        let acceleration = self.force / self.mass;
        self.force = Vec2::ZERO;
        self.vel += acceleration * delta;
        self.pos += self.vel * delta;
    }

    pub(crate) fn handle_collision(&mut self, collider: &FixedBody, config: &PhysicsEngineConfig) {
        if let Some(point) = collider.closest_point_on_edge(self.pos) {
            let normal = (point - self.pos).normalize_or_zero();
            self.pos = point;
            self.vel = (self.vel - 2.0 * (self.vel.dot(normal)) * normal) * config.friction;
        }
    }

    pub fn add_force(&mut self, force: Vec2) {
        self.force += force;
    }

    /// Get a reference to the mass point's pos.
    pub fn pos(&self) -> Vec2 {
        self.pos
    }

    /// Set the mass point's pos.
    pub fn set_pos(&mut self, pos: Vec2) {
        self.pos = pos;
    }
}
