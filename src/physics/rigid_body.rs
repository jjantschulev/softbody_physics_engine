use nannou::glam::Vec2;

use super::{
    bounding_box::{BoundingBox, HasBoundingBox},
    collision_detection::PointCollision,
    engine::PhysicsEngineConfig,
    fixed_body::FixedBody,
};

pub struct RigidBody {
    pos: Vec2,
    rot: f32,
    vel: Vec2,
    force: Vec2,
    mass: Vec2,
    shape: FixedBody,
}

impl RigidBody {
    fn update(
        &mut self,
        delta: f32,
        fixed_bodies: &[FixedBody],
        rigid_bodies: &mut [RigidBody],
        config: &PhysicsEngineConfig,
    ) {
        self.force += config.gravity * self.mass;

        let acceleration = self.force / self.mass;
        self.force = Vec2::ZERO;
        self.vel += acceleration / delta;
        self.pos += self.vel / delta;
    }
}

impl PointCollision for RigidBody {
    fn contains_point(&self, p: Vec2) -> bool {
        self.shape.contains_point(p - self.pos)
    }

    fn closest_point_on_edge(&self, point: Vec2) -> Option<Vec2> {
        self.shape.closest_point_on_edge(point - self.pos)
    }
}

impl HasBoundingBox for RigidBody {
    fn get_bounding_box(&self) -> BoundingBox {
        self.shape.get_bounding_box().offset(self.pos)
    }
}
