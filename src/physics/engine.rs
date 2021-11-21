use itertools::Itertools;
use nannou::glam::Vec2;

use super::{
    bounding_box::{BoundingBox, HasBoundingBox},
    fixed_body::FixedBody,
    pressure_body::PressureBody,
    rigid_body::RigidBody,
    soft_body::SoftBody,
};

#[derive(PartialEq, Clone, Copy)]
pub enum MassPointBodyType {
    Soft,
    Pressure,
}

pub type GroupedMassPoints = [(MassPointBodyType, usize, BoundingBox, Vec<(usize, Vec2)>)];

pub struct PhysicsEngine {
    world: World,
    config: PhysicsEngineConfig,
}

pub struct PhysicsEngineConfig {
    pub gravity: Vec2,
    pub drag_coefficient: f32,
    pub friction: f32,
}

impl PhysicsEngine {
    pub fn new(config: PhysicsEngineConfig) -> PhysicsEngine {
        PhysicsEngine {
            world: World::new(),
            config,
        }
    }
    pub fn update(&mut self, delta: f32) {
        let mut all_points = self
            .world
            .soft_bodies
            .iter()
            .enumerate()
            .map(|(i, b)| {
                (
                    MassPointBodyType::Soft,
                    i,
                    b.get_bounding_box(),
                    b.points()
                        .iter()
                        .enumerate()
                        .map(|(j, p)| (j, p.pos()))
                        .collect_vec(),
                )
            })
            .collect_vec();

        all_points.extend(
            self.world
                .pressure_bodies
                .iter()
                .enumerate()
                .map(|(i, b)| {
                    (
                        MassPointBodyType::Pressure,
                        i,
                        b.get_bounding_box(),
                        b.points()
                            .iter()
                            .enumerate()
                            .map(|(j, p)| (j, p.pos()))
                            .collect_vec(),
                    )
                })
                .collect_vec(),
        );
        for (i, soft_body) in self.world.soft_bodies.iter_mut().enumerate() {
            soft_body.update(
                delta,
                &self.world.fixed_bodies,
                &all_points,
                i,
                &self.config,
            );
        }

        for (i, pressure_body) in self.world.pressure_bodies.iter_mut().enumerate() {
            pressure_body.update(
                delta,
                &self.config,
                &self.world.fixed_bodies,
                &all_points,
                i,
            );
        }
    }

    /// Get a reference to the physics engine's world.
    pub fn world(&self) -> &World {
        &self.world
    }

    /// Get a mutable reference to the physics engine's world.
    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }
}

pub struct World {
    fixed_bodies: Vec<FixedBody>,
    soft_bodies: Vec<SoftBody>,
    rigid_bodies: Vec<RigidBody>,
    pressure_bodies: Vec<PressureBody>,
}

impl World {
    pub fn new() -> Self {
        Self {
            fixed_bodies: vec![],
            soft_bodies: vec![],
            rigid_bodies: vec![],
            pressure_bodies: vec![],
        }
    }

    pub fn add_entity(&mut self, entity: Entity) -> &mut World {
        match entity {
            Entity::Fixed(e) => self.fixed_bodies.push(e),
            Entity::Soft(e) => self.soft_bodies.push(e),
            Entity::Rigid(e) => self.rigid_bodies.push(e),
            Entity::Pressure(e) => self.pressure_bodies.push(e),
        };
        self
    }

    /// Get a reference to the world's fixed bodies.
    pub fn fixed_bodies(&self) -> &[FixedBody] {
        self.fixed_bodies.as_ref()
    }

    /// Get a reference to the world's soft bodies.
    pub fn soft_bodies(&self) -> &[SoftBody] {
        self.soft_bodies.as_ref()
    }

    /// Get a mutable reference to the world's soft bodies.
    pub fn soft_bodies_mut(&mut self) -> &mut Vec<SoftBody> {
        &mut self.soft_bodies
    }

    /// Get a reference to the world's pressure bodies.
    pub fn pressure_bodies(&self) -> &[PressureBody] {
        self.pressure_bodies.as_ref()
    }

    /// Get a mutable reference to the world's pressure bodies.
    pub fn pressure_bodies_mut(&mut self) -> &mut Vec<PressureBody> {
        &mut self.pressure_bodies
    }
}

pub enum Entity {
    Fixed(FixedBody),
    Soft(SoftBody),
    Rigid(RigidBody),
    Pressure(PressureBody),
}
