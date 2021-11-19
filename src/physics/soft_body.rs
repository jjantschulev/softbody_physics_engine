use nannou::glam::{IVec2, Vec2};

use super::{
    bounding_box::{BoundingBox, HasBoundingBox},
    collision_detection::PointCollision,
    engine::PhysicsEngineConfig,
    fixed_body::FixedBody,
};

pub struct SoftBody {
    points: Vec<MassPoint>,
    connections: Vec<(usize, usize, Spring)>,
    repel_force: f32,
    repel_distance: f32,
}

impl SoftBody {
    pub fn new_rect(
        pos: Vec2,
        size: Vec2,
        mass: f32,
        stiffness: f32,
        damping: f32,
        repel_force: f32,
        point_distance: f32,
    ) -> SoftBody {
        let mut body = SoftBody {
            points: vec![],
            connections: vec![],
            repel_force,
            repel_distance: point_distance,
        };

        let num_points = IVec2::new(
            (size.x / point_distance).round() as i32,
            (size.y / point_distance).round() as i32,
        );

        let points_dist = Vec2::new(size.x / num_points.x as f32, size.y / num_points.y as f32);

        for y in 0..num_points.y {
            for x in 0..num_points.x {
                let p = Vec2::new(x as f32, y as f32) * points_dist + pos;
                body.points.push(MassPoint::new(mass, p));
            }
        }

        let get_index = |x, y| (y * num_points.x + x) as usize;

        for x in 0..num_points.x - 1 {
            for y in 0..num_points.y - 1 {
                body.add_connection(
                    get_index(x, y),
                    get_index(x + 1, y),
                    Spring {
                        length: points_dist.x,
                        stiffness,
                        damping,
                    },
                );
                body.add_connection(
                    get_index(x, y),
                    get_index(x, y + 1),
                    Spring {
                        length: points_dist.y,
                        stiffness,
                        damping,
                    },
                );
                body.add_connection(
                    get_index(x, y),
                    get_index(x + 1, y + 1),
                    Spring {
                        length: points_dist.length(),
                        stiffness,
                        damping,
                    },
                );
                body.add_connection(
                    get_index(x + 1, y),
                    get_index(x, y + 1),
                    Spring {
                        length: points_dist.length(),
                        stiffness,
                        damping,
                    },
                );
            }
        }

        for x in 0..num_points.x - 1 {
            body.add_connection(
                get_index(x, num_points.y - 1),
                get_index(x + 1, num_points.y - 1),
                Spring {
                    length: points_dist.x,
                    stiffness,
                    damping,
                },
            )
        }

        for y in 0..num_points.y - 1 {
            body.add_connection(
                get_index(num_points.x - 1, y),
                get_index(num_points.x - 1, y + 1),
                Spring {
                    length: points_dist.y,
                    stiffness,
                    damping,
                },
            )
        }

        body
    }

    pub fn update(
        &mut self,
        delta: f32,
        colliders: &[FixedBody],
        other_soft_mass_points: &[(usize, BoundingBox, Vec<(usize, Vec2)>)],
        my_index: usize,
        config: &PhysicsEngineConfig,
    ) {
        for (a, b, spring) in self.connections.iter() {
            let force = spring.calculate_force(&self.points[*a], &self.points[*b]);
            self.points[*a].add_force(force);
            self.points[*b].add_force(-force);
        }

        for collider in colliders {
            if self
                .get_bounding_box()
                .overlapping(&collider.get_bounding_box())
            {
                for mass in self.points.iter_mut() {
                    mass.handle_collision(collider);
                }
            }
        }

        let my_bb = self.get_bounding_box();
        for (i, mass) in self.points.iter_mut().enumerate() {
            for (j, bb, points) in other_soft_mass_points {
                if bb.overlapping(&my_bb) {
                    for (k, other) in points {
                        if !(i == *k && my_index == *j) {
                            let dir = mass.pos - *other;
                            let len = dir.length();
                            if len < self.repel_distance {
                                mass.add_force(
                                    ((self.repel_distance - len) * dir) * self.repel_force,
                                );
                            }
                        }
                    }
                }
            }
        }

        for mass in self.points.iter_mut() {
            mass.update(delta, config);
        }
    }

    pub fn add_connection(&mut self, a: usize, b: usize, spring: Spring) {
        self.connections.push((a, b, spring))
    }

    /// Get a reference to the soft body's points.
    pub fn points(&self) -> &[MassPoint] {
        self.points.as_ref()
    }

    /// Get a reference to the soft body's connections.
    pub fn connections(&self) -> &[(usize, usize, Spring)] {
        self.connections.as_ref()
    }

    /// Get a mutable reference to the soft body's points.
    pub fn points_mut(&mut self) -> &mut Vec<MassPoint> {
        &mut self.points
    }
}

impl HasBoundingBox for SoftBody {
    fn get_bounding_box(&self) -> BoundingBox {
        let mut min = Vec2::new(f32::MAX, f32::MAX);
        let mut max = Vec2::new(f32::MIN, f32::MIN);
        for point in self.points.iter() {
            min = Vec2::min(min, point.pos);
            max = Vec2::max(max, point.pos);
        }
        let offset = Vec2::new(self.repel_distance / 2.0, self.repel_distance / 2.0);
        BoundingBox {
            start: min - offset,
            stop: max + offset,
        }
    }
}

pub struct MassPoint {
    mass: f32,
    pos: Vec2,
    vel: Vec2,
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

    fn update(&mut self, delta: f32, config: &PhysicsEngineConfig) {
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

    fn handle_collision(&mut self, collider: &FixedBody) {
        if let Some(point) = collider.closest_point_on_edge(self.pos) {
            let normal = (point - self.pos).normalize_or_zero();
            self.pos = point;
            self.vel = (self.vel - 2.0 * (self.vel.dot(normal)) * normal) * 0.99;
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
