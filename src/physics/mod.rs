use itertools::Itertools;
use nannou::glam::{IVec2, Vec2};

use self::utils::{axis_aligned_line_segments_overlap, closest_point_on_line, f32max, f32min};

pub mod utils;

pub struct PhysicsEngine {
    world: World,
    config: PhysicsEngineConfig,
}

pub struct PhysicsEngineConfig {
    pub gravity: Vec2,
    pub drag_coefficient: f32,
}

impl PhysicsEngine {
    pub fn new(config: PhysicsEngineConfig) -> PhysicsEngine {
        PhysicsEngine {
            world: World::new(),
            config,
        }
    }
    pub fn update(&mut self, delta: f32) {
        let all_points = self
            .world
            .soft_bodies
            .iter()
            .enumerate()
            .map(|(i, b)| {
                (
                    i,
                    b.get_bounding_box(),
                    b.points
                        .iter()
                        .enumerate()
                        .map(|(j, p)| (j, p.pos))
                        .collect_vec(),
                )
            })
            .collect_vec();
        for (i, soft_body) in self.world.soft_bodies.iter_mut().enumerate() {
            soft_body.update(
                delta,
                &self.world.fixed_bodies,
                &all_points,
                i,
                &self.config,
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
}

impl World {
    pub fn new() -> Self {
        Self {
            fixed_bodies: vec![],
            soft_bodies: vec![],
            rigid_bodies: vec![],
        }
    }

    pub fn add_entity(&mut self, entity: Entity) -> &mut World {
        match entity {
            Entity::Fixed(e) => self.fixed_bodies.push(e),
            Entity::Soft(e) => self.soft_bodies.push(e),
            Entity::Rigid(e) => self.rigid_bodies.push(e),
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
}

pub enum Entity {
    Fixed(FixedBody),
    Soft(SoftBody),
    Rigid(RigidBody),
}

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

    fn update(
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

pub enum FixedBody {
    Polygon { verts: Vec<Vec2> },
    Circle { center: Vec2, radius: f32 },
    Rect { pos: Vec2, size: Vec2 },
}

pub struct BoundingBox {
    start: Vec2,
    stop: Vec2,
}

pub trait PointCollision {
    fn contains_point(&self, p: Vec2) -> bool;
    fn closest_point_on_edge(&self, p: Vec2) -> Option<Vec2>;
}

pub trait HasBoundingBox {
    fn get_bounding_box(&self) -> BoundingBox;
}

impl BoundingBox {
    fn overlapping(&self, other: &BoundingBox) -> bool {
        !(other.stop.x < self.start.x
            || other.start.x > self.stop.x
            || other.stop.y < self.start.y
            || other.start.y > self.stop.y)
    }

    fn offset(&self, amt: Vec2) -> BoundingBox {
        BoundingBox {
            start: self.start + amt,
            stop: self.stop + amt,
        }
    }

    /// Get a reference to the bounding box's start.
    pub fn start(&self) -> Vec2 {
        self.start
    }

    /// Get a reference to the bounding box's stop.
    pub fn stop(&self) -> Vec2 {
        self.stop
    }
}

impl PointCollision for BoundingBox {
    fn contains_point(&self, p: Vec2) -> bool {
        p.x >= self.start.x && p.y >= self.start.y && p.x <= self.stop.x && p.y <= self.stop.y
    }

    fn closest_point_on_edge(&self, p: Vec2) -> Option<Vec2> {
        if !self.contains_point(p) {
            return None;
        }
        if p.x <= (self.start.x + self.stop.x) / 2.0 {
            if p.y <= (self.start.y + self.stop.y) / 2.0 {
                let dx = p.x - self.start.x;
                let dy = p.y - self.start.y;
                if dx <= dy {
                    Some(Vec2::new(self.start.x, p.y))
                } else {
                    Some(Vec2::new(p.x, self.start.y))
                }
            } else {
                let dx = p.x - self.start.x;
                let dy = self.stop.y - p.y;
                if dx <= dy {
                    Some(Vec2::new(self.start.x, p.y))
                } else {
                    Some(Vec2::new(p.x, self.stop.y))
                }
            }
        } else {
            if p.y <= (self.start.y + self.stop.y) / 2.0 {
                let dx = self.stop.x - p.x;
                let dy = p.y - self.start.y;
                if dx <= dy {
                    Some(Vec2::new(self.stop.x, p.y))
                } else {
                    Some(Vec2::new(p.x, self.start.y))
                }
            } else {
                let dx = self.stop.x - p.x;
                let dy = self.stop.y - p.y;
                if dx <= dy {
                    Some(Vec2::new(self.stop.x, p.y))
                } else {
                    Some(Vec2::new(p.x, self.stop.y))
                }
            }
        }
    }
}

impl HasBoundingBox for FixedBody {
    fn get_bounding_box(&self) -> BoundingBox {
        match self {
            FixedBody::Polygon { verts } => {
                let mut b = BoundingBox {
                    start: Vec2::new(f32::INFINITY, f32::INFINITY),
                    stop: Vec2::new(f32::NEG_INFINITY, f32::NEG_INFINITY),
                };

                for v in verts {
                    b.start = b.start.min(*v);
                    b.stop = b.stop.max(*v);
                }

                b
            }
            FixedBody::Circle { center, radius } => BoundingBox {
                start: Vec2::new(
                    f32min(center.x - radius, center.x + radius),
                    f32min(center.y - radius, center.y + radius),
                ),
                stop: Vec2::new(
                    f32max(center.x - radius, center.x + radius),
                    f32max(center.y - radius, center.y + radius),
                ),
            },
            FixedBody::Rect { pos, size } => BoundingBox {
                start: Vec2::new(f32min(pos.x, pos.x + size.x), f32min(pos.y, pos.y + size.y)),
                stop: Vec2::new(f32max(pos.x, pos.x + size.x), f32max(pos.y, pos.y + size.y)),
            },
        }
    }
}

impl PointCollision for FixedBody {
    fn contains_point(&self, p: Vec2) -> bool {
        let bb = self.get_bounding_box();
        if !bb.contains_point(p) {
            return false;
        }
        match self {
            FixedBody::Polygon { verts } => {
                let mut num_collisions = 0;
                let p1 = Vec2::new(bb.start.x - 1.0, p.y);
                for (v1, v2) in verts.iter().circular_tuple_windows() {
                    if axis_aligned_line_segments_overlap(p1, p.x - p1.x, *v1, *v2).is_some() {
                        num_collisions += 1;
                    }
                }
                num_collisions % 2 == 1
            }
            FixedBody::Circle { center, radius } => {
                let diff = p - *center;
                diff.length() <= *radius
            }
            FixedBody::Rect { pos: _, size: _ } => true,
        }
    }

    fn closest_point_on_edge(&self, point: Vec2) -> Option<Vec2> {
        let bb = self.get_bounding_box();
        if !bb.contains_point(point) {
            return None;
        }
        match self {
            FixedBody::Polygon { verts } => {
                let mut closest_point = Vec2::ZERO;
                let mut closest_distance = f32::MAX;
                let mut num_collisions = 0;
                let p1 = Vec2::new(bb.start.x - 1.0, point.y);
                for (v1, v2) in verts.iter().circular_tuple_windows() {
                    if axis_aligned_line_segments_overlap(p1, point.x - p1.x, *v1, *v2).is_some() {
                        num_collisions += 1;
                    }
                    let cp = closest_point_on_line(point, *v1, *v2);
                    let dist = cp.distance_squared(point);
                    if dist < closest_distance {
                        closest_distance = dist;
                        closest_point = cp;
                    }
                }

                if num_collisions % 2 == 1 {
                    Some(closest_point)
                } else {
                    None
                }
            }
            FixedBody::Circle { center, radius } => {
                let diff = point - *center;
                if diff.length() <= *radius {
                    Some(diff.normalize() * *radius)
                } else {
                    None
                }
            }
            FixedBody::Rect { pos: _, size: _ } => bb.closest_point_on_edge(point),
        }
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
            self.vel = (self.vel - 2.0 * (self.vel.dot(normal)) * normal) * 0.9;
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

// impl FixedBody {
//     fn transform(&self, pos: Vec2, rotation: f32) -> FixedBody {
//         match self {
//             FixedBody::Polygon { verts } => ,
//             FixedBody::Circle { center, radius } => todo!(),
//             FixedBody::Rect { pos, size } => todo!(),
//         }
//     }
// }
