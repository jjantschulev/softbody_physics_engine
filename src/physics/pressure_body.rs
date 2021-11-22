use itertools::Itertools;
use nannou::glam::{Mat2, Vec2};

use super::{
    bounding_box::{BoundingBox, HasBoundingBox},
    engine::{GroupedMassPoints, MassPointBodyType, PhysicsEngineConfig},
    fixed_body::FixedBody,
    mass_point::MassPoint,
    spring::Spring,
};

pub struct PressureBody {
    points: Vec<MassPoint>,
    spring: Spring,
    pressure_constant: f32,
    repel_force: f32,
    repel_distance: f32,
    original_area: f32,
}

impl PressureBody {
    pub fn new_circle(
        pos: Vec2,
        radius: f32,
        num_points: usize,
        pressure_constant: f32,
        mass: f32,
        damping: f32,
        repel_force: f32,
    ) -> PressureBody {
        let segment_len = (std::f32::consts::PI * radius * 2.0) / (num_points as f32);
        let mut points = vec![];
        for i in 0..num_points {
            let m = Mat2::from_angle(i as f32 / num_points as f32 * 2.0 * std::f32::consts::PI);
            let p = pos + m * (Vec2::new(radius, 0.0));
            points.push(MassPoint::new(mass, p));
        }
        let d_pos = points[0].pos - points[1].pos;
        let point_pressure = 2.0 * d_pos.y * pressure_constant;
        let stiffness = -point_pressure / (2.0 * d_pos.x);
        let mut body = PressureBody {
            points,
            pressure_constant,
            spring: Spring::new(0.0, stiffness, damping),
            repel_force,
            repel_distance: segment_len,
            original_area: 0.0,
        };
        body.original_area = body.area();
        body
    }

    pub fn update(
        &mut self,
        delta: f32,
        config: &PhysicsEngineConfig,
        colliders: &[FixedBody],
        other_mass_points: &GroupedMassPoints,
        my_index: usize,
    ) {
        // Update Springs
        for a in 0..self.points.len() {
            let b = (a + 1) % self.points.len();
            let force = self
                .spring
                .calculate_force(&self.points[a], &self.points[b]);
            self.points[a].add_force(force);
            self.points[b].add_force(-force);
        }

        // Apply inner pressure;
        let pressure_force = self.original_area * self.pressure_constant / self.area();
        for i in 0..self.points.len() {
            let next = (i as isize + 1).rem_euclid(self.points.len() as isize) as usize;
            let prev = (i as isize - 1).rem_euclid(self.points.len() as isize) as usize;
            let v1 = self.points[prev].pos - self.points[i].pos;
            let v2 = self.points[i].pos - self.points[next].pos;
            for v in [v1, v2] {
                let normal = Vec2::new(-v.y, v.x);
                self.points[i].add_force(normal * pressure_force)
            }
        }

        for collider in colliders {
            if self
                .get_bounding_box()
                .overlapping(&collider.get_bounding_box())
            {
                for mass in self.points.iter_mut() {
                    mass.handle_collision(collider, config);
                }
            }
        }

        let my_bb = self.get_bounding_box();
        for (i, mass) in self.points.iter_mut().enumerate() {
            for (t, j, bb, points) in other_mass_points {
                if bb.overlapping(&my_bb) {
                    for (k, other) in points {
                        if !(i == *k && my_index == *j && t == &MassPointBodyType::Pressure) {
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

    fn area(&self) -> f32 {
        let mut area = 0.0;
        for (a, b) in self.points.iter().circular_tuple_windows() {
            area += a.pos.x * b.pos.y - b.pos.x * a.pos.y;
        }
        area / 2.0
    }

    pub fn center_of_mass(&self) -> Vec2 {
        let mut center = Vec2::ZERO;
        for point in self.points.iter() {
            center += point.pos;
        }
        center / self.points.len() as f32
    }

    /// Get a reference to the pressure body's points.
    pub fn points(&self) -> &[MassPoint] {
        self.points.as_ref()
    }

    /// Get a mutable reference to the pressure body's points.
    pub fn points_mut(&mut self) -> &mut Vec<MassPoint> {
        &mut self.points
    }
}

impl HasBoundingBox for PressureBody {
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
