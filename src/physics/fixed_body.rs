use itertools::Itertools;
use nannou::glam::Vec2;

use super::bounding_box::{BoundingBox, HasBoundingBox};
use super::collision_detection::PointCollision;
use super::utils::{axis_aligned_line_segments_overlap, closest_point_on_line, f32max, f32min};

pub enum FixedBody {
    Polygon { verts: Vec<Vec2> },
    Circle { center: Vec2, radius: f32 },
    Rect { pos: Vec2, size: Vec2 },
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
                let p1 = Vec2::new(bb.start().x - 1.0, p.y);
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
                let p1 = Vec2::new(bb.start().x - 1.0, point.y);
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
