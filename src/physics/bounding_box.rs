use nannou::glam::Vec2;

use super::collision_detection::PointCollision;

pub struct BoundingBox {
    pub(super) start: Vec2,
    pub(super) stop: Vec2,
}

pub trait HasBoundingBox {
    fn get_bounding_box(&self) -> BoundingBox;
}

impl BoundingBox {
    pub fn overlapping(&self, other: &BoundingBox) -> bool {
        !(other.stop.x < self.start.x
            || other.start.x > self.stop.x
            || other.stop.y < self.start.y
            || other.start.y > self.stop.y)
    }

    pub fn offset(&self, amt: Vec2) -> BoundingBox {
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
