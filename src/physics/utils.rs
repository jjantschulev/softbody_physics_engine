use nannou::{
    glam::{Mat2, Vec2},
    math::clamp,
};
use std::ops::{Add, Div, Mul, Sub};

/// Given two line segments `p1` -> `q1` and `p2` -> `q2`, return their overlap point.
pub fn line_segments_overlap(p1: Vec2, mut q1: Vec2, mut p2: Vec2, mut q2: Vec2) -> Option<Vec2> {
    // Idea, transform the space, so that p1 and q1 lie on the x axis, and find the root of line segment p2 -> q2
    q1 -= p1;
    p2 -= p1;
    q2 -= p1;

    let l = q1.length();

    let a = f32::atan2(q1.y, q1.x);

    let m = Mat2::from_angle(-a);
    let mi = Mat2::from_angle(a);

    let np2 = m * p2;
    let nq2 = m * q2;

    axis_aligned_line_segments_overlap(Vec2::ZERO, l, np2, nq2).and_then(|p| Some(mi * p + p1))
}

pub fn axis_aligned_line_segments_overlap(p1: Vec2, len: f32, p2: Vec2, q2: Vec2) -> Option<Vec2> {
    // Both points are either fully above or fully below
    if (p2.y > p1.y && q2.y > p1.y) || (p2.y < p1.y && q2.y < p1.y) {
        return None;
    }

    // Both points are on the line.
    if p2.y == p1.y && q2.y == p1.y {
        return match len.partial_cmp(&0.0).unwrap() {
            std::cmp::Ordering::Less => {
                if !((p2.x < p1.x + len && q2.x < p1.x + len) || (p2.x > p1.x && q2.x > p1.x)) {
                    if (p2.x <= p1.x && q2.x >= p1.x) || (p2.x >= p1.x && q2.x <= p1.x) {
                        Some(p1)
                    } else {
                        Some(Vec2::new(p1.x + len, p1.y))
                    }
                } else {
                    None
                }
            }
            std::cmp::Ordering::Equal => {
                if (p2.x <= p1.x && q2.x >= p1.x) || (p2.x >= p1.x && q2.x <= p1.x) {
                    Some(p1)
                } else {
                    None
                }
            }
            std::cmp::Ordering::Greater => {
                if !((p2.x > p1.x + len && q2.x > p1.x + len) || (p2.x < p1.x && q2.x < p1.x)) {
                    if (p2.x <= p1.x && q2.x >= p1.x) || (p2.x >= p1.x && q2.x <= p1.x) {
                        Some(p1)
                    } else {
                        Some(Vec2::new(p1.x + len, p1.y))
                    }
                } else {
                    None
                }
            }
        };
    }

    // Calculate root.
    let x = lerp(p1.y, p2.y, q2.y, p2.x, q2.x);

    match len.partial_cmp(&0.0).unwrap() {
        std::cmp::Ordering::Less => {
            if x >= p1.x + len && x <= p1.x {
                Some(Vec2::new(x, p1.y))
            } else {
                None
            }
        }
        std::cmp::Ordering::Equal => {
            if p1.x == x {
                Some(p1)
            } else {
                None
            }
        }
        std::cmp::Ordering::Greater => {
            if x >= p1.x && x <= p1.x + len {
                Some(Vec2::new(x, p1.y))
            } else {
                None
            }
        }
    }
}

/// Calculate the distance from a point `p` to an infinitely long line
/// passing through `l1` and `l2`
pub fn dist_point_line(p: Vec2, l1: Vec2, l2: Vec2) -> f32 {
    let p = p - l1;
    let p0 = l2 - l1;
    let b = -p0.x / p0.y;
    let d = (1.0 + b * b).sqrt();
    f32::abs(p.x + b * p.y) / d
}

/// Given a point `p` and a line segment between `l1` and `l2`
/// return the point on the line segement closest to p
pub fn closest_point_on_line(p: Vec2, l1: Vec2, l2: Vec2) -> Vec2 {
    let p = p - l1;
    let l = l2 - l1;
    if l.x == 0.0 {
        l1 + Vec2::new(0.0, clamp(p.y, f32min(0.0, l.y), f32max(0.0, l.y)))
    } else if l.y == 0.0 {
        l1 + Vec2::new(clamp(p.x, f32min(0.0, l.x), f32max(0.0, l.x)), 0.0)
    } else {
        let len = l.length();
        let a = f32::atan2(l.y, l.x);
        let m = Mat2::from_angle(-a);
        let mi = Mat2::from_angle(a);
        let np = m * p;
        let closest = Vec2::new(clamp(np.x, 0.0, len), 0.0);
        mi * closest + l1
    }
}

pub fn f32min(a: f32, b: f32) -> f32 {
    if a <= b {
        a
    } else {
        b
    }
}

pub fn f32max(a: f32, b: f32) -> f32 {
    if a >= b {
        a
    } else {
        b
    }
}

pub fn lerp<T>(x: T, start1: T, stop1: T, start2: T, stop2: T) -> T
where
    T: Clone,
    T: Sub<T, Output = T>,
    T: Div<T, Output = T>,
    T: Mul<T, Output = T>,
    T: Add<T, Output = T>,
{
    start2.clone() + (stop2 - start2) * ((x - start1.clone()) / (stop1 - start1))
}

#[test]
fn test_intersection() {
    let (p1, q1) = (Vec2::new(0.0, 2.0), Vec2::new(2.0, 0.0));
    let (p2, q2) = (Vec2::new(0.0, 0.0), Vec2::new(2.0, 2.0));
    let o = line_segments_overlap(p1, q1, p2, q2);
    println!("Overlap point: {:?}", o);
    assert!(o.is_some());
    assert!(Vec2::new(1.0, 1.0).distance(o.unwrap()) < 0.001);
}

#[test]
fn test_intersection_edge() {
    let (p1, q1) = (Vec2::new(0.0, 2.0), Vec2::new(2.0, 0.0));
    let (p2, q2) = (Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
    let o = line_segments_overlap(p1, q1, p2, q2);
    println!("Overlap point: {:?}", o);
    assert!(o.is_some());
    assert!(Vec2::new(1.0, 1.0).distance(o.unwrap()) < 0.001);
}

#[test]
fn test_intersection_none() {
    let (p1, q1) = (Vec2::new(0.0, 2.0), Vec2::new(2.0, 0.0));
    let (p2, q2) = (Vec2::new(0.0, 0.0), Vec2::new(0.999, 0.999));
    let o = line_segments_overlap(p1, q1, p2, q2);
    println!("Overlap point: {:?}", o);
    assert!(o.is_none());
}
