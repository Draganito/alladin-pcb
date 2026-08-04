//! Path primitives for stroked traces and filled regions -- same role as
//! `gerber_writer.Path` (move/line only; Alladin has no arc tracks).

use alladin_geom::Point;

#[derive(Debug, Clone, PartialEq)]
pub enum PathOp {
    MoveTo(Point),
    LineTo(Point),
}

/// A polyline built from move/line operators. Used both for stroked
/// traces ([`crate::GerberLayer::add_traces_path`]) and for region
/// contours ([`crate::GerberLayer::add_region`]).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Path {
    pub ops: Vec<PathOp>,
}

impl Path {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn moveto(&mut self, p: Point) {
        self.ops.push(PathOp::MoveTo(p));
    }

    pub fn lineto(&mut self, p: Point) {
        self.ops.push(PathOp::LineTo(p));
    }

    /// Convenience: closed contour from a ring of points (first point
    /// repeated at the end if needed).
    pub fn from_closed_ring(points: &[Point]) -> Self {
        let mut path = Self::new();
        if points.is_empty() {
            return path;
        }
        path.moveto(points[0]);
        for p in &points[1..] {
            path.lineto(*p);
        }
        if points.first() != points.last() {
            path.lineto(points[0]);
        }
        path
    }
}
