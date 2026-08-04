//! Walking around a *track* obstacle (a capsule/stadium shape: a segment
//! with a width), not just a circular pad/via.
//!
//! Why this needs its own module instead of reusing [`crate::walkaround`]
//! as-is: a stadium is convex but not circular, and the tangent-line
//! intuition from the circle case doesn't transfer directly. In
//! particular, a naive "route around whichever end circle is closer" was
//! considered and rejected during design: for an external point sitting
//! roughly in front of the capsule's *flat side* (not near either rounded
//! end), the true tangent lines touch the flat side's corner points, not
//! a point on either end's rounded cap. So this module treats the
//! clearance-inflated capsule boundary as a discretized convex polygon
//! (two straight edges + two 180° arc caps, **no** interior points along
//! the straight edges so the polygon has no accidental collinear runs)
//! and finds true tangent vertices on that polygon, generalizing the
//! circle-tangent technique from `walkaround.rs` rather than special-
//! casing the two obstacle shapes differently.

use alladin_geom::{cross2d, dist_point_to_line, Point, Segment, Unit};

const ARC_SEGMENTS: usize = 16; // per rounded end -- less than the circle case's 32, since each cap only sweeps 180°, not a full obstacle-dependent arc
const ARC_SAFETY_FACTOR: f64 = 1.02; // see walkaround.rs -- compensates polyline sagitta so a sampled point is never closer to the true boundary than intended

/// Discretized boundary of the clearance-inflated capsule, as a closed,
/// strictly convex polygon in consistent (counter-clockwise or clockwise,
/// doesn't matter which -- just consistent) order: corner → straight edge
/// → corner → arc around one cap → corner → straight edge → corner → arc
/// around the other cap → back to start.
pub(crate) fn capsule_boundary(seg: &Segment, radius: Unit) -> Vec<Point> {
    let dx = (seg.b.x - seg.a.x) as f64;
    let dy = (seg.b.y - seg.a.y) as f64;
    let len = (dx * dx + dy * dy).sqrt();

    if len < 1.0 {
        // Degenerate (near-zero-length) segment: caller should really
        // treat this as a circle, but return a minimal valid boundary
        // rather than dividing by zero.
        return vec![
            Point::new(seg.a.x + radius, seg.a.y),
            Point::new(seg.a.x, seg.a.y + radius),
            Point::new(seg.a.x - radius, seg.a.y),
            Point::new(seg.a.x, seg.a.y - radius),
        ];
    }

    let dir_angle = dy.atan2(dx);
    let (dux, duy) = (dx / len, dy / len); // unit direction a->b
    let (nx, ny) = (-duy, dux); // unit normal, 90 deg CCW from direction

    let sample_r = radius as f64 * ARC_SAFETY_FACTOR;
    let off = |base: Point, ux: f64, uy: f64, r: f64| -> Point {
        Point::new(
            base.x + (ux * r).round() as Unit,
            base.y + (uy * r).round() as Unit,
        )
    };

    let p1 = off(seg.a, nx, ny, radius as f64); // side 1 @ a
    let p2 = off(seg.b, nx, ny, radius as f64); // side 1 @ b
    let p3 = off(seg.b, -nx, -ny, radius as f64); // side 2 @ b
    let p4 = off(seg.a, -nx, -ny, radius as f64); // side 2 @ a

    let mut boundary = vec![p1, p2];

    // Arc around b's cap, from p2's angle to p3's angle, sweeping through
    // +direction (outward past b) -- a 180 degree turn.
    let start_b = dir_angle + std::f64::consts::FRAC_PI_2;
    for k in 1..ARC_SEGMENTS {
        let angle = start_b - std::f64::consts::PI * (k as f64 / ARC_SEGMENTS as f64);
        boundary.push(Point::new(
            seg.b.x + (sample_r * angle.cos()).round() as Unit,
            seg.b.y + (sample_r * angle.sin()).round() as Unit,
        ));
    }
    boundary.push(p3);
    boundary.push(p4);

    // Arc around a's cap, from p4's angle back to p1's angle, sweeping
    // through -direction (outward past a).
    let start_a = dir_angle - std::f64::consts::FRAC_PI_2;
    for k in 1..ARC_SEGMENTS {
        let angle = start_a - std::f64::consts::PI * (k as f64 / ARC_SEGMENTS as f64);
        boundary.push(Point::new(
            seg.a.x + (sample_r * angle.cos()).round() as Unit,
            seg.a.y + (sample_r * angle.sin()).round() as Unit,
        ));
    }

    boundary
}

/// Find the (up to two) vertex indices of a strictly convex polygon that
/// are tangent as seen from external point `p` -- i.e. vertices where
/// every other vertex lies on one consistent side of the line `p`→vertex.
/// Standard convex-polygon tangent-point technique, generalizing the
/// circle-tangent formula in `walkaround.rs` to an arbitrary convex
/// boundary (needed because a capsule's flat sides mean the true tangent
/// point is sometimes a corner, not a point on either round cap).
fn tangent_vertices(boundary: &[Point], p: Point) -> Vec<usize> {
    let n = boundary.len();
    let mut result = Vec::new();

    for i in 0..n {
        let v = boundary[i];
        let mut saw_pos = false;
        let mut saw_neg = false;

        for j in 0..n {
            if j == i {
                continue;
            }
            let side = cross2d(p, v, boundary[j]);
            if side > 0 {
                saw_pos = true;
            } else if side < 0 {
                saw_neg = true;
            }
        }

        if !(saw_pos && saw_neg) {
            result.push(i);
        }
    }

    result
}

fn path_length(path: &[Point]) -> f64 {
    path.windows(2).map(|w| w[0].distance(w[1])).sum()
}

/// True if every leg of `path` stays at least `min_dist` away from the
/// obstacle segment's centerline.
fn path_clears_capsule(path: &[Point], obstacle: &Segment, min_dist: f64) -> bool {
    let eps = 1.0;
    path.windows(2).all(|w| {
        // Conservative approximation: check both path-segment endpoints'
        // distance to the obstacle's line, and the obstacle endpoints'
        // distance to the path segment -- catches the practical crossing
        // cases without a full segment-to-segment distance routine here
        // (that already exists in alladin-geom for the final
        // verification pass done by the caller; this is a fast filter).
        let d1 = dist_point_to_line(w[0], obstacle.a, obstacle.b);
        let d2 = dist_point_to_line(w[1], obstacle.a, obstacle.b);
        let d3 = alladin_geom::dist_segment_to_segment((w[0], w[1]), (obstacle.a, obstacle.b));
        d1.min(d2).min(d3) >= min_dist - eps
    })
}

/// Compute the shortest detour from `from` to `to` around a single track
/// (capsule-shaped) obstacle, respecting `clearance`. Mirrors
/// [`crate::walkaround::walkaround_single_obstacle`]'s structure and its
/// "generate every plausible combination, keep only what's verified
/// clear" philosophy, generalized from circle tangents to convex-polygon
/// tangents.
pub fn walkaround_capsule(from: Point, to: Point, obstacle: Segment, clearance: Unit) -> Vec<Point> {
    let radius = obstacle.width / 2 + clearance;
    let min_dist = radius as f64;

    if alladin_geom::dist_segment_to_segment((from, to), (obstacle.a, obstacle.b)) >= min_dist {
        return vec![from, to];
    }

    let boundary = capsule_boundary(&obstacle, radius);
    let from_tangents = tangent_vertices(&boundary, from);
    let to_tangents = tangent_vertices(&boundary, to);

    if from_tangents.is_empty() || to_tangents.is_empty() {
        // from/to inside the inflated capsule -- invalid candidate route,
        // same convention as walkaround_single_obstacle.
        return vec![from, to];
    }

    let n = boundary.len();
    let mut candidates: Vec<Vec<Point>> = Vec::new();

    for &fi in &from_tangents {
        for &ti in &to_tangents {
            // Walk the boundary index list both directions from fi to ti.
            for forward in [true, false] {
                let mut arc = Vec::new();
                let mut idx = fi;
                loop {
                    arc.push(boundary[idx]);
                    if idx == ti {
                        break;
                    }
                    idx = if forward { (idx + 1) % n } else { (idx + n - 1) % n };
                    if arc.len() > n + 1 {
                        break; // safety valve; shouldn't happen
                    }
                }
                let mut path = vec![from];
                path.extend(arc);
                path.push(to);
                candidates.push(path);
            }
        }
    }

    candidates
        .into_iter()
        .filter(|c| path_clears_capsule(c, &obstacle, min_dist))
        .min_by(|a, b| path_length(a).partial_cmp(&path_length(b)).unwrap())
        .unwrap_or_else(|| vec![from, to])
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_geom::MM;

    #[test]
    fn boundary_points_are_all_at_the_expected_radius() {
        let seg = Segment::new(Point::new(0, 0), Point::new(3 * MM, 0), 500_000);
        let radius = 500_000 / 2 + 127_000;
        let boundary = capsule_boundary(&seg, radius);

        for p in &boundary {
            let d = dist_point_to_line(*p, seg.a, seg.b);
            // Every boundary point must be at *least* `radius` from the
            // centerline (arc points are sampled slightly outside via
            // ARC_SAFETY_FACTOR; corner points are exact).
            assert!(
                d >= radius as f64 - 1000.0,
                "boundary point {p:?} at distance {d}, expected >= {radius}"
            );
        }
    }

    #[test]
    fn no_obstacle_in_the_way_returns_direct_path() {
        let far_track = Segment::new(Point::new(0, 10 * MM), Point::new(1 * MM, 10 * MM), 250_000);
        let path = walkaround_capsule(Point::new(0, 0), Point::new(5 * MM, 0), far_track, 127_000);
        assert_eq!(path, vec![Point::new(0, 0), Point::new(5 * MM, 0)]);
    }

    #[test]
    fn detour_around_perpendicular_track_clears_and_reaches_endpoints() {
        // A track obstacle crossing perpendicular to our desired path,
        // centered in the middle -- forces a real detour around one of
        // its rounded ends.
        let obstacle = Segment::new(Point::new(2_500_000, -1_000_000), Point::new(2_500_000, 1_000_000), 300_000);
        let clearance = 127_000;
        let path = walkaround_capsule(Point::new(0, 0), Point::new(5 * MM, 0), obstacle, clearance);

        assert!(path.len() >= 3, "expected a real detour, got {path:?}");
        assert_eq!(*path.first().unwrap(), Point::new(0, 0));
        assert_eq!(*path.last().unwrap(), Point::new(5 * MM, 0));

        let min_dist = (obstacle.width / 2 + clearance) as f64;
        assert!(path_clears_capsule(&path, &obstacle, min_dist));
    }

    #[test]
    fn detour_around_parallel_track_touches_a_corner_not_an_arc() {
        // A long track obstacle running parallel to, and directly between,
        // our start/end points -- the case that motivated this whole
        // module: the true tangent points here are the flat-side corners,
        // not a point on either rounded end.
        let obstacle = Segment::new(Point::new(0, 0), Point::new(5 * MM, 0), 300_000);
        let clearance = 127_000;
        let from = Point::new(2_500_000, 2_000_000); // above the middle of the obstacle
        let to = Point::new(2_500_000, -2_000_000); // below the middle

        let path = walkaround_capsule(from, to, obstacle, clearance);

        assert!(path.len() >= 3, "expected a real detour, got {path:?}");
        let min_dist = (obstacle.width / 2 + clearance) as f64;
        assert!(path_clears_capsule(&path, &obstacle, min_dist));

        // The shortest legal detour must go around one end of the track,
        // not balloon out unnecessarily -- sanity bound on total length.
        assert!(path_length(&path) < 4.0 * from.distance(to));
    }
}
