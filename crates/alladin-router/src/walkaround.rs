//! WALKAROUND: detour a straight A→B track around a single circular
//! obstacle, hugging its clearance boundary.
//!
//! This is the Rust re-implementation (from understood *logic*, not
//! copied code -- see the development log "drei realistische Strategien") of
//! the role KiCad's `PNS::WALKAROUND` plays: when a direct path is
//! blocked by a hard island, compute the shortest path that goes around
//! it instead of failing outright. `PNS::SHOVE`'s role (push *movable*
//! neighbouring tracks aside) now lives in its own [`crate::shove`]
//! module, as a first, deliberately narrow slice -- see that module's
//! doc comment for the exact scope.

use alladin_core::{LayerId, NetClass, NetId, Node, RuleResolver};
use alladin_geom::{dist_point_to_line, Circle, Point, Polygon, Unit};

/// Tangent points from an external point `p` to a circle. Returns `None`
/// if `p` is inside or on the circle (no tangent exists).
///
/// Derivation: in the right triangle formed by `p`, the circle centre
/// `c`, and a tangent point `t`, the angle at `t` is 90° (radius ⊥
/// tangent line). So the angle at `c` (between `c→p` and `c→t`) is
/// `acos(radius / dist(p, c))`. The two tangent points are the centre-
/// relative direction to `p` rotated by ± that angle.
pub fn tangent_points(p: Point, c: &Circle) -> Option<(Point, Point)> {
    let dx = (p.x - c.center.x) as f64;
    let dy = (p.y - c.center.y) as f64;
    let d = (dx * dx + dy * dy).sqrt();
    let r = c.radius as f64;

    if d <= r {
        return None;
    }

    let base_angle = dy.atan2(dx); // direction c -> p
    let offset = (r / d).acos();

    let t1 = rotate_point_on_circle(c, base_angle + offset);
    let t2 = rotate_point_on_circle(c, base_angle - offset);
    Some((t1, t2))
}

fn rotate_point_on_circle(c: &Circle, angle: f64) -> Point {
    Point::new(
        c.center.x + (c.radius as f64 * angle.cos()).round() as Unit,
        c.center.y + (c.radius as f64 * angle.sin()).round() as Unit,
    )
}

fn path_length(path: &[Point]) -> f64 {
    path.windows(2).map(|w| w[0].distance(w[1])).sum()
}

/// True if every leg of `path` stays at least `min_dist` away from
/// `center` (with a small epsilon for rounding slack from the tangent
/// computation above).
fn path_clears_circle(path: &[Point], center: Point, min_dist: f64) -> bool {
    let eps = 1.0; // 1 internal unit = 1 nanometre; way below manufacturing tolerance
    path.windows(2)
        .all(|w| dist_point_to_line(center, w[0], w[1]) >= min_dist - eps)
}

fn angle_of(p: Point, center: Point) -> f64 {
    ((p.y - center.y) as f64).atan2((p.x - center.x) as f64)
}

/// Normalize an angle delta to the shortest signed representation in
/// `(-pi, pi]`.
fn normalize_delta(mut d: f64) -> f64 {
    use std::f64::consts::PI;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d <= -PI {
        d += 2.0 * PI;
    }
    d
}

/// Sample a polyline approximation of the arc from `from_angle` to
/// `from_angle + delta`, at `radius` around `center`. Sampled at a
/// slightly larger radius than the true clearance boundary
/// (`ARC_SAFETY_FACTOR`) so that polyline chord "sagitta" error can never
/// bring a sample segment inside the true clearance circle -- we'd rather
/// hand back a handful of nanometres of extra detour than an unverified
/// claim of clearance.
const ARC_SEGMENTS: usize = 32;
const ARC_SAFETY_FACTOR: f64 = 1.02;

fn sample_arc(center: Point, radius: Unit, from_angle: f64, delta: f64) -> Vec<Point> {
    let sample_radius = radius as f64 * ARC_SAFETY_FACTOR;
    (0..=ARC_SEGMENTS)
        .map(|k| {
            let angle = from_angle + delta * (k as f64 / ARC_SEGMENTS as f64);
            Point::new(
                center.x + (sample_radius * angle.cos()).round() as Unit,
                center.y + (sample_radius * angle.sin()).round() as Unit,
            )
        })
        .collect()
}

/// Compute the shortest detour from `from` to `to` around a single
/// circular obstacle, respecting `clearance`. Returns the direct
/// two-point path unchanged if no detour is needed.
///
/// Geometry: straight tangent lines from `from` and `to` touch the
/// clearance-inflated obstacle circle at up to two points each; the
/// detour follows one tangent in, an arc around the obstacle boundary,
/// then the other tangent out. Connecting the two tangent points with a
/// straight chord instead (a tempting shortcut) is *wrong*: a chord
/// between two points on a circle always dips inside it, which would
/// silently violate clearance -- hence the explicit arc sampling here.
///
/// Rather than hand-deriving which of the 4 tangent-point pairings and 2
/// arc directions is "the correct one", every combination is generated
/// and then verified against the real clearance requirement; only a
/// combination proven clear by the same collision primitives the rest of
/// Alladin uses is ever returned. This mirrors the project's own
/// Correct-by-Construction principle at the walkaround level, not just
/// at the DRC level.
pub fn walkaround_single_obstacle(
    from: Point,
    to: Point,
    obstacle: Circle,
    clearance: Unit,
) -> Vec<Point> {
    let inflated_radius = obstacle.radius + clearance;
    let min_dist = inflated_radius as f64;

    if dist_point_to_line(obstacle.center, from, to) >= min_dist {
        return vec![from, to]; // already clear, no detour needed
    }

    let inflated = Circle::new(obstacle.center, inflated_radius);
    let (Some((fa1, fa2)), Some((tb1, tb2))) =
        (tangent_points(from, &inflated), tangent_points(to, &inflated))
    else {
        // `from` or `to` is inside the (clearance-inflated) obstacle --
        // this candidate route is fundamentally invalid; caller (the A*
        // layer) must pick a different waypoint. Signal via the direct
        // (still-colliding) path so callers can detect failure by
        // re-checking clearance, rather than panicking here.
        return vec![from, to];
    };

    let mut candidates: Vec<Vec<Point>> = Vec::new();
    for fa in [fa1, fa2] {
        for tb in [tb1, tb2] {
            let a0 = angle_of(fa, obstacle.center);
            let a1 = angle_of(tb, obstacle.center);
            let short = normalize_delta(a1 - a0);
            let long = if short > 0.0 {
                short - 2.0 * std::f64::consts::PI
            } else {
                short + 2.0 * std::f64::consts::PI
            };
            for delta in [short, long] {
                let arc = sample_arc(obstacle.center, inflated_radius, a0, delta);
                let mut path = vec![from];
                path.extend(arc);
                path.push(to);
                candidates.push(path);
            }
        }
    }

    candidates
        .into_iter()
        .filter(|c| path_clears_circle(c, obstacle.center, min_dist))
        .min_by(|a, b| path_length(a).partial_cmp(&path_length(b)).unwrap())
        .unwrap_or_else(|| vec![from, to]) // no valid detour found (shouldn't happen for a single circular obstacle)
}

/// Route a single net from `from` to `to` through `world`.
///
/// The actual pathfinding is now [`crate::astar::find_path_astar`]'s
/// visibility-graph A* search -- a real search over every alternative
/// waypoint at once, not the sequential "resolve one colliding leg,
/// re-check" loop this function used to run directly. That loop is
/// still very much alive, though: `find_path_astar` builds its candidate
/// waypoints from the exact same boundary-sampling building blocks
/// (`walkaround_single_obstacle`'s tangent geometry conceptually, and
/// `capsule_walkaround::capsule_boundary` literally) developed here.
/// This function's own remaining job is just the surrounding contract:
/// run the rubberband [`crate::optimizer::optimize_path`] pass afterwards
/// to remove redundant bends, and do a final, from-scratch
/// Correct-by-Construction verification before ever handing a path back.
///
/// Still a stand-in for KiCad's `PNS::LINE_PLACER` in one respect: this
/// function itself never shoves anything, only ever routes around fixed
/// pads/vias/tracks -- [`crate::shove::try_shove_blockers`] is the
/// fallback once this function has already given up (see that module
/// for the (still narrow) current scope of real SHOVE). Notably,
/// `try_shove_blockers` is itself built on top of *this very function*: shoving a blocker
/// means calling `route_single_net` again for the blocker's own net,
/// between its own unchanged endpoints, with the new route as a
/// stand-in obstacle.
pub fn route_single_net(
    world: &Node,
    from: Point,
    to: Point,
    width: Unit,
    net: NetId,
    layer: LayerId,
    class: NetClass,
    resolver: &dyn RuleResolver,
    outline: &[Polygon],
) -> Option<Vec<Point>> {
    let path =
        crate::astar::find_path_astar(world, from, to, width, net, layer, class, resolver, outline)?;

    // Correct-by-Construction: don't trust the search above blindly -- do
    // a final, from-scratch verification of every leg against the whole
    // world (*and* the board outline, if one was supplied) before ever
    // handing a path back to the caller.
    let verify = |p: &[Point]| -> bool {
        p.windows(2).all(|leg| {
            world.path_is_clear(leg[0], leg[1], width, Some(net), layer, class, resolver)
                && (outline.is_empty() || alladin_geom::contains_segment_evenodd(outline, leg[0], leg[1]))
        })
    };

    if !verify(&path) {
        return None; // should be unreachable: find_path_astar only accepts edges it verified itself
    }

    let optimized =
        crate::optimizer::optimize_path(world, &path, width, net, layer, class, resolver, outline);

    if !verify(&optimized) {
        return None; // should be unreachable given optimize_path's own visibility checks
    }

    Some(optimized)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::{FixedClearance, Item, PadShape};
    use alladin_geom::MM;

    #[test]
    fn no_obstacle_returns_direct_path() {
        let obstacle = Circle::new(Point::new(10 * MM, 10 * MM), 500_000); // far away
        let path = walkaround_single_obstacle(Point::new(0, 0), Point::new(5 * MM, 0), obstacle, 127_000);
        assert_eq!(path, vec![Point::new(0, 0), Point::new(5 * MM, 0)]);
    }

    #[test]
    fn detour_clears_centered_obstacle_and_reaches_endpoints() {
        let obstacle = Circle::new(Point::new(2_500_000, 0), 800_000);
        let clearance = 127_000;
        let path = walkaround_single_obstacle(Point::new(0, 0), Point::new(5 * MM, 0), obstacle, clearance);

        assert!(path.len() >= 3, "expected a detour with intermediate waypoints, got {path:?}");
        assert_eq!(*path.first().unwrap(), Point::new(0, 0));
        assert_eq!(*path.last().unwrap(), Point::new(5 * MM, 0));

        let min_dist = (obstacle.radius + clearance) as f64;
        assert!(path_clears_circle(&path, obstacle.center, min_dist));

        // A real detour must be longer than the (blocked) straight line --
        // otherwise something has gone geometrically wrong.
        assert!(path_length(&path) > 5.0 * MM as f64);
    }

    #[test]
    fn route_single_net_end_to_end_through_node() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        let net1 = NetId(1);
        let net2 = Some(NetId(2));

        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(2_500_000, 0), 800_000)),
            net: net2,
            layer: LayerId::FCu,
        });

        let path = route_single_net(
            &world,
            Point::new(0, 0),
            Point::new(5 * MM, 0),
            250_000,
            net1,
            LayerId::FCu,
            NetClass::C,
            &resolver,
            &[],
        )
        .expect("routing must succeed around a single circular obstacle");

        assert!(path.len() >= 3);
        assert_eq!(*path.first().unwrap(), Point::new(0, 0));
        assert_eq!(*path.last().unwrap(), Point::new(5 * MM, 0));
    }

    #[test]
    fn route_single_net_handles_two_sequential_obstacles() {
        // Two pads 10mm apart, with two separate blocking obstacles along
        // the straight line between them -- exercises the new multi-
        // obstacle iteration in `route_single_net` (previously only a
        // single blocker was handled).
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        let net1 = NetId(1);

        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(3 * MM, 0), 800_000)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });
        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(7 * MM, 0), 800_000)),
            net: Some(NetId(3)),
            layer: LayerId::FCu,
        });

        let path = route_single_net(
            &world,
            Point::new(0, 0),
            Point::new(10 * MM, 0),
            250_000,
            net1,
            LayerId::FCu,
            NetClass::C,
            &resolver,
            &[],
        )
        .expect("routing must succeed around two sequential obstacles");

        assert_eq!(*path.first().unwrap(), Point::new(0, 0));
        assert_eq!(*path.last().unwrap(), Point::new(10 * MM, 0));

        // Every leg of the final (optimized) path must be independently
        // verified clear of both obstacles -- re-derive the check here
        // rather than trusting route_single_net's internal verification,
        // to catch regressions in either walkaround or the optimizer.
        for leg in path.windows(2) {
            assert!(world.path_is_clear(
                leg[0], leg[1], 250_000, Some(net1), LayerId::FCu, NetClass::C, &resolver
            ));
        }
    }

    #[test]
    fn route_single_net_walks_around_a_blocking_track() {
        // Previously: colliding with an existing *track* (as opposed to
        // a pad/via) made `route_single_net` give up and return `None`.
        // This exercises the new `capsule_walkaround` path end-to-end.
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        let net1 = NetId(1);

        world.add(Item::Track {
            shape: alladin_geom::Segment::new(
                Point::new(2_500_000, -2_000_000),
                Point::new(2_500_000, 2_000_000),
                300_000,
            ),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let path = route_single_net(
            &world,
            Point::new(0, 0),
            Point::new(5 * MM, 0),
            250_000,
            net1,
            LayerId::FCu,
            NetClass::C,
            &resolver,
            &[],
        )
        .expect("routing must succeed around a blocking track, not give up");

        assert!(path.len() >= 3, "expected a real detour, got {path:?}");
        assert_eq!(*path.first().unwrap(), Point::new(0, 0));
        assert_eq!(*path.last().unwrap(), Point::new(5 * MM, 0));

        for leg in path.windows(2) {
            assert!(world.path_is_clear(
                leg[0], leg[1], 250_000, Some(net1), LayerId::FCu, NetClass::C, &resolver
            ));
        }
    }

    #[test]
    fn route_single_net_direct_when_clear() {
        let world = Node::new();
        let resolver = FixedClearance(127_000);

        let path = route_single_net(
            &world,
            Point::new(0, 0),
            Point::new(5 * MM, 0),
            250_000,
            NetId(1),
            LayerId::FCu,
            NetClass::C,
            &resolver,
            &[],
        )
        .unwrap();

        assert_eq!(path, vec![Point::new(0, 0), Point::new(5 * MM, 0)]);
    }
}
