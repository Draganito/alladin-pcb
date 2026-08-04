//! OPTIMIZER: the "Rubberband" pass from the original architecture note --
//! pull the routed polyline taut like an elastic band until it touches the
//! clearance boundary of the nearest obstacle, removing unnecessary
//! zigzags left over from chaining multiple walkarounds together.
//!
//! Algorithm: classic greedy "string pulling" / visibility shortcutting.
//! From the current anchor point, find the *farthest* later point in the
//! path that's still reachable via a straight, collision-free line; jump
//! there, and repeat. This provably never increases total path length
//! (it only replaces multi-segment detours with a single chord whenever
//! that chord is provably clear), and against a single convex obstacle it
//! correctly refuses to cut the corner (every point on the wrap-around
//! arc stays a required anchor, because any chord between two arc points
//! would cut inside the obstacle).

use alladin_core::{LayerId, NetClass, NetId, Node, RuleResolver};
use alladin_geom::{Point, Polygon, Unit};

/// `outline`: the board's boundary polygon(s), or `&[]` if none is
/// known (see [`crate::astar::find_path_astar`]'s doc comment for the
/// same convention). Without this check, a shortcut chord could easily
/// cut across a board edge even when the *unoptimized* path this
/// function was handed didn't -- string-pulling straight-lines a
/// multi-waypoint detour that a board-outline-aware search deliberately
/// routed around a concave board edge, exactly the kind of "obstacle"
/// this optimizer otherwise has no other way to know about (it only
/// ever sees `path`, not whatever produced it).
pub fn optimize_path(
    world: &Node,
    path: &[Point],
    width: Unit,
    net: NetId,
    layer: LayerId,
    class: NetClass,
    resolver: &dyn RuleResolver,
    outline: &[Polygon],
) -> Vec<Point> {
    if path.len() <= 2 {
        return path.to_vec();
    }

    let stays_on_board = |a: Point, b: Point| {
        outline.is_empty() || alladin_geom::contains_segment_evenodd(outline, a, b)
    };

    let mut result = vec![path[0]];
    let mut anchor = 0usize;

    while anchor < path.len() - 1 {
        let mut farthest = anchor + 1;
        for j in (anchor + 2)..path.len() {
            if world.path_is_clear(path[anchor], path[j], width, Some(net), layer, class, resolver)
                && stays_on_board(path[anchor], path[j])
            {
                farthest = j;
            }
        }
        result.push(path[farthest]);
        anchor = farthest;
    }

    result
}

#[cfg(test)]
fn path_length(path: &[Point]) -> f64 {
    path.windows(2).map(|w| w[0].distance(w[1])).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::{FixedClearance, Item, PadShape};
    use alladin_geom::{Circle, MM};

    #[test]
    fn zigzag_in_open_space_collapses_to_straight_line() {
        let world = Node::new();
        let resolver = FixedClearance(127_000);
        let zigzag = vec![
            Point::new(0, 0),
            Point::new(1 * MM, 1 * MM),
            Point::new(2 * MM, -1 * MM),
            Point::new(3 * MM, 0),
        ];

        let optimized = optimize_path(
            &world,
            &zigzag,
            250_000,
            NetId(1),
            LayerId::FCu,
            NetClass::C,
            &resolver,
            &[],
        );

        assert_eq!(optimized, vec![Point::new(0, 0), Point::new(3 * MM, 0)]);
    }

    #[test]
    fn optimizer_never_increases_length_or_breaks_clearance() {
        // Chain two single-obstacle walkarounds by hand (a simple stand-in
        // for what a multi-obstacle route builder would produce), then
        // confirm the optimizer only ever shortens it and never
        // reintroduces a collision.
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        let net1 = NetId(1);
        let obstacle = Circle::new(Point::new(2_500_000, 0), 800_000);
        world.add(Item::Pad {
            shape: PadShape::Circle(obstacle),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });

        let raw = crate::walkaround::walkaround_single_obstacle(
            Point::new(0, 0),
            Point::new(5 * MM, 0),
            obstacle,
            127_000 + 125_000, // clearance + half track width, as route_single_net does
        );

        let optimized = optimize_path(
            &world, &raw, 250_000, net1, LayerId::FCu, NetClass::C, &resolver, &[],
        );

        assert!(path_length(&optimized) <= path_length(&raw) + 1.0);

        for leg in optimized.windows(2) {
            assert!(world.path_is_clear(
                leg[0], leg[1], 250_000, Some(net1), LayerId::FCu, NetClass::C, &resolver
            ));
        }
    }
}
