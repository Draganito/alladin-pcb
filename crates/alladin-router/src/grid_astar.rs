//! Octilinear grid A* over a [`GridObstacleMap`] -- the "structurally
//! different approach" `astar.rs`'s own `build_adjacency_knn` doc
//! comment already names as necessary follow-up work for the
//! candidate-point-explosion class of failures on dense real boards
//! (see that module's doc comment, and the development log's
//! "Teil 28" entry, for the full story and the reference project this
//! was adapted from, `drandyhaas/KiCadRoutingTools`).
//!
//! Unlike `astar.rs`'s visibility graph, there is no "candidate point"
//! concept here at all: every grid cell is implicitly a graph node, and
//! `GridObstacleMap::is_blocked` answers collision queries in `O(1)`
//! regardless of how complex the obstacle that blocked a cell was. The
//! trade-off is quantization: a raw grid path is a staircase-y
//! octilinear polyline at grid resolution, not the clean, minimal-
//! waypoint diagonal the continuous engine produces -- [`smooth_path`]
//! exists to close that gap using the exact same collision oracle
//! (`Node::path_is_clear`) the rest of this crate already trusts.
//!
//! Deliberately offered as an **opt-in alternative** to
//! [`crate::astar::find_path_astar`], not a replacement -- see
//! [`find_path_grid`]'s doc comment and `astar.rs`'s own
//! `ALLADIN_GRID_FALLBACK` integration point.

use alladin_core::{Item, LayerId, NetClass, NetId, Node, RuleResolver};
use alladin_geom::{Aabb, Point, Polygon, Unit};
use rustc_hash::{FxHashMap, FxHashSet};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::grid_obstacle::GridObstacleMap;

/// Cost of an orthogonal (N/E/S/W) grid step, in the same "millicell"
/// units `KiCadRoutingTools` uses (`ORTHO_COST`/`DIAG_COST`) -- kept as
/// integers so `g`/`f` comparisons are exact, never subject to
/// floating-point drift between otherwise-tied paths.
const ORTHO_COST: i64 = 1000;
/// `sqrt(2) * 1000`, rounded -- cost of a diagonal grid step.
const DIAG_COST: i64 = 1414;

const DIRECTIONS: [(i32, i32); 8] =
    [(1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1)];

/// Sentinel "no parent" marker for a source cell, packed into the same
/// `u64` key space every other cell uses -- since a valid packed key's
/// top bit is always `0` for any in-bounds `i32` grid coordinate pair
/// this module ever produces, `u64::MAX` can never collide with a real
/// cell key.
const NO_PARENT: u64 = u64::MAX;

#[inline]
fn pack(gx: i32, gy: i32) -> u64 {
    ((gx as u32 as u64) << 32) | (gy as u32 as u64)
}

#[inline]
fn unpack(key: u64) -> (i32, i32) {
    ((key >> 32) as u32 as i32, key as u32 as i32)
}

#[derive(Clone, Copy)]
struct NodeState {
    g: i64,
    parent: u64,
}

#[derive(Clone, Copy)]
struct HeapEntry {
    f: i64,
    /// Tie-breaker for deterministic ordering (insertion order),
    /// exactly like `astar.rs::HeapEntry`'s own reasoning.
    counter: u32,
    key: u64,
}

impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f && self.counter == other.counter
    }
}
impl Eq for HeapEntry {}
impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reversed: `BinaryHeap` is a max-heap, and the lowest `f`
        // (ties broken by insertion order) must come out first.
        other.f.cmp(&self.f).then_with(|| other.counter.cmp(&self.counter))
    }
}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Admissible octile-distance heuristic in the same integer "millicell"
/// units as `g` -- never overestimates the true remaining grid-step
/// cost, so the search result is a provably shortest grid path, not
/// just *a* path (same guarantee `astar.rs::astar_search`'s Euclidean
/// heuristic gives over its own graph).
fn octile_heuristic(gx: i32, gy: i32, tx: i32, ty: i32) -> i64 {
    let dx = (gx - tx).unsigned_abs() as i64;
    let dy = (gy - ty).unsigned_abs() as i64;
    let diag = dx.min(dy);
    let orth = dx.max(dy) - diag;
    diag * DIAG_COST + orth * ORTHO_COST
}

/// Raw octilinear grid A* search: `from`/`to` are snapped to their
/// containing grid cells, and the returned path (if any) walks cell
/// centers, not the exact `from`/`to` points themselves -- see
/// [`find_path_grid`], the only intended caller, for why it overwrites
/// the first/last point before handing a path back to the rest of this
/// crate.
///
/// A diagonal move is rejected if either of the two orthogonal cells it
/// would "cut past" is blocked -- otherwise a diagonal step could
/// silently graze a blocked corner no orthogonal move would ever be
/// allowed to touch, which would be a real (if narrow) DRC violation in
/// the resulting track.
fn search(obstacles: &GridObstacleMap, from: Point, to: Point, layer: LayerId) -> Option<Vec<Point>> {
    let (sx, sy) = obstacles.to_grid(from);
    let (tx, ty) = obstacles.to_grid(to);
    if !obstacles.in_bounds(sx, sy) || !obstacles.in_bounds(tx, ty) {
        return None;
    }

    let start_key = pack(sx, sy);
    let goal_key = pack(tx, ty);

    let mut nodes: FxHashMap<u64, NodeState> = FxHashMap::default();
    let mut closed: FxHashSet<u64> = FxHashSet::default();
    let mut open: BinaryHeap<HeapEntry> = BinaryHeap::new();
    let mut counter = 0u32;

    nodes.insert(start_key, NodeState { g: 0, parent: NO_PARENT });
    open.push(HeapEntry { f: octile_heuristic(sx, sy, tx, ty), counter, key: start_key });

    while let Some(HeapEntry { key, .. }) = open.pop() {
        if closed.contains(&key) {
            continue; // stale heap entry from before a cheaper g was found
        }
        closed.insert(key);
        if key == goal_key {
            return Some(reconstruct(obstacles, &nodes, key));
        }
        let (gx, gy) = unpack(key);
        let g = nodes[&key].g;

        for &(dx, dy) in &DIRECTIONS {
            let nx = gx + dx;
            let ny = gy + dy;
            if !obstacles.in_bounds(nx, ny) {
                continue;
            }
            let nkey = pack(nx, ny);
            if closed.contains(&nkey) || obstacles.is_blocked(nx, ny, layer) {
                continue;
            }
            if dx != 0 && dy != 0 && (obstacles.is_blocked(gx + dx, gy, layer) || obstacles.is_blocked(gx, gy + dy, layer)) {
                continue; // would clip a blocked corner
            }
            let step_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
            let tentative = g + step_cost;
            let better = nodes.get(&nkey).is_none_or(|s| tentative < s.g);
            if better {
                nodes.insert(nkey, NodeState { g: tentative, parent: key });
                counter += 1;
                let f = tentative + octile_heuristic(nx, ny, tx, ty);
                open.push(HeapEntry { f, counter, key: nkey });
            }
        }
    }
    None
}

fn reconstruct(obstacles: &GridObstacleMap, nodes: &FxHashMap<u64, NodeState>, goal_key: u64) -> Vec<Point> {
    let mut path = Vec::new();
    let mut key = goal_key;
    loop {
        let (gx, gy) = unpack(key);
        path.push(obstacles.cell_center(gx, gy));
        let parent = nodes[&key].parent;
        if parent == NO_PARENT {
            break;
        }
        key = parent;
    }
    path.reverse();
    path
}

/// Greedily removes intermediate waypoints from a raw (grid-resolution)
/// path whenever a straight line from the current anchor to a farther
/// waypoint is already clear -- turns grid A*'s staircase-y octilinear
/// output into the same kind of clean, minimal-waypoint polyline
/// `astar.rs`/`optimizer::optimize_path` already produce, using the
/// exact same collision oracle ([`Node::path_is_clear`]) the rest of
/// this crate trusts for that decision. Never produces a colliding
/// shortcut: every accepted segment is individually re-verified here,
/// not assumed safe just because it skips ahead.
///
/// Greedy, not globally optimal (a "string-pulling" pass, not a full
/// shortest-path-in-the-visible-subgraph search): from each anchor it
/// extends forward while the line of sight stays clear and stops at the
/// first blocked probe, rather than trying every remaining waypoint --
/// `O(n)` in the common case instead of `astar.rs::try_route`'s own
/// `O(n^2)` fallback, which matters here because a raw grid path can
/// have one waypoint per grid step (thousands, for a long net). The
/// only cost of stopping at the first failure is an occasional extra
/// waypoint where visibility briefly re-opens further down the path --
/// never an incorrect (colliding) result.
#[allow(clippy::too_many_arguments)]
pub fn smooth_path(
    world: &Node,
    path: &[Point],
    width: Unit,
    net: NetId,
    layer: LayerId,
    class: NetClass,
    resolver: &dyn RuleResolver,
) -> Vec<Point> {
    if path.len() <= 2 {
        return path.to_vec();
    }
    let mut result = vec![path[0]];
    let mut anchor = 0usize;
    while anchor < path.len() - 1 {
        let mut farthest = anchor + 1;
        let mut probe = anchor + 2;
        while probe < path.len()
            && world.path_is_clear(path[anchor], path[probe], width, Some(net), layer, class, resolver)
        {
            farthest = probe;
            probe += 1;
        }
        result.push(path[farthest]);
        anchor = farthest;
    }
    result
}

/// Grid-search counterpart to [`crate::astar::find_path_astar`]:
/// rasterizes `items` into a fresh [`GridObstacleMap`] covering
/// `region` (see that type's own doc comment for why a fresh map per
/// call, rather than an incrementally-updated one, is the right choice
/// here), searches it with octilinear A*, and smooths the result before
/// returning it.
///
/// **Status: opt-in prototype, not the default engine.** Offered as an
/// alternative for exactly the cases that make the continuous
/// visibility-graph engine's candidate-point count explode (dense
/// corridors, real filled zones with tens of thousands of vertices) --
/// see `astar.rs`'s `ALLADIN_GRID_FALLBACK`-gated call site and
/// the development log's "Teil 28" entry for the benchmark this was
/// validated against and the reasoning for keeping it opt-in while
/// still a prototype.
///
/// Returns `None` if either endpoint's containing grid cell falls
/// outside `region`, or if no octilinear grid path connects them.
#[allow(clippy::too_many_arguments)]
pub fn find_path_grid<'a>(
    world: &Node,
    items: impl Iterator<Item = &'a Item>,
    from: Point,
    to: Point,
    width: Unit,
    net: NetId,
    layer: LayerId,
    class: NetClass,
    resolver: &dyn RuleResolver,
    outline: &[Polygon],
    region: Aabb,
    grid_step: Unit,
) -> Option<Vec<Point>> {
    let dbg = std::env::var("ALLADIN_DEBUG_TIMING").is_ok();
    let t0 = std::time::Instant::now();
    let mut obstacles = GridObstacleMap::new(region, grid_step);
    if dbg {
        eprintln!("  [grid timing] new: {:?} ({}x{} cells)", t0.elapsed(), obstacles.width(), obstacles.height());
    }

    let t1 = std::time::Instant::now();
    obstacles.rasterize_items(items, from, to, width, net, layer, class, resolver);
    if dbg {
        eprintln!("  [grid timing] rasterize_items: {:?}", t1.elapsed());
    }

    let t2 = std::time::Instant::now();
    obstacles.block_off_board(outline);
    if dbg {
        eprintln!("  [grid timing] block_off_board: {:?}", t2.elapsed());
    }

    let t3 = std::time::Instant::now();
    let mut raw = search(&obstacles, from, to, layer)?;
    if dbg {
        eprintln!("  [grid timing] search: {:?} ({} raw waypoints)", t3.elapsed(), raw.len());
    }
    if raw.len() == 1 {
        raw.push(to);
    }
    *raw.first_mut().expect("search() never returns an empty path") = from;
    *raw.last_mut().expect("search() never returns an empty path") = to;

    let t4 = std::time::Instant::now();
    let smoothed = smooth_path(world, &raw, width, net, layer, class, resolver);
    if dbg {
        eprintln!("  [grid timing] smooth_path: {:?} ({} waypoints)", t4.elapsed(), smoothed.len());
    }
    Some(smoothed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::PadShape;
    use alladin_core::{FixedClearance, Item, NetClass as Class};
    use alladin_geom::{Circle, MM};

    fn wide_region() -> Aabb {
        Aabb { min: Point::new(-20 * MM, -20 * MM), max: Point::new(20 * MM, 20 * MM) }
    }

    #[test]
    fn direct_path_when_nothing_blocks() {
        let world = Node::new();
        let resolver = FixedClearance(127_000);
        let path = find_path_grid(
            &world, std::iter::empty(), Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, Class::C, &resolver, &[], wide_region(), 100_000,
        )
        .unwrap();
        assert_eq!(path.first().copied(), Some(Point::new(0, 0)));
        assert_eq!(path.last().copied(), Some(Point::new(5 * MM, 0)));
    }

    #[test]
    fn routes_around_a_single_circular_obstacle() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(2 * MM, 0), 800_000)), net: Some(NetId(99)), layer: LayerId::FCu });

        let items: Vec<&Item> = world.iter().collect();
        let path = find_path_grid(
            &world, items.into_iter(), Point::new(0, 0), Point::new(4 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, Class::C, &resolver, &[], wide_region(), 100_000,
        )
        .unwrap();
        assert_eq!(path.first().copied(), Some(Point::new(0, 0)));
        assert_eq!(path.last().copied(), Some(Point::new(4 * MM, 0)));
        assert!(path.len() > 2, "must detour, not go straight through the obstacle");

        for w in path.windows(2) {
            assert!(
                world.path_is_clear(w[0], w[1], 250_000, Some(NetId(1)), LayerId::FCu, Class::C, &resolver),
                "every smoothed leg must be independently collision-free"
            );
        }
    }

    #[test]
    fn returns_none_when_the_target_is_fully_enclosed() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        // Four pads sealing off the origin in a tight diamond, none of
        // which individually overlaps it (so this isn't just the
        // endpoint-collision short-circuit) -- mirrors astar.rs's own
        // `sealed_diamond_around_origin` fixture.
        for (dx, dy) in [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)] {
            world.add(Item::Pad {
                shape: PadShape::Circle(Circle::new(Point::new(dx * 700_000, dy * 700_000), 500_000)),
                net: Some(NetId(99)),
                layer: LayerId::FCu,
            });
        }
        let items: Vec<&Item> = world.iter().collect();
        let result = find_path_grid(
            &world, items.into_iter(), Point::new(0, 0), Point::new(5 * MM, 5 * MM), 200_000,
            NetId(1), LayerId::FCu, Class::C, &resolver, &[], wide_region(), 50_000,
        );
        assert!(result.is_none(), "a genuinely sealed target must report no path, not a colliding one");
    }

    #[test]
    fn routes_around_a_dense_zone_that_would_explode_candidate_points() {
        // Exactly the case find_path_grid exists for: a filled zone
        // with a many-vertex boundary (astar.rs's real 42,415-vertex
        // board found this class of obstacle to be the bottleneck --
        // this test uses a much smaller synthetic comb polygon, just
        // enough vertices to be clearly impractical as per-search
        // candidate points, to keep the test itself fast).
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);

        // A "comb" polygon: a rectangle (x in [-3mm,3mm], y in
        // [-1mm,1mm]) with a zigzag of many teeth along its top edge
        // (up to y=1.5mm), generating hundreds of vertices -- narrow
        // enough that the region around it (wide_region, ±20mm) leaves
        // plenty of room to detour left/right, but wide/tall enough
        // that the direct line between `from`/`to` below and above it
        // is genuinely blocked, forcing that detour.
        let teeth: i64 = 200;
        let mut points = vec![Point::new(-3 * MM, -1 * MM), Point::new(3 * MM, -1 * MM)];
        for i in (0..teeth).rev() {
            let x0 = -3 * MM + (6 * MM * i) / teeth;
            let x1 = -3 * MM + (6 * MM * (i + 1)) / teeth;
            points.push(Point::new(x1, 1 * MM));
            points.push(Point::new(x1, 1_500_000));
            points.push(Point::new(x0, 1_500_000));
            points.push(Point::new(x0, 1 * MM));
        }
        world.add(Item::Zone {
            outline: Polygon::new(points),
            layer: LayerId::FCu,
            net: Some(NetId(99)),
        });

        let items: Vec<&Item> = world.iter().collect();
        let path = find_path_grid(
            &world, items.into_iter(), Point::new(0, -5 * MM), Point::new(0, 5 * MM), 200_000,
            NetId(1), LayerId::FCu, Class::C, &resolver, &[], wide_region(), 100_000,
        );
        assert!(path.is_some(), "must find a way around the dense comb zone");
    }
}
