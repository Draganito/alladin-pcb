//! Adaptive quadtree candidate-point generation -- a new, primary
//! source for the `points: Vec<(Point, usize)>` that
//! [`crate::astar::find_path_astar`]'s visibility-graph A* search feeds
//! into [`crate::astar`]'s (already parallelized) `build_adjacency_knn`/
//! `build_adjacency_full`/`astar_search`. See the accompanying plan
//! ("Quadtree-basierte Kandidatenpunkte als primärer Suchweg") for the
//! full design rationale; the short version:
//!
//! `astar.rs::candidate_points` today samples a fixed number of points
//! off every *obstacle's* clearance boundary (a circle gets
//! `CIRCLE_SEGMENTS` points regardless of context, a zone/polygon pad
//! gets up to `MAX_ZONE_CANDIDATE_POINTS_PER_ITEM`). That treats free
//! space as an afterthought -- it's implicitly "everywhere the sampled
//! boundary points don't forbid". This module inverts that: it
//! recursively subdivides the *search region itself* (not any one
//! obstacle) into an adaptive quadtree -- coarse "roads" through wide
//! open areas, fine resolution only where an obstacle's boundary
//! actually needs it -- and offers each free leaf's center as a
//! candidate waypoint. This is a genuinely different (and, for a dense
//! board, typically far smaller and more evenly distributed) point set
//! than obstacle-boundary sampling, but it is deliberately fed into the
//! *exact same*, already-proven KNN/A* pipeline -- no new pathfinding
//! logic, no new correctness surface beyond the classification below.
//!
//! **Correctness boundary, stated once, for every function here:**
//! nothing in this module ever decides whether a route is valid. Every
//! resulting candidate edge is still independently re-verified by
//! [`alladin_core::Node::path_is_clear`] (`is_valid_edge` in
//! `crate::astar::try_route`) before it's ever trusted, exactly like
//! every existing `candidate_points` boundary sample. Getting a
//! classification below "wrong" in the conservative direction (treating
//! something as `Ambiguous` that could safely have been called `Free`
//! or `Blocked`) only costs a slightly less optimal/complete graph,
//! never an accepted-but-actually-colliding path.

use alladin_core::{Item, LayerId, NetClass, NetId, PadShape, RuleResolver};
use alladin_geom::{
    circle_polygon_collides, circle_polygon_collides_indexed, dist_point_to_line,
    polygon_polygon_collides_indexed, segment_polygon_collides, Aabb, Circle, Point, Polygon,
    PolygonEdgeIndex, Segment, Unit,
};

/// Minimum quadtree leaf size. Deliberately reuses the grid fallback's
/// own tuned resolution constant ([`crate::grid_obstacle::DEFAULT_GRID_STEP`])
/// rather than inventing a second tunable: both exist to answer the
/// same question (how finely to resolve free space near an obstacle
/// boundary), so there's no reason for them to diverge without a
/// measured reason to.
pub const DEFAULT_QUADTREE_MIN_LEAF: Unit = crate::grid_obstacle::DEFAULT_GRID_STEP;

/// Hard ceiling on how many leaves a single [`build_quadtree`] call may
/// ever produce, regardless of how large `region` is relative to
/// `min_leaf` -- the same safety argument as
/// [`crate::grid_obstacle::GridObstacleMap`]'s `MAX_GRID_CELLS` (see
/// that constant's doc comment): a worst-case fully-ambiguous quadtree
/// (every leaf needs subdividing all the way down) has the same
/// `O((region / min_leaf)^2)` leaf count a uniform grid at that
/// resolution would. Rather than ever attempting that many leaves, the
/// effective minimum leaf size is silently doubled (see
/// [`effective_min_leaf`]) until the worst case fits this budget.
const MAX_QUADTREE_LEAF_BUDGET: i64 = 4_000_000;

#[derive(Clone, Copy, PartialEq, Eq)]
enum SquareStatus {
    Free,
    Blocked,
    Ambiguous,
}

/// One obstacle, pre-resolved (clearance already folded into its own
/// radius/margin) purely for quadtree square-vs-obstacle
/// classification -- the same per-item clearance probe
/// `astar.rs::candidate_points`/`grid_obstacle.rs::rasterize_item`
/// already use, just computed once up front here instead of per-point.
pub(crate) enum Obstacle {
    Circle { center: Point, r: Unit, bounds: Aabb },
    Capsule { a: Point, b: Point, r: Unit, bounds: Aabb },
    Polygon { index: PolygonEdgeIndex, clearance: Unit, bounds: Aabb },
}

impl Obstacle {
    fn bounds(&self) -> Aabb {
        match self {
            Obstacle::Circle { bounds, .. } => *bounds,
            Obstacle::Capsule { bounds, .. } => *bounds,
            Obstacle::Polygon { bounds, .. } => *bounds,
        }
    }

    /// Classifies `square` against this one obstacle. `Free` means
    /// *genuinely* no overlap at all (safe to drop this obstacle for
    /// every descendant of `square` too -- see [`build_node`]);
    /// `Blocked` means `square` is fully covered, convexity-proven, no
    /// further recursion needed; `Ambiguous` means "touches, but not
    /// provably either of the above" -- always safe, just costs a
    /// subdivision.
    ///
    /// **Why polygon obstacles never return `Blocked` here, only
    /// `Ambiguous`:** a real zone/[`PadShape::Polygon`] outline can be
    /// non-convex (thermal-relief/keepout notches -- see this crate's
    /// the development log, "Teil 28"'s scanline-regression lesson). For a
    /// non-convex shape, "every corner of `square` reads as inside the
    /// polygon" does *not* imply the whole square is inside it (a
    /// notch can still cut through the middle of an edge between two
    /// "inside" corners) -- so unlike the circle/capsule cases below
    /// (both provably convex), there is no cheap, sound full-
    /// containment shortcut here. Bottoming out at [`build_node`]'s
    /// minimum leaf size and resolving via [`Self::blocks_point`]
    /// (the same zero-radius-probe technique already validated in
    /// `grid_obstacle.rs::block_polygon`) is the conservative, correct
    /// trade-off: possibly a few extra subdivisions near a polygon
    /// obstacle, never a wrong answer.
    fn classify(&self, square: Aabb) -> SquareStatus {
        match self {
            Obstacle::Circle { center, r, .. } => {
                let square_poly = aabb_to_polygon(square);
                if !circle_polygon_collides(&Circle::new(*center, *r), &square_poly, 0) {
                    return SquareStatus::Free;
                }
                let farthest = square_corners(square)
                    .into_iter()
                    .map(|p| p.distance(*center))
                    .fold(0.0_f64, f64::max);
                if farthest <= *r as f64 {
                    SquareStatus::Blocked
                } else {
                    SquareStatus::Ambiguous
                }
            }
            Obstacle::Capsule { a, b, r, .. } => {
                let square_poly = aabb_to_polygon(square);
                let probe = Segment::new(*a, *b, r * 2);
                if !segment_polygon_collides(&probe, &square_poly, 0) {
                    return SquareStatus::Free;
                }
                let farthest = square_corners(square)
                    .into_iter()
                    .map(|p| dist_point_to_line(p, *a, *b))
                    .fold(0.0_f64, f64::max);
                if farthest <= *r as f64 {
                    SquareStatus::Blocked
                } else {
                    SquareStatus::Ambiguous
                }
            }
            Obstacle::Polygon { index, clearance, bounds } => {
                // `polygon_polygon_collides_indexed(candidate, index, _)`
                // only tests `candidate`'s own vertices against `index`
                // plus nearby-edge pairs (see its doc comment) -- it
                // never tests whether `index`'s polygon's own vertices
                // fall inside `candidate`. That asymmetry is invisible
                // for its original use (a small pad/segment `candidate`
                // against a huge zone `index`), but here `square` starts
                // *large* (the whole search region) and shrinks -- a
                // small polygon obstacle (e.g. a `PadShape::Polygon`
                // pad) fully nested inside `square`, with none of its
                // edges crossing `square`'s own boundary, would
                // otherwise be missed entirely and wrongly read as
                // `Free`. Caught cheaply here using the bounds already
                // computed in `build_obstacles`, without walking the
                // polygon's own (possibly tens-of-thousands-of-vertices)
                // point list.
                if aabb_contains_aabb(square, *bounds) {
                    return SquareStatus::Ambiguous;
                }
                let square_poly = aabb_to_polygon(square);
                if polygon_polygon_collides_indexed(&square_poly, index, *clearance) {
                    SquareStatus::Ambiguous
                } else {
                    SquareStatus::Free
                }
            }
        }
    }

    /// Exact point-vs-obstacle test, used only to resolve a leaf that
    /// bottomed out at the minimum size still `Ambiguous`.
    fn blocks_point(&self, p: Point) -> bool {
        match self {
            Obstacle::Circle { center, r, .. } => p.distance(*center) <= *r as f64,
            Obstacle::Capsule { a, b, r, .. } => dist_point_to_line(p, *a, *b) <= *r as f64,
            Obstacle::Polygon { index, clearance, .. } => {
                circle_polygon_collides_indexed(&Circle::new(p, 0), index, *clearance)
            }
        }
    }
}

fn aabb_intersects(a: Aabb, b: Aabb) -> bool {
    a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y && a.max.y >= b.min.y
}

fn aabb_contains_aabb(outer: Aabb, inner: Aabb) -> bool {
    outer.contains_point(inner.min) && outer.contains_point(inner.max)
}

fn square_corners(square: Aabb) -> [Point; 4] {
    [
        Point::new(square.min.x, square.min.y),
        Point::new(square.max.x, square.min.y),
        Point::new(square.max.x, square.max.y),
        Point::new(square.min.x, square.max.y),
    ]
}

fn aabb_to_polygon(square: Aabb) -> Polygon {
    Polygon::new(square_corners(square).to_vec())
}

fn split4(square: Aabb) -> [Aabb; 4] {
    let mx = (square.min.x + square.max.x) / 2;
    let my = (square.min.y + square.max.y) / 2;
    [
        Aabb { min: Point::new(square.min.x, square.min.y), max: Point::new(mx, my) },
        Aabb { min: Point::new(mx, square.min.y), max: Point::new(square.max.x, my) },
        Aabb { min: Point::new(square.min.x, my), max: Point::new(mx, square.max.y) },
        Aabb { min: Point::new(mx, my), max: Point::new(square.max.x, square.max.y) },
    ]
}

/// Recursive quadtree node. Deliberately doesn't store its own bounds
/// (recomputed by [`split4`] as callers walk the tree, exactly the
/// region they built it against) -- one less thing that could ever get
/// out of sync with the region a tree was actually built for.
pub(crate) enum QuadNode {
    Free,
    Blocked,
    Internal(Box<[QuadNode; 4]>),
}

pub(crate) struct QuadTree {
    root: QuadNode,
    region: Aabb,
}

/// Doubles `min_leaf` until a worst-case (fully ambiguous) quadtree over
/// `region` would produce at most [`MAX_QUADTREE_LEAF_BUDGET`] leaves --
/// the same "coarsen until it fits a fixed budget" safety net as
/// [`crate::grid_obstacle::GridObstacleMap::new`] (see that method's doc
/// comment); see [`MAX_QUADTREE_LEAF_BUDGET`]'s own doc comment for why
/// the worst case is the same shape as a uniform grid's cell count.
fn effective_min_leaf(region: Aabb, min_leaf: Unit) -> Unit {
    let mut step = min_leaf.max(1);
    loop {
        let w = (((region.max.x - region.min.x) as f64 / step as f64).ceil() as i64).max(1);
        let h = (((region.max.y - region.min.y) as f64 / step as f64).ceil() as i64).max(1);
        if w.saturating_mul(h) <= MAX_QUADTREE_LEAF_BUDGET {
            return step;
        }
        step *= 2;
    }
}

fn build_node(square: Aabb, min_leaf: Unit, obstacles: &[&Obstacle]) -> QuadNode {
    let mut relevant: Vec<&Obstacle> = Vec::new();
    for &obs in obstacles {
        if !aabb_intersects(square, obs.bounds()) {
            continue; // can't matter for this square or any of its descendants
        }
        match obs.classify(square) {
            SquareStatus::Blocked => return QuadNode::Blocked,
            SquareStatus::Ambiguous => relevant.push(obs),
            // Genuinely no overlap: like the AABB-reject case above,
            // this can never matter for a descendant (strict) subset of
            // `square` either, so it's safe to drop for the recursive
            // calls below.
            SquareStatus::Free => {}
        }
    }
    if relevant.is_empty() {
        return QuadNode::Free;
    }

    let w = square.max.x - square.min.x;
    let h = square.max.y - square.min.y;
    if w <= min_leaf && h <= min_leaf {
        let center = Point::new((square.min.x + square.max.x) / 2, (square.min.y + square.max.y) / 2);
        let blocked = relevant.iter().any(|o| o.blocks_point(center));
        return if blocked { QuadNode::Blocked } else { QuadNode::Free };
    }

    let quads = split4(square);
    QuadNode::Internal(Box::new([
        build_node(quads[0], min_leaf, &relevant),
        build_node(quads[1], min_leaf, &relevant),
        build_node(quads[2], min_leaf, &relevant),
        build_node(quads[3], min_leaf, &relevant),
    ]))
}

/// Builds an adaptive quadtree over `region`, classifying every node
/// against `obstacles` via [`Obstacle::classify`] (see that method's
/// doc comment for the full free/blocked/ambiguous rationale).
/// `min_leaf` is a *requested* minimum resolution, silently coarsened
/// (see [`effective_min_leaf`]) if `region` is too large relative to it
/// to stay within [`MAX_QUADTREE_LEAF_BUDGET`].
pub(crate) fn build_quadtree(region: Aabb, min_leaf: Unit, obstacles: &[Obstacle]) -> QuadTree {
    let min_leaf = effective_min_leaf(region, min_leaf);
    let refs: Vec<&Obstacle> = obstacles.iter().collect();
    let root = build_node(region, min_leaf, &refs);
    QuadTree { root, region }
}

/// Builds the [`Obstacle`] list `build_quadtree` classifies against,
/// from `items` -- mirrors `astar.rs::candidate_points`'s own per-item
/// loop exactly (same clearance probe, same "every item regardless of
/// net/layer contributes a candidate-generation hint, `is_valid_edge`
/// is what actually enforces correctness" philosophy -- see that
/// function's doc comment): no net/layer pre-filtering happens here
/// either, deliberately, to keep this a drop-in alternative point
/// *source* rather than a second, subtly-different collision oracle
/// like `GridObstacleMap` (which -- unlike this module -- *is* trusted
/// directly by the grid search, so it must filter net/layer itself).
#[allow(clippy::too_many_arguments)]
pub(crate) fn build_obstacles<'a>(
    items: impl Iterator<Item = &'a Item>,
    from: Point,
    to: Point,
    width: Unit,
    net: NetId,
    layer: LayerId,
    class: NetClass,
    resolver: &dyn RuleResolver,
) -> Vec<Obstacle> {
    let probe = Item::Track { shape: Segment::new(from, to, width), net: Some(net), layer, class };
    let mut obstacles = Vec::new();
    for item in items {
        let clearance = resolver.clearance(&probe, item) + width / 2;
        match item {
            Item::Pad { shape: PadShape::Circle(c), .. } | Item::Via { shape: c, .. } => {
                let r = c.radius + clearance;
                obstacles.push(Obstacle::Circle {
                    center: c.center,
                    r,
                    bounds: Aabb::from_circle(&Circle::new(c.center, r)),
                });
            }
            Item::Pad { shape: PadShape::Polygon { outline, .. }, .. } => {
                let index = PolygonEdgeIndex::build(outline);
                let bounds = Aabb::from_polygon(outline).inflate(clearance.max(0));
                obstacles.push(Obstacle::Polygon { index, clearance, bounds });
            }
            Item::Track { shape, .. } => {
                let r = shape.width / 2 + clearance;
                let bounds = Aabb::from_segment(&Segment::new(shape.a, shape.b, r * 2));
                obstacles.push(Obstacle::Capsule { a: shape.a, b: shape.b, r, bounds });
            }
            Item::Zone { outline, .. } => {
                let index = PolygonEdgeIndex::build(outline);
                let bounds = Aabb::from_polygon(outline).inflate(clearance.max(0));
                obstacles.push(Obstacle::Polygon { index, clearance, bounds });
            }
            Item::Hole { position, drill } => {
                let r = drill / 2 + clearance;
                obstacles.push(Obstacle::Circle { center: *position, r, bounds: Aabb::from_circle(&Circle::new(*position, r)) });
            }
        }
    }
    obstacles
}

fn collect_points(node: &QuadNode, square: Aabb, next_group: &mut usize, out: &mut Vec<(Point, usize)>) {
    match node {
        QuadNode::Free => {
            let center = Point::new((square.min.x + square.max.x) / 2, (square.min.y + square.max.y) / 2);
            let group = *next_group;
            *next_group += 1;
            out.push((center, group));
        }
        QuadNode::Blocked => {}
        QuadNode::Internal(children) => {
            for (child, child_square) in children.iter().zip(split4(square)) {
                collect_points(child, child_square, next_group, out);
            }
        }
    }
}

/// [`crate::astar::candidate_points`]'s quadtree-based counterpart: same
/// return shape (`from`/`to` first, at groups 0/1; every other point
/// with its own group; a `bool` reporting whether the global cap below
/// truncated the result), same tail behaviour (board-outline vertices
/// included, same global cap applied), so callers can swap one for the
/// other without touching anything downstream.
///
/// Where it differs is the *source* of everything between `from`/`to`
/// and the outline vertices: every free quadtree leaf's own center,
/// walked out of `tree` (see [`build_quadtree`]/[`collect_points`])
/// instead of `candidate_points`'s per-obstacle boundary sampling. A
/// free leaf that only resolved after subdividing near an obstacle
/// (small, since it bottomed out close to `tree`'s own minimum leaf
/// size) naturally reads as a "hug the obstacle's boundary" waypoint;
/// a free leaf that resolved immediately, without ever needing to
/// subdivide (only possible when *no* obstacle touches it at all, see
/// [`build_node`]), is naturally large and reads as a "road through
/// open space" waypoint -- exactly the plan's two waypoint kinds,
/// without needing to track which case produced each point separately:
/// leaf size alone already tells them apart, implicitly, for free.
pub(crate) fn quadtree_candidate_points(
    tree: &QuadTree,
    from: Point,
    to: Point,
    outline: &[Polygon],
) -> (Vec<(Point, usize)>, bool) {
    let mut points = vec![(from, 0usize), (to, 1usize)];
    let mut next_group = 2usize;
    collect_points(&tree.root, tree.region, &mut next_group, &mut points);

    // Same "hug a concave board edge with no other nearby obstacle"
    // gap-closer as `candidate_points` -- see that function's own doc
    // comment and the development log's "Teil 15" entry.
    for poly in outline {
        for vertex in poly.inward_vertices(crate::astar::OUTLINE_VERTEX_INSET) {
            points.push((vertex, next_group));
            next_group += 1;
        }
    }

    let mut truncated = false;
    if points.len() > 2 + crate::astar::MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE {
        truncated = true;
        let rest = points.split_off(2);
        points.extend(crate::astar::stride_sample(rest, crate::astar::MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE));
    }

    (points, truncated)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::{FixedClearance, NetClass as Class};
    use alladin_geom::MM;

    fn region(half: Unit) -> Aabb {
        Aabb { min: Point::new(-half, -half), max: Point::new(half, half) }
    }

    fn count_leaves(node: &QuadNode) -> (usize, usize) {
        // (free, blocked)
        match node {
            QuadNode::Free => (1, 0),
            QuadNode::Blocked => (0, 1),
            QuadNode::Internal(children) => {
                children.iter().map(count_leaves).fold((0, 0), |(f, b), (cf, cb)| (f + cf, b + cb))
            }
        }
    }

    #[test]
    fn an_empty_region_with_no_obstacles_is_a_single_free_leaf() {
        let tree = build_quadtree(region(5 * MM), DEFAULT_QUADTREE_MIN_LEAF, &[]);
        assert!(matches!(tree.root, QuadNode::Free), "no obstacles at all must resolve without ever subdividing");
    }

    #[test]
    fn a_circle_obstacle_covering_the_whole_region_is_a_single_blocked_leaf() {
        let obstacles = vec![Obstacle::Circle {
            center: Point::new(0, 0),
            r: 100 * MM, // far larger than the region below
            bounds: Aabb { min: Point::new(-100 * MM, -100 * MM), max: Point::new(100 * MM, 100 * MM) },
        }];
        let tree = build_quadtree(region(5 * MM), DEFAULT_QUADTREE_MIN_LEAF, &obstacles);
        assert!(
            matches!(tree.root, QuadNode::Blocked),
            "a circle obstacle (convex) fully covering the region must short-circuit to Blocked without subdividing"
        );
    }

    #[test]
    fn a_small_circle_obstacle_in_the_middle_produces_both_free_and_blocked_leaves() {
        let obstacles = vec![Obstacle::Circle {
            center: Point::new(0, 0),
            r: MM / 2,
            bounds: Aabb { min: Point::new(-MM / 2, -MM / 2), max: Point::new(MM / 2, MM / 2) },
        }];
        let tree = build_quadtree(region(5 * MM), DEFAULT_QUADTREE_MIN_LEAF, &obstacles);
        let (free, blocked) = count_leaves(&tree.root);
        assert!(free > 0, "must have free space far from the small obstacle");
        assert!(blocked > 0, "must have at least one blocked leaf where the obstacle sits");
    }

    #[test]
    fn a_rotated_rectangular_pad_is_never_misclassified_as_free_at_its_own_center() {
        // A 4mm x 1mm pad, rotated 45 degrees, centered at the origin --
        // exactly the "true corner sticks out further than a naive
        // inscribed-circle check would think" shape this whole slice
        // (Echte Pad-Geometrie) exists to get right.
        let rect = Polygon::rounded_rect(4 * MM, MM, 0, 1);
        let outline = Polygon::new(rect.points.into_iter().map(|p| p.rotated(45.0)).collect());
        let bounds = Aabb::from_polygon(&outline);
        let obstacles = vec![Obstacle::Polygon { index: PolygonEdgeIndex::build(&outline), clearance: 0, bounds }];
        let tree = build_quadtree(region(5 * MM), DEFAULT_QUADTREE_MIN_LEAF, &obstacles);

        // Walk the tree down to whichever leaf contains the origin and
        // confirm it reads as Blocked, not Free.
        fn leaf_at(node: &QuadNode, square: Aabb, p: Point) -> bool {
            match node {
                QuadNode::Free => false,
                QuadNode::Blocked => true,
                QuadNode::Internal(children) => {
                    for (child, child_square) in children.iter().zip(split4(square)) {
                        if child_square.contains_point(p) {
                            return leaf_at(child, child_square, p);
                        }
                    }
                    false
                }
            }
        }
        assert!(leaf_at(&tree.root, tree.region, Point::new(0, 0)), "the pad's own center must read as Blocked");
    }

    #[test]
    fn a_dense_comb_polygon_zone_never_produces_a_wrongly_free_leaf() {
        // Reuses the same "many-vertex, non-convex, deliberately
        // clustered" fixture shape `alladin_geom`'s own test suite
        // validates `PolygonEdgeIndex` against (see that crate's
        // `comb_polygon` doc comment) -- built by hand here since it's
        // a private test helper there.
        let mm = |v: f64| (v * MM as f64) as Unit;
        let mut points = vec![Point::new(mm(0.0), mm(0.0)), Point::new(mm(10.0), mm(0.0))];
        for i in 0..5 {
            let x0 = 10.0 - (i as f64 * 2.0 + 1.0) * 1.0;
            let x1 = 10.0 - (i as f64 * 2.0 + 2.0) * 1.0;
            points.push(Point::new(mm(x0 + 0.5), mm(5.0)));
            points.push(Point::new(mm(x0 + 0.5), mm(2.0)));
            points.push(Point::new(mm(x1 + 0.5), mm(2.0)));
            points.push(Point::new(mm(x1 + 0.5), mm(5.0)));
        }
        points.push(Point::new(mm(0.0), mm(5.0)));
        let zone = Polygon::new(points);
        let index = PolygonEdgeIndex::build(&zone);
        let bounds = Aabb::from_polygon(&zone);
        let obstacles = vec![Obstacle::Polygon { index, clearance: 0, bounds }];

        let search_region =
            Aabb { min: Point::new(mm(-2.0), mm(-2.0)), max: Point::new(mm(12.0), mm(7.0)) };
        let tree = build_quadtree(search_region, DEFAULT_QUADTREE_MIN_LEAF, &obstacles);

        // Every resulting Free leaf's own center must genuinely not be
        // inside the zone -- the actual correctness property this test
        // exists to pin down, cross-checked against the brute-force
        // `Polygon::contains_point`.
        fn check(node: &QuadNode, square: Aabb, zone: &Polygon) {
            match node {
                QuadNode::Free => {
                    let center =
                        Point::new((square.min.x + square.max.x) / 2, (square.min.y + square.max.y) / 2);
                    assert!(
                        !zone.contains_point(center),
                        "a Free leaf's center at {center:?} must never actually be inside the zone"
                    );
                }
                QuadNode::Blocked => {}
                QuadNode::Internal(children) => {
                    for (child, child_square) in children.iter().zip(split4(square)) {
                        check(child, child_square, zone);
                    }
                }
            }
        }
        check(&tree.root, tree.region, &zone);
    }

    #[test]
    fn effective_min_leaf_coarsens_a_pathologically_fine_request_to_fit_the_budget() {
        let huge = Aabb { min: Point::new(0, 0), max: Point::new(1000 * MM, 1000 * MM) };
        let step = effective_min_leaf(huge, 1); // 1nm, deliberately absurd
        let w = ((huge.max.x - huge.min.x) as f64 / step as f64).ceil() as i64;
        let h = ((huge.max.y - huge.min.y) as f64 / step as f64).ceil() as i64;
        assert!(w * h <= MAX_QUADTREE_LEAF_BUDGET);
    }

    #[test]
    fn quadtree_candidate_points_always_includes_from_and_to_first() {
        let obstacles = build_obstacles(
            std::iter::empty::<&Item>(),
            Point::new(0, 0),
            Point::new(5 * MM, 0),
            250_000,
            NetId(1),
            LayerId::FCu,
            Class::C,
            &FixedClearance(127_000),
        );
        let tree = build_quadtree(region(10 * MM), DEFAULT_QUADTREE_MIN_LEAF, &obstacles);
        let (points, truncated) = quadtree_candidate_points(&tree, Point::new(0, 0), Point::new(5 * MM, 0), &[]);
        assert!(!truncated);
        assert_eq!(points[0], (Point::new(0, 0), 0));
        assert_eq!(points[1], (Point::new(5 * MM, 0), 1));
        assert!(points.len() >= 3, "an open region must still offer at least one bridge waypoint");
    }

    #[test]
    fn build_obstacles_produces_one_entry_per_item() {
        let items = [
            Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)), net: Some(NetId(1)), layer: LayerId::FCu },
            Item::Track {
                shape: Segment::new(Point::new(0, 0), Point::new(MM, 0), 250_000),
                net: Some(NetId(2)),
                layer: LayerId::FCu,
                class: Class::C,
            },
            Item::Via { shape: Circle::new(Point::new(2 * MM, 0), 300_000), drill: 150_000, net: Some(NetId(3)) },
            Item::Zone { outline: Polygon::rounded_rect(2 * MM, 2 * MM, 0, 1), layer: LayerId::FCu, net: None },
            Item::Hole { position: Point::new(-2 * MM, 0), drill: 600_000 },
        ];
        let obstacles = build_obstacles(
            items.iter(),
            Point::new(-5 * MM, 0),
            Point::new(5 * MM, 0),
            250_000,
            NetId(9),
            LayerId::FCu,
            Class::C,
            &FixedClearance(127_000),
        );
        assert_eq!(obstacles.len(), items.len());
        assert!(
            matches!(obstacles.last(), Some(Obstacle::Circle { .. })),
            "a mounting hole must contribute a plain circle obstacle, same shape as a via"
        );
    }
}
