//! ASTAR: true graph search over a visibility graph, replacing the old
//! `route_single_net` engine's "resolve one colliding leg at a time"
//! loop with an actual search over every alternative waypoint at once.
//!
//! Why this matters beyond per-leg walkaround: the previous
//! `route_single_net` fixed each newly-discovered collision locally and
//! independently (walk around whichever single obstacle a straight leg
//! hits, re-check, repeat) -- it never compared that choice against a
//! genuinely different detour that might turn out globally shorter once
//! a *second* obstacle is factored in. Classic failure mode: two
//! obstacles staggered so the path must weave -- go around obstacle A's
//! accessible end, then around obstacle B's (different, possibly
//! opposite) accessible end -- where fixing A first without knowing
//! about B can lock in a locally-sensible but globally longer detour.
//!
//! The fix is the standard "visibility graph" technique: collect every
//! obstacle's clearance-inflated boundary as a set of candidate
//! waypoints, connect any two waypoints whose straight line is
//! *provably* clear (reusing the exact same [`Node::path_is_clear`]
//! check the rest of Alladin already trusts -- no separate, possibly
//! inconsistent collision logic here), and run A* over that graph. This
//! reuses the boundary-sampling building blocks from
//! `walkaround.rs`/`capsule_walkaround.rs` (they already know how to
//! turn a circle or a capsule into a set of boundary points) purely as a
//! *candidate generator* now, not as the whole algorithm.

use crate::failure::{Endpoint, FailureReason};
use alladin_core::{Item, LayerId, NetClass, NetId, Node, PadShape, RuleResolver};
use alladin_geom::{Aabb, Point, Polygon, Segment, Unit};
use rayon::prelude::*;
use rstar::{PointDistance, RTree, RTreeObject, AABB};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Whether the straight leg `a`->`b` stays on the board, given the
/// board's outline polygon(s) (see the sibling `alladin-kicad-io`
/// crate's `import_board_outline` for where these come from and how
/// they're chained). Uses [`alladin_geom::contains_segment_evenodd`], so
/// an internal cutout/hole in `outline` is correctly excluded rather
/// than treated as extra allowed area (see that function's doc comment
/// and the development log's "Teil 17" entry). An empty `outline` slice
/// means "no board-outline information available/supplied" --
/// deliberately permissive, not restrictive, matching every existing
/// caller's behaviour before this check existed (see this function's
/// own doc comment for why that's the right default here specifically,
/// unlike this codebase's usual "when unsure, be conservative" bias
/// elsewhere).
fn edge_stays_on_board(a: Point, b: Point, outline: &[Polygon]) -> bool {
    outline.is_empty() || alladin_geom::contains_segment_evenodd(outline, a, b)
}

/// How many points to sample around a circular obstacle's clearance
/// boundary. Unlike the analytic tangent formula in `walkaround.rs`,
/// approximate sampling is fine here: every candidate edge is still
/// verified exactly via `Node::path_is_clear` before being trusted, so a
/// sample point being a few degrees off the true tangent only costs a
/// slightly less optimal graph, never an incorrect path.
const CIRCLE_SEGMENTS: usize = 24;

/// Points are sampled slightly *outside* the exact minimum-clearance
/// radius, not exactly on it. Without this, a point sits precisely at
/// the pass/fail threshold `path_is_clear` checks against -- and two
/// adjacent boundary points' connecting chord dips even further inside
/// that threshold (classic polyline "sagitta" versus the true circle,
/// same issue `walkaround.rs`'s `ARC_SAFETY_FACTOR` exists for). Right at
/// that threshold, `.round() as Unit`'s sub-nanometre rounding is enough
/// to flip a handful of edges to "colliding", which can disconnect the
/// whole visibility graph (found by a real failing test, not by
/// inspection). A little extra clearance costs nothing here -- there's
/// no obstacle at that distance, only more graph -- so err generous.
const CIRCLE_SAFETY_FACTOR: f64 = 1.02;

fn circle_boundary(center: Point, radius: Unit) -> Vec<Point> {
    let sample_radius = radius as f64 * CIRCLE_SAFETY_FACTOR;
    (0..CIRCLE_SEGMENTS)
        .map(|k| {
            let angle = std::f64::consts::TAU * (k as f64) / (CIRCLE_SEGMENTS as f64);
            Point::new(
                center.x + (sample_radius * angle.cos()).round() as Unit,
                center.y + (sample_radius * angle.sin()).round() as Unit,
            )
        })
        .collect()
}

/// How far around the straight `from`→`to` corridor to look for
/// obstacles on the fast path's first attempt, expressed as a multiple
/// of the direct `from`-`to` distance. Generous enough for every
/// scenario built so far (chicanes, sequential obstacles all detour by
/// at most a couple of obstacle-widths) without pulling in every item
/// on a big board -- but this is a *performance* heuristic, not a
/// correctness guarantee: if the filtered graph has no solution,
/// [`find_path_astar`] widens the corridor in stages (see
/// `CORRIDOR_ESCALATION_FACTORS`) and, failing all of those too, falls
/// back to the complete, unfiltered candidate set (the original,
/// always-correct behaviour) before ever reporting failure. So getting
/// this constant "too small" only costs extra fallback passes, never a
/// wrong or missed route.
const CORRIDOR_MARGIN_FACTOR: f64 = 1.0;

/// Successive corridor margins [`find_path_astar`] retries with, each
/// strictly wider than `CORRIDOR_MARGIN_FACTOR`, before it ever falls
/// back to the complete, unfiltered item set. Doubling at each stage
/// means a detour that needed, say, factor 3.0 only pays for the 1.0,
/// 2.0 and 4.0 attempts, not an immediate jump to scanning the whole
/// board -- most boards have *some* wide region between the direct
/// corridor and "everything", and this reduces how often that most
/// expensive case is actually reached. This is purely a performance
/// staircase, not a correctness change: the final, always-tried stage
/// after this list is exhausted is the exact same full fallback
/// `find_path_astar` has always had, so a route that only the full
/// fallback can find is still found, just after a few more (cheap,
/// small-graph) failed attempts first.
const CORRIDOR_ESCALATION_FACTORS: [f64; 3] = [2.0, 4.0, 8.0];

/// The tight axis-aligned bounding box of every point in `points`. Used
/// to size the default grid pre-filter's grid (see [`try_route`]) to
/// the actual candidate points being connected rather than to a much
/// larger corridor-escalation region -- see that grid's own
/// construction site for why the distinction matters.
fn points_bbox(points: &[Point]) -> Aabb {
    let first = points.first().copied().unwrap_or(Point::new(0, 0));
    points.iter().fold(Aabb { min: first, max: first }, |acc, &p| Aabb {
        min: Point::new(acc.min.x.min(p.x), acc.min.y.min(p.y)),
        max: Point::new(acc.max.x.max(p.x), acc.max.y.max(p.y)),
    })
}

fn corridor_region(from: Point, to: Point, factor: f64) -> Aabb {
    let margin = (from.distance(to) * factor).round() as Unit;
    Aabb {
        min: Point::new(from.x.min(to.x), from.y.min(to.y)),
        max: Point::new(from.x.max(to.x), from.y.max(to.y)),
    }
    .inflate(margin.max(1)) // at least 1nm margin even for a zero-length corridor
}

/// Every candidate waypoint the search is allowed to route through,
/// paired with a "source group" id: `from` and `to` each get their own
/// unique group, and every point sampled from the same obstacle's
/// boundary shares one group. [`build_adjacency_knn`] uses this to
/// avoid wasting its nearest-neighbour budget on same-obstacle chords
/// (see that function's doc comment for why that matters) -- everything
/// else (`try_route`'s exhaustive fallback, `astar_search`) only ever
/// looks at the `Point` half and doesn't care about grouping at all.
///
/// How far inward (in internal units, i.e. nanometres) [`Polygon::inward_vertices`]
/// nudges each outline vertex before it's offered as a candidate
/// waypoint. Deliberately tiny -- purely enough to dodge
/// `Polygon::contains_point`'s documented boundary ambiguity, not a real
/// design-rule margin (there is no "stay N mm away from the board edge"
/// rule modelled anywhere in this codebase; see
/// [`candidate_points`]'s doc comment for the full story).
pub(crate) const OUTLINE_VERTEX_INSET: Unit = 1_000; // 1 micrometre

/// Every candidate waypoint the search is allowed to route through,
/// paired with a "source group" id: `from` and `to` each get their own
/// unique group, every point sampled from the same obstacle's boundary
/// shares one group, and every board-outline vertex gets its own unique
/// group too (adjacent outline vertices are perfectly valid to connect
/// to each other -- e.g. hugging two consecutive corners of a zigzag
/// board edge -- unlike two samples of the same circular obstacle, so
/// there's no reason to lump them together). [`build_adjacency_knn`]
/// uses this to avoid wasting its nearest-neighbour budget on
/// same-obstacle chords (see that function's doc comment for why that
/// matters) -- everything else (`try_route`'s exhaustive fallback,
/// `astar_search`) only ever looks at the `Point` half and doesn't care
/// about grouping at all.
///
/// `from`, `to`, the clearance-inflated boundary of every relevant item
/// (regardless of net/layer -- irrelevant items just contribute a few
/// harmless extra graph nodes; `path_is_clear` below is what actually
/// enforces net/layer/clearance correctness), and -- since a real board
/// outline is not always convex -- every vertex of `outline` itself,
/// nudged inward by [`OUTLINE_VERTEX_INSET`]. That last part closes a
/// previously documented gap (see the development log's "Teil 15"
/// entry): without it, a route whose only valid path hugs a concave
/// stretch of the board edge with no nearby obstacle to seed a waypoint
/// there could come back `None` even though a human router could find
/// one by eye -- `edge_stays_on_board` alone can *filter* a bad edge,
/// but was never able to *offer* a waypoint of its own. `items` is
/// supplied by the caller so it can be either the spatially pre-filtered
/// fast path or the complete fallback set -- this function itself
/// doesn't know or care which.
///
/// `corridor`: unlike every other item kind above (whose point *count*
/// is bounded by that one item's own geometry -- `CIRCLE_SEGMENTS` for a
/// pad/via, a handful of capsule points for a track, regardless of how
/// big the board around it is), an [`Item::Zone`]'s point count is
/// exactly its filled-copper polygon's own vertex count -- and a
/// *real* KiCad zone fill is nothing like the small hand-built test
/// polygons this was first validated against: found while routing a
/// real 109-LED panel's DATA daisy chain, whose 5V pour's
/// `filled_polygon` has **42,415** vertices (every pad/via/keepout
/// cutout the fill algorithm routed copper around adds its own cluster
/// of boundary segments). Feeding all of those into `outward_edge_boundary`
/// (three points per vertex, see that method's docs) yields well over a
/// hundred thousand candidate points for a *single* net -- both the KNN
/// graph build and, if that fails, `build_adjacency_full`'s O(n^2) pass
/// become effectively unusable at that size (this is what silently hung
/// the first real-board run, not an infinite loop: `build_adjacency_full`
/// alone is on the order of 10^10 pair checks at that n). Since
/// `find_path_astar` only ever wants an edge whose candidate points sit
/// somewhere it's actually searching, a zone's boundary points outside
/// the current search corridor can never usefully connect two points
/// both inside it anyway (any edge leaving the corridor already fails
/// `is_valid_edge`'s real-world -- not just candidate-graph -- checks
/// far less often than it fails simply for being enormous), so they're
/// dropped up front here rather than generated and then never used.
/// `None` means "no corridor filter" (every zone vertex is offered) --
/// used only when the caller has nothing better to bound the search
/// with; see `find_path_astar`'s own doc comment for how it picks one
/// even for its final, otherwise-unfiltered fallback pass.
/// Hard cap on how many boundary candidate points a *single*
/// [`Item::Zone`] (or, since the "Echte Pad-Geometrie" slice, a single
/// non-round [`alladin_core::PadShape::Polygon`] pad -- see
/// `polygon_boundary_candidates`, shared by both) may contribute to one
/// stage's candidate graph, regardless of how many of its own vertices
/// happen to fall inside `corridor`. Necessary for the same underlying
/// reason
/// [`MAX_FULL_FALLBACK_POINTS`] exists (see that constant's doc comment
/// for the real 42,415-vertex zone this was found against), but a
/// distinct problem from the one it solves: even the *corridor-filtered*
/// subset of a real zone's boundary can still be well over 100,000
/// points for a route spanning a large fraction of a densely-detailed
/// board -- measured 135,442 total candidate points, almost all from
/// one zone, for a single real net's first (smallest) corridor stage on
/// the 109-LED panel board. `build_adjacency_knn`'s cost is genuinely
/// linear in candidate count (`KNN_MAX_CONSIDERED` collision checks per
/// point), not the `O(n^2)` blowup `MAX_FULL_FALLBACK_POINTS` guards
/// against, but linear-in-135,442 is still far too slow in aggregate:
/// measured 62 seconds to build the KNN graph alone for that one stage,
/// which then still failed to find a path, before the *next*
/// (necessarily even larger) corridor stage would have to repeat the
/// same cost -- this is what actually hung real-board routing, not the
/// per-point collision-check cost `alladin_geom::PolygonEdgeIndex`
/// already fixed (that fix was necessary but insufficient on its own:
/// it makes each of those millions of checks cheap, but doesn't reduce
/// how many of them there are).
///
/// Safe to subsample down to this cap rather than keep every point:
/// every resulting candidate *edge* is still independently re-verified
/// by the exact `is_valid_edge`/[`crate::Node::path_is_clear`]-equivalent
/// check before ever being trusted (same guarantee
/// [`KNN_MAX_CONSIDERED`]'s own doc comment already relies on for its
/// own, smaller-scale pruning) -- discarding *which* boundary points get
/// offered as waypoints only changes which detours the search is able
/// to *find*, never whether an accepted one is actually clear.
const MAX_ZONE_CANDIDATE_POINTS_PER_ITEM: usize = 400;

/// Global safety net across *every* obstacle in a stage combined, not
/// just zones -- [`MAX_ZONE_CANDIDATE_POINTS_PER_ITEM`] alone isn't
/// always enough: a real, densely-populated board can have hundreds of
/// *individually* small, already-bounded obstacles (a plain circular
/// pad only ever contributes [`CIRCLE_SEGMENTS`] points, regardless of
/// board size -- see that constant's doc comment) whose combined count
/// still overwhelms `build_adjacency_knn` even with not a single
/// oversized zone in sight. Found on the real 109-LED panel board's
/// longest (83.7mm, versus an 8.8mm median) DATA net: its wide corridor
/// legitimately spans 944 nearby pads, 24 points each, ~23,000 points
/// total -- 9.7 seconds to build the KNN graph for just the first of
/// five escalating corridor stages, versus microseconds for every
/// shorter net on the same board. Set equal to
/// [`MAX_FULL_FALLBACK_POINTS`] deliberately: past that many points,
/// the full-fallback pass is already skipped as impractical (see that
/// constant's doc comment), so there is no reason for the KNN graph
/// alone to keep paying for more than that either.
///
/// Same safety argument as `MAX_ZONE_CANDIDATE_POINTS_PER_ITEM`: every
/// resulting candidate edge is still independently re-verified before
/// being trusted, so subsampling only changes which detours the search
/// is able to *find*, never whether an accepted one is actually clear.
pub(crate) const MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE: usize = MAX_FULL_FALLBACK_POINTS;

/// Deterministically thins `items` down to at most `max` entries by
/// taking an even stride through the sequence, rather than e.g. only
/// the first `max`. Used both for one zone's own boundary trace (or
/// however many separate runs of it survived the corridor filter) and
/// for the combined candidate list across every obstacle in a stage --
/// in both cases a stride keeps roughly even coverage along whatever
/// order the caller built the sequence in, instead of biasing toward
/// whichever part of it happened to come first.
pub(crate) fn stride_sample<T>(items: Vec<T>, max: usize) -> Vec<T> {
    if items.len() <= max || max == 0 {
        return items;
    }
    let stride = items.len().div_ceil(max);
    items.into_iter().step_by(stride).collect()
}

/// Shared "polygon offset boundary, filtered to the search corridor and
/// capped per item" candidate-point logic -- originally only needed for
/// [`Item::Zone`] outlines, now also reused for a [`PadShape::Polygon`]
/// pad (a non-round footprint's true, DFM-exact shape): both are a
/// filled polygon obstacle the A* visibility graph needs to trace
/// *around* a corner of, not a convex circle/capsule tangent-visible
/// from both sides at once. See [`MAX_ZONE_CANDIDATE_POINTS_PER_ITEM`]'s
/// doc comment for why the per-item cap exists and `candidate_points`'s
/// own former `Item::Zone`-only doc comment (now here) for why every
/// resulting point gets its own unique KNN group: unlike a circle's
/// boundary samples, a polygon's offset points genuinely need
/// corner-to-corner hops to route around a corner the direct line can't
/// see past, and every resulting edge is still independently
/// re-verified by `is_valid_edge` before being trusted regardless.
fn polygon_boundary_candidates(
    outline: &Polygon,
    clearance: Unit,
    corridor: Option<Aabb>,
    next_group: &mut usize,
    points: &mut Vec<(Point, usize)>,
) {
    let in_corridor: Vec<Point> = outline
        .outward_edge_boundary(clearance)
        .into_iter()
        .filter(|&p| !corridor.is_some_and(|c| !c.contains_point(p)))
        .collect();
    for p in stride_sample(in_corridor, MAX_ZONE_CANDIDATE_POINTS_PER_ITEM) {
        let group = *next_group;
        *next_group += 1;
        points.push((p, group));
    }
}

fn candidate_points<'a>(
    items: impl Iterator<Item = &'a Item>,
    from: Point,
    to: Point,
    width: Unit,
    net: NetId,
    layer: LayerId,
    class: NetClass,
    resolver: &dyn RuleResolver,
    outline: &[Polygon],
    corridor: Option<Aabb>,
) -> (Vec<(Point, usize)>, bool) {
    let mut points = vec![(from, 0usize), (to, 1usize)];
    let mut next_group = 2usize;
    let probe = Item::Track {
        shape: Segment::new(from, to, width),
        net: Some(net),
        layer,
        class,
    };

    for item in items {
        let clearance = resolver.clearance(&probe, item) + width / 2;
        match item {
            Item::Pad { shape: PadShape::Circle(c), .. } | Item::Via { shape: c, .. } => {
                let boundary = circle_boundary(c.center, c.radius + clearance);
                let group = next_group;
                next_group += 1;
                points.extend(boundary.into_iter().map(|p| (p, group)));
            }
            // A non-round pad's true, DFM-exact outline (see
            // `alladin_core::PadShape`'s doc comment for why this exists
            // at all): reuses the exact same "offset boundary, corridor-
            // filtered, per-item capped" logic as `Item::Zone` below --
            // see `polygon_boundary_candidates`'s doc comment for why a
            // polygon obstacle needs this rather than the plain
            // `circle_boundary` above. This is the one place the A*
            // search itself learns that a pad's corner may stick out
            // further than the old inscribed-circle approximation ever
            // did; without it, `Node::is_colliding` would correctly
            // reject a too-close edge but the search would never find a
            // real detour around the pad's *true* corner.
            Item::Pad { shape: PadShape::Polygon { outline, .. }, .. } => {
                polygon_boundary_candidates(outline, clearance, corridor, &mut next_group, &mut points);
            }
            Item::Track { shape, .. } => {
                let boundary =
                    crate::capsule_walkaround::capsule_boundary(shape, shape.width / 2 + clearance);
                let group = next_group;
                next_group += 1;
                points.extend(boundary.into_iter().map(|p| (p, group)));
            }
            Item::Zone { outline, .. } => {
                polygon_boundary_candidates(outline, clearance, corridor, &mut next_group, &mut points);
            }
            // A mounting hole is a plain circular keep-out, same shape
            // as the `Pad{Circle}`/`Via` arm above -- no copper, but
            // geometrically identical (see `Item::Hole`'s own doc
            // comment).
            Item::Hole { position, drill } => {
                let boundary = circle_boundary(*position, drill / 2 + clearance);
                let group = next_group;
                next_group += 1;
                points.extend(boundary.into_iter().map(|p| (p, group)));
            }
        };
    }

    // Note on hole polygons (see `edge_stays_on_board`'s doc comment and
    // `alladin_geom::contains_point_evenodd`): `inward_vertices` nudges
    // a vertex toward *that specific polygon's own* interior, which for
    // a hole polygon is the excluded cutout area itself, not the board.
    // A candidate point sitting just inside a hole is harmless, not
    // wrong: any edge touching it still has to pass `edge_stays_on_board`
    // (even-odd aware) to be accepted, which correctly rejects it --
    // it just becomes a dead-end graph node nobody ever connects
    // through, not a hole in the routing logic's actual correctness.
    for poly in outline {
        for vertex in poly.inward_vertices(OUTLINE_VERTEX_INSET) {
            let group = next_group;
            next_group += 1;
            points.push((vertex, group));
        }
    }

    // Global cap across every obstacle combined -- see
    // `MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE`'s doc comment for why the
    // per-zone cap above isn't always enough on its own. `from`/`to`
    // (always the first two entries pushed above) are exempt: unlike
    // every other point here, they're mandatory, not optional.
    //
    // The returned `bool` tells the caller whether this happened: it
    // means the *true* obstacle count was too large to fit even
    // `build_adjacency_full`'s own `MAX_FULL_FALLBACK_POINTS` budget
    // (this cap is set equal to it, see its doc comment), so callers
    // must not use the now-thinned `points.len()` alone to decide
    // whether the exhaustive fallback is still practical -- it would
    // wrongly look small enough *because* it was just thinned down to
    // fit, defeating the very budget it was thinned against.
    let mut truncated = false;
    if points.len() > 2 + MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE {
        truncated = true;
        let rest = points.split_off(2);
        points.extend(stride_sample(rest, MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE));
    }

    (points, truncated)
}

/// One [`Point`] wrapped for `rstar` indexing by its position in a
/// `candidate_points` output slice -- the whole reason for this wrapper
/// (rather than indexing `Point` itself) is that [`build_adjacency_knn`]
/// needs to map a query result straight back to an adjacency-list row.
#[derive(Clone, Copy)]
struct IndexedPoint {
    idx: usize,
    point: Point,
}

impl RTreeObject for IndexedPoint {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_point([self.point.x as f64, self.point.y as f64])
    }
}

impl PointDistance for IndexedPoint {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dx = self.point.x as f64 - point[0];
        let dy = self.point.y as f64 - point[1];
        dx * dx + dy * dy
    }
}

/// How many *successfully connectable* neighbours [`build_adjacency_knn`]
/// tries to find for each point before it stops looking (in nearest-
/// first order). This is a target, not a hard budget on attempts -- see
/// [`KNN_MAX_CONSIDERED`] for that.
///
/// This is the fix for a real performance trap found while validating
/// board-outline enforcement against a real 1511-item board
/// (`interf_u.kicad_pcb`, see the development log's "Teil 14" entry):
/// the previous "connect every candidate point to every other candidate
/// point" graph is O(n^2) in candidate count, and a long route on a
/// dense real board can easily produce several thousand candidate
/// points (many obstacles x `CIRCLE_SEGMENTS` boundary samples each) --
/// tens of millions of `Node::path_is_clear` calls, each itself a
/// non-trivial R-tree query. In practice almost none of those pairs
/// matter: a shortest path's next hop is essentially always one of a
/// handful of *geometrically nearby* candidates, never a point on the
/// far side of the board.
const KNN_TARGET_EDGES: usize = 6;

/// Hard cap on how many nearest *other-group* candidates
/// [`build_adjacency_knn`] will examine for a single point, win or lose.
/// Needed because [`KNN_TARGET_EDGES`] alone doesn't bound worst-case
/// cost: a point buried in a genuinely dense cluster (e.g. inside a
/// fine-pitch connector footprint, discovered while measuring this
/// against the real board above) can have dozens of its nearest
/// candidates all rejected in a row -- most of that cluster's own
/// candidate-to-candidate lines pass through a third pin's clearance --
/// so without a ceiling, that single point would keep consuming the
/// *entire* candidate list looking for a success that might not exist.
const KNN_MAX_CONSIDERED: usize = 80;

/// For each of `points`, keep pulling its geometrically nearest
/// *other-group* candidate points (see `candidate_points`'s doc comment
/// for what a "group" is) in nearest-first order, testing each against
/// `is_valid_edge`, until either [`KNN_TARGET_EDGES`] connections are
/// found or [`KNN_MAX_CONSIDERED`] candidates have been examined.
///
/// **Why "other-group", not just "other point":** an obstacle's own
/// boundary is sampled at `CIRCLE_SEGMENTS` points hugging its own
/// clearance circle -- so a point's geometrically nearest neighbours are
/// overwhelmingly *other samples of that same obstacle*, and the chord
/// between two nearby samples on the same circle almost always cuts
/// back inside that very obstacle's own clearance zone (the classic
/// "sagitta" issue `CIRCLE_SAFETY_FACTOR` exists for elsewhere).
/// Excluding same-group candidates (they don't count against the
/// budget either, exactly like skipping the point itself) was a real,
/// measured fix, not a hypothetical one: without it, every one of a
/// point's neighbour slots got wasted on always-invalid same-obstacle
/// chords, finding *zero* usable cross-obstacle edges for most points on
/// the real `interf_u.kicad_pcb` board this was tuned against (see
/// the development log's "Teil 14" entry), and falling straight through
/// to the exhaustive `O(n^2)` fallback on almost every non-trivial
/// query -- silently defeating the whole optimization.
///
/// **Measured, honest limit of this whole approach, not hidden:**
/// "nearest by raw distance" is still a fundamentally weak predictor of
/// "actually connectable" on a genuinely dense real board. Instrumented
/// runs against `interf_u.kicad_pcb` (1511 items) showed only ~1-3% of
/// *geometrically nearest* other-group candidate pairs actually pass
/// `is_valid_edge` -- most nearby cross-obstacle lines still cut through
/// a *third* obstacle sitting between them in a dense area (e.g. inside
/// a fine-pitch connector footprint) -- and quadrupling
/// [`KNN_MAX_CONSIDERED`] from 80 to 300 measurably grew per-point
/// search cost (~4x more `is_valid_edge` calls) while barely moving the
/// needle on how many points stayed completely isolated. In short: this
/// pruned graph reliably wins for small-to-moderate candidate sets (see
/// `alladin-router`'s test suite, all comfortably sub-100ms with it),
/// but for a route whose direct corridor spans most of a very densely
/// populated board, it usually fails outright and the exhaustive
/// fallback below still has to do the real (slow) work -- this is a
/// **documented, unsolved, real limitation**, not silently swept under
/// the rug; see the development log's "Status" section. Properly
/// fixing it needs a structurally different approach (e.g. a proper
/// polygon-sweep visibility-graph construction, or shrinking the
/// candidate-point count itself instead of pruning edges over a fixed
/// one), tracked as real follow-up work, not attempted here.
///
/// Never a correctness risk regardless: callers must (and
/// [`try_route`], the only caller, does) treat "no path found" from this
/// graph as "try the exact [`build_adjacency_full`] graph before
/// concluding there's truly no route".
///
/// **Parallel across CPU cores, via `rayon`:** every point's row is a
/// self-contained nearest-neighbour search against the read-only `tree`
/// plus calls to `is_valid_edge` (itself a read-only
/// [`alladin_core::Node::path_is_clear`] check) -- no data any two
/// points' rows both need to mutate. This directly targets the real,
/// documented cost of this function (see this module's own doc comment
/// above, and [`MAX_ZONE_CANDIDATE_POINTS_PER_ITEM`]/
/// [`MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE`]'s: up to ~4000 points times
/// up to [`KNN_MAX_CONSIDERED`] checks each, measured single-threaded at
/// several seconds to over a minute on a dense real board). One
/// consequence of going parallel: each point's search can no longer
/// "coast" on an edge a *different* point already found back to it (the
/// old single-threaded version's `found = adjacency[i].len()` seed) --
/// every point now always runs its own full search up to
/// [`KNN_TARGET_EDGES`]/[`KNN_MAX_CONSIDERED`], and [`symmetrize`]
/// reconciles the two directions afterwards. This can find a slightly
/// different (never a *worse* one -- KNN is already just a heuristic,
/// see this function's own doc comment above) edge set than before;
/// never a correctness concern, since [`build_adjacency_full`] remains
/// the exhaustive fallback whenever this graph doesn't yield a path.
fn build_adjacency_knn(
    points: &[(Point, usize)],
    is_valid_edge: &(dyn Fn(Point, Point) -> bool + Sync),
) -> Vec<Vec<(usize, f64)>> {
    let indexed: Vec<IndexedPoint> = points
        .iter()
        .enumerate()
        .map(|(idx, &(point, _group))| IndexedPoint { idx, point })
        .collect();
    let tree = RTree::bulk_load(indexed);

    let rows: Vec<Vec<(usize, f64)>> = points
        .par_iter()
        .enumerate()
        .map(|(i, &(pi, group_i))| {
            let query = [pi.x as f64, pi.y as f64];
            let mut considered = 0usize;
            let mut found = 0usize;
            let mut row = Vec::new();
            for neighbor in tree.nearest_neighbor_iter(&query) {
                if found >= KNN_TARGET_EDGES || considered >= KNN_MAX_CONSIDERED {
                    break;
                }
                let j = neighbor.idx;
                if j == i || points[j].1 == group_i {
                    continue; // self or same source obstacle: doesn't count against the budget
                }
                considered += 1;
                if is_valid_edge(pi, points[j].0) {
                    row.push((j, pi.distance(points[j].0)));
                    found += 1;
                }
            }
            row
        })
        .collect();

    symmetrize(rows)
}

/// Fills in whichever reverse `j -> i` edges are missing after
/// [`build_adjacency_knn`]/[`build_adjacency_full`]'s parallel phase
/// only ever wrote each point's own, one-directional `i -> j` findings
/// (each row written by exactly one thread, so no lock/contention is
/// needed during that phase) -- run single-threaded afterwards since the
/// total edge count here is small (a low multiple of `n`), so the
/// sequential cost is negligible next to the collision checks that
/// found them. `d` is a plain Euclidean distance, so `i`'s own
/// already-computed distance is exactly what the reverse edge needs
/// too, no recomputation required.
fn symmetrize(mut rows: Vec<Vec<(usize, f64)>>) -> Vec<Vec<(usize, f64)>> {
    let n = rows.len();
    for i in 0..n {
        let edges = rows[i].clone();
        for (j, d) in edges {
            if !rows[j].iter().any(|&(k, _)| k == i) {
                rows[j].push((i, d));
            }
        }
    }
    rows
}

/// The exhaustive, always-correct visibility graph: every pair of
/// `points` connected iff `is_valid_edge` allows it. `O(n^2)` in
/// candidate count -- see [`build_adjacency_knn`]'s doc comment for why
/// that matters and what tries to avoid paying this cost in practice.
///
/// Parallel across CPU cores, exactly like [`build_adjacency_knn`]: each
/// point `i` independently checks every `j > i` (a self-contained,
/// read-only slice of the full `O(n^2)` pair space), and [`symmetrize`]
/// adds the reverse `j -> i` edges afterwards.
fn build_adjacency_full(points: &[Point], is_valid_edge: &(dyn Fn(Point, Point) -> bool + Sync)) -> Vec<Vec<(usize, f64)>> {
    let n = points.len();
    let rows: Vec<Vec<(usize, f64)>> = (0..n)
        .into_par_iter()
        .map(|i| {
            let mut row = Vec::new();
            for j in (i + 1)..n {
                if is_valid_edge(points[i], points[j]) {
                    row.push((j, points[i].distance(points[j])));
                }
            }
            row
        })
        .collect();

    symmetrize(rows)
}

/// Hard ceiling on how many candidate points [`try_route`] will ever
/// build the exhaustive `O(n^2)` [`build_adjacency_full`] fallback graph
/// for. Found necessary routing a real board's DATA daisy chain (see
/// [`candidate_points`]'s doc comment on its `corridor` parameter for
/// the full story): once a net genuinely has no valid same-layer path at
/// all -- the documented real case being one of a handful of nets whose
/// actual solution needed a brief via/B.Cu hop, which `alladin-router`
/// can't insert on its own yet -- the KNN graph correctly finds nothing,
/// and *every* corridor-escalation stage (up to and including the
/// outline-bounded final fallback) is retried in turn, each one
/// rebuilding that same board's real, densely-detailed zone's candidate
/// set from scratch. Above this many points, `build_adjacency_full` is
/// squarely in "would not finish in a user's lifetime" territory (this
/// many points is already `8*10^6` pair checks; that real board's zone
/// alone produced well over `10^5`), so it's skipped outright rather
/// than attempted -- turning an indefinite hang into a bounded-time (if
/// occasionally over-eager) "not found" for a case that would have
/// taken forever to prove impossible either way. Deliberately
/// conservative, not tuned precisely: the KNN graph built just before
/// this already covers the overwhelming majority of real, findable
/// routes (see this module's own top-level doc comment); this cap only
/// ever discards the *exhaustive* graph's small residual chance of
/// finding something KNN missed, and only for exactly the pathological
/// candidate-count cases where paying for it isn't practical anyway.
const MAX_FULL_FALLBACK_POINTS: usize = 4_000;

#[allow(clippy::too_many_arguments)]
fn try_route<'a>(
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
    corridor: Option<Aabb>,
) -> Option<Vec<Point>> {
    let dbg = std::env::var("ALLADIN_DEBUG_TIMING").is_ok();
    // Collected once (rather than left as a lazy iterator) so it can be
    // walked twice: once for whichever candidate-point source runs
    // below, once more for the default grid pre-filter's rasterization
    // pass (see below) -- both need the exact same item set, and
    // re-querying `world`'s spatial index a second time would be
    // wasted work for no benefit.
    let items: Vec<&Item> = items.collect();
    let t0 = std::time::Instant::now();
    let (grouped_points, truncated) = if std::env::var("ALLADIN_QUADTREE_CANDIDATES").is_ok() {
        // Opt-in, alternative primary candidate-point source -- see
        // `quadtree_candidates` module doc comment and the
        // "Quadtree-basierte Kandidatenpunkte" plan for the full
        // rationale. Gated exactly like `ALLADIN_GRID_FALLBACK`/
        // `ALLADIN_DEBUG_TIMING` while it's still being validated
        // against real boards (see this module's own
        // `ALLADIN_GRID_FALLBACK`-era precedent for why: opt-in first,
        // default later, once measured).
        let region = corridor
            .unwrap_or_else(|| corridor_region(from, to, *CORRIDOR_ESCALATION_FACTORS.last().unwrap()));
        let obstacles = crate::quadtree_candidates::build_obstacles(
            items.iter().copied(),
            from,
            to,
            width,
            net,
            layer,
            class,
            resolver,
        );
        let tree = crate::quadtree_candidates::build_quadtree(
            region,
            crate::quadtree_candidates::DEFAULT_QUADTREE_MIN_LEAF,
            &obstacles,
        );
        crate::quadtree_candidates::quadtree_candidate_points(&tree, from, to, outline)
    } else {
        candidate_points(items.iter().copied(), from, to, width, net, layer, class, resolver, outline, corridor)
    };
    let points: Vec<Point> = grouped_points.iter().map(|&(p, _group)| p).collect();
    if dbg {
        eprintln!("  [timing] candidate_points: {} points in {:?}", points.len(), t0.elapsed());
    }
    let is_valid_edge = |a: Point, b: Point| {
        world.path_is_clear(a, b, width, Some(net), layer, class, resolver)
            && edge_stays_on_board(a, b, outline)
    };

    // Default (as of 2026-08-01; was `ALLADIN_KNN_GRID_PREFILTER`-gated
    // while being measured -- see the development log's "Neunzehnter
    // MVP-Slice" entry): a cheap early-reject pass in front of
    // `is_valid_edge` for `build_adjacency_knn` specifically (never for
    // the exhaustive `build_adjacency_full` fallback below, which stays
    // exactly as simple/correct as it's always been). Rasterizes the
    // same item set into a `GridObstacleMap` once, then lets
    // `GridObstacleMap::segment_definitely_blocked` skip the real,
    // exact collision check for whichever candidate edges it can already
    // prove blocked. See that method's own doc comment for the one-
    // directional safety argument (a grid "yes" may replace the real
    // check; a grid "no" never does) -- this can only reduce wasted
    // `Node::path_is_clear` calls, never accept an edge the real check
    // would have rejected; unlike `ALLADIN_GRID_FALLBACK`/
    // `ALLADIN_QUADTREE_CANDIDATES` (structurally different alternative
    // search paths, still opt-in pending further real-board validation),
    // this is a strictly one-directional filter in front of the
    // existing, unchanged correctness path, and was promoted to the
    // default after three synthetic benchmarks (dense grid, sparse long
    // route, pathological fine-pitch cluster) all measured real speedups
    // with zero regressions in path-finding outcome.
    //
    // **Deliberately sized to `points`' own bounding box, not
    // `corridor`:** a late escalation stage's `corridor` (see
    // `CORRIDOR_ESCALATION_FACTORS`) can be many times wider than the
    // candidate points it actually produced (its margin exists to widen
    // *which items get considered*, not to bound where points end up),
    // and `GridObstacleMap::new` silently coarsens its cell size to fit
    // `MAX_GRID_CELLS` -- found, while measuring this very feature, to
    // coarsen a real dense-scene grid 4x past `DEFAULT_GRID_STEP`
    // (0.05mm -> 0.2mm) purely because of an oversized region, at which
    // point quantization noise routinely exceeds a real clearance
    // margin and the pre-filter rejects nearly everything (still
    // *safe*, per this method's one-directional argument, but
    // pointless: an empty KNN graph forces the expensive exhaustive
    // fallback on every query instead of saving work). Sizing to the
    // points actually being connected keeps the grid at its intended
    // resolution for exactly the dense, tightly-packed scenes this
    // feature targets.
    let t0b = std::time::Instant::now();
    let region = points_bbox(&points).inflate(crate::grid_obstacle::DEFAULT_GRID_STEP * 4);
    let mut grid = crate::grid_obstacle::GridObstacleMap::new(region, crate::grid_obstacle::DEFAULT_GRID_STEP);
    grid.rasterize_items(items.iter().copied(), from, to, width, net, layer, class, resolver);
    if dbg {
        eprintln!("  [timing] grid_prefilter build: {:?}", t0b.elapsed());
    }
    let is_valid_edge_knn = |a: Point, b: Point| {
        if grid.segment_definitely_blocked(a, b, layer) {
            return false;
        }
        is_valid_edge(a, b)
    };

    let t1 = std::time::Instant::now();
    let pruned = build_adjacency_knn(&grouped_points, &is_valid_edge_knn);
    if dbg {
        eprintln!("  [timing] build_adjacency_knn: {:?}", t1.elapsed());
    }
    let t2 = std::time::Instant::now();
    if let Some(path) = astar_search(&points, &pruned, 0, 1) {
        if dbg {
            eprintln!("  [timing] astar_search (knn graph): {:?} -- FOUND", t2.elapsed());
        }
        return Some(path);
    }
    if dbg {
        eprintln!("  [timing] astar_search (knn graph): {:?} -- not found", t2.elapsed());
    }

    // The k-nearest-neighbour graph above is a performance heuristic,
    // not a correctness guarantee (see its own doc comment) -- fall
    // back to the exhaustive O(n^2) graph before ever reporting failure,
    // unless there are simply too many points for that to be practical
    // at all (see `MAX_FULL_FALLBACK_POINTS`'s doc comment). `truncated`
    // must be checked too, not just `points.len()`: see
    // `candidate_points`'s doc comment on its own return value for why.
    if truncated || points.len() > MAX_FULL_FALLBACK_POINTS {
        if dbg {
            eprintln!("  [timing] skipping full fallback: {} points (truncated={truncated}) > cap", points.len());
        }
        return None;
    }
    let t3 = std::time::Instant::now();
    let full = build_adjacency_full(&points, &is_valid_edge);
    if dbg {
        eprintln!("  [timing] build_adjacency_full: {:?}", t3.elapsed());
    }
    astar_search(&points, &full, 0, 1)
}

#[derive(Clone, Copy)]
struct HeapEntry {
    priority: f64,
    index: usize,
}

impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}
impl Eq for HeapEntry {}

impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reversed: `BinaryHeap` is a max-heap, and we want the entry
        // with the *smallest* priority (f = g + h) to come out first.
        other.priority.total_cmp(&self.priority)
    }
}

/// Standard A* over an explicit adjacency list, with the Euclidean
/// distance to `goal` as the heuristic -- admissible because it can
/// never overestimate the true remaining path length (straight-line
/// distance is always a lower bound), so the result is a provably
/// shortest path through the graph, not just *a* path.
fn astar_search(points: &[Point], adjacency: &[Vec<(usize, f64)>], start: usize, goal: usize) -> Option<Vec<Point>> {
    let n = points.len();
    let mut g_score = vec![f64::INFINITY; n];
    let mut came_from: Vec<Option<usize>> = vec![None; n];
    let mut closed = vec![false; n];
    let mut open = BinaryHeap::new();

    g_score[start] = 0.0;
    open.push(HeapEntry { priority: points[start].distance(points[goal]), index: start });

    while let Some(HeapEntry { index: current, .. }) = open.pop() {
        if current == goal {
            let mut path = vec![points[goal]];
            let mut cur = goal;
            while let Some(prev) = came_from[cur] {
                path.push(points[prev]);
                cur = prev;
            }
            path.reverse();
            return Some(path);
        }
        if closed[current] {
            continue; // stale heap entry from before a cheaper g_score was found
        }
        closed[current] = true;

        for &(neighbor, weight) in &adjacency[current] {
            if closed[neighbor] {
                continue;
            }
            let tentative = g_score[current] + weight;
            if tentative < g_score[neighbor] {
                g_score[neighbor] = tentative;
                came_from[neighbor] = Some(current);
                let priority = tentative + points[neighbor].distance(points[goal]);
                open.push(HeapEntry { priority, index: neighbor });
            }
        }
    }

    None
}

/// Find the shortest `from` → `to` path through `world`, routing around
/// every obstacle at once via a visibility-graph A* search (see module
/// docs). Returns `None` only if no combination of candidate waypoints
/// connects `from` to `to` at all (e.g. `from`/`to` themselves are inside
/// an obstacle's clearance zone, or obstacles fully enclose one of them).
///
/// `outline`: the board's boundary polygon(s), or `&[]` for "not
/// checked" (every caller before this parameter existed, and every
/// caller that doesn't have `.kicad_pcb` outline data to give). When
/// non-empty, every candidate edge must additionally stay fully inside
/// at least one outline polygon (see [`edge_stays_on_board`]) -- so a
/// route can no longer exit the physical board. The outline's own
/// vertices are also fed into [`candidate_points`] (nudged inward, see
/// [`OUTLINE_VERTEX_INSET`]) -- so a route whose only valid path
/// requires hugging a concave stretch of the board edge, with no other
/// obstacle nearby to seed a waypoint there, can still be found (see
/// the development log's "Teil 15" entry for the story of closing that
/// gap; it used to be a documented limitation here).
///
/// **Performance note, found while validating this against a real
/// 1511-item board:** if `from` or `to` itself isn't on the board at
/// all, this returns `None` immediately rather than building a
/// candidate graph -- every edge touching an off-board endpoint would
/// fail [`edge_stays_on_board`] anyway, so without this short-circuit
/// the corridor-limited fast path would *always* fail for such a
/// request (not because of any obstacle, purely because the endpoint is
/// off-board), unconditionally triggering the expensive whole-board
/// fallback pass on every single off-board query -- turning an instant
/// "obviously invalid" answer into a multi-second-or-worse one on a
/// real densely-populated board. This only short-circuits the "endpoint
/// is off-board" case; a request whose endpoints are both on-board but
/// whose only valid detour hugs a concave board edge still hits the
/// existing (already-documented) fallback cost.
pub fn find_path_astar(
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
    let on_board = |p: Point| outline.is_empty() || alladin_geom::contains_point_evenodd(outline, p);
    if !on_board(from) || !on_board(to) {
        return None;
    }

    // Second cheap short-circuit, found the same way as the outline one
    // above (see this function's doc comment): if `from` or `to` itself
    // already collides with an existing *different-net* item -- e.g. a
    // caller-supplied coordinate that turns out to sit inside another
    // net's pad -- then literally every candidate edge touching that
    // endpoint is doomed before it's even built, since a longer segment
    // sharing that same start point collides too (whatever the endpoint
    // overlaps doesn't get any less overlapped by extending the segment
    // away from it). Without this, such a request still correctly
    // returns `None` eventually, just only after paying the full
    // exhaustive-fallback cost below to prove it -- exactly the kind of
    // "instant rejection turned into a multi-second one" trap the
    // off-board check above already guards against.
    let endpoint_is_clear = |p: Point| {
        let probe = Item::Track { shape: Segment::new(p, p, width), net: Some(net), layer, class };
        !world.is_colliding(&probe, resolver)
    };
    if !endpoint_is_clear(from) || !endpoint_is_clear(to) {
        return None;
    }

    if world.path_is_clear(from, to, width, Some(net), layer, class, resolver)
        && edge_stays_on_board(from, to, outline)
    {
        return Some(vec![from, to]);
    }

    // Fast path: only consider obstacles near the direct corridor,
    // widening the corridor in stages (see `CORRIDOR_ESCALATION_FACTORS`)
    // before ever falling back to the complete, unfiltered item set.
    let dbg = std::env::var("ALLADIN_DEBUG_TIMING").is_ok();
    for factor in std::iter::once(CORRIDOR_MARGIN_FACTOR).chain(CORRIDOR_ESCALATION_FACTORS) {
        if dbg {
            eprintln!("[timing] stage factor={factor}: querying region...");
        }
        let region = corridor_region(from, to, factor);
        let t_region = std::time::Instant::now();
        let nearby = world.query_region(region);
        if dbg {
            eprintln!("[timing] stage factor={factor}: {} nearby item(s) in {:?}", nearby.len(), t_region.elapsed());
        }
        if let Some(path) = try_route(
            world, nearby.into_iter(), from, to, width, net, layer, class, resolver, outline,
            Some(region),
        ) {
            return Some(path);
        }
    }

    // Slow, always-correct fallback: every escalation stage above
    // missed something (e.g. the only valid detour swings wider than
    // even the widest `CORRIDOR_ESCALATION_FACTORS` step allows for) --
    // retry with every item in the world, exactly like `find_path_astar`
    // always did before the spatial pre-filter was added. Still bounded
    // to the board's own outline where one is available: a candidate
    // point outside every outline polygon can never survive
    // `edge_stays_on_board` anyway (see `candidate_points`'s doc comment
    // on `corridor` for why this specifically matters for a real zone's
    // vertex count), so this loses no correctness, only the points that
    // were already guaranteed to be useless. No outline supplied at all
    // -- `&[]`, meaning "not checked" -- means no such bound exists
    // either, so this last resort is genuinely unfiltered then, exactly
    // as it always was.
    let board_bounds = outline_bounds(outline);
    if let Some(path) = try_route(
        world, world.iter(), from, to, width, net, layer, class, resolver, outline, board_bounds,
    ) {
        return Some(path);
    }

    grid_fallback(world, from, to, width, net, layer, class, resolver, outline, board_bounds)
}

/// Opt-in, structurally different last resort: see `crate::grid_astar`'s
/// module doc comment and the development log's "Teil 28" entry. Every
/// stage above -- including [`find_path_astar`]'s own final, unfiltered
/// pass -- is a continuous visibility-graph search whose cost scales
/// with how many candidate points get sampled off nearby obstacles'
/// boundaries, exactly the mechanism [`FailureReason::SearchTooComplex`]
/// reports on. A discretized grid search has no candidate-point concept
/// to explode at all (see [`crate::grid_obstacle::GridObstacleMap`]), so
/// it's offered here as one more attempt before ever reporting failure
/// -- deliberately gated behind an explicit `ALLADIN_GRID_FALLBACK=1`
/// opt-in while it's still a benchmarked prototype (see
/// `crates/alladin-cli/examples/route_darkroom_panel_4x5.rs`), rather
/// than silently changing default behaviour for the already-passing
/// visibility-graph test suite.
#[allow(clippy::too_many_arguments)]
fn grid_fallback(
    world: &Node,
    from: Point,
    to: Point,
    width: Unit,
    net: NetId,
    layer: LayerId,
    class: NetClass,
    resolver: &dyn RuleResolver,
    outline: &[Polygon],
    board_bounds: Option<Aabb>,
) -> Option<Vec<Point>> {
    if std::env::var("ALLADIN_GRID_FALLBACK").is_err() {
        return None;
    }
    let region = board_bounds
        .unwrap_or_else(|| corridor_region(from, to, *CORRIDOR_ESCALATION_FACTORS.last().unwrap()));
    let items = world.query_region(region);
    crate::grid_astar::find_path_grid(
        world, items.into_iter(), from, to, width, net, layer, class, resolver, outline, region,
        crate::grid_obstacle::DEFAULT_GRID_STEP,
    )
}

/// Explains *why* [`find_path_astar`] (or anything built on top of it,
/// like [`crate::route_single_net`]) just returned `None` for this
/// exact request -- see
/// [`crate::failure`]'s module doc comment for the full rationale and
/// what each [`FailureReason`] variant is meant to tell its caller.
///
/// Deliberately a *separate* call, not a richer return type on
/// `find_path_astar` itself: threading a reason through every
/// `Option`-returning function in this crate (and updating every
/// existing test that pattern-matches on `Option`) would be a large,
/// invasive change for a purely diagnostic feature that only ever
/// matters on the failure path -- already the rare, already-slow one.
/// Instead, callers that got `None` back and want to know why call this
/// *afterwards*; it cheaply re-derives the answer from the same
/// primitives `find_path_astar` itself uses (`on_board`,
/// `endpoint_is_clear`/[`Node::query_colliding`], and one final
/// [`candidate_points`] call over the same outline-bounded region
/// `find_path_astar`'s own final fallback stage already searched). This
/// costs one extra `O(n)`-ish pass over an already-failed, already
/// comparatively rare request -- never anything on the success path.
#[allow(clippy::too_many_arguments)]
pub fn diagnose_failure(
    world: &Node,
    from: Point,
    to: Point,
    width: Unit,
    net: NetId,
    layer: LayerId,
    class: NetClass,
    resolver: &dyn RuleResolver,
    outline: &[Polygon],
) -> FailureReason {
    let on_board = |p: Point| outline.is_empty() || alladin_geom::contains_point_evenodd(outline, p);
    if !on_board(from) {
        return FailureReason::EndpointOffBoard { endpoint: Endpoint::From, at: from };
    }
    if !on_board(to) {
        return FailureReason::EndpointOffBoard { endpoint: Endpoint::To, at: to };
    }

    let blockers_at = |p: Point| -> Vec<alladin_core::ItemId> {
        let probe = Item::Track { shape: Segment::new(p, p, width), net: Some(net), layer, class };
        world.query_colliding(&probe, resolver)
    };
    let from_blockers = blockers_at(from);
    if !from_blockers.is_empty() {
        return FailureReason::EndpointBlocked {
            endpoint: Endpoint::From,
            at: from,
            blocking_items: from_blockers,
        };
    }
    let to_blockers = blockers_at(to);
    if !to_blockers.is_empty() {
        return FailureReason::EndpointBlocked { endpoint: Endpoint::To, at: to, blocking_items: to_blockers };
    }

    // Neither endpoint is the problem by itself -- re-derive the same
    // outline-bounded region `find_path_astar`'s own final fallback
    // stage already searched, purely to report *why* it came back
    // empty. `board_bounds` mirrors that stage's own `board_bounds`
    // exactly (`None` only when no outline was supplied at all, in
    // which case there is no real bound to report -- approximate with
    // a generous corridor around `from`/`to` instead, just for this
    // report's `region_searched` field).
    let board_bounds = outline_bounds(outline);
    let nearby: Vec<&Item> = match board_bounds {
        Some(region) => world.query_region(region),
        None => world.iter().collect(),
    };
    let nearby_items = nearby.len();
    let (points, truncated) = candidate_points(
        nearby.into_iter(), from, to, width, net, layer, class, resolver, outline, board_bounds,
    );
    let region_searched = board_bounds.unwrap_or_else(|| corridor_region(from, to, CORRIDOR_MARGIN_FACTOR));

    if truncated || points.len() > MAX_FULL_FALLBACK_POINTS {
        FailureReason::SearchTooComplex {
            region_searched,
            candidate_points: points.len(),
            nearby_items,
        }
    } else {
        FailureReason::NoPathExists {
            region_searched,
            candidate_points: points.len(),
            nearby_items,
        }
    }
}

/// The union bounding box of every polygon in `outline`, or `None` if
/// `outline` is empty (`find_path_astar`'s "no outline supplied" case --
/// deliberately distinct from "there's an outline but it's tiny").
fn outline_bounds(outline: &[Polygon]) -> Option<Aabb> {
    outline
        .iter()
        .map(Aabb::from_polygon)
        .reduce(|a, b| Aabb {
            min: Point::new(a.min.x.min(b.min.x), a.min.y.min(b.min.y)),
            max: Point::new(a.max.x.max(b.max.x), a.max.y.max(b.max.y)),
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::{FixedClearance, JlcpcbClearance};
    use alladin_geom::{Circle, MM};

    fn path_length(path: &[Point]) -> f64 {
        path.windows(2).map(|w| w[0].distance(w[1])).sum()
    }

    #[test]
    fn direct_path_when_nothing_blocks() {
        let world = Node::new();
        let resolver = FixedClearance(127_000);
        let path = find_path_astar(
            &world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        ).unwrap();
        assert_eq!(path, vec![Point::new(0, 0), Point::new(5 * MM, 0)]);
    }

    #[test]
    fn routes_around_a_single_circular_obstacle() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(2_500_000, 0), 800_000)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });

        let path = find_path_astar(
            &world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        ).expect("A* must find a way around a single obstacle");

        assert_eq!(*path.first().unwrap(), Point::new(0, 0));
        assert_eq!(*path.last().unwrap(), Point::new(5 * MM, 0));
        for leg in path.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        }
    }

    #[test]
    fn routes_around_a_mounting_hole() {
        // Proves `candidate_points`'s new `Item::Hole` arm actually
        // yields a usable graph -- same shape as the existing
        // `Item::Pad{Circle}`/`Item::Via` boundary-candidate arm, just
        // exercised via a genuine `Item::Hole` this time.
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        world.add(Item::Hole { position: Point::new(2_500_000, 0), drill: 1_600_000 });

        let path = find_path_astar(
            &world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        ).expect("A* must find a way around a single mounting hole");

        assert_eq!(*path.first().unwrap(), Point::new(0, 0));
        assert_eq!(*path.last().unwrap(), Point::new(5 * MM, 0));
        for leg in path.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        }
    }

    #[test]
    fn a_filled_zone_obstacle_never_forces_a_detour() {
        // Was `routes_around_a_filled_zone_obstacle`, back when
        // `alladin_core::Node::item_collides` still treated a
        // different-net `Item::Zone` as a hard obstacle. It no longer
        // does (see that method's own doc comment: a zone fill is a
        // point-in-time pour snapshot, not a live routing obstacle --
        // treating it as one would make it impossible to ever route a
        // second net across a layer that already has so much as one
        // full-board plane on it). So `find_path_astar`'s own direct-line
        // fast path (a couple lines up in this same file) now correctly
        // fires straight through what used to be a detour-forcing zone,
        // exactly like it already would for a same-net one.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;
        let p = |x_mm: f64, y_mm: f64| Point::new((x_mm * MM as f64) as Unit, (y_mm * MM as f64) as Unit);

        // A 4x4mm ground pour dead-center on the direct line.
        world.add(Item::Zone {
            outline: Polygon::new(vec![p(3.0, -2.0), p(7.0, -2.0), p(7.0, 2.0), p(3.0, 2.0)]),
            layer: LayerId::FCu,
            net: Some(NetId(2)),
        });

        let from = p(0.0, 0.0);
        let to = p(10.0, 0.0);
        let path = find_path_astar(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("A* must find a path straight across a filled zone obstacle");

        assert_eq!(path, vec![from, to], "must go straight through, not detour around, a zone that doesn't block");
    }

    #[test]
    fn routes_around_a_rotated_rectangular_pad_using_its_true_corner_not_the_old_inscribed_circle() {
        // Regression test for the actual reported bug (see
        // the development log's "Echte Pad-Geometrie" slice): a 4mm x
        // 1mm pad, rotated 30 degrees, whose old "inscribed circle"
        // collision radius (`min(width, height) / 2` = 0.5mm) left a
        // gap that used to read as clear even though the pad's *true*,
        // rotated corner reaches well past it. A route through that gap
        // must now be rejected and A* must find a real detour around
        // the true shape -- not just `Node::is_colliding` catching it
        // after the fact once a candidate edge happens to be tried.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;
        let mm = |v: f64| (v * MM as f64) as Unit;
        let center = Point::new(mm(5.0), 0);

        let half_width = mm(2.0); // 4mm-wide pad
        let half_height = mm(0.5); // 1mm-tall pad -- old inscribed radius = 0.5mm
        let local = [
            Point::new(-half_width, -half_height),
            Point::new(half_width, -half_height),
            Point::new(half_width, half_height),
            Point::new(-half_width, half_height),
        ];
        let outline = Polygon::new(local.iter().map(|&p| p.rotated(30.0).add(center)).collect());

        world.add(Item::Pad {
            shape: PadShape::Polygon { outline, center },
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });

        // A horizontal line 0.75mm above the pad's center: farther than
        // the old 0.5mm inscribed-circle radius plus this route's own
        // effective clearance (checked below), but still crossing the
        // pad's true, rotated corner.
        let from = Point::new(0, mm(0.75));
        let to = Point::new(mm(10.0), mm(0.75));
        let width = 250_000;

        let path = find_path_astar(
            &world, from, to, width, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("A* must find a way around the pad's true, rotated shape");

        assert_eq!(*path.first().unwrap(), from);
        assert_eq!(*path.last().unwrap(), to);
        assert!(path.len() > 2, "must actually detour around the true corner, not go straight through it");
        for leg in path.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], width, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        }

        // Sanity check the test's own premise: the direct line really
        // would have been (wrongly) accepted as clear under the old
        // inscribed-circle model, proving this is a genuine regression
        // test and not vacuously true regardless of pad shape.
        let mut old_model_world = Node::new();
        old_model_world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(center, half_height)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });
        assert!(
            old_model_world.path_is_clear(from, to, width, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver),
            "sanity check: the old inscribed-circle model must have missed this collision"
        );
    }

    #[test]
    fn candidate_points_drops_zone_boundary_points_outside_the_supplied_corridor() {
        // Regression test for the real-board hang found routing a
        // 109-LED panel's DATA daisy chain: a real KiCad zone fill's
        // polygon can have tens of thousands of vertices (see
        // `candidate_points`'s doc comment on its `corridor` parameter
        // for the exact number that was found), which without this
        // filter turns into an unusable graph for *every* net near that
        // zone, not just ones that actually need its boundary nearby.
        let resolver = JlcpcbClearance;
        let from = Point::new(0, 0);
        let to = Point::new(MM, 0);

        // A huge zone spanning far beyond any plausible corridor around
        // `from`/`to` -- a small stand-in for a real board-covering
        // copper pour.
        let huge_zone = Item::Zone {
            outline: Polygon::new(vec![
                Point::new(-100 * MM, -100 * MM),
                Point::new(100 * MM, -100 * MM),
                Point::new(100 * MM, 100 * MM),
                Point::new(-100 * MM, 100 * MM),
            ]),
            layer: LayerId::FCu,
            net: Some(NetId(2)),
        };

        let tight_corridor = corridor_region(from, to, CORRIDOR_MARGIN_FACTOR);
        let (filtered, _) = candidate_points(
            std::iter::once(&huge_zone), from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C,
            &resolver, &[], Some(tight_corridor),
        );
        assert_eq!(
            filtered.len(), 2,
            "a corridor far smaller than the zone must drop every one of its boundary points, leaving only from/to"
        );

        let (unfiltered, _) = candidate_points(
            std::iter::once(&huge_zone), from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C,
            &resolver, &[], None,
        );
        assert!(
            unfiltered.len() > 2,
            "with no corridor filter (`None`), the zone's own boundary points must still be offered"
        );
    }

    #[test]
    fn stride_sample_keeps_first_and_last_and_never_exceeds_the_cap() {
        let points: Vec<Point> = (0..1000).map(|i| Point::new(i, i)).collect();
        let sampled = stride_sample(points.clone(), 100);
        assert!(sampled.len() <= 100, "must never exceed the requested cap, got {}", sampled.len());
        assert!(sampled.len() >= 10, "a 1000/100 stride should still keep a reasonable spread, got {}", sampled.len());
        assert_eq!(sampled.first(), points.first(), "must keep the start of the sequence");

        let under_cap = stride_sample(points.clone(), 10_000);
        assert_eq!(under_cap.len(), points.len(), "a cap larger than the input must keep every point");

        assert!(
            stride_sample(Vec::<Point>::new(), 100).is_empty(),
            "an empty input must stay empty regardless of the cap"
        );
    }

    #[test]
    fn a_zone_with_far_more_boundary_points_than_the_cap_still_contributes_a_spread_of_waypoints() {
        // Regression test for the *other* half of the real-board hang
        // (see `candidate_points_drops_zone_boundary_points_outside_the_supplied_corridor`'s
        // doc comment): even after the corridor filter, a real zone's
        // *in-corridor* boundary alone measured well over 100,000 points
        // on the 109-LED panel board -- this fixture reproduces that
        // shape (a zone with thousands of vertices, all inside a
        // generous corridor) and checks `candidate_points` caps what it
        // actually offers per zone rather than passing every one of
        // them through to the KNN/full-fallback graph builders.
        let resolver = JlcpcbClearance;
        let from = Point::new(0, 0);
        let to = Point::new(50 * MM, 0);

        // A many-vertex zone comfortably inside a wide corridor around
        // `from`/`to` -- a fine-grained rectangle boundary standing in
        // for a real zone's thermal-relief-riddled outline.
        let mut zone_points = Vec::new();
        let steps = 2000;
        for i in 0..steps {
            let t = i as f64 / steps as f64;
            zone_points.push(Point::new((t * 40.0 * MM as f64) as Unit, (5.0 * MM as f64) as Unit));
        }
        for i in 0..steps {
            let t = i as f64 / steps as f64;
            zone_points.push(Point::new(((1.0 - t) * 40.0 * MM as f64) as Unit, (-5.0 * MM as f64) as Unit));
        }
        let dense_zone = Item::Zone {
            outline: Polygon::new(zone_points),
            layer: LayerId::FCu,
            net: Some(NetId(2)),
        };

        let wide_corridor = corridor_region(from, to, 4.0);
        let (result, _) = candidate_points(
            std::iter::once(&dense_zone), from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C,
            &resolver, &[], Some(wide_corridor),
        );

        // 2 (from/to) plus at most `MAX_ZONE_CANDIDATE_POINTS_PER_ITEM`
        // from this one zone, however many thousands of its own
        // in-corridor vertices there actually were.
        assert!(
            result.len() <= 2 + MAX_ZONE_CANDIDATE_POINTS_PER_ITEM,
            "a single dense zone must never contribute more than the cap, got {} total points",
            result.len()
        );
        // Still a real, useful spread of waypoints, not just `from`/`to`
        // themselves -- the cap must thin, not effectively delete, this
        // zone's contribution.
        assert!(result.len() > 100, "the cap should still leave a meaningful spread of waypoints, got {}", result.len());
    }

    #[test]
    fn many_small_obstacles_are_globally_capped_even_with_no_zone_involved() {
        // Regression test for the real-board hang's *other* other half:
        // the 109-LED panel's longest DATA net had no oversized zone in
        // its way at all, just 944 legitimately nearby real pads (24
        // points each, already within `MAX_ZONE_CANDIDATE_POINTS_PER_ITEM`
        // individually) -- proving the per-zone cap alone can't catch
        // this, only `MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE` can.
        let resolver = JlcpcbClearance;
        let from = Point::new(0, 0);
        let to = Point::new(50 * MM, 0);

        let pads: Vec<Item> = (0..500)
            .map(|i| Item::Pad {
                shape: PadShape::Circle(Circle::new(Point::new((i as Unit) * 100_000, 2 * MM), 200_000)),
                net: Some(NetId(100 + i as u32)),
                layer: LayerId::FCu,
            })
            .collect();

        let (result, truncated) = candidate_points(
            pads.iter(), from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C,
            &resolver, &[], None,
        );
        assert!(
            result.len() <= 2 + MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE,
            "500 small pads (24 points each = 12,000) must still be capped globally, got {} points",
            result.len()
        );
        assert!(truncated, "capping this many legitimately-small obstacles must still report truncation");
    }

    /// Not a correctness test (`#[ignore]`d, like this module's other
    /// `#[ignore]`-worthy real-board-scale fixtures): a manual benchmark
    /// for the `rayon`-parallel [`build_adjacency_knn`] rewrite, run via
    /// `cargo test --release -- --ignored knn_build_scales_across_cpu_cores --nocapture`.
    /// Reproduces the real "944 nearby pads, 24 points each, ~23,000
    /// points total" case documented on
    /// [`MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE`]'s doc comment directly
    /// (bypassing `candidate_points`'s own cap, which today would
    /// thin this down before it ever reached `build_adjacency_knn` --
    /// the point here is measuring this function's own scaling, not
    /// today's cap policy), with real `Node::path_is_clear` checks
    /// against hundreds of real pad obstacles, not an `always_valid`
    /// stand-in. Prints wall-clock time so it can be compared across
    /// `RAYON_NUM_THREADS` settings (e.g. `RAYON_NUM_THREADS=1` for a
    /// single-core baseline vs. unset for "use every core") on this
    /// machine.
    #[test]
    #[ignore]
    fn knn_build_scales_across_cpu_cores() {
        let resolver = JlcpcbClearance;

        let mut world = Node::new();
        let pads: Vec<Item> = (0..900)
            .map(|i| {
                let x = (i % 30) as Unit * 2_800_000;
                let y = (i / 30) as Unit * 2_800_000;
                Item::Pad {
                    shape: PadShape::Circle(Circle::new(Point::new(x, y), 200_000)),
                    net: Some(NetId(100 + i as u32)),
                    layer: LayerId::FCu,
                }
            })
            .collect();
        for pad in &pads {
            world.add(pad.clone());
        }

        // Every pad's own real clearance-inflated boundary (exactly what
        // `candidate_points` would generate for an `Item::Pad`), just
        // without the global truncation -- ~900 * 24 =~ 21,600 points,
        // matching the real board's measured order of magnitude.
        let mut points: Vec<(Point, usize)> = vec![(Point::new(-5 * MM, -5 * MM), 0), (Point::new(90 * MM, 90 * MM), 1)];
        let mut next_group = 2usize;
        for pad in &pads {
            let Item::Pad { shape: PadShape::Circle(c), .. } = pad else { unreachable!() };
            let boundary = circle_boundary(c.center, c.radius + 127_000 + 125_000);
            let group = next_group;
            next_group += 1;
            points.extend(boundary.into_iter().map(|p| (p, group)));
        }
        eprintln!("[bench] candidate points: {}", points.len());

        let is_valid_edge = |a: Point, b: Point| {
            world.path_is_clear(a, b, 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver)
        };

        let t0 = std::time::Instant::now();
        let adjacency = build_adjacency_knn(&points, &is_valid_edge);
        eprintln!(
            "[bench] build_adjacency_knn: {:?} (RAYON_NUM_THREADS={:?}, available_parallelism={:?})",
            t0.elapsed(),
            std::env::var("RAYON_NUM_THREADS"),
            std::thread::available_parallelism(),
        );
        assert!(!adjacency.is_empty());
    }

    /// Head-to-head benchmark, old obstacle-boundary sampling
    /// (`candidate_points`) vs. new adaptive quadtree sampling
    /// (`quadtree_candidates`), on the *same* synthetic dense scene as
    /// [`knn_build_scales_across_cpu_cores`] (900 pads on a 30x30 grid,
    /// ~21,600 old-style candidate points) -- see the "Quadtree-basierte
    /// Kandidatenpunkte" plan's benchmark step. `#[ignore]`d like every
    /// other real-board-scale fixture in this module; run via
    /// `cargo test --release -p alladin-router -- --ignored candidate_point_generation_old_vs_quadtree --nocapture`.
    #[test]
    #[ignore]
    fn candidate_point_generation_old_vs_quadtree() {
        let resolver = JlcpcbClearance;
        let mut world = Node::new();
        let pads: Vec<Item> = (0..900)
            .map(|i| {
                let x = (i % 30) as Unit * 2_800_000;
                let y = (i / 30) as Unit * 2_800_000;
                Item::Pad {
                    shape: PadShape::Circle(Circle::new(Point::new(x, y), 200_000)),
                    net: Some(NetId(100 + i as u32)),
                    layer: LayerId::FCu,
                }
            })
            .collect();
        for pad in &pads {
            world.add(pad.clone());
        }

        let from = Point::new(-5 * MM, -5 * MM);
        let to = Point::new(90 * MM, 90 * MM);
        let width = 250_000;
        let net = NetId(1);
        let layer = LayerId::FCu;
        let class = NetClass::C;
        let region = corridor_region(from, to, CORRIDOR_ESCALATION_FACTORS[0]);

        // Old path: obstacle-boundary sampling, exactly what `try_route`
        // does today.
        let t0 = std::time::Instant::now();
        let (old_points, old_truncated) = candidate_points(
            pads.iter(), from, to, width, net, layer, class, &resolver, &[], Some(region),
        );
        let old_gen_time = t0.elapsed();
        let is_valid_edge = |a: Point, b: Point| world.path_is_clear(a, b, width, Some(net), layer, class, &resolver);
        let t1 = std::time::Instant::now();
        let old_adjacency = build_adjacency_knn(&old_points, &is_valid_edge);
        let old_knn_time = t1.elapsed();
        let t2 = std::time::Instant::now();
        let old_points_flat: Vec<Point> = old_points.iter().map(|&(p, _)| p).collect();
        let old_path = astar_search(&old_points_flat, &old_adjacency, 0, 1);
        let old_search_time = t2.elapsed();

        // New path: adaptive quadtree sampling.
        let t3 = std::time::Instant::now();
        let obstacles =
            crate::quadtree_candidates::build_obstacles(pads.iter(), from, to, width, net, layer, class, &resolver);
        let tree = crate::quadtree_candidates::build_quadtree(
            region, crate::quadtree_candidates::DEFAULT_QUADTREE_MIN_LEAF, &obstacles,
        );
        let (new_points, new_truncated) =
            crate::quadtree_candidates::quadtree_candidate_points(&tree, from, to, &[]);
        let new_gen_time = t3.elapsed();
        let t4 = std::time::Instant::now();
        let new_adjacency = build_adjacency_knn(&new_points, &is_valid_edge);
        let new_knn_time = t4.elapsed();
        let t5 = std::time::Instant::now();
        let new_points_flat: Vec<Point> = new_points.iter().map(|&(p, _)| p).collect();
        let new_path = astar_search(&new_points_flat, &new_adjacency, 0, 1);
        let new_search_time = t5.elapsed();

        eprintln!(
            "[bench] OLD (boundary sampling): {} points (truncated={old_truncated}), \
             gen={old_gen_time:?} knn={old_knn_time:?} search={old_search_time:?} found={}",
            old_points.len(),
            old_path.is_some(),
        );
        eprintln!(
            "[bench] NEW (quadtree):          {} points (truncated={new_truncated}), \
             gen={new_gen_time:?} knn={new_knn_time:?} search={new_search_time:?} found={}",
            new_points.len(),
            new_path.is_some(),
        );

        assert!(old_path.is_some(), "old path must find a route across the dense grid (sanity check)");
        assert!(new_path.is_some(), "new quadtree path must find a route across the dense grid too");
    }

    /// The scenario [`candidate_point_generation_old_vs_quadtree`]'s
    /// uniformly-packed 30x30 pad grid deliberately does *not*
    /// represent: a long route across a mostly-*open* board with only a
    /// handful of obstacles scattered along the way (e.g. a long
    /// point-to-point net crossing empty board area between a few
    /// components) -- exactly the "free space is a sea, obstacles are
    /// occasional islands" case the quadtree's own adaptive resolution
    /// (coarse everywhere, fine only near a boundary) is meant to help
    /// most, unlike the uniformly-dense grid above where nearly
    /// everywhere needs fine resolution anyway. Same benchmark
    /// structure and invocation pattern as that test.
    #[test]
    #[ignore]
    fn candidate_point_generation_old_vs_quadtree_sparse_long_route() {
        let resolver = JlcpcbClearance;
        let mut world = Node::new();
        // 25 pads, deterministically scattered (xorshift32, no `rand`
        // dependency needed) across a 200mm x 60mm span -- sparse
        // relative to that area, unlike the dense grid above.
        struct XorShift32(u32);
        impl XorShift32 {
            fn next_unit_in(&mut self, lo: Unit, hi: Unit) -> Unit {
                self.0 ^= self.0 << 13;
                self.0 ^= self.0 >> 17;
                self.0 ^= self.0 << 5;
                lo + ((self.0 as f64 / u32::MAX as f64) * (hi - lo) as f64) as Unit
            }
        }
        let mut rng = XorShift32(0xC0FFEE);
        let pads: Vec<Item> = (0..25)
            .map(|i| {
                let x = rng.next_unit_in(10 * MM, 190 * MM);
                let y = rng.next_unit_in(5 * MM, 55 * MM);
                Item::Pad {
                    shape: PadShape::Circle(Circle::new(Point::new(x, y), 400_000)),
                    net: Some(NetId(100 + i as u32)),
                    layer: LayerId::FCu,
                }
            })
            .collect();
        for pad in &pads {
            world.add(pad.clone());
        }

        let from = Point::new(0, 30 * MM);
        let to = Point::new(200 * MM, 30 * MM);
        let width = 250_000;
        let net = NetId(1);
        let layer = LayerId::FCu;
        let class = NetClass::C;
        // The margin a real first attempt actually uses (`find_path_astar`'s
        // own first, non-escalated stage) -- more representative of the
        // common case than an already-escalated, wider corridor.
        let region = corridor_region(from, to, CORRIDOR_MARGIN_FACTOR);
        eprintln!(
            "[bench-sparse] region: {:.1}mm x {:.1}mm",
            (region.max.x - region.min.x) as f64 / MM as f64,
            (region.max.y - region.min.y) as f64 / MM as f64,
        );

        let t0 = std::time::Instant::now();
        let (old_points, old_truncated) = candidate_points(
            pads.iter(), from, to, width, net, layer, class, &resolver, &[], Some(region),
        );
        let old_gen_time = t0.elapsed();
        let is_valid_edge = |a: Point, b: Point| world.path_is_clear(a, b, width, Some(net), layer, class, &resolver);
        let t1 = std::time::Instant::now();
        let old_adjacency = build_adjacency_knn(&old_points, &is_valid_edge);
        let old_knn_time = t1.elapsed();
        let t2 = std::time::Instant::now();
        let old_points_flat: Vec<Point> = old_points.iter().map(|&(p, _)| p).collect();
        let old_path = astar_search(&old_points_flat, &old_adjacency, 0, 1);
        let old_search_time = t2.elapsed();

        let t3 = std::time::Instant::now();
        let obstacles =
            crate::quadtree_candidates::build_obstacles(pads.iter(), from, to, width, net, layer, class, &resolver);
        let tree = crate::quadtree_candidates::build_quadtree(
            region, crate::quadtree_candidates::DEFAULT_QUADTREE_MIN_LEAF, &obstacles,
        );
        let (new_points, new_truncated) =
            crate::quadtree_candidates::quadtree_candidate_points(&tree, from, to, &[]);
        let new_gen_time = t3.elapsed();
        let t4 = std::time::Instant::now();
        let new_adjacency = build_adjacency_knn(&new_points, &is_valid_edge);
        let new_knn_time = t4.elapsed();
        let t5 = std::time::Instant::now();
        let new_points_flat: Vec<Point> = new_points.iter().map(|&(p, _)| p).collect();
        let new_path = astar_search(&new_points_flat, &new_adjacency, 0, 1);
        let new_search_time = t5.elapsed();

        eprintln!(
            "[bench-sparse] OLD (boundary sampling): {} points (truncated={old_truncated}), \
             gen={old_gen_time:?} knn={old_knn_time:?} search={old_search_time:?} found={}",
            old_points.len(),
            old_path.is_some(),
        );
        eprintln!(
            "[bench-sparse] NEW (quadtree):          {} points (truncated={new_truncated}), \
             gen={new_gen_time:?} knn={new_knn_time:?} search={new_search_time:?} found={}",
            new_points.len(),
            new_path.is_some(),
        );

        assert!(old_path.is_some());
        assert!(new_path.is_some());
    }

    /// Head-to-head benchmark for the grid pre-filter
    /// (`GridObstacleMap::segment_definitely_blocked`, on by default in
    /// `try_route` since 2026-08-01):
    /// plain [`build_adjacency_knn`] vs. the same candidate points and
    /// the same real `Node::path_is_clear` checks, but with the grid
    /// pre-filter wrapped in front -- on the *same* dense 900-pad scene
    /// as [`candidate_point_generation_old_vs_quadtree`]. Candidate
    /// point count is identical in both runs (the pre-filter only
    /// changes how many of `build_adjacency_knn`'s own
    /// `is_valid_edge` calls reach the real, expensive check), so only
    /// `knn` time is meaningful to compare here. `#[ignore]`d like every
    /// other real-board-scale fixture in this module; run via `cargo
    /// test --release -p alladin-router -- --ignored
    /// knn_grid_prefilter_old_vs_new --nocapture`.
    #[test]
    #[ignore]
    fn knn_grid_prefilter_old_vs_new_dense_grid() {
        let resolver = JlcpcbClearance;
        let mut world = Node::new();
        let pads: Vec<Item> = (0..900)
            .map(|i| {
                let x = (i % 30) as Unit * 2_800_000;
                let y = (i / 30) as Unit * 2_800_000;
                Item::Pad {
                    shape: PadShape::Circle(Circle::new(Point::new(x, y), 200_000)),
                    net: Some(NetId(100 + i as u32)),
                    layer: LayerId::FCu,
                }
            })
            .collect();
        for pad in &pads {
            world.add(pad.clone());
        }

        let from = Point::new(-5 * MM, -5 * MM);
        let to = Point::new(90 * MM, 90 * MM);
        let width = 250_000;
        let net = NetId(1);
        let layer = LayerId::FCu;
        let class = NetClass::C;
        let region = corridor_region(from, to, CORRIDOR_ESCALATION_FACTORS[0]);

        let (points, _truncated) =
            candidate_points(pads.iter(), from, to, width, net, layer, class, &resolver, &[], Some(region));
        eprintln!("[bench-grid-prefilter] dense grid: {} points", points.len());

        let is_valid_edge = |a: Point, b: Point| world.path_is_clear(a, b, width, Some(net), layer, class, &resolver);
        let t0 = std::time::Instant::now();
        let old_adjacency = build_adjacency_knn(&points, &is_valid_edge);
        let old_knn_time = t0.elapsed();

        let points_flat: Vec<Point> = points.iter().map(|&(p, _)| p).collect();
        let grid_region = points_bbox(&points_flat).inflate(crate::grid_obstacle::DEFAULT_GRID_STEP * 4);
        let mut grid =
            crate::grid_obstacle::GridObstacleMap::new(grid_region, crate::grid_obstacle::DEFAULT_GRID_STEP);
        grid.rasterize_items(pads.iter(), from, to, width, net, layer, class, &resolver);
        eprintln!(
            "[bench-grid-prefilter] dense grid: grid step={} width={} height={}",
            grid.step(),
            grid.width(),
            grid.height()
        );
        let is_valid_edge_prefiltered = |a: Point, b: Point| {
            if grid.segment_definitely_blocked(a, b, layer) {
                return false;
            }
            is_valid_edge(a, b)
        };
        let t1 = std::time::Instant::now();
        let new_adjacency = build_adjacency_knn(&points, &is_valid_edge_prefiltered);
        let new_knn_time = t1.elapsed();

        eprintln!("[bench-grid-prefilter] dense grid: OLD knn={old_knn_time:?}, NEW (grid-prefiltered) knn={new_knn_time:?}");

        let same_edge_count: usize = old_adjacency.iter().map(|row| row.len()).sum();
        let new_edge_count: usize = new_adjacency.iter().map(|row| row.len()).sum();
        eprintln!("[bench-grid-prefilter] dense grid: OLD edges={same_edge_count}, NEW edges={new_edge_count}");

        let old_found = astar_search(&points_flat, &old_adjacency, 0, 1).is_some();
        let new_found = astar_search(&points_flat, &new_adjacency, 0, 1).is_some();
        eprintln!("[bench-grid-prefilter] dense grid: OLD found={old_found}, NEW found={new_found}");
    }

    /// Same comparison as [`knn_grid_prefilter_old_vs_new_dense_grid`],
    /// but on the sparse, mostly-open scene from
    /// [`candidate_point_generation_old_vs_quadtree_sparse_long_route`] --
    /// the case where most candidate points are far apart and the
    /// pre-filter should have little to reject.
    #[test]
    #[ignore]
    fn knn_grid_prefilter_old_vs_new_sparse_long_route() {
        let resolver = JlcpcbClearance;
        let mut world = Node::new();
        struct XorShift32(u32);
        impl XorShift32 {
            fn next_unit_in(&mut self, lo: Unit, hi: Unit) -> Unit {
                self.0 ^= self.0 << 13;
                self.0 ^= self.0 >> 17;
                self.0 ^= self.0 << 5;
                lo + ((self.0 as f64 / u32::MAX as f64) * (hi - lo) as f64) as Unit
            }
        }
        let mut rng = XorShift32(0xC0FFEE);
        let pads: Vec<Item> = (0..25)
            .map(|i| {
                let x = rng.next_unit_in(10 * MM, 190 * MM);
                let y = rng.next_unit_in(5 * MM, 55 * MM);
                Item::Pad {
                    shape: PadShape::Circle(Circle::new(Point::new(x, y), 400_000)),
                    net: Some(NetId(100 + i as u32)),
                    layer: LayerId::FCu,
                }
            })
            .collect();
        for pad in &pads {
            world.add(pad.clone());
        }

        let from = Point::new(0, 30 * MM);
        let to = Point::new(200 * MM, 30 * MM);
        let width = 250_000;
        let net = NetId(1);
        let layer = LayerId::FCu;
        let class = NetClass::C;
        let region = corridor_region(from, to, CORRIDOR_MARGIN_FACTOR);

        let (points, _truncated) =
            candidate_points(pads.iter(), from, to, width, net, layer, class, &resolver, &[], Some(region));
        eprintln!("[bench-grid-prefilter] sparse: {} points", points.len());

        let is_valid_edge = |a: Point, b: Point| world.path_is_clear(a, b, width, Some(net), layer, class, &resolver);
        let t0 = std::time::Instant::now();
        let old_adjacency = build_adjacency_knn(&points, &is_valid_edge);
        let old_knn_time = t0.elapsed();

        let points_flat: Vec<Point> = points.iter().map(|&(p, _)| p).collect();
        let grid_region = points_bbox(&points_flat).inflate(crate::grid_obstacle::DEFAULT_GRID_STEP * 4);
        let mut grid =
            crate::grid_obstacle::GridObstacleMap::new(grid_region, crate::grid_obstacle::DEFAULT_GRID_STEP);
        grid.rasterize_items(pads.iter(), from, to, width, net, layer, class, &resolver);
        let is_valid_edge_prefiltered = |a: Point, b: Point| {
            if grid.segment_definitely_blocked(a, b, layer) {
                return false;
            }
            is_valid_edge(a, b)
        };
        let t1 = std::time::Instant::now();
        let new_adjacency = build_adjacency_knn(&points, &is_valid_edge_prefiltered);
        let new_knn_time = t1.elapsed();

        eprintln!("[bench-grid-prefilter] sparse: OLD knn={old_knn_time:?}, NEW (grid-prefiltered) knn={new_knn_time:?}");

        let old_edge_count: usize = old_adjacency.iter().map(|row| row.len()).sum();
        let new_edge_count: usize = new_adjacency.iter().map(|row| row.len()).sum();
        eprintln!("[bench-grid-prefilter] sparse: OLD edges={old_edge_count}, NEW edges={new_edge_count}");

        let old_found = astar_search(&points_flat, &old_adjacency, 0, 1).is_some();
        let new_found = astar_search(&points_flat, &new_adjacency, 0, 1).is_some();
        eprintln!("[bench-grid-prefilter] sparse: OLD found={old_found}, NEW found={new_found}");
    }

    /// The pathological case [`KNN_MAX_CONSIDERED`]'s own doc comment
    /// names explicitly: a genuinely dense cluster (a fine-pitch
    /// connector/QFN-style footprint) where most candidate-to-candidate
    /// lines cut through a *third* nearby pin's clearance and get
    /// rejected -- exactly the scenario where wasted `path_is_clear`
    /// calls should be most numerous, so the pre-filter's benefit (if
    /// any) should show up most clearly here.
    #[test]
    #[ignore]
    fn knn_grid_prefilter_old_vs_new_fine_pitch_cluster() {
        let resolver = JlcpcbClearance;
        let mut world = Node::new();
        // 20x20 pads at 0.5mm pitch (a common fine-pitch/QFN spacing),
        // 0.2mm pads -- a 10mm x 10mm footprint the route must cross.
        let pitch: Unit = 500_000;
        let pad_radius: Unit = 100_000;
        let pads: Vec<Item> = (0..400)
            .map(|i| {
                let x = (i % 20) as Unit * pitch;
                let y = (i / 20) as Unit * pitch;
                Item::Pad {
                    shape: PadShape::Circle(Circle::new(Point::new(x, y), pad_radius)),
                    net: Some(NetId(100 + i as u32)),
                    layer: LayerId::FCu,
                }
            })
            .collect();
        for pad in &pads {
            world.add(pad.clone());
        }

        let from = Point::new(-5 * MM, 5 * MM);
        let to = Point::new(15 * MM, 5 * MM);
        let width = 150_000;
        let net = NetId(1);
        let layer = LayerId::FCu;
        let class = NetClass::C;
        let region = corridor_region(from, to, CORRIDOR_ESCALATION_FACTORS[0]);

        let (points, _truncated) =
            candidate_points(pads.iter(), from, to, width, net, layer, class, &resolver, &[], Some(region));
        eprintln!("[bench-grid-prefilter] fine-pitch cluster: {} points", points.len());

        let is_valid_edge = |a: Point, b: Point| world.path_is_clear(a, b, width, Some(net), layer, class, &resolver);
        let t0 = std::time::Instant::now();
        let old_adjacency = build_adjacency_knn(&points, &is_valid_edge);
        let old_knn_time = t0.elapsed();

        let points_flat: Vec<Point> = points.iter().map(|&(p, _)| p).collect();
        let grid_region = points_bbox(&points_flat).inflate(crate::grid_obstacle::DEFAULT_GRID_STEP * 4);
        let mut grid =
            crate::grid_obstacle::GridObstacleMap::new(grid_region, crate::grid_obstacle::DEFAULT_GRID_STEP);
        grid.rasterize_items(pads.iter(), from, to, width, net, layer, class, &resolver);
        let is_valid_edge_prefiltered = |a: Point, b: Point| {
            if grid.segment_definitely_blocked(a, b, layer) {
                return false;
            }
            is_valid_edge(a, b)
        };
        let t1 = std::time::Instant::now();
        let new_adjacency = build_adjacency_knn(&points, &is_valid_edge_prefiltered);
        let new_knn_time = t1.elapsed();

        eprintln!(
            "[bench-grid-prefilter] fine-pitch cluster: OLD knn={old_knn_time:?}, NEW (grid-prefiltered) knn={new_knn_time:?}"
        );

        let old_edge_count: usize = old_adjacency.iter().map(|row| row.len()).sum();
        let new_edge_count: usize = new_adjacency.iter().map(|row| row.len()).sum();
        eprintln!("[bench-grid-prefilter] fine-pitch cluster: OLD edges={old_edge_count}, NEW edges={new_edge_count}");

        let old_found = astar_search(&points_flat, &old_adjacency, 0, 1).is_some();
        let new_found = astar_search(&points_flat, &new_adjacency, 0, 1).is_some();
        eprintln!("[bench-grid-prefilter] fine-pitch cluster: OLD found={old_found}, NEW found={new_found}");
    }

    #[test]
    fn navigates_a_staggered_two_obstacle_chicane() {
        // The scenario that motivates this whole module: two "wall"
        // track obstacles whose accessible gaps are on *opposite* sides
        // (A leaves a gap above, B leaves a gap below) -- forcing a real
        // S-curve. A myopic per-leg walkaround would resolve A's
        // collision first without ever knowing B exists yet; A* instead
        // sees both obstacles' full boundaries in one graph from the
        // start.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(Item::Track {
            shape: Segment::new(Point::new(2 * MM, -3 * MM), Point::new(2 * MM, 500_000), 300_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        world.add(Item::Track {
            shape: Segment::new(Point::new(4 * MM, -500_000), Point::new(4 * MM, 3 * MM), 300_000),
            net: Some(NetId(3)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let from = Point::new(0, 0);
        let to = Point::new(6 * MM, 0);
        let path = find_path_astar(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        ).expect("A* must find a valid chicane path around both walls");

        assert_eq!(*path.first().unwrap(), from);
        assert_eq!(*path.last().unwrap(), to);
        for leg in path.windows(2) {
            assert!(
                world.path_is_clear(leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver),
                "leg {leg:?} must clear both walls"
            );
        }

        // Sanity bound: a correct chicane solution shouldn't need to
        // wander wildly far from the straight line to thread this gap.
        assert!(path_length(&path) < 3.0 * from.distance(to));
    }

    /// Runs the exact same "build candidate points -> KNN -> A*" pipeline
    /// `try_route` uses when `ALLADIN_QUADTREE_CANDIDATES=1` is set, but
    /// calling `crate::quadtree_candidates` directly instead of toggling
    /// the env var -- this crate's established pattern for testing an
    /// env-var-gated code path (see `grid_astar`/`grid_obstacle`'s own
    /// tests, none of which set `ALLADIN_GRID_FALLBACK` either): process-
    /// wide env vars are shared, mutable global state, and Rust tests run
    /// concurrently by default, so toggling one from an individual test
    /// would risk racing every other test that (transitively, via
    /// `find_path_astar`) reads it.
    #[allow(clippy::too_many_arguments)]
    fn route_via_quadtree_candidates(
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
        let region = outline_bounds(outline)
            .unwrap_or_else(|| corridor_region(from, to, *CORRIDOR_ESCALATION_FACTORS.last().unwrap()));
        let items: Vec<Item> = world.iter().cloned().collect();
        let obstacles = crate::quadtree_candidates::build_obstacles(
            items.iter(), from, to, width, net, layer, class, resolver,
        );
        let tree = crate::quadtree_candidates::build_quadtree(
            region, crate::quadtree_candidates::DEFAULT_QUADTREE_MIN_LEAF, &obstacles,
        );
        let (grouped_points, _truncated) =
            crate::quadtree_candidates::quadtree_candidate_points(&tree, from, to, outline);
        let points: Vec<Point> = grouped_points.iter().map(|&(p, _group)| p).collect();
        let is_valid_edge = |a: Point, b: Point| {
            world.path_is_clear(a, b, width, Some(net), layer, class, resolver)
                && edge_stays_on_board(a, b, outline)
        };
        let adjacency = build_adjacency_knn(&grouped_points, &is_valid_edge);
        astar_search(&points, &adjacency, 0, 1).or_else(|| {
            let full = build_adjacency_full(&points, &is_valid_edge);
            astar_search(&points, &full, 0, 1)
        })
    }

    #[test]
    fn quadtree_candidates_can_route_around_a_single_circular_obstacle() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(2_500_000, 0), 800_000)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });
        let (from, to) = (Point::new(0, 0), Point::new(5 * MM, 0));

        let path = route_via_quadtree_candidates(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("quadtree-derived candidates must still find a way around a single obstacle");

        assert_eq!(*path.first().unwrap(), from);
        assert_eq!(*path.last().unwrap(), to);
        for leg in path.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        }
    }

    #[test]
    fn quadtree_candidates_can_route_around_a_filled_zone_obstacle() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;
        let p = |x_mm: f64, y_mm: f64| Point::new((x_mm * MM as f64) as Unit, (y_mm * MM as f64) as Unit);

        world.add(Item::Zone {
            outline: Polygon::new(vec![p(3.0, -2.0), p(7.0, -2.0), p(7.0, 2.0), p(3.0, 2.0)]),
            layer: LayerId::FCu,
            net: Some(NetId(2)),
        });
        let (from, to) = (p(0.0, 0.0), p(10.0, 0.0));

        let path = route_via_quadtree_candidates(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("quadtree-derived candidates must still find a way around a filled zone");

        assert_eq!(*path.first().unwrap(), from);
        assert_eq!(*path.last().unwrap(), to);
        assert!(path.len() > 2, "must actually detour, not go straight through the zone");
        for leg in path.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        }
    }

    #[test]
    fn quadtree_candidates_can_route_around_a_rotated_rectangular_pad() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;
        let mm = |v: f64| (v * MM as f64) as Unit;
        let center = Point::new(mm(5.0), 0);
        let half_width = mm(2.0);
        let half_height = mm(0.5);
        let local = [
            Point::new(-half_width, -half_height),
            Point::new(half_width, -half_height),
            Point::new(half_width, half_height),
            Point::new(-half_width, half_height),
        ];
        let outline = Polygon::new(local.iter().map(|&p| p.rotated(30.0).add(center)).collect());
        world.add(Item::Pad { shape: PadShape::Polygon { outline, center }, net: Some(NetId(2)), layer: LayerId::FCu });

        let (from, to) = (Point::new(0, mm(0.75)), Point::new(mm(10.0), mm(0.75)));
        let width = 250_000;

        let path = route_via_quadtree_candidates(
            &world, from, to, width, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("quadtree-derived candidates must still find a way around the pad's true, rotated shape");

        assert_eq!(*path.first().unwrap(), from);
        assert_eq!(*path.last().unwrap(), to);
        assert!(path.len() > 2, "must actually detour around the true corner, not go straight through it");
        for leg in path.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], width, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        }
    }

    #[test]
    fn quadtree_candidates_can_navigate_a_staggered_two_obstacle_chicane() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(Item::Track {
            shape: Segment::new(Point::new(2 * MM, -3 * MM), Point::new(2 * MM, 500_000), 300_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        world.add(Item::Track {
            shape: Segment::new(Point::new(4 * MM, -500_000), Point::new(4 * MM, 3 * MM), 300_000),
            net: Some(NetId(3)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let (from, to) = (Point::new(0, 0), Point::new(6 * MM, 0));
        let path = route_via_quadtree_candidates(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("quadtree-derived candidates must still find a valid chicane path around both walls");

        assert_eq!(*path.first().unwrap(), from);
        assert_eq!(*path.last().unwrap(), to);
        for leg in path.windows(2) {
            assert!(
                world.path_is_clear(leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver),
                "leg {leg:?} must clear both walls"
            );
        }
    }

    /// Exactly the code path `try_route` now always uses by default
    /// (same items, same `points_bbox`-sized grid, same
    /// `segment_definitely_blocked` wrapping around `is_valid_edge`),
    /// but built standalone here so these regression tests keep
    /// exercising the exact mechanics in isolation, independent of
    /// whatever candidate-point source `try_route` happens to pick.
    #[allow(clippy::too_many_arguments)]
    fn route_via_grid_prefiltered_knn(
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
        let items: Vec<Item> = world.iter().cloned().collect();
        let region = outline_bounds(outline)
            .unwrap_or_else(|| corridor_region(from, to, *CORRIDOR_ESCALATION_FACTORS.last().unwrap()));
        let (grouped_points, _truncated) =
            candidate_points(items.iter(), from, to, width, net, layer, class, resolver, outline, Some(region));
        let points: Vec<Point> = grouped_points.iter().map(|&(p, _group)| p).collect();
        let is_valid_edge = |a: Point, b: Point| {
            world.path_is_clear(a, b, width, Some(net), layer, class, resolver)
                && edge_stays_on_board(a, b, outline)
        };
        let grid_region = points_bbox(&points).inflate(crate::grid_obstacle::DEFAULT_GRID_STEP * 4);
        let mut grid =
            crate::grid_obstacle::GridObstacleMap::new(grid_region, crate::grid_obstacle::DEFAULT_GRID_STEP);
        grid.rasterize_items(items.iter(), from, to, width, net, layer, class, resolver);
        let is_valid_edge_knn = |a: Point, b: Point| {
            if grid.segment_definitely_blocked(a, b, layer) {
                return false;
            }
            is_valid_edge(a, b)
        };
        let adjacency = build_adjacency_knn(&grouped_points, &is_valid_edge_knn);
        astar_search(&points, &adjacency, 0, 1).or_else(|| {
            let full = build_adjacency_full(&points, &is_valid_edge);
            astar_search(&points, &full, 0, 1)
        })
    }

    #[test]
    fn grid_prefilter_can_route_around_a_single_circular_obstacle() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(2_500_000, 0), 800_000)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });
        let (from, to) = (Point::new(0, 0), Point::new(5 * MM, 0));

        let path = route_via_grid_prefiltered_knn(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("grid-prefiltered KNN must still find a way around a single obstacle");

        assert_eq!(*path.first().unwrap(), from);
        assert_eq!(*path.last().unwrap(), to);
        for leg in path.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        }
    }

    #[test]
    fn grid_prefilter_can_route_around_a_filled_zone_obstacle() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;
        let p = |x_mm: f64, y_mm: f64| Point::new((x_mm * MM as f64) as Unit, (y_mm * MM as f64) as Unit);

        world.add(Item::Zone {
            outline: Polygon::new(vec![p(3.0, -2.0), p(7.0, -2.0), p(7.0, 2.0), p(3.0, 2.0)]),
            layer: LayerId::FCu,
            net: Some(NetId(2)),
        });
        let (from, to) = (p(0.0, 0.0), p(10.0, 0.0));

        let path = route_via_grid_prefiltered_knn(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("grid-prefiltered KNN must still find a way around a filled zone");

        assert_eq!(*path.first().unwrap(), from);
        assert_eq!(*path.last().unwrap(), to);
        assert!(path.len() > 2, "must actually detour, not go straight through the zone");
        for leg in path.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        }
    }

    #[test]
    fn grid_prefilter_can_route_around_a_rotated_rectangular_pad() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;
        let mm = |v: f64| (v * MM as f64) as Unit;
        let center = Point::new(mm(5.0), 0);
        let half_width = mm(2.0);
        let half_height = mm(0.5);
        let local = [
            Point::new(-half_width, -half_height),
            Point::new(half_width, -half_height),
            Point::new(half_width, half_height),
            Point::new(-half_width, half_height),
        ];
        let outline = Polygon::new(local.iter().map(|&p| p.rotated(30.0).add(center)).collect());
        world.add(Item::Pad { shape: PadShape::Polygon { outline, center }, net: Some(NetId(2)), layer: LayerId::FCu });

        let (from, to) = (Point::new(0, mm(0.75)), Point::new(mm(10.0), mm(0.75)));
        let width = 250_000;

        let path = route_via_grid_prefiltered_knn(
            &world, from, to, width, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("grid-prefiltered KNN must still find a way around the pad's true, rotated shape");

        assert_eq!(*path.first().unwrap(), from);
        assert_eq!(*path.last().unwrap(), to);
        assert!(path.len() > 2, "must actually detour around the true corner, not go straight through it");
        for leg in path.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], width, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        }
    }

    #[test]
    fn grid_prefilter_can_navigate_a_staggered_two_obstacle_chicane() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(Item::Track {
            shape: Segment::new(Point::new(2 * MM, -3 * MM), Point::new(2 * MM, 500_000), 300_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        world.add(Item::Track {
            shape: Segment::new(Point::new(4 * MM, -500_000), Point::new(4 * MM, 3 * MM), 300_000),
            net: Some(NetId(3)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let (from, to) = (Point::new(0, 0), Point::new(6 * MM, 0));
        let path = route_via_grid_prefiltered_knn(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("grid-prefiltered KNN must still find a valid chicane path around both walls");

        assert_eq!(*path.first().unwrap(), from);
        assert_eq!(*path.last().unwrap(), to);
        for leg in path.windows(2) {
            assert!(
                world.path_is_clear(leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver),
                "leg {leg:?} must clear both walls"
            );
        }
    }

    /// Direct asymmetry-safety proof, not just an indirect "the found
    /// path happens to be clear" check: constructs a case where the
    /// grid pre-filter's own coarse-quantization approximation (see
    /// `GridObstacleMap::segment_definitely_blocked`'s doc comment) is
    /// deliberately provoked -- a real obstacle sits close enough to a
    /// candidate edge that a coarse grid cell could plausibly disagree
    /// with the exact geometric answer in *either* direction -- and
    /// confirms every edge [`build_adjacency_knn`] actually accepts
    /// through the grid-prefiltered path still independently passes the
    /// real, exact `Node::path_is_clear` check. This is the property
    /// the whole feature's safety rests on: a grid "yes, blocked" may
    /// substitute for the real check, but nothing here ever lets a grid
    /// "no" substitute for it -- so no accepted edge can ever depend on
    /// the grid being right.
    #[test]
    fn grid_prefilter_never_accepts_an_edge_the_real_check_would_reject() {
        let resolver = JlcpcbClearance;
        let mut world = Node::new();
        // A ring of pads whose gaps are only *just* wide enough for the
        // real geometric check to pass -- exactly the tight-margin case
        // most likely to expose a grid quantization disagreement.
        let pads: Vec<Item> = (0..24)
            .map(|i| {
                let angle = std::f64::consts::TAU * (i as f64) / 24.0;
                let r = 3.0 * MM as f64;
                Item::Pad {
                    shape: PadShape::Circle(Circle::new(
                        Point::new((r * angle.cos()) as Unit, (r * angle.sin()) as Unit),
                        350_000,
                    )),
                    net: Some(NetId(100 + i as u32)),
                    layer: LayerId::FCu,
                }
            })
            .collect();
        for pad in &pads {
            world.add(pad.clone());
        }

        let from = Point::new(-5 * MM, 0);
        let to = Point::new(5 * MM, 0);
        let width = 200_000;
        let net = NetId(1);
        let layer = LayerId::FCu;
        let class = NetClass::C;
        let region = corridor_region(from, to, CORRIDOR_ESCALATION_FACTORS[1]);

        let (grouped_points, _truncated) =
            candidate_points(pads.iter(), from, to, width, net, layer, class, &resolver, &[], Some(region));
        let points: Vec<Point> = grouped_points.iter().map(|&(p, _group)| p).collect();
        let is_valid_edge = |a: Point, b: Point| world.path_is_clear(a, b, width, Some(net), layer, class, &resolver);

        let grid_region = points_bbox(&points).inflate(crate::grid_obstacle::DEFAULT_GRID_STEP * 4);
        let mut grid =
            crate::grid_obstacle::GridObstacleMap::new(grid_region, crate::grid_obstacle::DEFAULT_GRID_STEP);
        grid.rasterize_items(pads.iter(), from, to, width, net, layer, class, &resolver);
        let is_valid_edge_knn = |a: Point, b: Point| {
            if grid.segment_definitely_blocked(a, b, layer) {
                return false;
            }
            is_valid_edge(a, b)
        };

        let adjacency = build_adjacency_knn(&grouped_points, &is_valid_edge_knn);
        let mut checked = 0usize;
        for (i, row) in adjacency.iter().enumerate() {
            for &(j, _) in row {
                assert!(
                    is_valid_edge(points[i], points[j]),
                    "edge {:?} -> {:?} was accepted by the grid-prefiltered KNN build but fails the real check",
                    points[i],
                    points[j]
                );
                checked += 1;
            }
        }
        assert!(checked > 0, "sanity: this scene must produce at least some accepted edges to check");
    }

    #[test]
    fn corridor_region_is_the_endpoint_bbox_inflated_by_the_margin() {
        let from = Point::new(0, 0);
        let to = Point::new(10 * MM, 0);
        let region = corridor_region(from, to, CORRIDOR_MARGIN_FACTOR);
        let expected_margin = (10.0 * MM as f64 * CORRIDOR_MARGIN_FACTOR).round() as Unit;

        assert_eq!(region.min, Point::new(0 - expected_margin, 0 - expected_margin));
        assert_eq!(region.max, Point::new(10 * MM + expected_margin, 0 + expected_margin));
    }

    #[test]
    fn corridor_region_grows_with_a_larger_factor() {
        let from = Point::new(0, 0);
        let to = Point::new(10 * MM, 0);
        let factor = 4.0;
        let region = corridor_region(from, to, factor);
        let expected_margin = (10.0 * MM as f64 * factor).round() as Unit;

        assert_eq!(region.min, Point::new(0 - expected_margin, 0 - expected_margin));
        assert_eq!(region.max, Point::new(10 * MM + expected_margin, 0 + expected_margin));
    }

    #[test]
    fn outline_bounds_is_none_for_no_outline_and_the_union_box_otherwise() {
        assert_eq!(outline_bounds(&[]), None, "no outline supplied means no bound available");

        let p = |x_mm: i64, y_mm: i64| Point::new(x_mm * MM, y_mm * MM);
        let a = Polygon::new(vec![p(0, 0), p(10, 0), p(10, 10), p(0, 10)]);
        let b = Polygon::new(vec![p(-5, -5), p(-3, -5), p(-3, -3), p(-5, -3)]);
        let bounds = outline_bounds(&[a, b]).expect("two polygons must yield a union bound");
        assert_eq!(bounds.min, p(-5, -5), "union must extend to the second polygon's own min corner");
        assert_eq!(bounds.max, p(10, 10), "union must extend to the first polygon's own max corner");
    }

    #[test]
    fn query_region_only_picks_up_a_distant_obstacle_once_the_corridor_escalates() {
        // Real (not synthetic) proof of the escalation loop's mechanics:
        // an obstacle positioned comfortably outside the base-margin
        // corridor's box but comfortably inside the first escalation
        // stage's wider one -- verified via `Node::query_region` itself,
        // the exact call `find_path_astar`'s escalation loop makes, not
        // just `corridor_region`'s bounds in isolation.
        let mut world = Node::new();
        let from = Point::new(0, 0);
        let to = Point::new(10 * MM, 0);

        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(5 * MM, 15 * MM), 2 * MM)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });

        let base = world.query_region(corridor_region(from, to, CORRIDOR_MARGIN_FACTOR));
        assert!(
            base.is_empty(),
            "the base-margin corridor must not see an obstacle this far from the direct line"
        );

        let escalated = world.query_region(corridor_region(from, to, CORRIDOR_ESCALATION_FACTORS[0]));
        assert_eq!(
            escalated.len(), 1,
            "the first escalation stage's wider corridor must pick up the same obstacle"
        );
    }

    #[test]
    fn try_route_can_use_an_obstacle_only_a_wider_corridor_would_have_included() {
        // Companion to `spatial_filter_excluding_a_needed_obstacle_falls_back_correctly`
        // above, but this time the excluded obstacle's *raw* extent
        // (no clearance -- exactly what `Node::query_region` tests) is
        // verified to sit entirely outside the base-margin corridor and
        // entirely inside the first escalation stage's one, rather than
        // being a same-margin stand-in. Its *clearance* zone still
        // reaches back to graze the direct line (verified below), so
        // this is a real instance of the failure mode
        // `CORRIDOR_ESCALATION_FACTORS` exists for: a graph built from
        // only the base corridor's items has no obstacle boundary to
        // route around at all and must fail outright, not just find a
        // worse detour -- `path_is_clear` (used to validate every
        // candidate edge) always checks the complete `world`, never just
        // the candidate-point source list, so the direct line is
        // (correctly) rejected either way.
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        let width = 250_000;
        let net = NetId(1);

        let from = Point::new(0, 0);
        let to = Point::new(230_000, 0);

        let blocker = Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(115_000, 260_000), 19_000)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        };
        world.add(blocker.clone());

        // Fixture sanity check: raw-extent exclusion/inclusion, exactly
        // as `find_path_astar`'s escalation loop would see it.
        let base_region = corridor_region(from, to, CORRIDOR_MARGIN_FACTOR);
        assert!(
            world.query_region(base_region).is_empty(),
            "fixture sanity check: the base-margin corridor must not see this obstacle"
        );
        let escalated_region = corridor_region(from, to, CORRIDOR_ESCALATION_FACTORS[0]);
        assert_eq!(
            world.query_region(escalated_region).len(), 1,
            "fixture sanity check: the first escalation stage's corridor must see it"
        );

        // Fixture sanity check: it still actually blocks the direct
        // line via real collision, regardless of graph membership.
        assert!(
            !world.path_is_clear(from, to, width, Some(net), LayerId::FCu, NetClass::C, &resolver),
            "fixture sanity check: the obstacle must actually block the direct line"
        );

        // Incomplete graph (as if only the base-margin corridor had been
        // searched): no candidate points besides from/to exist, and the
        // only edge available (the direct line) is blocked.
        let incomplete = try_route(
            &world, std::iter::empty::<&Item>(), from, to, width, net, LayerId::FCu, NetClass::C, &resolver, &[],
            None,
        );
        assert!(
            incomplete.is_none(),
            "a graph missing this obstacle's boundary must fail to find any path at all"
        );

        // Complete graph (as if the first escalation stage had been
        // searched): the obstacle's own clearance boundary now supplies
        // a way around it.
        let complete = try_route(
            &world, std::iter::once(&blocker), from, to, width, net, LayerId::FCu, NetClass::C, &resolver, &[],
            None,
        )
        .expect("the obstacle's own boundary must provide a valid detour");
        assert_eq!(*complete.first().unwrap(), from);
        assert_eq!(*complete.last().unwrap(), to);
        for leg in complete.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], width, Some(net), LayerId::FCu, NetClass::C, &resolver));
        }

        // And the public API -- which performs exactly this escalation
        // automatically -- must also succeed on this fixture.
        assert!(
            find_path_astar(&world, from, to, width, net, LayerId::FCu, NetClass::C, &resolver, &[]).is_some(),
            "find_path_astar must succeed via its own corridor escalation"
        );
    }

    #[test]
    fn spatial_filter_excluding_a_needed_obstacle_falls_back_correctly() {
        // White-box test of the actual failure mode the fast/fallback
        // split exists for: obstacle A alone is easily routed around,
        // but a *second* obstacle positioned right where A's detour
        // needs to go (here: symmetric obstacles above *and* below, so
        // neither direction around A is free) means a candidate graph
        // that's missing that second obstacle's boundary points can't
        // find a path at all, even though the true collision check
        // (`Node::path_is_clear`) never depends on which points made it
        // into the graph.
        //
        // This is deliberately tested at the `try_route` level rather
        // than through the public `find_path_astar`: with
        // `CORRIDOR_MARGIN_FACTOR = 1.0`, the default corridor is
        // generous enough (by design -- see that constant's docs) that
        // this particular scenario's secondary obstacles still fall
        // inside it, so the public fast path already succeeds without
        // ever needing the fallback. That's a *good* property of the
        // margin choice, not a gap in coverage -- so this test instead
        // directly proves the mechanism `find_path_astar` relies on:
        // build the candidate graph from an explicit item subset (a
        // stand-in for "what a stricter filter would have kept") and
        // confirm the graph-completeness/correctness relationship holds
        // in both directions.
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);

        let a = Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(5 * MM, 0), 3 * MM)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        };
        let b_top = Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(5 * MM, 4 * MM), 1_500_000)),
            net: Some(NetId(3)),
            layer: LayerId::FCu,
        };
        let b_bottom = Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(5 * MM, -4 * MM), 1_500_000)),
            net: Some(NetId(4)),
            layer: LayerId::FCu,
        };
        world.add(a.clone());
        world.add(b_top);
        world.add(b_bottom);

        let from = Point::new(0, 0);
        let to = Point::new(10 * MM, 0);
        let width = 250_000;
        let net = NetId(1);

        // Only A's boundary in the graph (as if B had been filtered
        // out): A's own tangent route runs straight into B's clearance
        // zone on both sides, and `path_is_clear` correctly rejects
        // every such edge -- so no path exists in *this* graph.
        let incomplete = try_route(
            &world, [a].iter(), from, to, width, net, LayerId::FCu, NetClass::C, &resolver, &[], None,
        );
        assert!(
            incomplete.is_none(),
            "a graph missing the blocking secondary obstacle's boundary must fail to find a path"
        );

        // The same query, with every item's boundary available (what
        // `find_path_astar`'s fallback pass does), must succeed and
        // produce a fully verified path.
        let complete = try_route(
            &world, world.iter(), from, to, width, net, LayerId::FCu, NetClass::C, &resolver, &[], None,
        )
        .expect("the complete candidate graph must find a valid path around all three obstacles");

        assert_eq!(*complete.first().unwrap(), from);
        assert_eq!(*complete.last().unwrap(), to);
        for leg in complete.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], width, Some(net), LayerId::FCu, NetClass::C, &resolver));
        }

        // And `find_path_astar` itself must also succeed on this world
        // (via its fast path here, since the margin happens to cover
        // it) -- confirming the public API gives the same guarantee.
        assert!(find_path_astar(&world, from, to, width, net, LayerId::FCu, NetClass::C, &resolver, &[]).is_some());
    }

    #[test]
    fn full_fallback_is_skipped_once_the_candidate_graph_is_too_large_to_be_practical() {
        // Regression test for the real-board hang found routing a
        // 109-LED panel's DATA daisy chain: some of its nets genuinely
        // have no valid same-layer path at all (their real solution
        // needed a brief via/B.Cu hop `alladin-router` can't insert
        // itself yet) -- so *every* graph-building strategy correctly
        // reports no path, but without `MAX_FULL_FALLBACK_POINTS`,
        // `try_route` would still always build the exhaustive `O(n^2)`
        // graph before giving up, which is a many-minute-or-worse hang
        // once the candidate count is in the hundred-thousands (a real
        // zone's own vertex count, see `candidate_points`'s doc comment
        // on `corridor`).
        //
        // `to` sits exactly on a different-net pad's own centre, so
        // *every* edge that could ever terminate there collides
        // immediately -- no path can exist regardless of which graph
        // (KNN or exhaustive) is searched, so this isolates the cap's
        // one actual job (stay fast) from the separate, already-covered
        // question of KNN-vs-exhaustive correctness. Padding is done
        // with many small, separate, far-away pads rather than one huge
        // polygon deliberately: a pad-vs-track check is `O(1)` (unlike a
        // zone's `O(vertex count)`, see `alladin_geom::segment_polygon_collides`),
        // so this stays a fast unit test even at several thousand
        // candidate points.
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        let width = 250_000;
        let net = NetId(1);
        let from = Point::new(0, 0);
        let to = Point::new(20 * MM, 0);

        world.add(Item::Pad { shape: PadShape::Circle(Circle::new(to, 500_000)), net: Some(NetId(2)), layer: LayerId::FCu });

        let pad_count = MAX_FULL_FALLBACK_POINTS / CIRCLE_SEGMENTS + 50;
        for i in 0..pad_count {
            let angle = std::f64::consts::TAU * (i as f64) / (pad_count as f64);
            let r = 1_000.0 * MM as f64;
            let center = Point::new(
                (500 * MM) + (r * angle.cos()) as Unit,
                (500 * MM) + (r * angle.sin()) as Unit,
            );
            world.add(Item::Pad { shape: PadShape::Circle(Circle::new(center, 300_000)), net: Some(NetId(100 + i as u32)), layer: LayerId::FCu });
        }

        let started = std::time::Instant::now();
        let result = try_route(&world, world.iter(), from, to, width, net, LayerId::FCu, NetClass::C, &resolver, &[], None);
        let elapsed = started.elapsed();

        assert!(
            result.is_none(),
            "`to` collides with a different-net pad centred exactly on it -- no path can ever exist"
        );
        assert!(
            elapsed < std::time::Duration::from_secs(5),
            "an oversized candidate graph must skip the O(n^2) fallback and stay fast, not hang trying to (re-)prove the obvious -- took {elapsed:.2?}"
        );
    }

    #[test]
    fn many_distant_irrelevant_obstacles_do_not_prevent_finding_the_real_route() {
        // The actual practical payoff of `corridor_region`/`query_region`:
        // a board with lots of components far from this particular net's
        // path shouldn't force every candidate-graph edge check to
        // reckon with all of them. This test only asserts *correctness*
        // (routing still works, still verified clear) -- proving the
        // filtered fast path doesn't silently drop a result -- not
        // performance, which would need timing and is out of scope for
        // a unit test.
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);

        // 40 small pads scattered far away in every direction, well
        // outside any plausible corridor for the from/to pair below.
        for i in 0..40i64 {
            let angle = (i as f64) * 0.157; // spread them around a big circle
            let radius_mm = 50.0 + (i % 5) as f64 * 10.0;
            let x = (2_500_000.0 + radius_mm * MM as f64 * angle.cos()) as Unit;
            let y = (radius_mm * MM as f64 * angle.sin()) as Unit;
            world.add(Item::Pad {
                shape: PadShape::Circle(Circle::new(Point::new(x, y), 300_000)),
                net: Some(NetId(100 + i as u32)),
                layer: LayerId::FCu,
            });
        }

        // The one obstacle that actually matters for this route.
        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(2_500_000, 0), 800_000)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });

        let path = find_path_astar(
            &world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        ).expect("40 irrelevant distant obstacles must not prevent routing around the real one");

        assert_eq!(*path.first().unwrap(), Point::new(0, 0));
        assert_eq!(*path.last().unwrap(), Point::new(5 * MM, 0));
        for leg in path.windows(2) {
            assert!(world.path_is_clear(leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        }
    }

    /// An L-shaped board (10x10mm square missing its top-left 5x5mm
    /// corner) -- same shape as the CLI demo's outline scenario, chosen
    /// again here because it's the simplest fixture that actually
    /// exercises `contains_segment`'s "both endpoints inside, straight
    /// line between them isn't" case, not just `contains_point`.
    fn l_shaped_board() -> Polygon {
        let p = |x: i64, y: i64| Point::new(x * MM, y * MM);
        Polygon::new(vec![p(0, 0), p(10, 0), p(10, 10), p(5, 10), p(5, 5), p(0, 5)])
    }

    #[test]
    fn direct_path_leaving_the_board_detours_via_the_outline_s_own_concave_vertex() {
        // Regression/feature test for the "Teil 15" fix: this exact
        // scenario used to be a documented limitation -- no obstacle
        // exists anywhere near the missing corner to seed a detour
        // waypoint, so before `candidate_points` fed in the outline's
        // own (inward-nudged) vertices, this came back `None` outright,
        // even though a human router could obviously thread it via the
        // concave corner at (5mm, 5mm).
        let world = Node::new();
        let resolver = FixedClearance(127_000);
        let outline = l_shaped_board();

        // Nothing blocks this collision-wise -- only the outline (and
        // the missing top-left corner specifically) makes the direct
        // line invalid.
        let from = Point::new(1 * MM, 4 * MM);
        let to = Point::new(9 * MM, 9 * MM);
        assert!(world.path_is_clear(from, to, 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));
        assert!(
            !outline.contains_segment(from, to),
            "sanity check on the fixture itself: the direct line must actually cut through the missing corner"
        );

        let with_outline = find_path_astar(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver,
            std::slice::from_ref(&outline),
        )
        .expect("must find a detour around the missing corner via the outline's own concave vertex");

        assert_eq!(*with_outline.first().unwrap(), from);
        assert_eq!(*with_outline.last().unwrap(), to);
        for leg in with_outline.windows(2) {
            assert!(
                outline.contains_segment(leg[0], leg[1]),
                "every leg of the detour must actually stay on the board: {leg:?}"
            );
            assert!(world.path_is_clear(
                leg[0], leg[1], 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver
            ));
        }
    }

    #[test]
    fn an_endpoint_outside_the_outline_fails_immediately_without_building_a_candidate_graph() {
        // Regression test for the performance issue found while
        // validating this against a real 1511-item board (see
        // `find_path_astar`'s own doc comment): `to` here is off the
        // L-shaped board's outline entirely (x=15mm, well past its
        // x<=10mm extent) -- this must be rejected outright, not by
        // building a full visibility graph and having every single
        // candidate edge fail the outline check one at a time.
        let world = Node::new();
        let resolver = FixedClearance(127_000);
        let outline = l_shaped_board();

        let result = find_path_astar(
            &world, Point::new(1 * MM, 1 * MM), Point::new(15 * MM, 1 * MM), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[outline],
        );
        assert!(result.is_none(), "an endpoint off the board must never produce a route");
    }

    #[test]
    fn knn_graph_connects_across_different_obstacles_not_just_within_one() {
        // Regression test for a real bug found while performance-tuning
        // `build_adjacency_knn` against a real 1511-item board (see its
        // own doc comment): a point's raw-nearest neighbours are usually
        // *other samples of its own obstacle's boundary*, and connecting
        // those wastes the whole neighbour budget on chords that (in
        // practice) almost always cut back inside that very obstacle.
        // Two obstacles close enough together that their boundary
        // samples interleave in raw distance order reproduces the setup
        // directly, using an always-valid edge predicate so this test
        // isolates the KNN/grouping mechanics from collision geometry.
        let a_center = Point::new(0, 0);
        let b_center = Point::new(2 * MM, 0);
        let radius = 500_000;

        let mut points: Vec<(Point, usize)> =
            circle_boundary(a_center, radius).into_iter().map(|p| (p, 0usize)).collect();
        points.extend(circle_boundary(b_center, radius).into_iter().map(|p| (p, 1usize)));

        let always_valid = |_a: Point, _b: Point| true;
        let adjacency = build_adjacency_knn(&points, &always_valid);

        for (i, &(_, group)) in points.iter().enumerate() {
            assert!(!adjacency[i].is_empty(), "point {i} (group {group}) ended up with no edges at all");
            assert!(
                adjacency[i].iter().any(|&(j, _)| points[j].1 != group),
                "point {i} (group {group}) never connected to the other obstacle -- \
same-group candidates were not correctly excluded from the neighbour budget"
            );
        }
    }

    #[test]
    fn an_endpoint_already_colliding_with_a_different_net_pad_fails_immediately() {
        // Regression test for the second short-circuit found during the
        // same real-board investigation (see `find_path_astar`'s own doc
        // comment): `from` here sits *inside* an existing different-net
        // pad's clearance zone -- every candidate edge starting there is
        // doomed before any graph is even built, so this must come back
        // `None` immediately rather than after paying the full
        // exhaustive-fallback cost to (correctly but slowly) discover
        // the same thing.
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);

        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), 800_000)),
            net: Some(NetId(2)), // different net than the route below
            layer: LayerId::FCu,
        });

        let result = find_path_astar(
            &world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        );
        assert!(result.is_none(), "a start point buried in another net's pad must never produce a route");

        // Sanity check: the same pad on `from`'s *own* net must not
        // trigger this at all (same-net pairs never collide).
        let same_net = find_path_astar(
            &world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(2), LayerId::FCu, NetClass::C, &resolver, &[],
        );
        assert!(same_net.is_some(), "routing from a pad on its own net must still work");
    }

    #[test]
    fn a_route_fully_inside_the_outline_is_unaffected() {
        let world = Node::new();
        let resolver = FixedClearance(127_000);
        let outline = l_shaped_board();

        // Entirely within the lower rectangle, nowhere near the missing
        // corner -- an outline constraint must not reject this.
        let path = find_path_astar(
            &world, Point::new(1 * MM, 1 * MM), Point::new(9 * MM, 4 * MM), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[outline],
        ).expect("a route that never leaves the board must still succeed with an outline supplied");
        assert_eq!(path, vec![Point::new(1 * MM, 1 * MM), Point::new(9 * MM, 4 * MM)]);
    }

    #[test]
    fn an_obstacle_straddling_the_missing_corner_forces_the_on_board_detour_side() {
        // A circular obstacle centred exactly on the L-shaped board's
        // missing-corner vertex (5,5), with `from`/`to` placed
        // symmetrically across it on the diagonal `y = x`. That
        // symmetry means both ways around the obstacle are the *same
        // length* -- but only one of them (the lower-right side, into
        // the always-allowed main rectangle / upper-right sub-rectangle)
        // stays on the board; the other (upper-left) cuts straight
        // through the missing corner. Without an outline, `path_is_clear`
        // alone can't tell the difference and may pick either side
        // (proven routable at all by the `without_outline` sub-case);
        // with the outline supplied, the forbidden side's candidate
        // edges must all be rejected, forcing (and still finding) the
        // on-board detour.
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        let outline = l_shaped_board();

        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(5 * MM, 5 * MM), 300_000)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });

        let from = Point::new(3 * MM, 3 * MM);
        let to = Point::new(8 * MM, 8 * MM);

        let without_outline = find_path_astar(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        );
        assert!(
            without_outline.is_some(),
            "sanity check: without an outline this obstacle must be routable around at all"
        );

        let with_outline = find_path_astar(
            &world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[outline.clone()],
        ).expect("a fully on-board detour exists on the other side of the obstacle and must be found");

        for leg in with_outline.windows(2) {
            assert!(
                outline.contains_segment(leg[0], leg[1]),
                "every leg of an outline-constrained route must stay on the board: {leg:?}"
            );
        }
    }

    #[test]
    fn diagnose_failure_reports_endpoint_off_board() {
        let world = Node::new();
        let resolver = FixedClearance(127_000);
        let outline = vec![Polygon::new(vec![
            Point::new(0, 0), Point::new(10 * MM, 0), Point::new(10 * MM, 10 * MM), Point::new(0, 10 * MM),
        ])];

        let from = Point::new(-5 * MM, 5 * MM); // outside the outline
        let to = Point::new(5 * MM, 5 * MM);
        assert!(find_path_astar(&world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &outline).is_none());

        let reason = diagnose_failure(&world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &outline);
        assert!(
            matches!(reason, FailureReason::EndpointOffBoard { endpoint: Endpoint::From, at } if at == from),
            "expected EndpointOffBoard for `from`, got {reason:?}"
        );
    }

    #[test]
    fn diagnose_failure_reports_endpoint_blocked_and_names_the_blocker() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);

        let blocker = Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(5 * MM, 0), 900_000)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        };
        let blocker_id = world.add(blocker);

        let from = Point::new(0, 0);
        let to = Point::new(5 * MM, 0); // sits exactly on the different-net pad's own centre
        assert!(find_path_astar(&world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[]).is_none());

        let reason = diagnose_failure(&world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[]);
        match reason {
            FailureReason::EndpointBlocked { endpoint: Endpoint::To, at, blocking_items } => {
                assert_eq!(at, to);
                assert!(blocking_items.contains(&blocker_id), "must name the actual blocking pad, got {blocking_items:?}");
            }
            other => panic!("expected EndpointBlocked for `to`, got {other:?}"),
        }
    }

    /// Four pads placed in a diamond around `Point::new(0, 0)`, each
    /// individually far enough away that `to` itself is *not*
    /// clearance-blocked by any single one of them (unlike a tighter
    /// box that would block `to` directly), but with each pair of
    /// adjacent pads left too close
    /// together for a 250,000-wide track plus clearance to ever squeeze
    /// through the gap between them. Isolates "point itself is fine,
    /// but genuinely walled in" from "point itself already collides".
    fn sealed_diamond_around_origin(world: &mut Node) {
        for (dx, dy) in [(3 * MM, 0), (-3 * MM, 0), (0, 3 * MM), (0, -3 * MM)] {
            world.add(Item::Pad {
                shape: PadShape::Circle(Circle::new(Point::new(dx, dy), 2 * MM)),
                net: Some(NetId(99)),
                layer: LayerId::FCu,
            });
        }
    }

    #[test]
    fn diagnose_failure_reports_no_path_exists_when_genuinely_enclosed() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        sealed_diamond_around_origin(&mut world);

        let from = Point::new(20 * MM, 0);
        let to = Point::new(0, 0);

        // Sanity check on the fixture itself: `to` must not be
        // considered blocked by any single pad's own clearance zone --
        // only the *combination* of all four seals it in.
        let probe = Item::Track { shape: Segment::new(to, to, 250_000), net: Some(NetId(1)), layer: LayerId::FCu, class: NetClass::C };
        assert!(!world.is_colliding(&probe, &resolver), "fixture sanity check: `to` must not directly collide with any single pad");

        assert!(find_path_astar(&world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[]).is_none());

        let reason = diagnose_failure(&world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[]);
        assert!(
            matches!(reason, FailureReason::NoPathExists { .. }),
            "expected NoPathExists, got {reason:?}"
        );
    }

    #[test]
    fn diagnose_failure_reports_search_too_complex_for_an_oversized_graph() {
        // Same genuinely-sealed diamond as the `NoPathExists` test above
        // (so this failure is real either way, not an artefact of this
        // fixture), plus several hundred extra, far-away, otherwise
        // irrelevant pads -- enough to push the *final, unbounded*
        // diagnostic pass (no outline supplied, so it considers every
        // item in the world, exactly like `find_path_astar`'s own
        // last-resort fallback stage) past
        // `MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE`. The point: diagnosing
        // this must recognise the candidate graph was too large to
        // fully search, not silently call it a proven `NoPathExists`,
        // even though -- as it happens -- the diamond really does seal
        // `to` in regardless.
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        sealed_diamond_around_origin(&mut world);

        let extra_pad_count = MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE / CIRCLE_SEGMENTS + 50;
        for i in 0..extra_pad_count {
            let angle = std::f64::consts::TAU * (i as f64) / (extra_pad_count as f64);
            let r = 1_000.0 * MM as f64; // far outside any corridor around `from`/`to` below
            let center = Point::new(
                (500 * MM) + (r * angle.cos()) as Unit,
                (500 * MM) + (r * angle.sin()) as Unit,
            );
            world.add(Item::Pad { shape: PadShape::Circle(Circle::new(center, 300_000)), net: Some(NetId(1000 + i as u32)), layer: LayerId::FCu });
        }

        let from = Point::new(20 * MM, 0);
        let to = Point::new(0, 0);
        assert!(find_path_astar(&world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[]).is_none());

        let reason = diagnose_failure(&world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[]);
        assert!(
            matches!(reason, FailureReason::SearchTooComplex { .. }),
            "expected SearchTooComplex, got {reason:?}"
        );
    }
}

