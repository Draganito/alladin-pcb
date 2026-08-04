//! A discretized, bitmap-backed obstacle map -- a structurally
//! different alternative to `astar.rs`'s continuous visibility-graph
//! candidate points (see that module's `candidate_points` doc comment,
//! and the development log's "Teil 28" entry).
//!
//! **The core idea, credited to `drandyhaas/KiCadRoutingTools`** (MIT
//! licensed; see that project's `rust_router/src/obstacle_map.rs`):
//! instead of *sampling* a fixed number of boundary points off every
//! obstacle for every single search (which is where a real 42,415-
//! vertex filled zone or a corridor with 1000+ nearby pads blows up --
//! see `astar.rs`'s `MAX_ZONE_CANDIDATE_POINTS_PER_ITEM`/
//! `MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE` doc comments), rasterize every
//! obstacle **once** into a set of blocked `(gx, gy)` grid cells on its
//! own layer. After that one-time cost, "is this cell blocked?" is an
//! `O(1)` bit lookup regardless of how geometrically complex the
//! obstacle that blocked it was -- there is no candidate-point count to
//! explode at all.
//!
//! **Deliberately simpler than the reference project's own
//! `GridObstacleMap`, and why that's still correct here:**
//! `KiCadRoutingTools` maintains one long-lived, incrementally-updated
//! (refcounted) obstacle map shared across an entire multi-hundred-net
//! routing run, so it needs `add`/`remove` and `clone()` for its
//! per-net obstacle churn. Alladin's [`GridObstacleMap`] is instead
//! built **fresh, once, per search call** -- exactly the same lifecycle
//! `astar.rs::candidate_points` already has (`alladin_core::Node` is
//! the one actual persistent world; this is just another way to view a
//! *snapshot* of it for one query) -- so there is no need for
//! refcounting or incremental removal, only a one-shot rasterization
//! pass followed by read-only lookups during the search that follows.
//!
//! Two layers only ([`LayerId::FCu`]/[`LayerId::BCu`]), matching
//! Alladin's current single-layer-per-net, no-via-insertion scope (see
//! the development log's "Teil 27" entry on why the router itself never
//! inserts vias).

use alladin_core::{Item, LayerId, NetClass, NetId, RuleResolver};
use alladin_geom::{
    circle_polygon_collides_indexed, dist_point_to_line, Aabb, Circle, Point, Polygon,
    PolygonEdgeIndex, Segment, Unit,
};

/// Hard cap on how many cells (across both layers combined) a single
/// [`GridObstacleMap`] will ever allocate, regardless of how large a
/// `region`/how fine a `step` is requested. Without this, a caller
/// asking for the whole-board fallback region at a very fine step could
/// request a multi-gigabyte bitmap; [`GridObstacleMap::new`] instead
/// silently coarsens `step` (doubling it until the cell count fits)
/// rather than ever allocating past this budget. `16_000_000` cells is
/// 2 MB per layer as a packed bitmap (`u64` words) -- generous for any
/// real board's corridor at a sane grid step, negligible next to the
/// multi-second-per-net costs this module exists to avoid.
const MAX_GRID_CELLS: i64 = 16_000_000;

/// Default grid resolution: 0.05 mm. Fine enough that the grid path's
/// own quantization error is well under typical clearance margins
/// (JLCPCB's own minimum is 0.127 mm), coarse enough that a
/// board-spanning corridor still fits comfortably under
/// [`MAX_GRID_CELLS`]. Exposed as a tunable, not hard-coded into
/// [`GridObstacleMap::new`], so benchmarking a different resolution
/// doesn't require touching this module.
pub const DEFAULT_GRID_STEP: Unit = 50_000;

/// A discretized snapshot of every obstacle relevant to one A* search,
/// as a per-layer bitmap of blocked cells. See this module's own doc
/// comment for the full rationale.
pub struct GridObstacleMap {
    /// World-space coordinate of grid cell `(0, 0)`'s min corner.
    origin: Point,
    /// Cell size, internal units (nanometres).
    step: Unit,
    /// Cells in x/y. Always at least `1x1`.
    width: i32,
    height: i32,
    blocked_fcu: Vec<u64>,
    blocked_bcu: Vec<u64>,
}

impl GridObstacleMap {
    /// Builds an empty (nothing blocked yet) map covering `region`, at
    /// `step` resolution -- coarsened automatically if that would
    /// exceed [`MAX_GRID_CELLS`] (see that constant's doc comment).
    pub fn new(region: Aabb, step: Unit) -> Self {
        let mut step = step.max(1);
        let (width, height) = loop {
            let w = (((region.max.x - region.min.x) as f64 / step as f64).ceil() as i64).max(1);
            let h = (((region.max.y - region.min.y) as f64 / step as f64).ceil() as i64).max(1);
            if w * h <= MAX_GRID_CELLS {
                break (w as i32, h as i32);
            }
            step *= 2;
        };
        let words_per_layer = (width as usize * height as usize).div_ceil(64).max(1);
        Self {
            origin: region.min,
            step,
            width,
            height,
            blocked_fcu: vec![0u64; words_per_layer],
            blocked_bcu: vec![0u64; words_per_layer],
        }
    }

    pub fn step(&self) -> Unit {
        self.step
    }

    pub fn width(&self) -> i32 {
        self.width
    }

    pub fn height(&self) -> i32 {
        self.height
    }

    /// World point -> grid cell containing it (floor division; a point
    /// exactly on a cell boundary belongs to the cell above/right of
    /// it, consistent throughout this module).
    pub fn to_grid(&self, p: Point) -> (i32, i32) {
        let gx = ((p.x - self.origin.x) as f64 / self.step as f64).floor();
        let gy = ((p.y - self.origin.y) as f64 / self.step as f64).floor();
        (clamp_to_i32(gx), clamp_to_i32(gy))
    }

    /// Grid cell -> its own center, in world coordinates. The inverse
    /// of [`Self::to_grid`] up to quantization -- used both to place
    /// obstacle-vs-cell distance checks and to turn a found grid path
    /// back into real board coordinates.
    pub fn cell_center(&self, gx: i32, gy: i32) -> Point {
        Point::new(
            self.origin.x + gx as Unit * self.step + self.step / 2,
            self.origin.y + gy as Unit * self.step + self.step / 2,
        )
    }

    pub fn in_bounds(&self, gx: i32, gy: i32) -> bool {
        gx >= 0 && gy >= 0 && gx < self.width && gy < self.height
    }

    fn cell_index(&self, gx: i32, gy: i32) -> usize {
        gy as usize * self.width as usize + gx as usize
    }

    fn bits(&self, layer: LayerId) -> &[u64] {
        match layer {
            LayerId::FCu => &self.blocked_fcu,
            LayerId::BCu => &self.blocked_bcu,
        }
    }

    fn bits_mut(&mut self, layer: LayerId) -> &mut [u64] {
        match layer {
            LayerId::FCu => &mut self.blocked_fcu,
            LayerId::BCu => &mut self.blocked_bcu,
        }
    }

    /// The hot-path query [`crate::grid_astar`]'s search calls for
    /// every neighbour it considers: one bounds check plus one bit
    /// load, regardless of what obstacle(s) set that bit or how
    /// geometrically complex they were. Cells outside the map's own
    /// window count as blocked -- a search must never wander past the
    /// region it was actually rasterized for.
    pub fn is_blocked(&self, gx: i32, gy: i32, layer: LayerId) -> bool {
        if !self.in_bounds(gx, gy) {
            return true;
        }
        let i = self.cell_index(gx, gy);
        (self.bits(layer)[i >> 6] >> (i & 63)) & 1 != 0
    }

    pub fn set_blocked(&mut self, gx: i32, gy: i32, layer: LayerId) {
        if !self.in_bounds(gx, gy) {
            return;
        }
        let i = self.cell_index(gx, gy);
        self.bits_mut(layer)[i >> 6] |= 1u64 << (i & 63);
    }

    /// Rasterizes a filled disc (a pad or via's clearance-inflated
    /// footprint) onto every layer in `layers`.
    pub fn block_disc(&mut self, center: Point, radius: Unit, layers: &[LayerId]) {
        if radius <= 0 {
            return;
        }
        let Some((gx0, gy0, gx1, gy1)) = self.clip_window(Aabb {
            min: Point::new(center.x - radius, center.y - radius),
            max: Point::new(center.x + radius, center.y + radius),
        }) else {
            return;
        };
        let r2 = (radius as f64) * (radius as f64);
        for gy in gy0..=gy1 {
            for gx in gx0..=gx1 {
                let c = self.cell_center(gx, gy);
                let dx = (c.x - center.x) as f64;
                let dy = (c.y - center.y) as f64;
                if dx * dx + dy * dy <= r2 {
                    for &layer in layers {
                        self.set_blocked(gx, gy, layer);
                    }
                }
            }
        }
    }

    /// Rasterizes a clearance-inflated track capsule (a swept segment)
    /// onto every layer in `layers`.
    pub fn block_capsule(&mut self, a: Point, b: Point, radius: Unit, layers: &[LayerId]) {
        if radius <= 0 {
            return;
        }
        let Some((gx0, gy0, gx1, gy1)) = self.clip_window(Aabb {
            min: Point::new(a.x.min(b.x) - radius, a.y.min(b.y) - radius),
            max: Point::new(a.x.max(b.x) + radius, a.y.max(b.y) + radius),
        }) else {
            return;
        };
        let r = radius as f64;
        for gy in gy0..=gy1 {
            for gx in gx0..=gx1 {
                let c = self.cell_center(gx, gy);
                if dist_point_to_line(c, a, b) <= r {
                    for &layer in layers {
                        self.set_blocked(gx, gy, layer);
                    }
                }
            }
        }
    }

    /// Rasterizes a clearance-inflated filled polygon (a copper zone)
    /// onto `layer`. This is the actual fix for the real 42,415-vertex
    /// zone case documented in `astar.rs`: rather than sampling a
    /// (capped, lossy) subset of the polygon's boundary as candidate
    /// *points* on every search, its edges are indexed **once**
    /// ([`PolygonEdgeIndex::build`], already `O(log n)`-per-query --
    /// see that type's own doc comment for the same 42,415-vertex board
    /// this was built against), and only the cells inside the polygon's
    /// own clearance-inflated bounding box are ever tested against it --
    /// proportional to that zone's own footprint at this grid's
    /// resolution, not the whole search region and not its vertex
    /// count.
    ///
    /// **Deliberately not a hand-rolled scanline fill**, unlike
    /// [`Self::block_off_board`]: an early prototype of this method
    /// tried exactly that (one edge-crossing pass per row, `O(rows *
    /// edges)`, matching that sibling method) and it *was* dramatically
    /// faster on this board's own real 5V/GND pour -- but it also
    /// silently mis-rasterized part of that same real (non-trivially-
    /// shaped, pad-keepout-notched) zone, which only showed up as a
    /// correctness regression in the end-to-end `route_darkroom_panel_4x5`
    /// benchmark (several nets that the un-optimized version routed
    /// correctly instead produced jagged 400+-waypoint paths and
    /// ultimately failed) -- see the development log's "Teil 28" entry
    /// for the full story. A copper zone's boundary is not a small,
    /// simple board outline; per-cell exact queries against the
    /// already-tested [`PolygonEdgeIndex`] are the correct trade-off
    /// here even though they're the grid engine's current dominant cost
    /// for a real, whole-board-spanning pour -- see that entry for the
    /// benchmarked follow-up options (e.g. limiting the grid fallback's
    /// own search region instead of always using the full board
    /// outline).
    pub fn block_polygon(&mut self, polygon: &Polygon, layer: LayerId, clearance: Unit) {
        if polygon.points.len() < 3 {
            return;
        }
        let Some((gx0, gy0, gx1, gy1)) =
            self.clip_window(Aabb::from_polygon(polygon).inflate(clearance.max(0)))
        else {
            return;
        };
        let index = PolygonEdgeIndex::build(polygon);
        for gy in gy0..=gy1 {
            for gx in gx0..=gx1 {
                let c = self.cell_center(gx, gy);
                // A zero-radius "probe circle" reuses the existing,
                // already-tested `circle_polygon_collides_indexed`
                // (contains-or-within-clearance) rather than
                // duplicating its logic here.
                if circle_polygon_collides_indexed(&Circle::new(c, 0), &index, clearance) {
                    self.set_blocked(gx, gy, layer);
                }
            }
        }
    }

    /// Blocks (on both layers) every cell whose center does not lie on
    /// the board at all -- the grid-search equivalent of
    /// `astar.rs::edge_stays_on_board`/`alladin_geom::contains_point_evenodd`.
    /// `outline.is_empty()` means "no outline data supplied", matching
    /// every other caller's "not checked" convention in this crate: no
    /// cells get blocked by this call at all.
    ///
    /// Deliberately a real per-row scanline fill, **not**
    /// [`Self::block_polygon`]'s per-cell `PolygonEdgeIndex` query --
    /// found by benchmarking to matter a lot here (see
    /// the development log's "Teil 28" entry): a board outline has only
    /// a handful of vertices (unlike a filled zone's tens of
    /// thousands), so paying one `O(log n)` R-tree query *per grid
    /// cell* -- potentially millions of them, for the largest,
    /// whole-board fallback region -- is far more expensive than
    /// finding each row's edge crossings directly and filling the
    /// (cheap, branch-free) cell ranges between them. Multiple outline
    /// polygons combine via the same even-odd rule as
    /// [`alladin_geom::contains_point_evenodd`] (a hole/cutout's edges
    /// just contribute more crossings on the rows they span).
    pub fn block_off_board(&mut self, outline: &[Polygon]) {
        if outline.is_empty() {
            return;
        }
        let edges: Vec<(Point, Point)> = outline.iter().flat_map(|p| p.edges()).collect();

        for gy in 0..self.height {
            let y = self.cell_center(0, gy).y;
            let mut crossings: Vec<Unit> = edges
                .iter()
                .filter(|&&(a, b)| (a.y > y) != (b.y > y))
                .map(|&(a, b)| {
                    let x_at_y = a.x as f64 + (b.x - a.x) as f64 * (y - a.y) as f64 / (b.y - a.y) as f64;
                    x_at_y.round() as Unit
                })
                .collect();
            crossings.sort_unstable();

            // Between each pair of crossings is "on the board" (even-odd
            // rule); everything else on this row -- including a
            // leftover unpaired crossing, which can only be
            // floating-point noise from a scanline landing exactly on a
            // vertex -- is off-board and gets blocked. Mirrors
            // `Polygon::contains_point`'s own doc comment on that same
            // boundary-point edge case.
            let mut gx = 0i32;
            for pair in crossings.chunks_exact(2) {
                let (inside_start, _) = self.to_grid(Point::new(pair[0], y));
                let inside_start = inside_start.clamp(0, self.width);
                for x in gx.max(0)..inside_start {
                    self.set_blocked(x, gy, LayerId::FCu);
                    self.set_blocked(x, gy, LayerId::BCu);
                }
                let (inside_end, _) = self.to_grid(Point::new(pair[1], y));
                gx = inside_end.clamp(0, self.width);
            }
            for x in gx.max(0)..self.width {
                self.set_blocked(x, gy, LayerId::FCu);
                self.set_blocked(x, gy, LayerId::BCu);
            }
        }
    }

    /// Rasterizes one obstacle `item` -- mirrors `astar.rs::candidate_points`'s
    /// per-item loop body (same clearance probe, same net/layer
    /// semantics), but blocks grid cells instead of sampling boundary
    /// points. Same-net items are skipped entirely (never rasterized),
    /// matching `alladin_core::Node`'s own same-net collision skip --
    /// unlike `candidate_points`, which can afford to sample every item
    /// indiscriminately because a later exact `path_is_clear` call
    /// re-validates every candidate edge, this map *is* the collision
    /// oracle the grid search trusts directly, so it must get net/layer
    /// filtering right itself.
    #[allow(clippy::too_many_arguments)]
    pub fn rasterize_item(
        &mut self,
        item: &Item,
        from: Point,
        to: Point,
        width: Unit,
        net: NetId,
        layer: LayerId,
        class: NetClass,
        resolver: &dyn RuleResolver,
    ) {
        if item.net() == Some(net) {
            return;
        }
        let probe = Item::Track { shape: Segment::new(from, to, width), net: Some(net), layer, class };
        let clearance = resolver.clearance(&probe, item) + width / 2;
        match item {
            Item::Pad { shape, layer: item_layer, .. } => {
                if *item_layer == layer {
                    // Deliberately a bounding-circle approximation even
                    // for a non-round `alladin_core::PadShape::Polygon`
                    // pad -- unlike `astar.rs`'s `candidate_points`
                    // (which now traces a `PadShape::Polygon`'s *true*
                    // outline via `polygon_boundary_candidates`, the
                    // actual fix for the reported pad-collision bug),
                    // this whole grid-search fallback is still the
                    // opt-in, "benchmarked but not yet full-board-
                    // validated" `ALLADIN_GRID_FALLBACK=1` prototype
                    // (see `../../README.md`'s "Status" section) that
                    // standard routing never exercises. A real
                    // `block_polygon` rasterization (already used for
                    // `Item::Zone` below) would work here too, but
                    // isn't worth doing until this fallback itself
                    // graduates out of prototype status -- see
                    // the development log's "Echte Pad-Geometrie" slice
                    // entry for the full reasoning.
                    self.block_disc(shape.center(), shape.bounding_radius() + clearance, &[layer]);
                }
            }
            Item::Track { shape, layer: item_layer, .. } => {
                if *item_layer == layer {
                    self.block_capsule(shape.a, shape.b, shape.width / 2 + clearance, &[layer]);
                }
            }
            // A via sits on both copper layers (see `Item::layers`), so
            // it blocks whichever layer this search is on regardless of
            // which layer that happens to be -- no `item_layer` check
            // needed, unlike Pad/Track/Zone above.
            Item::Via { shape, .. } => {
                self.block_disc(shape.center, shape.radius + clearance, &[layer]);
            }
            Item::Zone { outline, layer: item_layer, .. } => {
                if *item_layer == layer {
                    self.block_polygon(outline, layer, clearance);
                }
            }
            // A mounting hole drills through both copper layers, same
            // as a via (see `Item::Hole`'s own doc comment) -- no
            // `item_layer` check needed, exactly like the `Via` arm
            // above.
            Item::Hole { position, drill } => {
                self.block_disc(*position, drill / 2 + clearance, &[layer]);
            }
        }
    }

    /// Batch [`Self::rasterize_item`] over every item in `items`.
    #[allow(clippy::too_many_arguments)]
    pub fn rasterize_items<'a>(
        &mut self,
        items: impl Iterator<Item = &'a Item>,
        from: Point,
        to: Point,
        width: Unit,
        net: NetId,
        layer: LayerId,
        class: NetClass,
        resolver: &dyn RuleResolver,
    ) {
        for item in items {
            self.rasterize_item(item, from, to, width, net, layer, class, resolver);
        }
    }

    /// Cheap early-reject probe for `astar.rs::build_adjacency_knn`'s
    /// candidate-edge validation loop: walks every grid cell the
    /// straight line `a`->`b` passes through (Amanatides-Woo voxel
    /// traversal) and returns `true` the moment it finds one already
    /// marked blocked on `layer`, `false` if it reaches `b` without
    /// finding one.
    ///
    /// **The one safety rule every caller must follow: a `true` result
    /// may be trusted to skip the real, exact
    /// [`alladin_core::Node::path_is_clear`] check and treat the edge as
    /// invalid outright, but a `false` result must never be trusted to
    /// skip that real check and treat the edge as valid.** This method
    /// only ever needs to answer "definitely blocked", never "definitely
    /// clear" -- so every source of approximation below only costs
    /// *performance* (a few more real checks than strictly necessary),
    /// never *correctness* (an accepted edge that shouldn't have been):
    /// - This map's own rasterization already deliberately over-approximates
    ///   some obstacles (e.g. a non-round [`alladin_core::PadShape::Polygon`]
    ///   pad is blocked as its bounding circle, see [`Self::rasterize_item`]) --
    ///   the safe direction for a filter that only ever rejects early.
    /// - [`Self::is_blocked`] treats any cell outside this map's own
    ///   window as blocked; if `a`/`b` sit right at this map's edge, a
    ///   floating-point rounding step could nudge the walk one cell past
    ///   the window and return `true` for an edge the real check would
    ///   have accepted -- vanishingly rare (callers build this map to
    ///   cover the same corridor candidate points already come from) and
    ///   still safe, just a missed fast-path opportunity, not a wrong
    ///   answer: `build_adjacency_knn`'s corridor escalation and
    ///   [`crate::astar::build_adjacency_full`] fallback exist precisely
    ///   to catch whatever the fast KNN pass misses.
    /// - The traversal below visits every cell the segment truly
    ///   crosses (not a coarser subsample), so it has no *false negative*
    ///   risk of its own beyond the two points above -- but even if a
    ///   future change made it coarser, that would only mean fewer
    ///   caught cells (fewer early rejects), never a wrongly accepted
    ///   one, by the same one-directional argument.
    pub fn segment_definitely_blocked(&self, a: Point, b: Point, layer: LayerId) -> bool {
        let step = self.step as f64;
        let ax = (a.x - self.origin.x) as f64 / step;
        let ay = (a.y - self.origin.y) as f64 / step;
        let bx = (b.x - self.origin.x) as f64 / step;
        let by = (b.y - self.origin.y) as f64 / step;

        let mut x = ax.floor() as i32;
        let mut y = ay.floor() as i32;
        let end_x = bx.floor() as i32;
        let end_y = by.floor() as i32;

        let dx = bx - ax;
        let dy = by - ay;

        let (step_x, mut t_max_x, t_delta_x) = axis_step(x, ax, dx);
        let (step_y, mut t_max_y, t_delta_y) = axis_step(y, ay, dy);

        // Bounded by the number of cells the map itself can possibly
        // have, so a degenerate/NaN input can't spin forever -- always
        // falls through to "not proven blocked" (safe, see this
        // method's own doc comment) rather than looping.
        let max_steps = self.width as i64 + self.height as i64 + 2;
        for _ in 0..=max_steps {
            if self.is_blocked(x, y, layer) {
                return true;
            }
            if x == end_x && y == end_y {
                return false;
            }
            if t_max_x < t_max_y {
                x += step_x;
                t_max_x += t_delta_x;
            } else {
                y += step_y;
                t_max_y += t_delta_y;
            }
        }
        false
    }

    /// Intersects `bounds` with this map's own cell window, returning
    /// `(gx0, gy0, gx1, gy1)` inclusive grid bounds to iterate, or
    /// `None` if the two don't overlap at all. Shared by every
    /// `block_*` method above so each only ever visits cells it could
    /// possibly need to touch.
    fn clip_window(&self, bounds: Aabb) -> Option<(i32, i32, i32, i32)> {
        let (gx0, gy0) = self.to_grid(bounds.min);
        let (gx1, gy1) = self.to_grid(bounds.max);
        let gx0 = gx0.max(0);
        let gy0 = gy0.max(0);
        let gx1 = gx1.min(self.width - 1);
        let gy1 = gy1.min(self.height - 1);
        if gx0 > gx1 || gy0 > gy1 {
            None
        } else {
            Some((gx0, gy0, gx1, gy1))
        }
    }
}

/// One axis of the Amanatides-Woo voxel traversal `segment_definitely_blocked`
/// uses: given the starting cell `cell`, the fractional grid-space
/// starting coordinate `start` (i.e. `(a.x - origin.x) / step`), and the
/// grid-space delta `delta` to the segment's other endpoint on this axis,
/// returns `(step_direction, t_max, t_delta)` -- `step_direction` is
/// which way `cell` moves (`-1`/`0`/`1`), `t_max` is how far along the
/// segment (in the same `0..=1`-ish units as `delta`) the next cell
/// boundary on this axis is, and `t_delta` is how far one full cell
/// spans in those units. `delta == 0` (segment doesn't move on this
/// axis at all) returns `t_max`/`t_delta` of `f64::INFINITY` so the
/// traversal loop always advances on the *other* axis instead.
fn axis_step(cell: i32, start: f64, delta: f64) -> (i32, f64, f64) {
    if delta > 0.0 {
        let next_boundary = (cell + 1) as f64;
        (1, (next_boundary - start) / delta, 1.0 / delta)
    } else if delta < 0.0 {
        let next_boundary = cell as f64;
        (-1, (next_boundary - start) / delta, 1.0 / -delta)
    } else {
        (0, f64::INFINITY, f64::INFINITY)
    }
}

fn clamp_to_i32(v: f64) -> i32 {
    if v.is_nan() {
        0
    } else {
        v.clamp(i32::MIN as f64, i32::MAX as f64) as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::PadShape;
    use alladin_core::{FixedClearance, NetClass as Class};
    use alladin_geom::MM;

    fn region(half: Unit) -> Aabb {
        Aabb { min: Point::new(-half, -half), max: Point::new(half, half) }
    }

    #[test]
    fn a_fresh_map_has_nothing_blocked() {
        let map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        assert!(!map.is_blocked(0, 0, LayerId::FCu));
        assert!(!map.is_blocked(map.width() / 2, map.height() / 2, LayerId::BCu));
    }

    #[test]
    fn out_of_bounds_cells_read_as_blocked() {
        let map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        assert!(map.is_blocked(-1, 0, LayerId::FCu));
        assert!(map.is_blocked(map.width(), 0, LayerId::FCu));
    }

    #[test]
    fn block_disc_marks_only_cells_within_radius_on_the_given_layer() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        let center = Point::new(0, 0);
        let radius = 1 * MM;
        map.block_disc(center, radius, &[LayerId::FCu]);

        let (cgx, cgy) = map.to_grid(center);
        assert!(map.is_blocked(cgx, cgy, LayerId::FCu), "center cell must be blocked");
        assert!(!map.is_blocked(cgx, cgy, LayerId::BCu), "must not leak onto the other layer");

        let far = map.to_grid(Point::new(4 * MM, 4 * MM));
        assert!(!map.is_blocked(far.0, far.1, LayerId::FCu), "far cell must stay clear");
    }

    #[test]
    fn block_capsule_covers_a_swept_segment() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        map.block_capsule(Point::new(-2 * MM, 0), Point::new(2 * MM, 0), 200_000, &[LayerId::FCu]);

        let (mgx, mgy) = map.to_grid(Point::new(0, 0));
        assert!(map.is_blocked(mgx, mgy, LayerId::FCu), "midpoint of the swept segment must be blocked");

        let (fgx, fgy) = map.to_grid(Point::new(0, 3 * MM));
        assert!(!map.is_blocked(fgx, fgy, LayerId::FCu), "far off the segment's line must stay clear");
    }

    #[test]
    fn block_polygon_fills_the_interior_and_its_clearance_ring() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        let square = Polygon::new(vec![
            Point::new(-1 * MM, -1 * MM),
            Point::new(1 * MM, -1 * MM),
            Point::new(1 * MM, 1 * MM),
            Point::new(-1 * MM, 1 * MM),
        ]);
        map.block_polygon(&square, LayerId::FCu, 200_000);

        let (cgx, cgy) = map.to_grid(Point::new(0, 0));
        assert!(map.is_blocked(cgx, cgy, LayerId::FCu), "interior must be blocked");

        // Just outside the square, within the 0.2mm clearance ring.
        let (rgx, rgy) = map.to_grid(Point::new(1 * MM + 100_000, 0));
        assert!(map.is_blocked(rgx, rgy, LayerId::FCu), "clearance ring must be blocked");

        let (fgx, fgy) = map.to_grid(Point::new(4 * MM, 4 * MM));
        assert!(!map.is_blocked(fgx, fgy, LayerId::FCu), "far corner must stay clear");
    }

    #[test]
    fn segment_definitely_blocked_is_false_across_an_empty_map() {
        let map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        assert!(!map.segment_definitely_blocked(Point::new(-4 * MM, -4 * MM), Point::new(4 * MM, 4 * MM), LayerId::FCu));
    }

    #[test]
    fn segment_definitely_blocked_detects_a_horizontal_line_through_an_obstacle() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        map.block_disc(Point::new(0, 0), 500_000, &[LayerId::FCu]);
        assert!(map.segment_definitely_blocked(Point::new(-4 * MM, 0), Point::new(4 * MM, 0), LayerId::FCu));
        // Same obstacle, but the query is on the layer it was never
        // rasterized onto.
        assert!(!map.segment_definitely_blocked(Point::new(-4 * MM, 0), Point::new(4 * MM, 0), LayerId::BCu));
    }

    #[test]
    fn segment_definitely_blocked_detects_a_vertical_line_through_an_obstacle() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        map.block_disc(Point::new(0, 0), 500_000, &[LayerId::FCu]);
        assert!(map.segment_definitely_blocked(Point::new(0, -4 * MM), Point::new(0, 4 * MM), LayerId::FCu));
    }

    #[test]
    fn segment_definitely_blocked_detects_a_diagonal_line_through_an_obstacle() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        map.block_disc(Point::new(0, 0), 500_000, &[LayerId::FCu]);
        assert!(map.segment_definitely_blocked(Point::new(-4 * MM, -4 * MM), Point::new(4 * MM, 4 * MM), LayerId::FCu));
    }

    #[test]
    fn segment_definitely_blocked_is_false_when_the_obstacle_is_off_to_the_side() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        // Obstacle sits well clear of the straight line between the two
        // endpoints below.
        map.block_disc(Point::new(0, 3 * MM), 500_000, &[LayerId::FCu]);
        assert!(!map.segment_definitely_blocked(Point::new(-4 * MM, 0), Point::new(4 * MM, 0), LayerId::FCu));
    }

    #[test]
    fn segment_definitely_blocked_handles_a_zero_length_segment() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        map.block_disc(Point::new(0, 0), 500_000, &[LayerId::FCu]);
        assert!(map.segment_definitely_blocked(Point::new(0, 0), Point::new(0, 0), LayerId::FCu));
        assert!(!map.segment_definitely_blocked(Point::new(3 * MM, 3 * MM), Point::new(3 * MM, 3 * MM), LayerId::FCu));
    }

    #[test]
    fn segment_definitely_blocked_walks_every_cell_along_a_shallow_line() {
        // A near-horizontal, but not exactly horizontal, line -- the
        // case most likely to skip cells with a naive/buggy stepper.
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        map.block_disc(Point::new(3 * MM, 100_000), 150_000, &[LayerId::FCu]);
        assert!(map.segment_definitely_blocked(Point::new(-4 * MM, 0), Point::new(4 * MM, 200_000), LayerId::FCu));
    }

    #[test]
    fn segment_definitely_blocked_never_reports_false_positives_against_real_rasterization() {
        // Cross-check against `is_blocked` directly: densely sample the
        // segment itself and confirm every truly-blocked sample point's
        // own cell was indeed visited (i.e. the walk's `true` verdicts
        // agree with brute-force sampling) -- guards against a stepper
        // bug that would falsely report "blocked" for a genuinely clear
        // line, which (unlike a missed cell) would violate this
        // method's core safety contract.
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        map.block_disc(Point::new(2 * MM, MM), 300_000, &[LayerId::FCu]);
        let a = Point::new(-4 * MM, -4 * MM);
        let b = Point::new(4 * MM, 3 * MM);

        let mut any_sample_blocked = false;
        for i in 0..=1000 {
            let t = i as f64 / 1000.0;
            let p = Point::new(
                (a.x as f64 + (b.x - a.x) as f64 * t).round() as Unit,
                (a.y as f64 + (b.y - a.y) as f64 * t).round() as Unit,
            );
            let (gx, gy) = map.to_grid(p);
            if map.is_blocked(gx, gy, LayerId::FCu) {
                any_sample_blocked = true;
                break;
            }
        }

        assert_eq!(map.segment_definitely_blocked(a, b, LayerId::FCu), any_sample_blocked);
    }

    #[test]
    fn rasterize_item_skips_same_net_items() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        let resolver = FixedClearance(127_000);
        let same_net_pad = Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)), net: Some(NetId(1)), layer: LayerId::FCu };
        map.rasterize_item(
            &same_net_pad, Point::new(-2 * MM, 0), Point::new(2 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, Class::C, &resolver,
        );
        let (cgx, cgy) = map.to_grid(Point::new(0, 0));
        assert!(!map.is_blocked(cgx, cgy, LayerId::FCu), "same-net item must never be rasterized as an obstacle");
    }

    #[test]
    fn rasterize_item_ignores_the_wrong_layer() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        let resolver = FixedClearance(127_000);
        let other_layer_pad = Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)), net: Some(NetId(2)), layer: LayerId::BCu };
        map.rasterize_item(
            &other_layer_pad, Point::new(-2 * MM, 0), Point::new(2 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, Class::C, &resolver,
        );
        let (cgx, cgy) = map.to_grid(Point::new(0, 0));
        assert!(!map.is_blocked(cgx, cgy, LayerId::FCu), "a B.Cu-only pad must not block an F.Cu search");
    }

    #[test]
    fn rasterize_item_blocks_a_via_on_either_layer() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        let resolver = FixedClearance(127_000);
        let via = Item::Via { shape: Circle::new(Point::new(0, 0), 300_000), drill: 150_000, net: Some(NetId(2)) };
        map.rasterize_item(
            &via, Point::new(-2 * MM, 0), Point::new(2 * MM, 0), 250_000,
            NetId(1), LayerId::BCu, Class::C, &resolver,
        );
        let (cgx, cgy) = map.to_grid(Point::new(0, 0));
        assert!(map.is_blocked(cgx, cgy, LayerId::BCu), "a via must block whichever layer is being searched");
    }

    #[test]
    fn rasterize_item_blocks_a_mounting_hole_on_either_layer() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        let resolver = FixedClearance(127_000);
        let hole = Item::Hole { position: Point::new(0, 0), drill: 600_000 };
        map.rasterize_item(
            &hole, Point::new(-2 * MM, 0), Point::new(2 * MM, 0), 250_000,
            NetId(1), LayerId::BCu, Class::C, &resolver,
        );
        let (cgx, cgy) = map.to_grid(Point::new(0, 0));
        assert!(map.is_blocked(cgx, cgy, LayerId::BCu), "a mounting hole must block whichever layer is being searched, same as a via");
    }

    #[test]
    fn block_off_board_blocks_everything_outside_the_outline() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        let board = Polygon::new(vec![
            Point::new(-3 * MM, -3 * MM),
            Point::new(3 * MM, -3 * MM),
            Point::new(3 * MM, 3 * MM),
            Point::new(-3 * MM, 3 * MM),
        ]);
        map.block_off_board(std::slice::from_ref(&board));

        let (cgx, cgy) = map.to_grid(Point::new(0, 0));
        assert!(!map.is_blocked(cgx, cgy, LayerId::FCu), "on-board cell must stay clear");

        let (fgx, fgy) = map.to_grid(Point::new(4 * MM, 4 * MM));
        assert!(map.is_blocked(fgx, fgy, LayerId::FCu), "off-board cell must be blocked");
    }

    #[test]
    fn block_off_board_is_a_no_op_when_no_outline_is_supplied() {
        let mut map = GridObstacleMap::new(region(5 * MM), DEFAULT_GRID_STEP);
        map.block_off_board(&[]);
        let (fgx, fgy) = map.to_grid(Point::new(4 * MM, 4 * MM));
        assert!(!map.is_blocked(fgx, fgy, LayerId::FCu), "no outline supplied means nothing gets blocked");
    }

    #[test]
    fn a_pathologically_fine_step_over_a_huge_region_is_coarsened_to_fit_the_budget() {
        // A whole-board-sized region at an absurdly fine step would
        // otherwise ask for a multi-gigabyte bitmap.
        let huge = Aabb { min: Point::new(0, 0), max: Point::new(1000 * MM, 1000 * MM) };
        let map = GridObstacleMap::new(huge, 1); // 1 nanometre step, deliberately absurd
        assert!((map.width() as i64) * (map.height() as i64) <= MAX_GRID_CELLS);
    }
}
