//! Alladin geometry primitives.
//!
//! Internal unit convention deliberately mirrors KiCad's own: **integer
//! nanometres** (`i64`), not floating-point millimetres. This is not
//! cosmetic — it's the reason KiCad (and Alladin) can promise
//! "correct-by-construction" clearance: comparisons on a fixed integer
//! grid have no accumulated floating-point drift near the DRC boundary.
//! 1 mm == 1_000_000 nm.

use serde::{Deserialize, Serialize};

pub mod fill;

pub type Unit = i64;

/// 1 millimetre in internal units, for readable test/CLI code.
pub const MM: Unit = 1_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Point {
    pub x: Unit,
    pub y: Unit,
}

impl Point {
    pub const fn new(x: Unit, y: Unit) -> Self {
        Self { x, y }
    }

    pub fn sub(self, other: Point) -> Point {
        Point::new(self.x - other.x, self.y - other.y)
    }

    pub fn add(self, other: Point) -> Point {
        Point::new(self.x + other.x, self.y + other.y)
    }

    pub fn dot(self, other: Point) -> i128 {
        self.x as i128 * other.x as i128 + self.y as i128 * other.y as i128
    }

    pub fn length_sq(self) -> i128 {
        self.dot(self)
    }

    pub fn length(self) -> f64 {
        (self.length_sq() as f64).sqrt()
    }

    pub fn distance(self, other: Point) -> f64 {
        self.sub(other).length()
    }

    /// Scale a point by a float factor (used for interpolation along a
    /// segment); rounds to the nearest internal unit.
    pub fn lerp(self, other: Point, t: f64) -> Point {
        Point::new(
            self.x + ((other.x - self.x) as f64 * t).round() as Unit,
            self.y + ((other.y - self.y) as f64 * t).round() as Unit,
        )
    }

    /// Rotates `self` by `degrees` (counter-clockwise, this crate's
    /// board-space convention) around the origin, rounding to the
    /// nearest internal unit -- the one canonical "rotate then round"
    /// implementation this workspace needs. Rotate around a pivot other
    /// than the origin by subtracting it first and adding it back after
    /// (see `alladin_pcb::footprint::pad_world_position`, this
    /// function's first real caller, for that composition).
    pub fn rotated(self, degrees: f64) -> Point {
        let rad = degrees.to_radians();
        let (sin, cos) = rad.sin_cos();
        Point::new(
            (self.x as f64 * cos - self.y as f64 * sin).round() as Unit,
            (self.x as f64 * sin + self.y as f64 * cos).round() as Unit,
        )
    }
}

/// Axis-aligned bounding box, in internal units. Used as the rstar spatial
/// index key (equivalent role to KiCad's `PNS::INDEX` R-tree).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Aabb {
    pub min: Point,
    pub max: Point,
}

impl Aabb {
    pub fn from_circle(c: &Circle) -> Self {
        Aabb {
            min: Point::new(c.center.x - c.radius, c.center.y - c.radius),
            max: Point::new(c.center.x + c.radius, c.center.y + c.radius),
        }
    }

    pub fn from_segment(s: &Segment) -> Self {
        let r = s.width / 2;
        Aabb {
            min: Point::new(s.a.x.min(s.b.x) - r, s.a.y.min(s.b.y) - r),
            max: Point::new(s.a.x.max(s.b.x) + r, s.a.y.max(s.b.y) + r),
        }
    }

    /// Bounding box of every vertex of `poly` -- used for a filled
    /// zone/copper-pour [`Polygon`] the same way [`Self::from_circle`]/
    /// [`Self::from_segment`] are used for pads/tracks. Unlike those two,
    /// there's no clearance-independent "radius" to add here: a zone's
    /// outline vertices already define its full extent exactly.
    ///
    /// # Panics
    /// If `poly.points` is empty -- a real zone outline always has at
    /// least 3 vertices; an empty one is a caller bug, not a shape this
    /// crate has any sensible bounding box for.
    pub fn from_polygon(poly: &Polygon) -> Self {
        let mut points = poly.points.iter();
        let first = *points.next().expect("Aabb::from_polygon: polygon has no points");
        let (min, max) = points.fold((first, first), |(min, max), &p| {
            (
                Point::new(min.x.min(p.x), min.y.min(p.y)),
                Point::new(max.x.max(p.x), max.y.max(p.y)),
            )
        });
        Aabb { min, max }
    }

    pub fn inflate(self, by: Unit) -> Self {
        Aabb {
            min: Point::new(self.min.x - by, self.min.y - by),
            max: Point::new(self.max.x + by, self.max.y + by),
        }
    }

    pub fn contains_point(&self, p: Point) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }
}

/// A round pad or via footprint, on the water/island model from the
/// original architecture note: this is a "hard island".
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Circle {
    pub center: Point,
    pub radius: Unit,
}

impl Circle {
    pub const fn new(center: Point, radius: Unit) -> Self {
        Self { center, radius }
    }
}

/// A routed track: a capsule (stadium shape) between two endpoints with a
/// given copper width. Equivalent to KiCad's `PNS::SEGMENT`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Segment {
    pub a: Point,
    pub b: Point,
    pub width: Unit,
}

impl Segment {
    pub const fn new(a: Point, b: Point, width: Unit) -> Self {
        Self { a, b, width }
    }

    pub fn length(&self) -> f64 {
        self.a.distance(self.b)
    }
}

/// A closed, simple (non-self-intersecting) polygon -- `points` is
/// implicitly closed, i.e. the last point connects back to the first.
/// Today's only consumer is the board outline (`Edge.Cuts`), imported
/// by `alladin-kicad-io`; see that crate's doc comments for the real
/// data this was validated against (a real board's 9-edge non-convex
/// outline, chained from individually-unordered `gr_line` forms).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Polygon {
    pub points: Vec<Point>,
}

impl Polygon {
    pub fn new(points: Vec<Point>) -> Self {
        Self { points }
    }

    /// Standard even-odd ("crossing number") ray-casting point-in-polygon
    /// test, cast along +x. Handles non-convex polygons correctly (the
    /// real board outline this was validated against has an inward
    /// notch, not just a rectangle). **Boundary points are an
    /// undefined-by-design edge case**: floating-point ray casting can
    /// go either way for a point exactly on an edge, and this is never
    /// relied on for exact-boundary decisions -- callers needing a
    /// "fully inside, clearance included" answer should inflate/deflate
    /// their own query geometry first, the same pattern
    /// `alladin_core::RuleResolver`'s clearance model already uses
    /// elsewhere.
    pub fn contains_point(&self, p: Point) -> bool {
        let n = self.points.len();
        if n < 3 {
            return false;
        }
        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let pi = self.points[i];
            let pj = self.points[j];
            let straddles = (pi.y > p.y) != (pj.y > p.y);
            if straddles {
                let x_at_p_y = pi.x as f64
                    + (pj.x - pi.x) as f64 * (p.y - pi.y) as f64 / (pj.y - pi.y) as f64;
                if (p.x as f64) < x_at_p_y {
                    inside = !inside;
                }
            }
            j = i;
        }
        inside
    }

    /// This polygon's own edges as `(a, b)` pairs, wrapping the last
    /// point back to the first.
    pub fn edges(&self) -> impl Iterator<Item = (Point, Point)> + '_ {
        let n = self.points.len();
        (0..n).map(move |i| (self.points[i], self.points[(i + 1) % n]))
    }

    /// Whether the straight line from `a` to `b` lies **entirely**
    /// within this polygon (handles non-convex shapes, unlike checking
    /// only the two endpoints): both endpoints must be inside, *and*
    /// the segment must not cross any of this polygon's own boundary
    /// edges -- a segment with both endpoints inside a non-convex
    /// polygon can still exit and re-enter through a notch, exactly the
    /// shape of the one real outline this was validated against.
    pub fn contains_segment(&self, a: Point, b: Point) -> bool {
        if !self.contains_point(a) || !self.contains_point(b) {
            return false;
        }
        !self.edges().any(|(p1, p2)| segments_intersect((a, b), (p1, p2)))
    }

    /// This polygon's own signed area (shoelace formula): positive if
    /// `points` wind counter-clockwise, negative if clockwise. Purely an
    /// internal helper for [`Self::inward_vertices`] -- this codebase's
    /// outline data (see `alladin_kicad_io::import_board_outline`'s doc
    /// comment) doesn't guarantee either winding direction, so anything
    /// that needs to know "which side is the interior" has to work it
    /// out itself rather than assume one.
    fn signed_area(&self) -> f64 {
        let n = self.points.len();
        (0..n)
            .map(|i| {
                let a = self.points[i];
                let b = self.points[(i + 1) % n];
                (a.x as f64) * (b.y as f64) - (b.x as f64) * (a.y as f64)
            })
            .sum::<f64>()
            * 0.5
    }

    /// Every vertex of this polygon, nudged `epsilon` internal units
    /// toward the interior along its own angle bisector -- candidate
    /// waypoints for a route that needs to hug a concave stretch of this
    /// polygon's own boundary, where no nearby obstacle happens to seed
    /// one (see `alladin-router::candidate_points`, the only consumer,
    /// and the development log's "Teil 15" entry for why this exists).
    ///
    /// Why nudge at all rather than returning the exact vertex:
    /// [`Self::contains_point`] documents boundary points as an
    /// undefined-by-design edge case (floating-point ray casting can go
    /// either way exactly on an edge) -- a waypoint that only
    /// *sometimes* reads as "on the board" would be a real, if rare,
    /// source of route flakiness. `epsilon` should be small relative to
    /// real clearances (nanometre-scale slack, not a design-rule
    /// margin) so this never meaningfully changes where a route can
    /// physically go.
    ///
    /// **Known limitation for extremely sharp corners, not hidden:**
    /// this averages the two adjacent edges' inward normals rather than
    /// computing a true polygon-offset/miter intersection -- for a very
    /// sharp reflex angle this can under- or (rarely) over-shoot the
    /// nudge distance. Not an issue for the real, gently-angled board
    /// outlines this was validated against (see this crate's own tests
    /// and `alladin-router`'s); a proper miter-based offset would be the
    /// fix if a pathological board outline ever needs it.
    pub fn inward_vertices(&self, epsilon: Unit) -> Vec<Point> {
        let n = self.points.len();
        if n < 3 {
            return Vec::new();
        }
        let ccw = self.signed_area() > 0.0;

        let normalize = |(x, y): (f64, f64)| -> (f64, f64) {
            let len = (x * x + y * y).sqrt();
            if len < 1e-9 {
                (0.0, 0.0)
            } else {
                (x / len, y / len)
            }
        };
        // The inward-pointing perpendicular of a directed edge: for a
        // CCW polygon the interior is on the edge's left (rotate the
        // direction +90°); for CW, the right (rotate -90°).
        let inward_normal = |(dx, dy): (f64, f64)| -> (f64, f64) {
            if ccw {
                (-dy, dx)
            } else {
                (dy, -dx)
            }
        };

        (0..n)
            .map(|i| {
                let prev = self.points[(i + n - 1) % n];
                let cur = self.points[i];
                let next = self.points[(i + 1) % n];

                let in_dir = normalize(((cur.x - prev.x) as f64, (cur.y - prev.y) as f64));
                let out_dir = normalize(((next.x - cur.x) as f64, (next.y - cur.y) as f64));
                let n1 = inward_normal(in_dir);
                let n2 = inward_normal(out_dir);
                let (bx, by) = normalize((n1.0 + n2.0, n1.1 + n2.1));

                Point::new(
                    cur.x + (bx * epsilon as f64).round() as Unit,
                    cur.y + (by * epsilon as f64).round() as Unit,
                )
            })
            .collect()
    }

    /// Approximate clearance-inflated boundary of `self`, for seeding a
    /// router's visibility-graph candidate points around a filled zone
    /// treated as an obstacle (`alladin_router`'s `candidate_points`) --
    /// the polygon-shaped counterpart to that module's circle/capsule
    /// boundary sampling for round pad/track obstacles.
    ///
    /// Two kinds of points, per vertex:
    ///
    /// 1. **Per-edge offsets:** each edge's own two endpoints, offset
    ///    outward by `clearance` along that one edge's outward normal.
    ///    Every such point is *exactly* `clearance` from the edge that
    ///    produced it -- unlike [`Self::inward_vertices`]'s "average the
    ///    two adjacent edges' normals into one nudged vertex" approach,
    ///    which is fine for "is this vertex still inside/outside"
    ///    sanity nudges but provably *wrong* as a real offset distance
    ///    at anything but a 180° (straight) join (e.g. at a rectangle's
    ///    90° corner it only reaches `clearance / sqrt(2)` of real
    ///    perpendicular clearance).
    /// 2. **Per-vertex miters:** the classic straight-line offset "miter
    ///    join" of the two edges meeting at a vertex -- where the two
    ///    edges' own offset *lines* (not just their sampled endpoints)
    ///    would intersect. Needed because the straight chord between two
    ///    per-edge points flanking a convex corner cuts *inside* the
    ///    true (rounded) clearance boundary there -- the same "chord
    ///    cuts inside the true curve" issue `CIRCLE_SAFETY_FACTOR`/
    ///    `ARC_SAFETY_FACTOR` exist for elsewhere in `alladin-router`,
    ///    just for a polygon corner instead of a circular arc. Skipped
    ///    when the join is too sharp/reflex (`1 + n_in·n_out` below a
    ///    small threshold): the standard failure mode of miter joins in
    ///    general (a near-180° reflex corner sends the miter point to
    ///    infinity) -- see [`Self::inward_vertices`]'s doc comment for
    ///    the same instability, there avoided by never computing a true
    ///    miter at all. The per-edge offsets above still cover that
    ///    corner, just without the extra, unnecessary-for-a-reflex-
    ///    corner miter point.
    ///
    /// Trade-off, and why it's acceptable here: this is still not a true
    /// polygon buffer/Minkowski-sum offset (no rounded arcs, no
    /// self-intersection cleanup for a very non-convex outline) -- but
    /// every visibility-graph edge these points end up seeding is
    /// independently re-verified against the real polygon via
    /// [`crate::segment_polygon_collides`]-based checks before ever
    /// being trusted, so an occasional missing or slightly-short
    /// candidate near an unusual corner only costs a missing graph edge
    /// there, never an accepted-but-actually-colliding one.
    pub fn outward_edge_boundary(&self, clearance: Unit) -> Vec<Point> {
        let n = self.points.len();
        if n < 2 {
            return Vec::new();
        }
        let ccw = self.signed_area() > 0.0;

        let normalize = |(x, y): (f64, f64)| -> (f64, f64) {
            let len = (x * x + y * y).sqrt();
            if len < 1e-9 {
                (0.0, 0.0)
            } else {
                (x / len, y / len)
            }
        };
        // Outward-pointing perpendicular of a directed edge: for a CCW
        // polygon the interior is on the edge's left, so outward is to
        // the right (rotate direction -90°); for CW, the opposite.
        // Mirror image of `inward_vertices`'s `inward_normal` closure.
        let outward_normal = |(dx, dy): (f64, f64)| -> (f64, f64) {
            if ccw {
                (dy, -dx)
            } else {
                (-dy, dx)
            }
        };
        let edge_dir = |i: usize| -> (f64, f64) {
            let a = self.points[i];
            let b = self.points[(i + 1) % n];
            normalize(((b.x - a.x) as f64, (b.y - a.y) as f64))
        };
        // `offset` is the raw displacement vector to add, already scaled
        // to its final magnitude -- callers pass either a unit normal
        // times `clearance` (per-edge points) or a pre-scaled miter
        // vector (per-vertex points).
        let displace = |p: Point, (ox, oy): (f64, f64)| {
            Point::new(p.x + ox.round() as Unit, p.y + oy.round() as Unit)
        };
        let scale = |(x, y): (f64, f64), s: f64| (x * s, y * s);

        let mut result = Vec::with_capacity(n * 3);

        for i in 0..n {
            let dir = edge_dir(i);
            if dir == (0.0, 0.0) {
                continue; // degenerate (near-zero-length) edge: no well-defined normal
            }
            let normal = scale(outward_normal(dir), clearance as f64);
            result.push(displace(self.points[i], normal));
            result.push(displace(self.points[(i + 1) % n], normal));
        }

        for i in 0..n {
            let n_in = outward_normal(edge_dir((i + n - 1) % n));
            let n_out = outward_normal(edge_dir(i));
            if n_in == (0.0, 0.0) || n_out == (0.0, 0.0) {
                continue;
            }
            let denom = 1.0 + (n_in.0 * n_out.0 + n_in.1 * n_out.1);
            if denom < 0.15 {
                continue; // too sharp/reflex a join: miter would blow up, skip it
            }
            let k = clearance as f64 / denom;
            let miter = scale((n_in.0 + n_out.0, n_in.1 + n_out.1), k);
            result.push(displace(self.points[i], miter));
        }

        result
    }

    /// A `width` x `height` rectangle centred on the origin, with each
    /// corner rounded to `corner_radius` (clamped to at most half the
    /// shorter side, so an oversized request degrades to the largest
    /// radius that still fits rather than producing a self-intersecting
    /// shape). `corner_radius <= 0` yields a plain sharp-cornered
    /// rectangle (4 points, no arc sampling at all). The board-outline
    /// generator for `alladin-pcb`'s "New board" flow -- kept here rather
    /// than in that crate since it's pure geometry with no GUI/board-doc
    /// knowledge, exactly like this type's other constructors.
    ///
    /// Winds counter-clockwise in this crate's own (mathematical, not
    /// screen) `y`-down convention, i.e. `signed_area() < 0.0` -- matches
    /// every other outline this codebase builds internally (KiCad's own
    /// `Edge.Cuts` chaining doesn't guarantee a winding direction at all,
    /// which is exactly why `inward_vertices` computes it per-polygon
    /// instead of assuming one).
    pub fn rounded_rect(width: Unit, height: Unit, corner_radius: Unit, segments_per_corner: usize) -> Self {
        let hw = width / 2;
        let hh = height / 2;
        let r = corner_radius.max(0).min(hw.min(hh));

        if r == 0 {
            return Self::new(vec![
                Point::new(-hw, -hh),
                Point::new(hw, -hh),
                Point::new(hw, hh),
                Point::new(-hw, hh),
            ]);
        }

        // One arc per corner, walked in top-left -> top-right -> bottom-right
        // -> bottom-left order; each arc's centre is inset by `r` from both
        // edges it joins, sweeping the quarter-turn between them.
        let corners: [(Unit, Unit, f64); 4] = [
            (-hw + r, -hh + r, 180.0_f64.to_radians()),
            (hw - r, -hh + r, 270.0_f64.to_radians()),
            (hw - r, hh - r, 0.0_f64.to_radians()),
            (-hw + r, hh - r, 90.0_f64.to_radians()),
        ];
        let segments = segments_per_corner.max(1);

        let mut points = Vec::with_capacity(4 * (segments + 1));
        for &(cx, cy, start_angle) in &corners {
            for step in 0..=segments {
                let angle = start_angle + (step as f64 / segments as f64) * 90.0_f64.to_radians();
                points.push(Point::new(
                    cx + (r as f64 * angle.cos()).round() as Unit,
                    cy + (r as f64 * angle.sin()).round() as Unit,
                ));
            }
        }
        Self::new(points)
    }
}

/// Even-odd ("crossing number", same name as SVG/PostScript's `evenodd`
/// fill rule) containment test across a whole *set* of polygons at
/// once, rather than a single [`Polygon::contains_point`] call: counts
/// how many of `polygons` individually contain `p`, and returns whether
/// that count is odd.
///
/// This is the standard trick for representing "an outer board boundary
/// with a hole/cutout in it" -- or, just as validly with the exact same
/// rule, several genuinely disjoint separate allowed regions -- without
/// ever having to work out *which* polygon is whose hole, or track a
/// consistent winding direction (this codebase's own outline-chaining
/// algorithm doesn't guarantee one -- see
/// `alladin_kicad_io::import_board_outline`'s doc comment): a point
/// inside only the outer boundary is counted once (odd -> on the
/// board); a point *also* inside a hole polygon nested within that
/// boundary is counted twice (even -> correctly excluded); a point
/// inside a second, wholly separate region is counted once there too
/// (odd -> on the board, same as before this rule existed). Found to be
/// necessary, not just theoretically nicer, via a real board
/// (`RaspberryPi-HAT.kicad_pcb`) that has exactly such an internal
/// cutout once `gr_arc` support let its outline chain into closed
/// polygons at all -- see the development log's "Teil 17" entry.
///
/// An empty `polygons` slice returns `false` for every point (0 is
/// even) -- this function's own honest answer to "is this point inside
/// nothing at all", not a stand-in for "no outline supplied, so don't
/// restrict anything" (that's a caller-level default, e.g.
/// `alladin_router::astar::edge_stays_on_board`'s `outline.is_empty() ||
/// ...` pattern -- deliberately kept as the caller's choice, not baked
/// in here, since it's a routing-domain default, not a geometric one).
pub fn contains_point_evenodd(polygons: &[Polygon], p: Point) -> bool {
    polygons.iter().filter(|poly| poly.contains_point(p)).count() % 2 == 1
}

/// [`contains_point_evenodd`]'s equivalent of
/// [`Polygon::contains_segment`]: the straight line from `a` to `b` is
/// "on the board" only if *both* endpoints have odd containment count,
/// *and* the segment never crosses any polygon's own boundary edge at
/// all -- a crossing would flip the parity partway along the segment,
/// which two endpoint checks alone can't detect (the exact reason
/// `Polygon::contains_segment` itself does more than check just its two
/// endpoints).
pub fn contains_segment_evenodd(polygons: &[Polygon], a: Point, b: Point) -> bool {
    contains_point_evenodd(polygons, a)
        && contains_point_evenodd(polygons, b)
        && !polygons
            .iter()
            .any(|poly| poly.edges().any(|(p1, p2)| segments_intersect((a, b), (p1, p2))))
}

/// Number of boundary samples [`circle_within_outline`] checks -- cheap
/// (a handful of `contains_point_evenodd` calls per placement/drag frame)
/// while still catching a pad whose edge, not just its center, has
/// crept past the board boundary; matches the sampling-based approach
/// `alladin-router`'s own `astar::circle_boundary` already uses for the
/// same "is this round shape actually clear" question, just re-purposed
/// here for on-board containment instead of obstacle clearance.
const CIRCLE_ON_BOARD_SAMPLES: usize = 16;

/// Whether a circular pad (`center`, `radius`) lies **entirely** within
/// `outline` (even-odd rule, see [`contains_point_evenodd`] -- so a hole/
/// cutout correctly excludes it too): the center plus
/// [`CIRCLE_ON_BOARD_SAMPLES`] points around its circumference must all
/// individually read as on-board. An empty `outline` always returns
/// `true` -- "no outline defined yet" is deliberately permissive here,
/// matching this codebase's existing `outline.is_empty() || ...` default
/// pattern (see [`contains_point_evenodd`]'s own doc comment) rather than
/// silently forbidding every placement before a board shape even exists.
pub fn circle_within_outline(center: Point, radius: Unit, outline: &[Polygon]) -> bool {
    if outline.is_empty() {
        return true;
    }
    if !contains_point_evenodd(outline, center) {
        return false;
    }
    (0..CIRCLE_ON_BOARD_SAMPLES).all(|k| {
        let angle = std::f64::consts::TAU * (k as f64) / (CIRCLE_ON_BOARD_SAMPLES as f64);
        let sample = Point::new(
            center.x + (radius as f64 * angle.cos()).round() as Unit,
            center.y + (radius as f64 * angle.sin()).round() as Unit,
        );
        contains_point_evenodd(outline, sample)
    })
}

/// Whether a track's entire copper -- a `width`-wide capsule along
/// centerline `a`->`b` -- stays fully within `outline`, with at least
/// `clearance` of margin beyond the raw board edge (e.g. JLCPCB's real
/// `copper_to_routed_edge` minimum -- this crate doesn't depend on
/// `alladin-core`, so the caller supplies the number; see
/// `alladin_core::JlcpcbDfm::COPPER_TO_ROUTED_EDGE`).
///
/// Exact, not sampled -- unlike [`circle_within_outline`]'s boundary
/// sampling (16 points is plenty for a pad, but a long track could need
/// arbitrarily many samples to catch a near-miss along a straight run).
/// The capsule is the Minkowski sum of the segment with a disk of radius
/// `width/2 + clearance`; that shape lies entirely inside a simple
/// polygon (holes included) iff (a) the raw centerline itself stays on
/// the board at all (reusing [`contains_segment_evenodd`], which also
/// catches the centerline crossing back out through a concave notch) and
/// (b) no boundary edge of any outline polygon comes closer than
/// `width/2 + clearance` to the centerline -- the same reasoning
/// [`circle_within_outline`] relies on for a single point, generalized
/// from point-to-edge to segment-to-edge distance via
/// [`dist_segment_to_segment`]. Checking every polygon's edges (not just
/// the outer boundary) is what makes a hole/cutout correctly repel the
/// track too, not just the outer board edge.
pub fn segment_within_outline_with_clearance(
    a: Point,
    b: Point,
    width: Unit,
    clearance: Unit,
    outline: &[Polygon],
) -> bool {
    if outline.is_empty() {
        return true;
    }
    if !contains_segment_evenodd(outline, a, b) {
        return false;
    }
    let min_dist = (width / 2 + clearance) as f64;
    outline
        .iter()
        .flat_map(|poly| poly.edges())
        .all(|(p1, p2)| dist_segment_to_segment((a, b), (p1, p2)) >= min_dist)
}

/// [`segment_within_outline_with_clearance`]'s polygon equivalent: is
/// `poly` (a pad's own true, filled outline -- rectangular/oval/rotated,
/// not a centerline+width capsule) fully on the board with at least
/// `clearance` of margin to the raw edge? Exact, not sampled, same as
/// the segment version -- unlike [`circle_within_outline`]'s boundary
/// sampling, this can't miss a near-miss along a long straight pad edge.
///
/// Reuses [`contains_segment_evenodd`] once per edge of `poly` rather
/// than duplicating its point-containment-plus-crossing logic: checking
/// every edge that way both (a) confirms every vertex is on-board (each
/// vertex is an endpoint of two edges) and (b) catches an edge cutting
/// back out through a concave notch in `outline`, exactly like that
/// function already does for a single segment. No `width` term here
/// (unlike the segment version) -- `poly` already *is* the shape's full
/// filled extent, not a centerline that still needs a half-width added.
pub fn polygon_within_outline_with_clearance(poly: &Polygon, clearance: Unit, outline: &[Polygon]) -> bool {
    if outline.is_empty() {
        return true;
    }
    if !poly.edges().all(|(a, b)| contains_segment_evenodd(outline, a, b)) {
        return false;
    }
    let min_dist = clearance as f64;
    outline
        .iter()
        .flat_map(|op| op.edges())
        .all(|(p1, p2)| poly.edges().all(|(a, b)| dist_segment_to_segment((a, b), (p1, p2)) >= min_dist))
}

/// Shortest distance from `p` to the segment's centerline (ignores width).
pub fn dist_point_to_line(p: Point, seg_a: Point, seg_b: Point) -> f64 {
    let ab = seg_b.sub(seg_a);
    let ab_len_sq = ab.length_sq();
    if ab_len_sq == 0 {
        return p.distance(seg_a);
    }
    let ap = p.sub(seg_a);
    let t = (ap.dot(ab) as f64 / ab_len_sq as f64).clamp(0.0, 1.0);
    let closest = seg_a.lerp(seg_b, t);
    p.distance(closest)
}

/// Shortest distance between the centerlines of two segments (2D segment-
/// segment distance; 0.0 if they cross).
pub fn dist_segment_to_segment(s1: (Point, Point), s2: (Point, Point)) -> f64 {
    if segments_intersect(s1, s2) {
        return 0.0;
    }
    let d1 = dist_point_to_line(s1.0, s2.0, s2.1);
    let d2 = dist_point_to_line(s1.1, s2.0, s2.1);
    let d3 = dist_point_to_line(s2.0, s1.0, s1.1);
    let d4 = dist_point_to_line(s2.1, s1.0, s1.1);
    d1.min(d2).min(d3).min(d4)
}

/// 2D cross product z-component of (a-o) x (b-o). Positive means `b` is
/// counter-clockwise from `a` as seen from `o`; zero means `o`, `a`, `b`
/// are collinear. Exposed publicly because it's the building block for
/// convex-boundary tangent-point queries (see `alladin-router`'s
/// capsule walkaround), not just the segment-intersection test below.
pub fn cross2d(o: Point, a: Point, b: Point) -> i128 {
    let ab = a.sub(o);
    let ac = b.sub(o);
    ab.x as i128 * ac.y as i128 - ab.y as i128 * ac.x as i128
}

fn cross(o: Point, a: Point, b: Point) -> i128 {
    a.sub(o).dot(Point::new(b.y - o.y, -(b.x - o.x)))
}

fn orient(o: Point, a: Point, b: Point) -> i8 {
    let v = cross(o, a, b);
    if v > 0 {
        1
    } else if v < 0 {
        -1
    } else {
        0
    }
}

fn on_segment(p: Point, a: Point, b: Point) -> bool {
    // Bounding-box check against a/b's own extent -- NOT p's, which would
    // make the comparison vacuous (p.x is trivially <= max(p.x, ...)).
    a.x.min(b.x) <= p.x
        && p.x <= a.x.max(b.x)
        && a.y.min(b.y) <= p.y
        && p.y <= a.y.max(b.y)
        && orient(a, b, p) == 0
}

/// Classic O(1) segment-segment intersection test (including collinear
/// touching cases).
pub fn segments_intersect(s1: (Point, Point), s2: (Point, Point)) -> bool {
    let (p1, q1) = s1;
    let (p2, q2) = s2;
    let o1 = orient(p1, q1, p2);
    let o2 = orient(p1, q1, q2);
    let o3 = orient(p2, q2, p1);
    let o4 = orient(p2, q2, q1);

    if o1 != o2 && o3 != o4 {
        return true;
    }

    (o1 == 0 && on_segment(p2, p1, q1))
        || (o2 == 0 && on_segment(q2, p1, q1))
        || (o3 == 0 && on_segment(p1, p2, q2))
        || (o4 == 0 && on_segment(q1, p2, q2))
}

/// True if two circles overlap once `clearance` (in internal units, e.g.
/// the JLCPCB 0.127 mm minimum spacing) is added on top of both radii.
/// This is the "islands grow" step from the water/island model.
pub fn circle_circle_collides(a: &Circle, b: &Circle, clearance: Unit) -> bool {
    let min_dist = (a.radius + b.radius + clearance) as f64;
    a.center.distance(b.center) < min_dist
}

/// True if a circle and a track capsule overlap with clearance added.
pub fn circle_segment_collides(c: &Circle, s: &Segment, clearance: Unit) -> bool {
    let min_dist = (c.radius + s.width / 2 + clearance) as f64;
    dist_point_to_line(c.center, s.a, s.b) < min_dist
}

/// True if two track capsules overlap with clearance added.
pub fn segment_segment_collides(s1: &Segment, s2: &Segment, clearance: Unit) -> bool {
    let min_dist = (s1.width / 2 + s2.width / 2 + clearance) as f64;
    dist_segment_to_segment((s1.a, s1.b), (s2.a, s2.b)) < min_dist
}

/// Shortest distance from `p` to `poly`'s own *filled area* -- `0.0` if
/// `p` is inside it (matching [`Polygon::contains_point`]'s definition
/// of "inside"), otherwise the shortest distance to any of its boundary
/// edges. This is the "distance to a filled region" counterpart to
/// [`dist_point_to_line`] (distance to an unfilled centerline) --
/// needed because a static copper zone/ground-pour (see
/// `alladin_core::Item::Zone`) is a *filled* obstacle: anything strictly
/// inside its outline is already colliding with it, not just anything
/// touching its boundary.
pub fn dist_point_to_polygon(p: Point, poly: &Polygon) -> f64 {
    if poly.contains_point(p) {
        return 0.0;
    }
    poly.edges()
        .map(|(a, b)| dist_point_to_line(p, a, b))
        .fold(f64::INFINITY, f64::min)
}

/// [`dist_point_to_polygon`]'s segment equivalent: `0.0` if either
/// endpoint of `seg_a`-`seg_b` is inside `poly`, or the segment crosses
/// any of `poly`'s own boundary edges (i.e. it passes through the
/// filled area even if both endpoints happen to be outside it);
/// otherwise the shortest distance from the segment to any boundary
/// edge.
pub fn dist_segment_to_polygon(seg_a: Point, seg_b: Point, poly: &Polygon) -> f64 {
    if poly.contains_point(seg_a) || poly.contains_point(seg_b) {
        return 0.0;
    }
    if poly
        .edges()
        .any(|(p1, p2)| segments_intersect((seg_a, seg_b), (p1, p2)))
    {
        return 0.0;
    }
    poly.edges()
        .map(|(p1, p2)| dist_segment_to_segment((seg_a, seg_b), (p1, p2)))
        .fold(f64::INFINITY, f64::min)
}

/// True if a circle (pad/via) overlaps a filled zone polygon with
/// clearance added. Only the circle's own radius is inflated by
/// `clearance` -- unlike [`circle_circle_collides`]/[`circle_segment_collides`],
/// a zone has no "width" of its own to also add: its outline already
/// defines its full filled extent.
pub fn circle_polygon_collides(c: &Circle, poly: &Polygon, clearance: Unit) -> bool {
    let min_dist = (c.radius + clearance) as f64;
    dist_point_to_polygon(c.center, poly) < min_dist
}

/// True if a track capsule overlaps a filled zone polygon with
/// clearance added. Same asymmetry as [`circle_polygon_collides`]: only
/// the segment's own half-width is inflated, not the zone.
pub fn segment_polygon_collides(s: &Segment, poly: &Polygon, clearance: Unit) -> bool {
    let min_dist = (s.width / 2 + clearance) as f64;
    dist_segment_to_polygon(s.a, s.b, poly) < min_dist
}

/// True if two filled polygons (e.g. a non-round pad's true outline
/// against another non-round pad's, or against a zone's) overlap with
/// `clearance` added -- neither shape has a "width" of its own to
/// inflate (both are already their own full filled extent), so unlike
/// [`segment_polygon_collides`]/[`circle_polygon_collides`] `clearance`
/// is the *entire* margin, symmetric between `a` and `b`.
///
/// Two-part check, same shape as [`dist_segment_to_polygon`] generalized
/// to two closed polygons instead of one open segment:
/// 1. **Containment fallback:** if any vertex of `a` lies inside `b` (or
///    vice versa), the shapes already overlap regardless of `clearance`
///    -- real footprints never nest one pad fully inside another, but a
///    malformed/overlapping placement must still be reported as
///    colliding, not silently missed just because no edge pair happens
///    to cross.
/// 2. Otherwise, the shapes collide iff some pair of edges (one from
///    each polygon) actually crosses, or comes closer than `clearance`
///    -- reusing the existing [`segments_intersect`]/
///    [`dist_segment_to_segment`] once per edge pair. The explicit
///    intersection test is *not* redundant with the distance one: at
///    `clearance == 0`, `dist < 0.0` can never be true, so without it
///    two polygons whose edges cross *without* either containing one
///    of the other's vertices (a narrow rectangle crossing a wide flat
///    one into a plus shape) were silently reported as clear --
///    [`PolygonEdgeIndex::any_edge_within_of_segment`] (the indexed
///    twin's building block) always carried this intersection test,
///    so this also restores the "exact same boolean answer" contract
///    the indexed variants promise.
pub fn polygon_polygon_collides(a: &Polygon, b: &Polygon, clearance: Unit) -> bool {
    if a.points.iter().any(|&p| b.contains_point(p)) || b.points.iter().any(|&p| a.contains_point(p)) {
        return true;
    }
    let min_dist = clearance as f64;
    a.edges()
        .any(|ea| b.edges().any(|eb| segments_intersect(ea, eb) || dist_segment_to_segment(ea, eb) < min_dist))
}

/// A spatial index over one [`Polygon`]'s own edges, letting
/// [`circle_polygon_collides_indexed`]/[`segment_polygon_collides_indexed`]
/// answer the exact same question as their un-indexed counterparts above
/// while only ever examining the *geometrically nearby* handful of edges,
/// not the whole polygon.
///
/// **Why this exists**: [`dist_point_to_polygon`]/[`dist_segment_to_polygon`]
/// are `O(vertex count)` -- fine for a hand-built test polygon, but a
/// *real* KiCad zone fill routinely has tens of thousands of vertices
/// (a real 109-LED panel's 5V pour: 42,415 -- see
/// `alladin_router::astar`'s `candidate_points` doc comment for the full
/// story of where that number came from). Alladin's A* visibility-graph
/// construction calls the equivalent of a collision check up to
/// `KNN_MAX_CONSIDERED` times *per candidate point*, so on a route whose
/// corridor overlaps such a zone, that `O(n)` scan was being paid
/// thousands of times over -- easily billions of edge-distance
/// evaluations for a single net, which is what silently hung real-board
/// routing (see the development log's "Status" section and
/// `alladin-viewer`'s own doc comment on exactly this discovery). Building
/// one `rstar` R-tree of a zone's edges once and reusing it for every one
/// of those checks turns each check into an `O(log n)`-ish spatially
/// pruned query instead.
///
/// **Correctness, not just speed**: every function that takes an index
/// produces *exactly* the same boolean answer as its un-indexed
/// counterpart above (see this module's
/// `indexed_collision_checks_agree_with_the_brute_force_ones_on_a_dense_comb_polygon`
/// test, which cross-checks both against a many-vertex synthetic
/// polygon and hundreds of random query points/segments) -- the R-tree
/// query is only ever used to *narrow* which edges get the exact same
/// exact-geometry test the brute-force version already runs on every
/// edge, never to approximate the answer itself.
#[derive(Clone)]
pub struct PolygonEdgeIndex {
    tree: rstar::RTree<PolyEdge>,
}

#[derive(Debug, Clone, Copy)]
struct PolyEdge {
    a: Point,
    b: Point,
}

impl rstar::RTreeObject for PolyEdge {
    type Envelope = rstar::AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        rstar::AABB::from_corners(
            [self.a.x.min(self.b.x) as f64, self.a.y.min(self.b.y) as f64],
            [self.a.x.max(self.b.x) as f64, self.a.y.max(self.b.y) as f64],
        )
    }
}

impl PolygonEdgeIndex {
    pub fn build(poly: &Polygon) -> Self {
        let edges: Vec<PolyEdge> = poly.edges().map(|(a, b)| PolyEdge { a, b }).collect();
        Self { tree: rstar::RTree::bulk_load(edges) }
    }

    /// Same even-odd ray-casting rule as [`Polygon::contains_point`]
    /// (see that method's doc comment for the boundary-point caveat),
    /// but only ever examines edges whose own bounding box's y-range
    /// straddles `p.y` -- provably every other edge cannot possibly
    /// satisfy the crossing test's own `(pi.y > p.y) != (pj.y > p.y)`
    /// condition, so narrowing to this subset first changes nothing
    /// about the result, only how many edges get examined to compute
    /// it. Since the final answer is an XOR/parity over every
    /// qualifying edge, the (R-tree-query-determined, not
    /// insertion-order) iteration order doesn't affect the result
    /// either.
    pub fn contains_point(&self, p: Point) -> bool {
        let band = rstar::AABB::from_corners([f64::NEG_INFINITY, p.y as f64], [f64::INFINITY, p.y as f64]);
        let mut inside = false;
        for edge in self.tree.locate_in_envelope_intersecting(&band) {
            let straddles = (edge.a.y > p.y) != (edge.b.y > p.y);
            if straddles {
                let x_at_p_y = edge.a.x as f64
                    + (edge.b.x - edge.a.x) as f64 * (p.y - edge.a.y) as f64 / (edge.b.y - edge.a.y) as f64;
                if (p.x as f64) < x_at_p_y {
                    inside = !inside;
                }
            }
        }
        inside
    }

    /// True if any edge lies within `min_dist` of `p` -- narrows to
    /// edges whose bounding box intersects the `min_dist`-inflated box
    /// around `p` before running the exact [`dist_point_to_line`] check.
    /// Sound, not approximate: any edge whose closest point to `p` is
    /// within `min_dist` must have *that closest point* -- which always
    /// lies on the edge itself, hence inside the edge's own bounding
    /// box -- within `min_dist` on both axes of `p` too, so it can never
    /// be missed by this box query (standard broad-phase/narrow-phase
    /// range-query reasoning, not a heuristic).
    fn any_edge_within(&self, p: Point, min_dist: f64) -> bool {
        let query = rstar::AABB::from_corners(
            [p.x as f64 - min_dist, p.y as f64 - min_dist],
            [p.x as f64 + min_dist, p.y as f64 + min_dist],
        );
        self.tree
            .locate_in_envelope_intersecting(&query)
            .any(|edge| dist_point_to_line(p, edge.a, edge.b) < min_dist)
    }

    /// [`Self::any_edge_within`]'s segment equivalent: also catches a
    /// crossing edge (distance `0.0`, always `< min_dist` for any
    /// non-negative `min_dist`), using the same sound bounding-box
    /// narrowing (the segment's own bbox inflated by `min_dist`).
    fn any_edge_within_of_segment(&self, a: Point, b: Point, min_dist: f64) -> bool {
        let query = rstar::AABB::from_corners(
            [a.x.min(b.x) as f64 - min_dist, a.y.min(b.y) as f64 - min_dist],
            [a.x.max(b.x) as f64 + min_dist, a.y.max(b.y) as f64 + min_dist],
        );
        self.tree.locate_in_envelope_intersecting(&query).any(|edge| {
            segments_intersect((a, b), (edge.a, edge.b))
                || dist_segment_to_segment((a, b), (edge.a, edge.b)) < min_dist
        })
    }
}

/// [`circle_polygon_collides`], but using a prebuilt [`PolygonEdgeIndex`]
/// instead of scanning every one of `poly`'s edges -- see that struct's
/// doc comment for why/when this matters. Produces the exact same
/// answer as `circle_polygon_collides(c, poly, clearance)` for the
/// `poly` `index` was built from.
pub fn circle_polygon_collides_indexed(c: &Circle, index: &PolygonEdgeIndex, clearance: Unit) -> bool {
    let min_dist = (c.radius + clearance) as f64;
    index.contains_point(c.center) || index.any_edge_within(c.center, min_dist)
}

/// [`segment_polygon_collides`], but using a prebuilt [`PolygonEdgeIndex`]
/// -- see [`circle_polygon_collides_indexed`]'s doc comment.
pub fn segment_polygon_collides_indexed(s: &Segment, index: &PolygonEdgeIndex, clearance: Unit) -> bool {
    let min_dist = (s.width / 2 + clearance) as f64;
    index.contains_point(s.a) || index.contains_point(s.b) || index.any_edge_within_of_segment(s.a, s.b, min_dist)
}

/// [`polygon_polygon_collides`], but using a prebuilt [`PolygonEdgeIndex`]
/// for the `b`-side polygon instead of scanning every one of its edges --
/// see [`PolygonEdgeIndex`]'s doc comment for why/when this matters (a
/// non-round pad checked against a large zone fill). Mirrors
/// [`segment_polygon_collides_indexed`]'s structure -- containment via
/// any one of `candidate`'s own vertices, then a per-edge nearby-edge
/// query -- generalized from a single segment's two endpoints/one edge
/// to `candidate`'s full vertex/edge set. Produces the exact same answer
/// as `polygon_polygon_collides(candidate, poly, clearance)` for the
/// `poly` `index` was built from.
pub fn polygon_polygon_collides_indexed(candidate: &Polygon, index: &PolygonEdgeIndex, clearance: Unit) -> bool {
    let min_dist = clearance as f64;
    candidate.points.iter().any(|&p| index.contains_point(p))
        || candidate
            .edges()
            .any(|(a, b)| index.any_edge_within_of_segment(a, b, min_dist))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aabb_contains_point_is_inclusive_of_its_own_edges() {
        let bounds = Aabb { min: Point::new(0, 0), max: Point::new(1_000_000, 2_000_000) };
        assert!(bounds.contains_point(Point::new(500_000, 500_000)), "interior point");
        assert!(bounds.contains_point(Point::new(0, 0)), "min corner is inclusive");
        assert!(bounds.contains_point(Point::new(1_000_000, 2_000_000)), "max corner is inclusive");
        assert!(!bounds.contains_point(Point::new(-1, 500_000)), "just outside on x");
        assert!(!bounds.contains_point(Point::new(500_000, 2_000_001)), "just outside on y");
    }

    #[test]
    fn segments_intersect_ignores_collinear_points_beyond_the_segment_extent() {
        // Regression test for a real bug: `on_segment`'s bounding-box
        // check compared `p` against itself (`p.x <= p.x.max(...)`,
        // always vacuously true) instead of against `a`/`b`'s extent, so
        // any point merely *collinear* with a segment's line -- even far
        // past either endpoint -- was wrongly treated as lying on the
        // segment. Found via `alladin-router`'s capsule walkaround: a
        // vertical obstacle segment from (2.5mm, -1mm) to (2.5mm, 1mm)
        // and an unrelated leg touching the same vertical line at
        // y = 1.28mm (clearly beyond the obstacle's own span) was
        // reported as intersecting.
        let obstacle = (Point::new(2_500_000, -1_000_000), Point::new(2_500_000, 1_000_000));
        let far_above = (Point::new(2_000_000, 1_282_540), Point::new(3_000_000, 1_282_540));
        assert!(
            !segments_intersect(far_above, obstacle),
            "a horizontal segment at y=1.28mm must not intersect a vertical segment spanning only y in [-1mm, 1mm]"
        );

        // Sanity check the positive case still works: shift it down so it
        // actually crosses the obstacle's real span.
        let crossing = (Point::new(2_000_000, 0), Point::new(3_000_000, 0));
        assert!(segments_intersect(crossing, obstacle));
    }

    #[test]
    fn mm_constant_matches_kicad_convention() {
        // KiCad's own test suite writes VECTOR2I(1000000, 0) to mean 1mm
        // (see qa/tests/pcbnew/test_pns_basics.cpp) -- we mirror that unit
        // convention exactly so distances/clearances translate 1:1.
        assert_eq!(MM, 1_000_000);
    }

    #[test]
    fn touching_circles_with_zero_clearance_do_not_collide() {
        let a = Circle::new(Point::new(0, 0), MM / 2);
        let b = Circle::new(Point::new(MM, 0), MM / 2); // exactly touching
        assert!(!circle_circle_collides(&a, &b, 0));
    }

    #[test]
    fn overlapping_circles_collide() {
        let a = Circle::new(Point::new(0, 0), MM / 2);
        let b = Circle::new(Point::new(MM - 1, 0), MM / 2);
        assert!(circle_circle_collides(&a, &b, 0));
    }

    #[test]
    fn jlcpcb_clearance_is_enforced() {
        // Two 0.5mm-radius pads 1.1mm apart (0.1mm gap) must violate the
        // JLCPCB 0.127mm minimum clearance rule, even though they don't
        // geometrically overlap.
        let jlc_min_clearance = (0.127 * MM as f64) as Unit;
        let a = Circle::new(Point::new(0, 0), MM / 2);
        let b = Circle::new(Point::new((1.1 * MM as f64) as Unit, 0), MM / 2);
        assert!(circle_circle_collides(&a, &b, jlc_min_clearance));
    }

    #[test]
    fn straight_line_between_two_pads_is_blocked_by_obstacle() {
        // Same scenario as the C++ probe (probe-build/probe_main.cpp):
        // padA at (0,0), padB at (5mm,0), an obstacle dead-center at
        // (2.5mm, 0) with 0.8mm radius. A straight track must be reported
        // as colliding.
        let obstacle = Circle::new(Point::new(2_500_000, 0), 800_000);
        let straight_track = Segment::new(Point::new(0, 0), Point::new(5_000_000, 0), 250_000);
        assert!(circle_segment_collides(&obstacle, &straight_track, 127_000));
    }

    #[test]
    fn detour_above_obstacle_is_clear() {
        let obstacle = Circle::new(Point::new(2_500_000, 0), 800_000);
        // Detour segment: (0,0) -> (2.5mm, 1.5mm) stays well clear.
        let detour = Segment::new(Point::new(0, 0), Point::new(2_500_000, 1_500_000), 250_000);
        assert!(!circle_segment_collides(&obstacle, &detour, 127_000));
    }

    /// Real board outline from `interf_u.kicad_pcb` (a KiCad 9 demo
    /// board), reproduced here as a fixed test fixture -- a non-convex
    /// rectangle with a protruding tab, not a trivial box. Vertex order
    /// and point-containment results below are both ground-truth
    /// verified against `pcbnew`'s own `BOARD::GetBoardPolygonOutlines()`
    /// / `SHAPE_POLY_SET::Contains()` (see
    /// `alladin_kicad_io::import_board_outline`'s doc comment and
    /// the development log's corresponding update for the full story of
    /// how this was chained together from individually-unordered
    /// `gr_line` forms).
    fn real_board_outline() -> Polygon {
        let mm = |v: f64| (v * MM as f64).round() as Unit;
        let p = |x: f64, y: f64| Point::new(mm(x), mm(y));
        Polygon::new(vec![
            p(194.9450, 133.3500),
            p(172.0850, 133.3500),
            p(172.0850, 142.4940),
            p(90.8050, 142.4940),
            p(90.8050, 140.9700),
            p(90.8050, 133.3500),
            p(79.3750, 133.3500),
            p(79.3750, 34.2900),
            p(194.9450, 34.2900),
        ])
    }

    #[test]
    fn polygon_contains_point_matches_pcbnew_on_a_real_non_convex_outline() {
        let outline = real_board_outline();
        let mm = |v: f64| (v * MM as f64).round() as Unit;
        let p = |x: f64, y: f64| Point::new(mm(x), mm(y));

        assert!(outline.contains_point(p(130.0, 50.0)), "inside the main rectangle");
        assert!(outline.contains_point(p(130.0, 138.0)), "inside the protruding tab");
        assert!(
            !outline.contains_point(p(85.0, 138.0)),
            "in the notch next to the tab -- outside despite being between the outer x-bounds"
        );
        assert!(!outline.contains_point(p(0.0, 0.0)), "far outside the board entirely");
    }

    #[test]
    fn polygon_contains_segment_rejects_a_line_that_cuts_through_the_notch() {
        let outline = real_board_outline();
        let mm = |v: f64| (v * MM as f64).round() as Unit;
        let p = |x: f64, y: f64| Point::new(mm(x), mm(y));

        // Both endpoints inside the polygon (main rectangle), but the
        // straight line between them cuts across the notch corner --
        // must be rejected even though `contains_point` alone would
        // pass both endpoints.
        assert!(!outline.contains_segment(p(85.0, 130.0), p(95.0, 141.0)));
        // A route safely inside the main rectangle, nowhere near the
        // notch or tab, must be accepted.
        assert!(outline.contains_segment(p(100.0, 40.0), p(150.0, 120.0)));
    }

    /// Simple L-shape (10x10mm square missing its top-left 5x5mm corner)
    /// -- small and easy to reason about by hand, unlike the 9-edge real
    /// board fixture above.
    fn l_shaped_board() -> Polygon {
        let p = |x: i64, y: i64| Point::new(x * MM, y * MM);
        Polygon::new(vec![p(0, 0), p(10, 0), p(10, 10), p(5, 10), p(5, 5), p(0, 5)])
    }

    #[test]
    fn inward_vertices_all_read_as_inside_the_polygon() {
        // The whole point of nudging rather than returning the exact
        // vertex: every single one of them must reliably pass
        // `contains_point`, on both a convex corner and the one
        // concave (reflex) corner this fixture has at (5mm, 5mm).
        for outline in [l_shaped_board(), real_board_outline()] {
            for vertex in outline.inward_vertices(1_000) {
                assert!(
                    outline.contains_point(vertex),
                    "nudged vertex {vertex:?} must read as inside its own polygon"
                );
            }
        }
    }

    #[test]
    fn inward_vertices_nudges_the_concave_corner_toward_the_present_notch_side() {
        // At the L-shape's one reflex vertex (5mm, 5mm), the interior is
        // toward increasing x / decreasing y (into the main rectangle,
        // away from the missing top-left corner) -- pin that down
        // directly rather than only checking `contains_point`, since a
        // nudge in a wildly wrong direction that *happens* to still
        // land inside wouldn't be caught by that alone.
        let outline = l_shaped_board();
        let concave = outline
            .inward_vertices(100_000) // a generous 0.1mm, easy to check by eye
            .into_iter()
            .find(|p| p.distance(Point::new(5 * MM, 5 * MM)) < MM as f64)
            .expect("the concave vertex must produce a nudged point near (5mm, 5mm)");

        assert!(concave.x > 5 * MM, "expected nudge toward +x, got {concave:?}");
        assert!(concave.y < 5 * MM, "expected nudge toward -y, got {concave:?}");
    }

    #[test]
    fn inward_vertices_produces_one_point_per_polygon_vertex() {
        let outline = l_shaped_board();
        assert_eq!(outline.inward_vertices(1_000).len(), outline.points.len());
    }

    #[test]
    fn rounded_rect_with_zero_radius_is_a_plain_sharp_cornered_rectangle() {
        let rect = Polygon::rounded_rect(10 * MM, 6 * MM, 0, 8);
        assert_eq!(rect.points.len(), 4);
        assert!(rect.contains_point(Point::new(0, 0)));
        assert!(rect.contains_point(Point::new(4 * MM, 2 * MM)));
        assert!(!rect.contains_point(Point::new(6 * MM, 0)));
    }

    #[test]
    fn rounded_rect_contains_its_own_center_and_stays_within_its_bounding_box() {
        let (w, h, r) = (20 * MM, 12 * MM, 2 * MM);
        let rect = Polygon::rounded_rect(w, h, r, 8);
        assert!(rect.contains_point(Point::new(0, 0)));
        for p in &rect.points {
            assert!(p.x.abs() <= w / 2, "vertex {p:?} escaped the requested width");
            assert!(p.y.abs() <= h / 2, "vertex {p:?} escaped the requested height");
        }
    }

    #[test]
    fn rounded_rect_corner_is_actually_rounded_not_sharp() {
        // A large radius should carve the exact corner point away: with a
        // 2mm radius on a 10x10mm square, (5mm, 5mm) itself must be
        // outside the polygon even though it's inside the raw bounding box.
        let rect = Polygon::rounded_rect(10 * MM, 10 * MM, 2 * MM, 8);
        assert!(!rect.contains_point(Point::new(5 * MM, 5 * MM)), "a rounded corner must cut off the sharp tip");
        assert!(rect.contains_point(Point::new(4 * MM, 4 * MM)), "just inside the rounded corner should stay inside");
    }

    #[test]
    fn rounded_rect_clamps_an_oversized_radius_instead_of_self_intersecting() {
        // Requesting a radius larger than half the shorter side must not
        // panic or produce a degenerate/self-intersecting shape -- it
        // should clamp to the largest radius that still fits (here: a
        // "stadium" shape, r == half the height).
        let rect = Polygon::rounded_rect(10 * MM, 4 * MM, 100 * MM, 8);
        assert!(rect.contains_point(Point::new(0, 0)));
        for p in &rect.points {
            assert!(p.y.abs() <= 2 * MM + 1_000, "clamped radius must not exceed half the height");
        }
    }

    #[test]
    fn outward_edge_boundary_is_exactly_clearance_from_a_rectangle_s_edges() {
        let mm = |v: f64| (v * MM as f64) as Unit;
        let rect = square(0.0, 10.0);
        let clearance = mm(0.3);

        let boundary = rect.outward_edge_boundary(clearance);
        assert_eq!(boundary.len(), rect.points.len() * 3, "two edge-offset points plus one miter point per vertex");

        for p in &boundary {
            assert!(!rect.contains_point(*p), "an outward-offset point must be outside the polygon");
            // Never *closer* than `clearance` -- the whole point of
            // this boundary. The per-edge points land exactly on it;
            // the corner miter points deliberately land a bit farther
            // out (a 90° corner's miter is `clearance * sqrt(2)` from
            // the polygon, not `clearance`, see the miter-vs-arc doc
            // comment on `outward_edge_boundary` itself) -- both are
            // fine, only "too close" would be a bug.
            let dist = dist_point_to_polygon(*p, &rect);
            assert!(
                dist >= clearance as f64 - 10.0,
                "expected at least ~{clearance} distance from the rectangle, got {dist} at {p:?}"
            );
        }

        // The 4 per-edge points landing exactly on the corner (one
        // pair per adjacent edge) must be at *precisely* `clearance`,
        // not overshot -- pin that down separately from the miters.
        let on_edge_count = boundary
            .iter()
            .filter(|p| (dist_point_to_polygon(**p, &rect) - clearance as f64).abs() < 10.0)
            .count();
        assert_eq!(on_edge_count, rect.points.len() * 2, "the 8 per-edge points, exactly at clearance");
    }

    #[test]
    fn outward_edge_boundary_of_a_triangle_stays_outside_and_roughly_clearance_away() {
        // A non-rectangular (60mm-ish angle) shape, to sanity-check the
        // per-edge approach doesn't only coincidentally work on
        // rectangles' convenient 90-degree corners.
        let mm = |v: f64| (v * MM as f64) as Unit;
        let p = |x: f64, y: f64| Point::new(mm(x), mm(y));
        let triangle = Polygon::new(vec![p(0.0, 0.0), p(10.0, 0.0), p(5.0, 8.0)]);
        let clearance = mm(0.5);

        let boundary = triangle.outward_edge_boundary(clearance);
        assert_eq!(boundary.len(), 9, "two edge-offset points plus one miter point per vertex");
        for pt in &boundary {
            assert!(!triangle.contains_point(*pt));
            assert!(dist_point_to_polygon(*pt, &triangle) > 0.0);
        }
    }

    fn square(min: f64, max: f64) -> Polygon {
        let p = |x: f64, y: f64| Point::new((x * MM as f64) as Unit, (y * MM as f64) as Unit);
        Polygon::new(vec![p(min, min), p(max, min), p(max, max), p(min, max)])
    }

    #[test]
    fn evenodd_excludes_a_hole_nested_inside_a_boundary() {
        // A 10x10mm outer boundary with a 2x2mm hole cut into its
        // middle -- exactly the real-world shape a board-outline-plus-
        // internal-cutout combination produces (see
        // `alladin_kicad_io::ImportedBoard::outline`'s doc comment).
        let boundary = square(0.0, 10.0);
        let hole = square(4.0, 6.0);
        let polygons = [boundary, hole];
        let p = |x: f64, y: f64| Point::new((x * MM as f64) as Unit, (y * MM as f64) as Unit);

        assert!(contains_point_evenodd(&polygons, p(1.0, 1.0)), "inside the boundary, outside the hole -> on the board");
        assert!(!contains_point_evenodd(&polygons, p(5.0, 5.0)), "inside the hole -> excluded");
        assert!(!contains_point_evenodd(&polygons, p(20.0, 20.0)), "outside everything");
    }

    #[test]
    fn evenodd_still_unions_two_genuinely_disjoint_regions() {
        // The other real use case this same rule has to keep working:
        // two separate, non-nested allowed regions (e.g. a board's main
        // body plus a physically separate island), which must each
        // independently count as "on the board", same as the plain-OR
        // behaviour this generalizes.
        let region_a = square(0.0, 5.0);
        let region_b = square(20.0, 25.0);
        let polygons = [region_a, region_b];
        let p = |x: f64, y: f64| Point::new((x * MM as f64) as Unit, (y * MM as f64) as Unit);

        assert!(contains_point_evenodd(&polygons, p(2.0, 2.0)));
        assert!(contains_point_evenodd(&polygons, p(22.0, 22.0)));
        assert!(!contains_point_evenodd(&polygons, p(10.0, 10.0)), "the gap between the two regions");
    }

    #[test]
    fn evenodd_segment_rejects_a_line_that_dips_through_a_hole() {
        let boundary = square(0.0, 10.0);
        let hole = square(4.0, 6.0);
        let polygons = [boundary, hole];
        let p = |x: f64, y: f64| Point::new((x * MM as f64) as Unit, (y * MM as f64) as Unit);

        // Both endpoints are individually "on the board" (odd count),
        // but the straight line between them cuts straight through the
        // hole -- must still be rejected, exactly like
        // `Polygon::contains_segment`'s own notch-crossing case.
        assert!(!contains_segment_evenodd(&polygons, p(1.0, 5.0), p(9.0, 5.0)));
        // A route safely away from the hole must be accepted.
        assert!(contains_segment_evenodd(&polygons, p(1.0, 1.0), p(9.0, 1.0)));
    }

    #[test]
    fn evenodd_of_an_empty_polygon_set_contains_nothing() {
        assert!(!contains_point_evenodd(&[], Point::new(0, 0)));
    }

    #[test]
    fn circle_within_outline_accepts_a_pad_well_inside_the_board() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        assert!(circle_within_outline(Point::new(mm(5.0), mm(5.0)), mm(1.0), &outline));
    }

    #[test]
    fn circle_within_outline_rejects_a_pad_whose_edge_crosses_the_boundary() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        // Center is on-board, but a 2mm-radius pad this close to the edge
        // sticks out past it -- must be rejected even though the center
        // alone would pass.
        assert!(!circle_within_outline(Point::new(mm(1.0), mm(5.0)), mm(2.0), &outline));
    }

    #[test]
    fn circle_within_outline_rejects_a_pad_whose_center_is_off_board() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        assert!(!circle_within_outline(Point::new(mm(50.0), mm(50.0)), mm(1.0), &outline));
    }

    #[test]
    fn circle_within_outline_with_no_outline_is_permissive() {
        assert!(circle_within_outline(Point::new(0, 0), MM, &[]));
    }

    #[test]
    fn circle_within_outline_rejects_a_pad_that_reaches_into_a_hole() {
        let boundary = square(0.0, 10.0);
        let hole = square(4.0, 6.0);
        let outline = [boundary, hole];
        let mm = |v: f64| (v * MM as f64) as Unit;
        // Center at (3mm, 5mm) is on-board, but a 1.2mm-radius pad
        // reaches into the hole at x=4mm.
        assert!(!circle_within_outline(Point::new(mm(3.0), mm(5.0)), mm(1.2), &outline));
    }

    #[test]
    fn segment_within_outline_with_clearance_accepts_a_track_well_inside_the_board() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        assert!(segment_within_outline_with_clearance(
            Point::new(mm(3.0), mm(5.0)),
            Point::new(mm(7.0), mm(5.0)),
            mm(0.25),
            mm(0.2),
            &outline,
        ));
    }

    #[test]
    fn segment_within_outline_with_clearance_rejects_a_track_hugging_the_edge() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        // Centerline at x=0.1mm is comfortably on-board (bare
        // `contains_segment_evenodd` would pass it), but a 0.25mm-wide
        // track's copper edge sits at x=0.1-0.125=-0.025mm -- already
        // off-board before `clearance` is even added on top.
        assert!(!segment_within_outline_with_clearance(
            Point::new(mm(0.1), mm(3.0)),
            Point::new(mm(0.1), mm(7.0)),
            mm(0.25),
            mm(0.2),
            &outline,
        ));
    }

    #[test]
    fn segment_within_outline_with_clearance_rejects_a_track_whose_copper_alone_is_on_board_but_too_close() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        // Centerline at x=0.3mm: a 0.25mm-wide track's copper edge
        // reaches x=0.175mm, still comfortably positive (on-board), but
        // JLCPCB's 0.2mm `copper_to_routed_edge` margin needs the edge
        // at x>=0.2mm -- must still be rejected.
        assert!(!segment_within_outline_with_clearance(
            Point::new(mm(0.3), mm(3.0)),
            Point::new(mm(0.3), mm(7.0)),
            mm(0.25),
            mm(0.2),
            &outline,
        ));
    }

    #[test]
    fn segment_within_outline_with_clearance_accepts_a_track_exactly_at_the_margin() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        // Centerline at x=0.325mm: copper edge at exactly x=0.2mm
        // (0.325 - 0.125 = 0.2) -- exactly the minimum, must still pass
        // (`>=`, not `>`).
        assert!(segment_within_outline_with_clearance(
            Point::new(mm(0.325), mm(3.0)),
            Point::new(mm(0.325), mm(7.0)),
            mm(0.25),
            mm(0.2),
            &outline,
        ));
    }

    #[test]
    fn segment_within_outline_with_clearance_rejects_a_track_whose_centerline_leaves_the_board() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        assert!(!segment_within_outline_with_clearance(
            Point::new(mm(5.0), mm(5.0)),
            Point::new(mm(50.0), mm(50.0)),
            mm(0.25),
            mm(0.2),
            &outline,
        ));
    }

    #[test]
    fn segment_within_outline_with_clearance_rejects_a_track_that_passes_too_close_to_a_hole() {
        let boundary = square(0.0, 10.0);
        let hole = square(4.0, 6.0);
        let outline = [boundary, hole];
        let mm = |v: f64| (v * MM as f64) as Unit;
        // Runs along y=3.9mm, well clear of the hole's own edge test
        // (hole spans y=4..6mm) by bare containment, but a 0.25mm track
        // plus 0.2mm clearance needs 0.325mm of margin -- only 0.1mm
        // away from the hole's y=4mm edge.
        assert!(!segment_within_outline_with_clearance(
            Point::new(mm(1.0), mm(3.9)),
            Point::new(mm(9.0), mm(3.9)),
            mm(0.25),
            mm(0.2),
            &outline,
        ));
    }

    #[test]
    fn segment_within_outline_with_clearance_with_no_outline_is_permissive() {
        assert!(segment_within_outline_with_clearance(
            Point::new(0, 0),
            Point::new(100 * MM, 100 * MM),
            250_000,
            200_000,
            &[],
        ));
    }

    #[test]
    fn dist_point_to_polygon_is_zero_strictly_inside_and_positive_outside() {
        let zone = square(0.0, 10.0); // 0..10mm on each axis
        let mm = |v: f64| (v * MM as f64) as Unit;

        assert_eq!(dist_point_to_polygon(Point::new(mm(5.0), mm(5.0)), &zone), 0.0, "dead center: strictly inside");
        assert!(dist_point_to_polygon(Point::new(mm(20.0), mm(5.0)), &zone) > 0.0, "well outside");
        // 3mm outside the right edge -> nearest boundary point is exactly 3mm away.
        assert!((dist_point_to_polygon(Point::new(mm(13.0), mm(5.0)), &zone) - mm(3.0) as f64).abs() < 1.0);
    }

    #[test]
    fn dist_segment_to_polygon_is_zero_if_either_endpoint_or_a_crossing_touches_the_zone() {
        let zone = square(0.0, 10.0);
        let mm = |v: f64| (v * MM as f64) as Unit;

        // One endpoint strictly inside.
        assert_eq!(dist_segment_to_polygon(Point::new(mm(5.0), mm(5.0)), Point::new(mm(20.0), mm(20.0)), &zone), 0.0);
        // Both endpoints outside, but the segment cuts straight through.
        assert_eq!(dist_segment_to_polygon(Point::new(mm(-5.0), mm(5.0)), Point::new(mm(15.0), mm(5.0)), &zone), 0.0);
        // Fully clear of the zone.
        assert!(dist_segment_to_polygon(Point::new(mm(20.0), mm(0.0)), Point::new(mm(20.0), mm(10.0)), &zone) > 0.0);
    }

    #[test]
    fn circle_polygon_collides_matches_the_clearance_inflated_distance() {
        let zone = square(0.0, 10.0);
        let mm = |v: f64| (v * MM as f64) as Unit;
        let clearance = mm(0.2);

        // Pad edge sits exactly 0.1mm from the zone's right edge
        // (12.1mm center - 2mm radius - 10mm zone edge = 0.1mm gap) --
        // narrower than the 0.2mm clearance, so must collide.
        let close_pad = Circle::new(Point::new(mm(12.1), mm(5.0)), mm(2.0));
        assert!(circle_polygon_collides(&close_pad, &zone, clearance));

        // Same pad, moved far enough away that even with clearance
        // there's no overlap.
        let far_pad = Circle::new(Point::new(mm(20.0), mm(5.0)), mm(2.0));
        assert!(!circle_polygon_collides(&far_pad, &zone, clearance));

        // A pad centered *inside* the zone always collides, regardless
        // of clearance (matches "0 distance = colliding").
        let inside_pad = Circle::new(Point::new(mm(5.0), mm(5.0)), mm(0.5));
        assert!(circle_polygon_collides(&inside_pad, &zone, 0));
    }

    #[test]
    fn segment_polygon_collides_matches_the_clearance_inflated_distance() {
        let zone = square(0.0, 10.0);
        let mm = |v: f64| (v * MM as f64) as Unit;
        let clearance = mm(0.2);

        // Track running parallel to the zone's right edge, 0.1mm clear
        // of it with its own half-width included -- narrower than the
        // 0.2mm clearance, so must collide.
        let close_track = Segment::new(Point::new(mm(10.3), mm(-5.0)), Point::new(mm(10.3), mm(15.0)), mm(0.4));
        assert!(segment_polygon_collides(&close_track, &zone, clearance));

        let far_track = Segment::new(Point::new(mm(20.0), mm(-5.0)), Point::new(mm(20.0), mm(15.0)), mm(0.4));
        assert!(!segment_polygon_collides(&far_track, &zone, clearance));

        // A track passing straight through the zone always collides.
        let through_track = Segment::new(Point::new(mm(-5.0), mm(5.0)), Point::new(mm(15.0), mm(5.0)), mm(0.2));
        assert!(segment_polygon_collides(&through_track, &zone, 0));
    }

    /// A many-vertex, non-convex "comb" polygon -- a rectangle with
    /// `teeth` inward-pointing notches cut into its top edge -- standing
    /// in for a real KiCad zone fill's thermal-relief-riddled boundary
    /// (see [`PolygonEdgeIndex`]'s doc comment for the real 42,415-vertex
    /// case this is modelling). Deliberately *not* spatially uniform:
    /// most of its detail is bunched along one edge, exactly the kind of
    /// clustering that would defeat a naive "just check every edge"
    /// substitute but is exactly what an R-tree's spatial partitioning
    /// should still handle correctly.
    fn comb_polygon(teeth: usize) -> Polygon {
        let mm = |v: f64| (v * MM as f64) as Unit;
        let width = 40.0;
        let height = 20.0;
        let tooth_w = width / (teeth as f64 * 2.0);
        let mut points = vec![Point::new(mm(0.0), mm(0.0)), Point::new(mm(width), mm(0.0))];
        // Walk back along the top edge (right to left), zigzagging a
        // notch down and back up for every tooth.
        for i in 0..teeth {
            let x0 = width - (i as f64 * 2.0 + 1.0) * tooth_w;
            let x1 = width - (i as f64 * 2.0 + 2.0) * tooth_w;
            points.push(Point::new(mm(x0 + tooth_w * 0.5), mm(height)));
            points.push(Point::new(mm(x0 + tooth_w * 0.5), mm(height * 0.4)));
            points.push(Point::new(mm(x1 + tooth_w * 0.5), mm(height * 0.4)));
            points.push(Point::new(mm(x1 + tooth_w * 0.5), mm(height)));
        }
        points.push(Point::new(mm(0.0), mm(height)));
        Polygon::new(points)
    }

    /// Deterministic, dependency-free pseudo-random stream (xorshift32) --
    /// good enough for a reproducible property-style cross-check test,
    /// no need for an actual `rand` crate dependency just for this.
    struct XorShift32(u32);
    impl XorShift32 {
        fn next_f64_in(&mut self, lo: f64, hi: f64) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            lo + (self.0 as f64 / u32::MAX as f64) * (hi - lo)
        }
    }

    #[test]
    fn indexed_contains_point_agrees_with_the_brute_force_version_on_a_dense_comb_polygon() {
        let poly = comb_polygon(50);
        let index = PolygonEdgeIndex::build(&poly);
        let mut rng = XorShift32(0xC0FFEE);

        let mm = MM as f64;
        for _ in 0..500 {
            let p = Point::new((rng.next_f64_in(-5.0, 45.0) * mm) as Unit, (rng.next_f64_in(-5.0, 25.0) * mm) as Unit);
            assert_eq!(
                index.contains_point(p),
                poly.contains_point(p),
                "mismatch at {p:?} for a {}-vertex comb polygon",
                poly.points.len()
            );
        }
    }

    #[test]
    fn indexed_collision_checks_agree_with_the_brute_force_ones_on_a_dense_comb_polygon() {
        let poly = comb_polygon(50);
        let index = PolygonEdgeIndex::build(&poly);
        let mm = MM as f64;
        let mut rng = XorShift32(0x5EED);

        for _ in 0..300 {
            let cx = (rng.next_f64_in(-5.0, 45.0) * mm) as Unit;
            let cy = (rng.next_f64_in(-5.0, 25.0) * mm) as Unit;
            let radius = (rng.next_f64_in(0.05, 1.0) * mm) as Unit;
            let clearance = (rng.next_f64_in(0.0, 0.5) * mm) as Unit;
            let circle = Circle::new(Point::new(cx, cy), radius);
            assert_eq!(
                circle_polygon_collides_indexed(&circle, &index, clearance),
                circle_polygon_collides(&circle, &poly, clearance),
                "circle mismatch: center=({cx},{cy}) radius={radius} clearance={clearance}"
            );

            let ax = (rng.next_f64_in(-5.0, 45.0) * mm) as Unit;
            let ay = (rng.next_f64_in(-5.0, 25.0) * mm) as Unit;
            let bx = (rng.next_f64_in(-5.0, 45.0) * mm) as Unit;
            let by = (rng.next_f64_in(-5.0, 25.0) * mm) as Unit;
            let width = (rng.next_f64_in(0.05, 1.0) * mm) as Unit;
            let seg = Segment::new(Point::new(ax, ay), Point::new(bx, by), width);
            assert_eq!(
                segment_polygon_collides_indexed(&seg, &index, clearance),
                segment_polygon_collides(&seg, &poly, clearance),
                "segment mismatch: a=({ax},{ay}) b=({bx},{by}) width={width} clearance={clearance}"
            );
        }
    }

    #[test]
    fn indexed_checks_match_the_exact_known_cases_the_brute_force_tests_already_cover() {
        let zone = square(0.0, 10.0);
        let index = PolygonEdgeIndex::build(&zone);
        let mm = |v: f64| (v * MM as f64) as Unit;
        let clearance = mm(0.2);

        let close_pad = Circle::new(Point::new(mm(12.1), mm(5.0)), mm(2.0));
        assert!(circle_polygon_collides_indexed(&close_pad, &index, clearance));
        let far_pad = Circle::new(Point::new(mm(20.0), mm(5.0)), mm(2.0));
        assert!(!circle_polygon_collides_indexed(&far_pad, &index, clearance));
        let inside_pad = Circle::new(Point::new(mm(5.0), mm(5.0)), mm(0.5));
        assert!(circle_polygon_collides_indexed(&inside_pad, &index, 0));

        let close_track = Segment::new(Point::new(mm(10.3), mm(-5.0)), Point::new(mm(10.3), mm(15.0)), mm(0.4));
        assert!(segment_polygon_collides_indexed(&close_track, &index, clearance));
        let far_track = Segment::new(Point::new(mm(20.0), mm(-5.0)), Point::new(mm(20.0), mm(15.0)), mm(0.4));
        assert!(!segment_polygon_collides_indexed(&far_track, &index, clearance));
        let through_track = Segment::new(Point::new(mm(-5.0), mm(5.0)), Point::new(mm(15.0), mm(5.0)), mm(0.2));
        assert!(segment_polygon_collides_indexed(&through_track, &index, 0));
    }

    #[test]
    fn rotated_point_by_90_degrees_swaps_axes_as_expected() {
        let mm = |v: f64| (v * MM as f64) as Unit;
        let p = Point::new(mm(2.0), mm(0.0));
        let rotated = p.rotated(90.0);
        assert!(rotated.x.abs() < 10, "x should collapse to ~0, got {}", rotated.x);
        assert!((rotated.y - mm(2.0)).abs() < 10, "y should become ~2mm, got {}", rotated.y);
    }

    #[test]
    fn rotated_point_by_360_degrees_is_the_identity() {
        let mm = |v: f64| (v * MM as f64) as Unit;
        let p = Point::new(mm(3.0), mm(-1.5));
        let rotated = p.rotated(360.0);
        assert!((rotated.x - p.x).abs() < 10);
        assert!((rotated.y - p.y).abs() < 10);
    }

    #[test]
    fn polygon_polygon_collides_true_when_close_false_when_clearly_separated() {
        let a = square(0.0, 10.0);
        let b_close = square(10.05, 20.0); // 0.05mm gap to `a`'s right edge
        let b_far = square(30.0, 40.0);
        let clearance = (0.2 * MM as f64) as Unit;
        assert!(polygon_polygon_collides(&a, &b_close, clearance), "0.05mm gap must collide under 0.2mm clearance");
        assert!(!polygon_polygon_collides(&a, &b_far, clearance), "20mm gap must never collide");
    }

    #[test]
    fn polygon_polygon_collides_exactly_at_the_clearance_boundary() {
        let mm = |v: f64| (v * MM as f64) as Unit;
        let clearance = mm(0.2);
        let a = square(0.0, 10.0);
        // `a`'s right edge is at x=10mm; `b`'s left edge exactly `clearance` away.
        let b = square(10.2, 20.0);
        assert!(!polygon_polygon_collides(&a, &b, clearance), "exactly at clearance: must not collide (>=, not >)");

        // One internal unit closer than the clearance margin: must flip to colliding.
        let b_closer = Polygon::new(vec![
            Point::new(mm(10.2) - 1, mm(0.0)),
            Point::new(mm(20.0), mm(0.0)),
            Point::new(mm(20.0), mm(10.0)),
            Point::new(mm(10.2) - 1, mm(10.0)),
        ]);
        assert!(polygon_polygon_collides(&a, &b_closer, clearance), "1 internal unit closer than clearance must collide");
    }

    #[test]
    fn polygon_polygon_collides_detects_full_containment_even_with_no_crossing_edges() {
        let outer = square(0.0, 10.0);
        let inner = square(3.0, 6.0); // fully inside `outer`, no edges cross
        assert!(polygon_polygon_collides(&outer, &inner, 0), "fully nested polygons must collide even at zero clearance");
        assert!(polygon_polygon_collides(&inner, &outer, 0), "symmetric: argument order must not matter");
    }

    #[test]
    fn polygon_polygon_collides_detects_crossing_edges_at_zero_clearance_even_with_no_contained_vertex() {
        // Regression test for a real bug: a narrow tall rectangle
        // crossing a wide flat one into a plus shape contains none of
        // the other's vertices, and at zero clearance the edge
        // *distance* check (`dist < 0.0`) can never fire either -- so
        // this genuinely overlapping pair was reported as clear. The
        // explicit `segments_intersect` test is what catches it.
        let mm = |v: f64| (v * MM as f64) as Unit;
        let wide = Polygon::new(vec![
            Point::new(mm(-5.0), mm(-1.0)),
            Point::new(mm(5.0), mm(-1.0)),
            Point::new(mm(5.0), mm(1.0)),
            Point::new(mm(-5.0), mm(1.0)),
        ]);
        let tall = Polygon::new(vec![
            Point::new(mm(-1.0), mm(-5.0)),
            Point::new(mm(1.0), mm(-5.0)),
            Point::new(mm(1.0), mm(5.0)),
            Point::new(mm(-1.0), mm(5.0)),
        ]);
        assert!(polygon_polygon_collides(&wide, &tall, 0), "a plus-shaped overlap must collide at zero clearance");
        assert!(polygon_polygon_collides(&tall, &wide, 0), "symmetric: argument order must not matter");
    }

    #[test]
    fn polygon_polygon_collides_is_invariant_under_a_shared_rotation() {
        // Two axis-aligned 10x4mm rectangles with a known, deliberately
        // narrow 0.1mm gap between them.
        let mm = |v: f64| (v * MM as f64) as Unit;
        let clearance = mm(0.2);
        let a = Polygon::rounded_rect(mm(10.0), mm(4.0), 0, 1); // centred on the origin
        let translate = |poly: &Polygon, dx: Unit, dy: Unit| {
            Polygon::new(poly.points.iter().map(|&p| Point::new(p.x + dx, p.y + dy)).collect())
        };
        // a's right edge is at x=5mm; offset b so its left edge (own
        // half-width 5mm) sits 0.1mm beyond that.
        let b = translate(&a, mm(5.0) + mm(0.1) + mm(5.0), 0);
        assert!(polygon_polygon_collides(&a, &b, clearance), "sanity check, unrotated: 0.1mm gap collides under 0.2mm clearance");

        // Rotating both shapes together by the same angle around the
        // same pivot must not change a purely relative-geometry answer
        // -- this is what actually exercises `Point::rotated` the same
        // way real pad-polygon construction will.
        let rotate_poly = |poly: &Polygon, deg: f64| Polygon::new(poly.points.iter().map(|&p| p.rotated(deg)).collect());
        let a_rot = rotate_poly(&a, 37.0);
        let b_rot = rotate_poly(&b, 37.0);
        assert!(polygon_polygon_collides(&a_rot, &b_rot, clearance), "rotating both shapes together must not change the collision outcome");

        let b_far = rotate_poly(&translate(&a, mm(50.0), 0), 37.0);
        assert!(!polygon_polygon_collides(&a_rot, &b_far, clearance), "a far-apart pair must stay non-colliding after the same rotation");
    }

    #[test]
    fn indexed_polygon_polygon_collides_agrees_with_the_brute_force_version_on_a_dense_comb_polygon() {
        let poly = comb_polygon(50);
        let index = PolygonEdgeIndex::build(&poly);
        let mm = MM as f64;
        let mut rng = XorShift32(0xBADC0DE);

        for _ in 0..200 {
            let cx = rng.next_f64_in(-5.0, 45.0) * mm;
            let cy = rng.next_f64_in(-5.0, 25.0) * mm;
            let hw = rng.next_f64_in(0.1, 2.0) * mm;
            let hh = rng.next_f64_in(0.1, 2.0) * mm;
            let clearance = (rng.next_f64_in(0.0, 0.5) * mm) as Unit;
            let candidate = Polygon::new(vec![
                Point::new((cx - hw) as Unit, (cy - hh) as Unit),
                Point::new((cx + hw) as Unit, (cy - hh) as Unit),
                Point::new((cx + hw) as Unit, (cy + hh) as Unit),
                Point::new((cx - hw) as Unit, (cy + hh) as Unit),
            ]);
            assert_eq!(
                polygon_polygon_collides_indexed(&candidate, &index, clearance),
                polygon_polygon_collides(&candidate, &poly, clearance),
                "mismatch for candidate centered at ({cx},{cy}) hw={hw} hh={hh} clearance={clearance}"
            );
        }
    }

    #[test]
    fn polygon_within_outline_with_clearance_accepts_a_pad_well_inside_the_board() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        let pad = Polygon::new(vec![
            Point::new(mm(4.0), mm(4.0)),
            Point::new(mm(6.0), mm(4.0)),
            Point::new(mm(6.0), mm(6.0)),
            Point::new(mm(4.0), mm(6.0)),
        ]);
        assert!(polygon_within_outline_with_clearance(&pad, mm(0.2), &outline));
    }

    #[test]
    fn polygon_within_outline_with_clearance_rejects_a_rotated_pad_whose_true_corner_crosses_the_edge() {
        // A 4mm x 1mm pad, rotated 45 degrees, centered close enough to
        // the right edge (x=10mm) that its *true*, rotated corner pokes
        // past it -- even though its inscribed circle (radius 0.5mm)
        // would have stayed comfortably clear of the same edge. This is
        // exactly the real-world bug this function exists to catch.
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        let rect = Polygon::rounded_rect(mm(4.0), mm(1.0), 0, 1);
        let center = Point::new(mm(9.0), mm(5.0)); // 1mm inset from the edge
        let pad = Polygon::new(
            rect.points
                .into_iter()
                .map(|p| p.rotated(45.0))
                .map(|p| Point::new(p.x + center.x, p.y + center.y))
                .collect(),
        );
        assert!(!polygon_within_outline_with_clearance(&pad, mm(0.2), &outline));
    }

    #[test]
    fn polygon_within_outline_with_clearance_accepts_a_pad_exactly_at_the_margin() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        let clearance = mm(0.2);
        // Pad's right edge sits at exactly x = 10mm - 0.2mm = 9.8mm.
        let pad = Polygon::new(vec![
            Point::new(mm(9.0), mm(4.0)),
            Point::new(mm(9.8), mm(4.0)),
            Point::new(mm(9.8), mm(6.0)),
            Point::new(mm(9.0), mm(6.0)),
        ]);
        assert!(polygon_within_outline_with_clearance(&pad, clearance, &outline), "exactly at the margin must pass (>=, not >)");
    }

    #[test]
    fn polygon_within_outline_with_clearance_rejects_a_pad_just_inside_the_margin() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        let clearance = mm(0.2);
        let pad = Polygon::new(vec![
            Point::new(mm(9.0), mm(4.0)),
            Point::new(mm(9.81), mm(4.0)),
            Point::new(mm(9.81), mm(6.0)),
            Point::new(mm(9.0), mm(6.0)),
        ]);
        assert!(!polygon_within_outline_with_clearance(&pad, clearance, &outline));
    }

    #[test]
    fn polygon_within_outline_with_clearance_rejects_a_pad_that_pokes_off_the_board() {
        let outline = [square(0.0, 10.0)];
        let mm = |v: f64| (v * MM as f64) as Unit;
        let pad = Polygon::new(vec![
            Point::new(mm(9.0), mm(4.0)),
            Point::new(mm(11.0), mm(4.0)),
            Point::new(mm(11.0), mm(6.0)),
            Point::new(mm(9.0), mm(6.0)),
        ]);
        assert!(!polygon_within_outline_with_clearance(&pad, mm(0.2), &outline));
    }

    #[test]
    fn polygon_within_outline_with_clearance_rejects_a_pad_that_reaches_into_a_hole() {
        let boundary = square(0.0, 10.0);
        let hole = square(4.0, 6.0);
        let outline = [boundary, hole];
        let mm = |v: f64| (v * MM as f64) as Unit;
        let pad = Polygon::new(vec![
            Point::new(mm(3.0), mm(4.5)),
            Point::new(mm(3.9), mm(4.5)),
            Point::new(mm(3.9), mm(5.5)),
            Point::new(mm(3.0), mm(5.5)),
        ]);
        assert!(
            !polygon_within_outline_with_clearance(&pad, mm(0.2), &outline),
            "0.1mm from the hole edge, under the 0.2mm clearance"
        );
    }

    #[test]
    fn polygon_within_outline_with_clearance_with_no_outline_is_permissive() {
        let pad = square(0.0, 10.0);
        assert!(polygon_within_outline_with_clearance(&pad, MM, &[]));
    }
}
