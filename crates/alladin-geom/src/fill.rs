//! Adapter between `alladin_geom::{Point, Polygon}` and the `i_overlay`
//! crate's polygon boolean/buffer engine -- infrastructure Feature B
//! (copper pour zones, see the development log's "Vias und
//! Kupferflächen im Editor" plan) needs and nothing else in this
//! workspace has ever needed before: union/intersection/difference of
//! arbitrary polygons, and buffering (offsetting) a polygon outward by
//! a clearance distance -- the two primitives a real zone-fill algorithm
//! is built from (clip to the board outline, inflate every obstacle by
//! its clearance, union them, subtract from the zone outline).
//!
//! # Why `i_overlay`
//! Pure Rust, actively maintained, and -- uniquely among the
//! alternatives surveyed for this plan -- offers both boolean ops *and*
//! buffering in one crate, avoiding a second dependency (or a
//! hand-rolled Minkowski-sum offset) just for "inflate this pad by its
//! clearance".
//!
//! # On "no floating-point drift"
//! `i_overlay`'s boolean operations ([`union`]/[`intersection`]/
//! [`difference`]) have a genuinely raw-integer API (`Overlay<i64>`),
//! used directly here -- no `f64` involved at all. Its *buffering*
//! (`OutlineOffset`) API, however, is only exposed over
//! `FloatPointCompatible` points in `i_overlay` v7, even though its
//! internal solver is fully integer and selectable as `i16`/`i32`/`i64`
//! (via the `*_as::<I>` methods this module always uses explicitly).
//! Alladin's own `Unit` (`i64` nanometres) converts to `f64` exactly for
//! any physically sane board size (values well under `f64`'s 2^53
//! exact-integer range), so [`buffer`]'s round trip through `f64`
//! introduces no measurable error at PCB scale -- but it is a real,
//! documented departure from "purely integer end to end", forced by
//! what `i_overlay` v7 actually exposes rather than a choice made here.

use crate::{segments_intersect, Point, Polygon, Unit};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay::{Overlay, ShapeType};
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::i_float::int::point::IntPoint;
use i_overlay::mesh::outline::offset::OutlineOffset;
use i_overlay::mesh::style::{LineJoin, OutlineStyle};

fn to_int_contour(polygon: &Polygon) -> Vec<IntPoint<i64>> {
    polygon.points.iter().map(|p| IntPoint::new(p.x, p.y)).collect()
}

fn from_int_contour(points: &[IntPoint<i64>]) -> Polygon {
    Polygon::new(points.iter().map(|p| Point::new(p.x, p.y)).collect())
}

/// One boolean-op result region: a filled area that may itself contain
/// holes -- `i_overlay`'s own "first contour is the outer boundary,
/// every following one is a hole" shape convention, kept explicit here
/// (rather than flattened into a bare `Vec<Polygon>`) so a caller never
/// has to guess which contour is which. Use [`Self::sealed`] to turn
/// this into the single hole-less [`Polygon`] `alladin_core::Item::Zone`
/// actually stores.
pub struct FilledRegion {
    pub outer: Polygon,
    pub holes: Vec<Polygon>,
}

impl FilledRegion {
    /// This region as one simple, hole-less polygon: every hole is
    /// "keyholed" into the outer boundary via a thin zero-width slit --
    /// the exact same convention real KiCad's own zone filler already
    /// uses for a `filled_polygon` block with a hole in it (ground-truth
    /// confirmed on a real board, see `alladin-kicad-io::import_zone`'s
    /// doc comment), and what lets a hole-bearing fill result still fit
    /// [`alladin_core::Item::Zone`]'s single, hole-less `outline: Polygon`
    /// field without changing that type. See [`seal_holes`].
    pub fn sealed(&self) -> Polygon {
        seal_holes(&self.outer, &self.holes)
    }
}

fn shapes_to_regions(shapes: Vec<Vec<Vec<IntPoint<i64>>>>) -> Vec<FilledRegion> {
    shapes
        .into_iter()
        .filter_map(|mut contours| {
            if contours.is_empty() {
                return None;
            }
            let outer = from_int_contour(&contours.remove(0));
            let holes = contours.iter().map(|c| from_int_contour(c)).collect();
            Some(FilledRegion { outer, holes })
        })
        .collect()
}

/// Unions every polygon in `polygons` together. Uses the `NonZero` fill
/// rule rather than even-odd: every polygon here (a buffered obstacle,
/// or the zone/board outline itself) is a simple, consistently-wound,
/// non-self-overlapping ring, so two of them overlapping must still
/// read as "filled" everywhere they cover -- which is exactly what
/// `NonZero` guarantees and `EvenOdd` does not (even-odd would punch a
/// hole wherever an *even* number of inputs happen to overlap, which is
/// never the union anyone actually wants here).
pub fn union(polygons: &[Polygon]) -> Vec<FilledRegion> {
    boolean_op(polygons, &[], OverlayRule::Union)
}

/// The area covered by both `subject` and `clip` -- what clipping a
/// user-drawn zone outline to the board's own outline (including any
/// cutouts/holes in it) needs. See [`union`]'s doc comment for why
/// `NonZero`, not even-odd, is the right fill rule here too.
pub fn intersection(subject: &[Polygon], clip: &[Polygon]) -> Vec<FilledRegion> {
    boolean_op(subject, clip, OverlayRule::Intersect)
}

/// `subject` minus `clip` -- what carving buffered obstacle keepouts out
/// of a zone's already board-clipped fill area needs. See [`union`]'s
/// doc comment for why `NonZero`, not even-odd, is the right fill rule
/// here too.
pub fn difference(subject: &[Polygon], clip: &[Polygon]) -> Vec<FilledRegion> {
    boolean_op(subject, clip, OverlayRule::Difference)
}

fn boolean_op(subject: &[Polygon], clip: &[Polygon], rule: OverlayRule) -> Vec<FilledRegion> {
    if subject.is_empty() && clip.is_empty() {
        return Vec::new();
    }
    let subj: Vec<Vec<IntPoint<i64>>> = subject.iter().map(to_int_contour).collect();
    let clp: Vec<Vec<IntPoint<i64>>> = clip.iter().map(to_int_contour).collect();
    let mut overlay = Overlay::<i64>::new(0);
    overlay.add_contours(&subj, ShapeType::Subject);
    overlay.add_contours(&clp, ShapeType::Clip);
    shapes_to_regions(overlay.overlay(rule, FillRule::NonZero))
}

/// Buffers (grows, or with a negative `distance`, shrinks) `polygon`
/// outward by `distance` -- "give this obstacle its clearance margin
/// before subtracting it from a zone" (the fill algorithm's own step 2).
/// A round join is used so an already many-sided circle/oval pad
/// approximation stays round rather than growing visible facets; can
/// return more than one polygon if a large negative `distance` splits
/// `polygon` into disjoint pieces, or none at all if it shrinks it out
/// of existence entirely.
pub fn buffer(polygon: &Polygon, distance: Unit) -> Vec<Polygon> {
    if polygon.points.len() < 3 {
        return Vec::new();
    }
    let path: Vec<[f64; 2]> = polygon.points.iter().map(|p| [p.x as f64, p.y as f64]).collect();
    let style = OutlineStyle::new(distance as f64).line_join(LineJoin::Round(0.2));
    let shapes = path.outline_as::<i64>(&style);
    shapes
        .into_iter()
        .flatten()
        .map(|contour| Polygon::new(contour.into_iter().map(|[x, y]| Point::new(x.round() as Unit, y.round() as Unit)).collect()))
        .collect()
}

/// "Seals" `outer`'s `holes` into it by cutting a thin double-back slit
/// from each hole's boundary to the outer boundary (or to a hole already
/// merged in), producing one simple (non-self-intersecting) polygon.
/// See [`FilledRegion::sealed`]'s doc comment for why this convention
/// (rather than an explicit holes field) is what this workspace uses.
///
/// For each hole, every candidate bridge (one outer-boundary vertex,
/// one hole vertex) is tried shortest-first, skipping any that would
/// cross the outer boundary, the hole itself, or pass outside the outer
/// boundary/inside the hole -- so nearby, unrelated geometry is never
/// clipped through. Real pour obstacles (buffered pads/tracks/vias) are
/// small and well-separated relative to a typical zone, so this always
/// finds a valid bridge in practice; if a pathological input somehow
/// leaves none, the closest candidate is used anyway (a zero-width
/// overlap is still far closer to correct than silently dropping the
/// hole's keepout).
pub fn seal_holes(outer: &Polygon, holes: &[Polygon]) -> Polygon {
    let mut boundary = outer.points.clone();
    for hole in holes {
        if hole.points.len() < 3 {
            continue;
        }
        merge_hole_into_boundary(&mut boundary, &hole.points);
    }
    Polygon::new(boundary)
}

fn merge_hole_into_boundary(boundary: &mut Vec<Point>, hole: &[Point]) {
    let mut candidates: Vec<(i128, usize, usize)> = Vec::with_capacity(boundary.len() * hole.len());
    for (i, &bp) in boundary.iter().enumerate() {
        for (j, &hp) in hole.iter().enumerate() {
            candidates.push((bp.sub(hp).length_sq(), i, j));
        }
    }
    candidates.sort_by_key(|&(dist, _, _)| dist);

    let bridge = candidates
        .iter()
        .find(|&&(_, i, j)| bridge_is_valid(boundary, hole, i, j))
        .or_else(|| candidates.first())
        .copied();

    if let Some((_, i, j)) = bridge {
        splice_hole(boundary, hole, i, j);
    }
}

fn bridge_is_valid(boundary: &[Point], hole: &[Point], i: usize, j: usize) -> bool {
    let a = boundary[i];
    let b = hole[j];
    if a == b {
        return true; // touching corner: degenerate zero-length bridge, harmless
    }

    let n = boundary.len();
    for k in 0..n {
        let k2 = (k + 1) % n;
        if k == i || k2 == i {
            continue; // edges incident to `a` legitimately touch the bridge there
        }
        if segments_intersect((a, b), (boundary[k], boundary[k2])) {
            return false;
        }
    }

    let m = hole.len();
    for k in 0..m {
        let k2 = (k + 1) % m;
        if k == j || k2 == j {
            continue; // edges incident to `b` legitimately touch the bridge there
        }
        if segments_intersect((a, b), (hole[k], hole[k2])) {
            return false;
        }
    }

    let mid = Point::new((a.x + b.x) / 2, (a.y + b.y) / 2);
    Polygon::new(boundary.to_vec()).contains_point(mid) && !Polygon::new(hole.to_vec()).contains_point(mid)
}

/// Splices `hole` into `boundary` at the bridge `(boundary[i], hole[j])`:
/// walk `hole` starting at `j` all the way around back to `j`, then
/// return to `boundary[i]` -- the classic "duplicate both bridge
/// endpoints" keyhole construction, which turns an outer ring plus one
/// hole into a single simple ring.
fn splice_hole(boundary: &mut Vec<Point>, hole: &[Point], i: usize, j: usize) {
    let m = hole.len();
    let mut insert = Vec::with_capacity(m + 2);
    for step in 0..=m {
        insert.push(hole[(j + step) % m]);
    }
    insert.push(boundary[i]);
    boundary.splice(i + 1..i + 1, insert);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MM;

    fn square(cx: f64, cy: f64, half: f64) -> Polygon {
        let mm = |v: f64| (v * MM as f64).round() as Unit;
        Polygon::new(vec![
            Point::new(mm(cx - half), mm(cy - half)),
            Point::new(mm(cx + half), mm(cy - half)),
            Point::new(mm(cx + half), mm(cy + half)),
            Point::new(mm(cx - half), mm(cy + half)),
        ])
    }

    fn area_mm2(polygon: &Polygon) -> f64 {
        let n = polygon.points.len();
        let sum: f64 = (0..n)
            .map(|i| {
                let a = polygon.points[i];
                let b = polygon.points[(i + 1) % n];
                (a.x as f64) * (b.y as f64) - (b.x as f64) * (a.y as f64)
            })
            .sum();
        (sum.abs() / 2.0) / (MM as f64 * MM as f64)
    }

    #[test]
    fn union_of_two_overlapping_squares_is_one_region_with_no_holes() {
        let a = square(0.0, 0.0, 5.0);
        let b = square(8.0, 0.0, 5.0);
        let regions = union(&[a, b]);
        assert_eq!(regions.len(), 1);
        assert!(regions[0].holes.is_empty());
        // 10x10 + 10x10 overlapping by 2x10 -> 100 + 100 - 20 = 180mm^2.
        assert!((area_mm2(&regions[0].outer) - 180.0).abs() < 1.0);
    }

    #[test]
    fn union_of_two_disjoint_squares_is_two_regions() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(100.0, 0.0, 2.0);
        let regions = union(&[a, b]);
        assert_eq!(regions.len(), 2);
    }

    #[test]
    fn difference_of_a_square_minus_a_centered_smaller_square_leaves_a_hole() {
        let outer = square(0.0, 0.0, 10.0);
        let inner = square(0.0, 0.0, 3.0);
        let regions = difference(&[outer], &[inner]);
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].holes.len(), 1, "the smaller square must come back as a hole, not vanish");
        // 20x20 minus 6x6 = 400 - 36 = 364mm^2.
        assert!((area_mm2(&regions[0].outer) - 400.0).abs() < 1.0);
    }

    #[test]
    fn difference_of_disjoint_shapes_is_unchanged() {
        let outer = square(0.0, 0.0, 5.0);
        let elsewhere = square(100.0, 0.0, 2.0);
        let regions = difference(&[outer], &[elsewhere]);
        assert_eq!(regions.len(), 1);
        assert!(regions[0].holes.is_empty());
        assert!((area_mm2(&regions[0].outer) - 100.0).abs() < 1.0);
    }

    #[test]
    fn intersection_of_two_overlapping_squares_is_the_shared_rectangle() {
        let a = square(0.0, 0.0, 5.0);
        let b = square(8.0, 0.0, 5.0);
        let regions = intersection(&[a], &[b]);
        assert_eq!(regions.len(), 1);
        // Overlap is 2mm (x) by 10mm (y).
        assert!((area_mm2(&regions[0].outer) - 20.0).abs() < 1.0);
    }

    #[test]
    fn intersection_of_disjoint_shapes_is_empty() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(100.0, 0.0, 2.0);
        assert!(intersection(&[a], &[b]).is_empty());
    }

    #[test]
    fn buffer_grows_a_square_outward_by_roughly_the_requested_distance() {
        let original = square(0.0, 0.0, 5.0);
        let grown = buffer(&original, MM);
        assert_eq!(grown.len(), 1);
        // A round-jointed outward offset by r is the Minkowski sum of
        // the original shape with a disk of radius r: area + perimeter*r
        // + pi*r^2 -- strictly less than the naive (sharp-cornered) 12x12
        // square's 144mm^2, since rounding shaves off the sharp corners
        // a miter join would otherwise leave in place.
        let expected = 100.0 + 40.0 * 1.0 + std::f64::consts::PI * 1.0 * 1.0;
        let area = area_mm2(&grown[0]);
        assert!(area > 100.0, "must grow past the original 10x10 square");
        assert!((area - expected).abs() < 1.0, "area {area} must be close to the Minkowski-sum estimate {expected}");
    }

    #[test]
    fn buffer_shrinks_with_a_negative_distance() {
        let original = square(0.0, 0.0, 5.0);
        let shrunk = buffer(&original, -MM);
        assert_eq!(shrunk.len(), 1);
        // 8x8 (10mm square - 1mm margin on every side) = 64mm^2.
        assert!((area_mm2(&shrunk[0]) - 64.0).abs() < 2.0);
    }

    #[test]
    fn seal_holes_on_a_shape_with_no_holes_returns_the_outer_boundary_unchanged() {
        let outer = square(0.0, 0.0, 5.0);
        let sealed = seal_holes(&outer, &[]);
        assert_eq!(sealed.points, outer.points);
    }

    #[test]
    fn seal_holes_produces_a_single_simple_polygon_around_a_centered_hole() {
        let outer = square(0.0, 0.0, 10.0);
        let hole = square(0.0, 0.0, 3.0);
        let sealed = seal_holes(&outer, &[hole]);

        // Splicing duplicates the two bridge endpoints -- outer's 4
        // points + hole's 4 points + 2 duplicated bridge endpoints.
        assert_eq!(sealed.points.len(), 10);
        // A valid keyhole ring: no two edges may genuinely *cross*. They
        // may still *touch* at a shared endpoint -- both at ordinary
        // adjacent vertices, and, degenerately, where the zero-width
        // bridge slit touches itself (the duplicated bridge point) --
        // that touch is the whole point of the keyhole construction, not
        // a self-intersection.
        let edges: Vec<(Point, Point)> = sealed.edges().collect();
        for (a, edge_a) in edges.iter().enumerate() {
            for (b, edge_b) in edges.iter().enumerate() {
                let shares_endpoint =
                    edge_a.0 == edge_b.0 || edge_a.0 == edge_b.1 || edge_a.1 == edge_b.0 || edge_a.1 == edge_b.1;
                if a == b || shares_endpoint {
                    continue;
                }
                assert!(!segments_intersect(*edge_a, *edge_b), "sealed polygon must not self-intersect ({a} x {b})");
            }
        }
        // The hole's own area must genuinely be missing from the sealed
        // shape's interior (sample its center).
        assert!(!sealed.contains_point(Point::new(0, 0)), "the hole's center must read as outside the sealed shape");
        assert!(sealed.contains_point(Point::new(0, (8.0 * MM as f64) as Unit)), "well inside the ring (outside the hole) must still read as inside");
    }

    #[test]
    fn union_then_seal_round_trips_a_pour_with_a_pad_shaped_hole() {
        // The realistic Feature B pipeline in miniature: fill a zone,
        // subtract one buffered obstacle, seal the resulting hole -- the
        // sealed result must still exactly exclude the obstacle's area.
        let zone = square(0.0, 0.0, 10.0);
        let obstacle = square(0.0, 0.0, 2.0);
        let regions = difference(&[zone], &[obstacle]);
        assert_eq!(regions.len(), 1);
        let sealed = regions[0].sealed();

        assert!(!sealed.contains_point(Point::new(0, 0)));
        assert!(sealed.contains_point(Point::new(0, (9.0 * MM as f64) as Unit)));
    }
}
