//! Turns a user-drawn zone outline into one or more real, DFM-correct
//! `alladin_core::Item::Zone` fill islands.
//!
//! This is deliberately **not** "just store the outline the user drew as
//! copper": that would silently short every different-net pad/track
//! underneath it. Instead, [`fill_zone`] runs the same board-clip /
//! obstacle-buffer / union / difference pipeline a real copper pour
//! needs, using [`alladin_geom::fill`]'s `i_overlay` adapter for the
//! actual polygon boolean/buffer math.
//!
//! Deliberately **not** modelled (same scope cut as for imported zone
//! fills): thermal-relief spokes (same-net
//! pads/vias are simply left solidly connected, no spoke gap), and
//! priority between multiple zones of different nets overlapping each
//! other (an existing [`Item::Zone`] is not itself treated as an
//! obstacle here -- only `Pad`/`Via`/`Track`).

use alladin_core::{Item, LayerId, NetId, Node, PadShape, RuleResolver};
use alladin_geom::{fill, Point, Polygon, Unit};

/// Segment count for a fill-time circle/capsule-cap approximation.
/// Deliberately higher than hot-path clearance-check circle budgets
/// (those choose smaller counts for graph cost): zone filling runs
/// once per explicit user action, never in a search hot loop, so it
/// can afford the extra smoothness without any real performance cost.
const ZONE_CIRCLE_SEGMENTS: usize = 32;

/// A regular `ZONE_CIRCLE_SEGMENTS`-gon approximating a circle of
/// `radius` around `center` -- used for round pads, vias, and (via
/// [`capsule_polygon`]) a track's two end caps.
fn circle_polygon(center: Point, radius: Unit) -> Polygon {
    let points = (0..ZONE_CIRCLE_SEGMENTS)
        .map(|k| {
            let angle = std::f64::consts::TAU * k as f64 / ZONE_CIRCLE_SEGMENTS as f64;
            Point::new(center.x + (radius as f64 * angle.cos()).round() as Unit, center.y + (radius as f64 * angle.sin()).round() as Unit)
        })
        .collect();
    Polygon::new(points)
}

/// The exact stadium/capsule outline of a track segment from `a` to `b`
/// with the given `radius` (already clearance-inflated by the caller) --
/// built directly at the true final radius rather than discretized then
/// buffered, so this carries no extra faceting error beyond the
/// `ZONE_CIRCLE_SEGMENTS`-gon approximation of its own two round caps.
/// Degenerates to a plain [`circle_polygon`] for a zero-length segment
/// (a same-point track, never produced by real routing but not worth a
/// panic over).
fn capsule_polygon(a: Point, b: Point, radius: Unit) -> Polygon {
    let dx = (b.x - a.x) as f64;
    let dy = (b.y - a.y) as f64;
    if dx == 0.0 && dy == 0.0 {
        return circle_polygon(a, radius);
    }
    let base = dy.atan2(dx);
    let half = ZONE_CIRCLE_SEGMENTS / 2;
    let mut points = Vec::with_capacity(2 * (half + 1));
    let arc = |center: Point, start: f64, points: &mut Vec<Point>| {
        for k in 0..=half {
            let angle = start + std::f64::consts::PI * (k as f64) / (half as f64);
            points.push(Point::new(center.x + (radius as f64 * angle.cos()).round() as Unit, center.y + (radius as f64 * angle.sin()).round() as Unit));
        }
    };
    // The cap around `b` faces away from `a` (sweeps through `base`);
    // the cap around `a` faces away from `b` (sweeps through `base` +
    // 180deg) -- together with the two implicit straight sides between
    // them, this is the exact Minkowski sum of the segment with a disk.
    arc(b, base - std::f64::consts::FRAC_PI_2, &mut points);
    arc(a, base + std::f64::consts::FRAC_PI_2, &mut points);
    Polygon::new(points)
}

fn item_on_layer(item: &Item, layer: LayerId) -> bool {
    match item.layers() {
        (a, None) => a == layer,
        (a, Some(b)) => a == layer || b == layer,
    }
}

/// The exact clearance-inflated obstacle polygon for one `Pad`/`Via`/
/// `Track` item, or `None` for any other item kind (only these three
/// ever need to be kept clear of a pour -- see this module's own doc
/// comment for why an existing [`Item::Zone`] is deliberately excluded
/// too). `clearance` is the already-resolved [`RuleResolver`] distance
/// for "this item kind vs. a zone".
fn obstacle_polygon(item: &Item, clearance: Unit) -> Option<Polygon> {
    match item {
        Item::Pad { shape: PadShape::Circle(c), .. } => Some(circle_polygon(c.center, c.radius + clearance)),
        Item::Pad { shape: PadShape::Polygon { outline, .. }, .. } => fill::buffer(outline, clearance).into_iter().next(),
        Item::Via { shape, .. } => Some(circle_polygon(shape.center, shape.radius + clearance)),
        Item::Track { shape, .. } => Some(capsule_polygon(shape.a, shape.b, shape.width / 2 + clearance)),
        // A mounting hole keeps the pour out of the full screw-head
        // circle (radius = drill diameter, matching
        // `alladin_core::hole_keepout_circle`), not just the drilled
        // barrel -- the screw head that will sit here must never land
        // on live zone copper.
        Item::Hole { position, drill } => Some(circle_polygon(*position, *drill + clearance)),
        Item::Zone { .. } => None,
    }
}

/// Fills a user-drawn `outline` on `layer` for `net` against the current
/// board state, returning zero or more [`Item::Zone`]s -- one per
/// disjoint filled island, matching how a KiCad-compatible
/// `filled_polygon` list maps to one `Item::Zone` per entry. Returns an
/// empty `Vec` if the outline doesn't overlap the board outline at all,
/// or if obstacles consume the entire clipped area.
///
/// Pipeline (see this module's own doc comment for what's deliberately
/// out of scope):
/// 1. Clip `outline` to `board_outline` (so a pour drawn past the
///    board's own edge, or across one of its cutouts, comes back
///    trimmed to what's physically there).
/// 2. Buffer every same-layer, different-net `Pad`/`Via`/`Track` in
///    `node` by its resolver-determined clearance from a zone.
/// 3. Union those buffered obstacles, then subtract the union from the
///    board-clipped fill area.
/// 4. Seal any resulting holes into each island's single `outline`
///    polygon via [`fill::FilledRegion::sealed`] (this project's
///    convention for representing a filled-with-holes shape as one
///    simple ring, matching a real KiCad `filled_polygon`'s own keyhole
///    convention -- see that function's doc comment).
pub fn fill_zone(outline: &Polygon, layer: LayerId, net: NetId, board_outline: &[Polygon], node: &Node, resolver: &dyn RuleResolver) -> Vec<Item> {
    let clipped = fill::intersection(std::slice::from_ref(outline), board_outline);
    if clipped.is_empty() {
        return Vec::new();
    }

    // A zero-geometry stand-in purely to select `RuleResolver::clearance`'s
    // "zone vs. X" match arms (see `alladin_core::JlcpcbClearance::clearance`)
    // -- its own outline/net content is never inspected, only its variant.
    let zone_stub = Item::Zone { outline: Polygon::new(Vec::new()), layer, net: None };

    let obstacles: Vec<Polygon> = node
        .iter()
        .filter(|item| item.net() != Some(net))
        .filter(|item| item_on_layer(item, layer))
        .filter_map(|item| obstacle_polygon(item, resolver.clearance(&zone_stub, item)))
        .collect();

    let obstacle_union: Vec<Polygon> = fill::union(&obstacles).iter().map(fill::FilledRegion::sealed).collect();

    clipped
        .iter()
        .flat_map(|region| fill::difference(std::slice::from_ref(&region.sealed()), &obstacle_union))
        .map(|island| Item::Zone { outline: island.sealed(), layer, net: Some(net) })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::{JlcpcbClearance, NetClass};
    use alladin_geom::{Circle, Segment, MM};

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
    fn fills_the_whole_outline_when_there_are_no_obstacles() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let node = Node::new();
        let islands = fill_zone(&outline, LayerId::FCu, NetId(1), &board, &node, &JlcpcbClearance);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, net, layer } = &islands[0] else { panic!("expected a zone") };
        assert_eq!(*net, Some(NetId(1)));
        assert_eq!(*layer, LayerId::FCu);
        assert!((area_mm2(filled) - 400.0).abs() < 1.0);
    }

    #[test]
    fn clips_the_outline_to_the_board_edge() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 5.0)]; // board smaller than the drawn outline
        let node = Node::new();
        let islands = fill_zone(&outline, LayerId::FCu, NetId(1), &board, &node, &JlcpcbClearance);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };
        assert!((area_mm2(filled) - 100.0).abs() < 1.0, "must be clipped down to the 10x10mm board, not the 20x20mm drawn outline");
    }

    #[test]
    fn punches_a_clearance_hole_around_a_different_nets_pad() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        let pad_radius = MM;
        node.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), pad_radius)), net: Some(NetId(2)), layer: LayerId::FCu });

        let islands = fill_zone(&outline, LayerId::FCu, NetId(1), &board, &node, &JlcpcbClearance);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };

        assert!(!filled.contains_point(Point::new(0, 0)), "the pad's own footprint must read as excluded");
        let clearance_mm = JlcpcbClearance::PAD_TO_TRACK as f64 / MM as f64;
        let expected_hole_area = std::f64::consts::PI * (1.0 + clearance_mm).powi(2);
        // 20x20 minus the (pad radius + clearance) circle.
        let expected = 400.0 - expected_hole_area;
        assert!((area_mm2(filled) - expected).abs() < 2.0, "area {} should be close to {expected}", area_mm2(filled));
    }

    #[test]
    fn does_not_exclude_a_same_net_pad() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        node.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), MM)), net: Some(NetId(1)), layer: LayerId::FCu });

        let islands = fill_zone(&outline, LayerId::FCu, NetId(1), &board, &node, &JlcpcbClearance);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };
        assert!(filled.contains_point(Point::new(0, 0)), "a same-net pad must stay solidly connected, not excluded");
    }

    #[test]
    fn ignores_an_obstacle_on_a_different_layer() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        node.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), MM)), net: Some(NetId(2)), layer: LayerId::BCu });

        let islands = fill_zone(&outline, LayerId::FCu, NetId(1), &board, &node, &JlcpcbClearance);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };
        assert!(filled.contains_point(Point::new(0, 0)), "a different-layer obstacle must not affect this layer's fill");
    }

    #[test]
    fn excludes_a_track_running_through_the_zone() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        let mm = |v: f64| (v * MM as f64).round() as Unit;
        node.add(Item::Track {
            shape: Segment::new(Point::new(mm(-8.0), 0), Point::new(mm(8.0), 0), mm(0.2)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let islands = fill_zone(&outline, LayerId::FCu, NetId(1), &board, &node, &JlcpcbClearance);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };
        assert!(!filled.contains_point(Point::new(0, 0)), "the track's own path must read as excluded");
        assert!(filled.contains_point(Point::new(0, mm(5.0))), "well away from the track must still be filled");
    }

    #[test]
    fn excludes_a_mounting_hole_from_the_pour() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        node.add(Item::Hole { position: Point::new(0, 0), drill: 2 * MM });

        let islands = fill_zone(&outline, LayerId::FCu, NetId(1), &board, &node, &JlcpcbClearance);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };
        assert!(!filled.contains_point(Point::new(0, 0)), "the mounting hole's own footprint must read as excluded, same as a pad/via");
        assert!(
            !filled.contains_point(Point::new(0, 3 * MM / 2)),
            "1.5mm from center is past the 1mm drill wall but inside the 2mm screw-head keep-out -- the head must never sit on zone copper"
        );
        assert!(filled.contains_point(Point::new(0, 5 * MM)), "well away from the hole must still be filled");
    }

    #[test]
    fn returns_nothing_when_the_outline_does_not_touch_the_board_at_all() {
        let outline = square(100.0, 100.0, 5.0);
        let board = vec![square(0.0, 0.0, 10.0)];
        let node = Node::new();
        assert!(fill_zone(&outline, LayerId::FCu, NetId(1), &board, &node, &JlcpcbClearance).is_empty());
    }
}
