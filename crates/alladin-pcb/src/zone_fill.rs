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
//! Same-net pads with [`ZoneConnection::Thermal`] get an annular gap
//! ([`thermal::GAP`]) plus pad-local spokes of width `spoke_width`, but
//! only when at least two spoke directions have room for a pour neck
//! (`GAP + spoke_width` clear of any same-layer copper, including
//! same-net). Crowded pads fall back to solid flood for that fill
//! (property stays Thermal). [`ZoneConnection::Solid`] pads and all
//! same-net vias stay fully flooded. Deliberately **not** modelled:
//! priority between multiple zones of different nets overlapping each
//! other (an existing [`Item::Zone`] is not itself treated as an
//! obstacle here -- only `Pad`/`Via`/`Track`/`Hole`).

use alladin_core::{thermal, Item, LayerId, NetId, Node, PadShape, RuleResolver, ZoneConnection};
use alladin_geom::{
    circle_circle_collides, circle_polygon_collides, circle_segment_collides, Circle, Point, Polygon, Unit,
};
use alladin_geom::fill;

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

/// Oriented rectangle from `center` along unit vector `(ux, uy)` for
/// `length` (full extent from center), width `width` (full).
fn oriented_spoke_stub(center: Point, ux: f64, uy: f64, length: Unit, width: Unit) -> Polygon {
    let px = -uy;
    let py = ux;
    let hl = length as f64;
    let hw = (width as f64) / 2.0;
    let corner = |along: f64, across: f64| {
        Point::new(
            center.x + (along * ux + across * px).round() as Unit,
            center.y + (along * uy + across * py).round() as Unit,
        )
    };
    Polygon::new(vec![corner(0.0, -hw), corner(hl, -hw), corner(hl, hw), corner(0.0, hw)])
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

/// Pad-local unit X axis: world +X for circles; direction of the first
/// outline edge for polygons (so spokes follow a rotated rect's sides).
fn pad_local_x(shape: &PadShape) -> (f64, f64) {
    match shape {
        PadShape::Circle(_) => (1.0, 0.0),
        PadShape::Polygon { outline, .. } => {
            if outline.points.len() >= 2 {
                let a = outline.points[0];
                let b = outline.points[1];
                let dx = (b.x - a.x) as f64;
                let dy = (b.y - a.y) as f64;
                let len = (dx * dx + dy * dy).sqrt();
                if len > 1.0 {
                    return (dx / len, dy / len);
                }
            }
            (1.0, 0.0)
        }
    }
}

/// Half-extent of `shape` from its center along unit vector `(ux, uy)`.
fn pad_extent_along(shape: &PadShape, ux: f64, uy: f64) -> Unit {
    match shape {
        PadShape::Circle(c) => c.radius,
        PadShape::Polygon { outline, .. } => {
            let c = shape.center();
            outline
                .points
                .iter()
                .map(|p| {
                    let dx = (p.x - c.x) as f64;
                    let dy = (p.y - c.y) as f64;
                    (dx * ux + dy * uy).abs().round() as Unit
                })
                .max()
                .unwrap_or(0)
        }
    }
}

fn copper_hits_probe(probe: &Circle, item: &Item) -> bool {
    match item {
        Item::Pad { shape: PadShape::Circle(c), .. } => circle_circle_collides(probe, c, 0),
        Item::Pad { shape: PadShape::Polygon { outline, .. }, .. } => circle_polygon_collides(probe, outline, 0),
        Item::Via { shape, .. } => circle_circle_collides(probe, shape, 0),
        Item::Track { shape, .. } => circle_segment_collides(probe, shape, 0),
        Item::Hole { position, drill } => {
            // Match fill obstacle: screw-head keep-out radius = drill diameter.
            circle_circle_collides(probe, &Circle::new(*position, *drill), 0)
        }
        Item::Zone { .. } => false,
    }
}

/// Whether a spoke stub along `(ux, uy)` has a pour neck: from the pad
/// edge outward, no other same-layer copper (incl. same-net) closer than
/// `GAP + spoke_width`.
fn spoke_direction_free(shape: &PadShape, ux: f64, uy: f64, layer: LayerId, node: &Node, spoke_width: Unit) -> bool {
    let center = shape.center();
    let extent = pad_extent_along(shape, ux, uy);
    let probe_r = spoke_width / 2;
    let dist = extent + thermal::GAP + probe_r;
    let probe = Circle::new(
        Point::new(center.x + (dist as f64 * ux).round() as Unit, center.y + (dist as f64 * uy).round() as Unit),
        probe_r,
    );
    for item in node.iter().filter(|item| item_on_layer(item, layer)) {
        // Skip the pad under consideration (same center).
        if let Item::Pad { shape: other, .. } = item {
            let oc = other.center();
            if oc.x == center.x && oc.y == center.y {
                continue;
            }
        }
        if copper_hits_probe(&probe, item) {
            return false;
        }
    }
    true
}

/// Collect gap obstacle + spoke stubs for one Thermal pad, or `None` if
/// fewer than two directions are free (caller treats the pad as solid).
fn adaptive_thermal_for_pad(item: &Item, layer: LayerId, node: &Node, spoke_width: Unit) -> Option<(Polygon, Vec<Polygon>)> {
    let Item::Pad { shape, .. } = item else { return None };
    let (ux, uy) = pad_local_x(shape);
    let dirs = [(ux, uy), (-ux, -uy), (-uy, ux), (uy, -ux)];
    let free: Vec<(f64, f64)> = dirs.into_iter().filter(|(dx, dy)| spoke_direction_free(shape, *dx, *dy, layer, node, spoke_width)).collect();
    if free.len() < 2 {
        return None;
    }
    let gap = obstacle_polygon(item, thermal::GAP)?;
    let center = shape.center();
    let spokes: Vec<Polygon> = free
        .into_iter()
        .map(|(dx, dy)| {
            let half = pad_extent_along(shape, dx, dy) + thermal::GAP;
            oriented_spoke_stub(center, dx, dy, half, spoke_width)
        })
        .collect();
    Some((gap, spokes))
}

/// Fills a user-drawn `outline` on `layer` for `net` against the current
/// board state, returning zero or more [`Item::Zone`]s -- one per
/// disjoint filled island, matching how a KiCad-compatible
/// `filled_polygon` list maps to one `Item::Zone` per entry. Returns an
/// empty `Vec` if the outline doesn't overlap the board outline at all,
/// or if obstacles consume the entire clipped area.
///
/// `spoke_width` is the pour-neck width for thermal spokes (see
/// [`thermal::spoke_width`] / `BoardDoc::thermal_spoke_width`).
///
/// Pipeline:
/// 1. Clip `outline` to `board_outline`.
/// 2. Buffer every same-layer, different-net `Pad`/`Via`/`Track`/`Hole`
///    by its resolver clearance; also buffer same-net
///    [`ZoneConnection::Thermal`] pads by [`thermal::GAP`] when adaptive
///    thermals keep them thermal (≥2 free spoke directions).
/// 3. Subtract the obstacle union from the clipped area.
/// 4. Union thermal spoke stubs back in, then re-clip to the
///    board-clipped outline so spokes never leave the pour.
/// 5. Seal holes into each island via [`fill::FilledRegion::sealed`].
pub fn fill_zone(
    outline: &Polygon,
    layer: LayerId,
    net: NetId,
    board_outline: &[Polygon],
    node: &Node,
    resolver: &dyn RuleResolver,
    spoke_width: Unit,
) -> Vec<Item> {
    let clipped = fill::intersection(std::slice::from_ref(outline), board_outline);
    if clipped.is_empty() {
        return Vec::new();
    }
    let clipped_sealed: Vec<Polygon> = clipped.iter().map(fill::FilledRegion::sealed).collect();

    // A zero-geometry stand-in purely to select `RuleResolver::clearance`'s
    // "zone vs. X" match arms (see `alladin_core::JlcpcbClearance::clearance`)
    // -- its own outline/net content is never inspected, only its variant.
    let zone_stub = Item::Zone { outline: Polygon::new(Vec::new()), layer, net: None };

    let mut obstacles: Vec<Polygon> = node
        .iter()
        .filter(|item| item.net() != Some(net))
        .filter(|item| item_on_layer(item, layer))
        .filter_map(|item| obstacle_polygon(item, resolver.clearance(&zone_stub, item)))
        .collect();

    let mut spokes: Vec<Polygon> = Vec::new();
    for item in node.iter().filter(|item| item.net() == Some(net)).filter(|item| item_on_layer(item, layer)) {
        let Item::Pad { zone_connection, .. } = item else { continue };
        if *zone_connection != ZoneConnection::Thermal {
            continue;
        }
        if let Some((gap, pad_spokes)) = adaptive_thermal_for_pad(item, layer, node, spoke_width) {
            obstacles.push(gap);
            spokes.extend(pad_spokes);
        }
        // else: solid-like for this fill — no gap, no spokes
    }

    let obstacle_union: Vec<Polygon> = fill::union(&obstacles).iter().map(fill::FilledRegion::sealed).collect();

    let mut base: Vec<Polygon> = clipped_sealed
        .iter()
        .flat_map(|region| fill::difference(std::slice::from_ref(region), &obstacle_union))
        .map(|island| island.sealed())
        .collect();

    if !spokes.is_empty() {
        let mut with_spokes = base;
        with_spokes.extend(spokes);
        let united: Vec<Polygon> = fill::union(&with_spokes).iter().map(fill::FilledRegion::sealed).collect();
        base = fill::intersection(&united, &clipped_sealed).into_iter().map(|r| r.sealed()).collect();
    }

    base.into_iter().map(|outline| Item::Zone { outline, layer, net: Some(net) }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::{JlcpcbClearance, NetClass};
    use alladin_geom::{Circle, Segment, MM};

    fn default_spoke() -> Unit {
        thermal::spoke_width(alladin_core::JlcpcbDfm::MIN_TRACK_WIDTH)
    }

    fn fill(outline: &Polygon, board: &[Polygon], node: &Node) -> Vec<Item> {
        fill_zone(outline, LayerId::FCu, NetId(1), board, node, &JlcpcbClearance, default_spoke())
    }

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
        let islands = fill(&outline, &board, &node);
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
        let islands = fill(&outline, &board, &node);
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
        node.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), pad_radius)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });

        let islands = fill(&outline, &board, &node);
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
    fn solid_same_net_pad_stays_flooded() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        node.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), MM)),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Solid,
        });

        let islands = fill(&outline, &board, &node);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };
        assert!(filled.contains_point(Point::new(0, 0)), "a solid same-net pad must stay solidly connected, not excluded");
    }

    #[test]
    fn thermal_same_net_pad_has_gap_but_spoke_connects() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        let pad_radius = MM;
        node.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), pad_radius)),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });

        let islands = fill(&outline, &board, &node);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };

        // Mid-gap on the diagonal (between spokes): must be empty copper.
        let gap_mid = pad_radius + thermal::GAP / 2;
        let diag = ((gap_mid as f64) / std::f64::consts::SQRT_2).round() as Unit;
        assert!(!filled.contains_point(Point::new(diag, diag)), "thermal gap between spokes must clear the pour");
        // On the +X spoke, just outside the pad: must be pour copper.
        let on_spoke = Point::new(pad_radius + thermal::GAP / 2, 0);
        assert!(filled.contains_point(on_spoke), "thermal spoke must reconnect pad to pour");
        // Far from the pad: pour remains.
        assert!(filled.contains_point(Point::new(5 * MM, 5 * MM)));
    }

    #[test]
    fn crowded_thermal_pads_fall_back_to_solid() {
        // Edge-to-edge gap < 2*GAP + spoke → facing directions blocked;
        // with only the shared axis free on each side of a close pair,
        // free count can drop below 2 when a third pad hems them in —
        // simplest: two pads so close the probes collide with the neighbour
        // in every cardinal direction that matters. Place them almost
        // touching so all four probes on each pad hit the other pad.
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        let r = MM / 2; // 0.5 mm
        // Center distance 1.05 mm → edge gap 0.05 mm ≪ GAP+spoke (0.40).
        // Probes along ±X hit the neighbour; ±Y still free → 2 free → still thermal.
        // Surround with four neighbours so every direction is blocked.
        let centers = [
            Point::new(0, 0),
            Point::new((1.05 * MM as f64) as Unit, 0),
            Point::new(-(1.05 * MM as f64) as Unit, 0),
            Point::new(0, (1.05 * MM as f64) as Unit),
            Point::new(0, -(1.05 * MM as f64) as Unit),
        ];
        for c in centers {
            node.add(Item::Pad {
                shape: PadShape::Circle(Circle::new(c, r)),
                net: Some(NetId(1)),
                layer: LayerId::FCu,
                zone_connection: ZoneConnection::Thermal,
            });
        }

        let islands = fill(&outline, &board, &node);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };
        // Center pad: solid-like → mid-gap on diagonal must be pour (no gap).
        let gap_mid = r + thermal::GAP / 2;
        let diag = ((gap_mid as f64) / std::f64::consts::SQRT_2).round() as Unit;
        assert!(
            filled.contains_point(Point::new(diag, diag)),
            "crowded thermal pad must fall back to solid (no gap on diagonal)"
        );
        assert!(filled.contains_point(Point::new(0, 0)));
    }

    #[test]
    fn rotated_rect_thermal_follows_pad_axes() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        // 1.0 x 0.6 mm rect rotated 45° — local X follows first edge.
        let angle = std::f64::consts::FRAC_PI_4;
        let (c, s) = (angle.cos(), angle.sin());
        let hw = 0.5 * MM as f64;
        let hh = 0.3 * MM as f64;
        let corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)];
        let points: Vec<Point> = corners
            .into_iter()
            .map(|(x, y)| Point::new((x * c - y * s).round() as Unit, (x * s + y * c).round() as Unit))
            .collect();
        let shape = PadShape::Polygon {
            outline: Polygon::new(points),
            center: Point::new(0, 0),
        };
        node.add(Item::Pad {
            shape: shape.clone(),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });

        let islands = fill(&outline, &board, &node);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };

        // Local +X is first edge direction: (-hw,-hh)->(hw,-hh) rotated 45°
        // → (cos, sin) = (√2/2, √2/2).
        let ux = std::f64::consts::FRAC_1_SQRT_2;
        let uy = ux;
        let extent = pad_extent_along(&shape, ux, uy);
        let on_local = Point::new(
            ((extent + thermal::GAP / 2) as f64 * ux).round() as Unit,
            ((extent + thermal::GAP / 2) as f64 * uy).round() as Unit,
        );
        assert!(filled.contains_point(on_local), "spoke must follow rotated pad local axis");

        // World +X mid-gap (between local axes at 45°): should be empty.
        let on_world = Point::new(extent + thermal::GAP / 2, 0);
        assert!(!filled.contains_point(on_world), "diagonal/world gap between local spokes must stay empty");
    }

    #[test]
    fn ignores_an_obstacle_on_a_different_layer() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        node.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), MM)),
            net: Some(NetId(2)),
            layer: LayerId::BCu,
            zone_connection: ZoneConnection::Thermal,
        });

        let islands = fill(&outline, &board, &node);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };
        assert!(filled.contains_point(Point::new(0, 0)), "a different-layer obstacle must not affect this layer's fill");
    }

    #[test]
    fn excludes_a_track_running_through_the_zone() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        node.add(Item::Track {
            shape: Segment::new(Point::new(-5 * MM, 0), Point::new(5 * MM, 0), MM / 5),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let islands = fill(&outline, &board, &node);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };
        assert!(!filled.contains_point(Point::new(0, 0)), "a different-net track must clear a corridor through the pour");
    }

    #[test]
    fn same_net_via_stays_solidly_connected() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        node.add(Item::Via {
            shape: Circle::new(Point::new(0, 0), 400_000),
            drill: 200_000,
            net: Some(NetId(1)),
        });
        let islands = fill(&outline, &board, &node);
        assert_eq!(islands.len(), 1);
        let Item::Zone { outline: filled, .. } = &islands[0] else { panic!("expected a zone") };
        assert!(filled.contains_point(Point::new(0, 0)), "same-net vias stay solid (stitching)");
    }

    #[test]
    fn returns_empty_when_obstacles_consume_the_entire_clipped_area() {
        let outline = square(0.0, 0.0, 1.0);
        let board = vec![square(0.0, 0.0, 1.0)];
        let mut node = Node::new();
        // Huge different-net pad covering the whole tiny zone.
        node.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), 5 * MM)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });
        assert!(fill(&outline, &board, &node).is_empty());
    }

    #[test]
    fn spoke_width_respects_min_track_floor() {
        assert_eq!(thermal::spoke_width(100_000), thermal::SPOKE_WIDTH);
        assert_eq!(thermal::spoke_width(160_000), thermal::SPOKE_WIDTH);
        assert_eq!(thermal::spoke_width(250_000), 250_000);
    }
}
