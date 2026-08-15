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
//! ([`thermal::GAP`]) plus pad-local spokes of width `spoke_width` when
//! at least [`crate::thermal_relief::MIN_FREE_SPOKE_DIRS`] spoke
//! directions are free. Crowded Thermals are a hard fill failure (no
//! silent solid). [`ZoneConnection::Solid`] pads and all same-net vias
//! stay fully flooded. After clipping to the board outline, pour copper
//! is inset by [`ZONE_EDGE_COMFORT_MARGIN`] (1.0 mm) from the absolute
//! routed edge so planes do not sit on the cut line (pads/tracks/vias
//! still use the harder 0.20 mm fab floor separately). Deliberately
//! **not** modelled: priority between multiple zones of different nets
//! overlapping each other (an existing [`Item::Zone`] is not itself
//! treated as an obstacle here -- only `Pad`/`Via`/`Track`/`Hole`).

use alladin_core::{thermal, Item, LayerId, NetId, Node, PadShape, RuleResolver, ZoneConnection};
use alladin_geom::fill;
use alladin_geom::{Circle, Point, Polygon, Unit, MM};

/// Pour keep-out from the absolute board outline. Same 1.0 mm comfort
/// as MCP routing's edge default (fab hard floor for pads/tracks/vias
/// remains [`alladin_core::JlcpcbDfm::COPPER_TO_ROUTED_EDGE`] = 0.20 mm).
pub const ZONE_EDGE_COMFORT_MARGIN: Unit = MM;

use crate::thermal_relief::{
    first_illegal_thermal_on_layer, free_spoke_directions_in_node, pad_extent_along, pad_local_x,
    self_exclude, MIN_FREE_SPOKE_DIRS,
};

/// Why [`fill_zone`] refused to produce copper.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FillZoneError {
    /// A same-net Thermal pad on this layer has fewer than
    /// [`MIN_FREE_SPOKE_DIRS`] free spoke corridors.
    IllegalThermal { center: Point },
}

impl std::fmt::Display for FillZoneError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FillZoneError::IllegalThermal { center } => write!(
                f,
                "thermal pad at ({:.2}, {:.2}) mm needs at least {MIN_FREE_SPOKE_DIRS} free spoke directions for a legal pour",
                center.x as f64 / 1_000_000.0,
                center.y as f64 / 1_000_000.0,
            ),
        }
    }
}

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
            Point::new(
                center.x + (radius as f64 * angle.cos()).round() as Unit,
                center.y + (radius as f64 * angle.sin()).round() as Unit,
            )
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
    Polygon::new(vec![
        corner(0.0, -hw),
        corner(hl, -hw),
        corner(hl, hw),
        corner(0.0, hw),
    ])
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
            points.push(Point::new(
                center.x + (radius as f64 * angle.cos()).round() as Unit,
                center.y + (radius as f64 * angle.sin()).round() as Unit,
            ));
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
        Item::Pad {
            shape: PadShape::Circle(c),
            ..
        } => Some(circle_polygon(c.center, c.radius + clearance)),
        Item::Pad {
            shape: PadShape::Polygon { outline, .. },
            ..
        } => fill::buffer(outline, clearance).into_iter().next(),
        Item::Via { shape, .. } => Some(circle_polygon(shape.center, shape.radius + clearance)),
        Item::Track { shape, .. } => Some(capsule_polygon(
            shape.a,
            shape.b,
            shape.width / 2 + clearance,
        )),
        // A mounting hole keeps the pour out of the full screw-head
        // circle (radius = drill diameter, matching
        // `alladin_core::hole_keepout_circle`), not just the drilled
        // barrel -- the screw head that will sit here must never land
        // on live zone copper.
        Item::Hole { position, drill } => Some(circle_polygon(*position, *drill + clearance)),
        Item::Zone { .. } => None,
    }
}

/// Gap obstacle + spoke stubs for one Thermal pad. Caller must have
/// already verified ≥[`MIN_FREE_SPOKE_DIRS`] free directions.
fn thermal_geometry_for_pad(
    item: &Item,
    layer: LayerId,
    node: &Node,
    spoke_width: Unit,
    exclude_centers: &[(Unit, Unit)],
) -> Option<(Polygon, Vec<Polygon>)> {
    let Item::Pad { shape, .. } = item else {
        return None;
    };
    let (ux, uy) = pad_local_x(shape);
    let dirs = [(ux, uy), (-ux, -uy), (-uy, ux), (uy, -ux)];
    let free: Vec<(f64, f64)> = dirs
        .into_iter()
        .filter(|(dx, dy)| {
            crate::thermal_relief::spoke_direction_free(
                shape,
                *dx,
                *dy,
                layer,
                spoke_width,
                node.iter(),
                exclude_centers,
            )
        })
        .collect();
    debug_assert!(free.len() >= MIN_FREE_SPOKE_DIRS);
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
/// disjoint filled island. Returns [`FillZoneError::IllegalThermal`] if
/// any same-net Thermal pad on `layer` has fewer than
/// [`MIN_FREE_SPOKE_DIRS`] free spoke corridors (no silent solid).
///
/// `exclude_for_center` supplies sibling/self centers skipped by the
/// spoke corridor probe (see [`crate::thermal_relief`]).
///
/// `spoke_width` is the pour-neck width for thermal spokes (see
/// [`thermal::spoke_width`] / `BoardDoc::thermal_spoke_width`).
pub fn fill_zone(
    outline: &Polygon,
    layer: LayerId,
    net: NetId,
    board_outline: &[Polygon],
    node: &Node,
    resolver: &dyn RuleResolver,
    spoke_width: Unit,
    exclude_for_center: &dyn Fn(Point) -> Vec<(Unit, Unit)>,
) -> Result<Vec<Item>, FillZoneError> {
    if let Some(center) =
        first_illegal_thermal_on_layer(node, layer, net, spoke_width, exclude_for_center)
    {
        return Err(FillZoneError::IllegalThermal { center });
    }

    let clipped = fill::intersection(std::slice::from_ref(outline), board_outline);
    if clipped.is_empty() {
        return Ok(Vec::new());
    }
    // Pull pour copper back from the absolute cut line (comfort margin),
    // matching MCP routing's default edge keep-out — not the tighter
    // 0.20 mm fab floor that pads/tracks/vias already enforce.
    let clipped_sealed: Vec<Polygon> = clipped
        .iter()
        .map(fill::FilledRegion::sealed)
        .flat_map(|sealed| fill::buffer(&sealed, -ZONE_EDGE_COMFORT_MARGIN))
        .collect();
    if clipped_sealed.is_empty() {
        return Ok(Vec::new());
    }

    // A zero-geometry stand-in purely to select `RuleResolver::clearance`'s
    // "zone vs. X" match arms (see `alladin_core::JlcpcbClearance::clearance`)
    // -- its own outline/net content is never inspected, only its variant.
    let zone_stub = Item::Zone {
        outline: Polygon::new(Vec::new()),
        layer,
        net: None,
    };

    let mut obstacles: Vec<Polygon> = node
        .iter()
        .filter(|item| item.net() != Some(net))
        .filter(|item| item_on_layer(item, layer))
        .filter_map(|item| obstacle_polygon(item, resolver.clearance(&zone_stub, item)))
        .collect();

    let mut spokes: Vec<Polygon> = Vec::new();
    for item in node
        .iter()
        .filter(|item| item.net() == Some(net))
        .filter(|item| item_on_layer(item, layer))
    {
        let Item::Pad {
            zone_connection,
            shape,
            ..
        } = item
        else {
            continue;
        };
        if *zone_connection != ZoneConnection::Thermal {
            continue;
        }
        let excludes = exclude_for_center(shape.center());
        let (gap, pad_spokes) = thermal_geometry_for_pad(item, layer, node, spoke_width, &excludes)
            .expect("thermal legality pre-checked");
        obstacles.push(gap);
        spokes.extend(pad_spokes);
    }

    let obstacle_union: Vec<Polygon> = fill::union(&obstacles)
        .iter()
        .map(fill::FilledRegion::sealed)
        .collect();

    let mut base: Vec<Polygon> = clipped_sealed
        .iter()
        .flat_map(|region| fill::difference(std::slice::from_ref(region), &obstacle_union))
        .map(|island| island.sealed())
        .collect();

    if !spokes.is_empty() {
        let mut with_spokes = base;
        with_spokes.extend(spokes);
        let united: Vec<Polygon> = fill::union(&with_spokes)
            .iter()
            .map(fill::FilledRegion::sealed)
            .collect();
        base = fill::intersection(&united, &clipped_sealed)
            .into_iter()
            .map(|r| r.sealed())
            .collect();
    }

    Ok(base
        .into_iter()
        .map(|outline| Item::Zone {
            outline,
            layer,
            net: Some(net),
        })
        .collect())
}

/// [`fill_zone`] with self-center-only excludes (no multi-pad siblings).
pub fn fill_zone_simple(
    outline: &Polygon,
    layer: LayerId,
    net: NetId,
    board_outline: &[Polygon],
    node: &Node,
    resolver: &dyn RuleResolver,
    spoke_width: Unit,
) -> Result<Vec<Item>, FillZoneError> {
    fill_zone(
        outline,
        layer,
        net,
        board_outline,
        node,
        resolver,
        spoke_width,
        &|c| self_exclude(c),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::{JlcpcbClearance, NetClass};
    use alladin_geom::{Segment, MM};

    fn default_spoke() -> Unit {
        thermal::spoke_width(alladin_core::JlcpcbDfm::MIN_TRACK_WIDTH)
    }

    fn fill(outline: &Polygon, board: &[Polygon], node: &Node) -> Result<Vec<Item>, FillZoneError> {
        fill_zone_simple(
            outline,
            LayerId::FCu,
            NetId(1),
            board,
            node,
            &JlcpcbClearance,
            default_spoke(),
        )
    }

    fn fill_on(
        outline: &Polygon,
        board: &[Polygon],
        node: &Node,
        layer: LayerId,
    ) -> Result<Vec<Item>, FillZoneError> {
        fill_zone_simple(
            outline,
            layer,
            NetId(1),
            board,
            node,
            &JlcpcbClearance,
            default_spoke(),
        )
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
        let islands = fill(&outline, &board, &node).unwrap();
        assert_eq!(islands.len(), 1);
        let Item::Zone {
            outline: filled,
            net,
            layer,
        } = &islands[0]
        else {
            panic!("expected a zone")
        };
        assert_eq!(*net, Some(NetId(1)));
        assert_eq!(*layer, LayerId::FCu);
        // 20×20 outline inset by 1 mm comfort → ~18×18 (round join shaves corners).
        let expected = 18.0 * 18.0;
        assert!(
            (area_mm2(filled) - expected).abs() < 8.0,
            "area {} should be near {expected}",
            area_mm2(filled)
        );
        assert!(filled.contains_point(Point::new(0, 0)));
    }

    #[test]
    fn clips_the_outline_to_the_board_edge() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 5.0)]; // board smaller than the drawn outline
        let node = Node::new();
        let islands = fill(&outline, &board, &node).unwrap();
        assert_eq!(islands.len(), 1);
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };
        // Clipped to 10×10 then inset 1 mm → ~8×8.
        let expected = 8.0 * 8.0;
        assert!(
            (area_mm2(filled) - expected).abs() < 6.0,
            "must clip to the board then apply comfort inset, area {} vs {expected}",
            area_mm2(filled)
        );
        assert!(filled.contains_point(Point::new(0, 0)));
        assert!(
            !filled.contains_point(Point::new((4.7 * MM as f64) as Unit, 0)),
            "comfort margin must clear copper near the clipped board edge"
        );
    }

    #[test]
    fn keeps_comfort_margin_from_absolute_board_edge() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 10.0)];
        let node = Node::new();
        let islands = fill(&outline, &board, &node).unwrap();
        assert_eq!(islands.len(), 1);
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };
        assert!(filled.contains_point(Point::new(0, 0)));
        assert!(
            filled.contains_point(Point::new((8 * MM) as Unit, 0)),
            "copper should remain well inside the comfort inset"
        );
        assert!(
            !filled.contains_point(Point::new((9.5 * MM as f64) as Unit, 0)),
            "pour must not enter the 1 mm comfort keep-out at the absolute board edge"
        );
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
            hole_diameter: None,
        });

        let islands = fill(&outline, &board, &node).unwrap();
        assert_eq!(islands.len(), 1);
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };

        assert!(
            !filled.contains_point(Point::new(0, 0)),
            "the pad's own footprint must read as excluded"
        );
        let clearance_mm = JlcpcbClearance::PAD_TO_TRACK as f64 / MM as f64;
        let expected_hole_area = std::f64::consts::PI * (1.0 + clearance_mm).powi(2);
        // Comfort-inset ~18×18 minus the (pad radius + clearance) circle.
        let expected = 18.0 * 18.0 - expected_hole_area;
        assert!(
            (area_mm2(filled) - expected).abs() < 10.0,
            "area {} should be close to {expected}",
            area_mm2(filled)
        );
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
            hole_diameter: None,
        });

        let islands = fill(&outline, &board, &node).unwrap();
        assert_eq!(islands.len(), 1);
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };
        assert!(
            filled.contains_point(Point::new(0, 0)),
            "a solid same-net pad must stay solidly connected, not excluded"
        );
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
            hole_diameter: None,
        });

        let islands = fill(&outline, &board, &node).unwrap();
        assert_eq!(islands.len(), 1);
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };

        // Mid-gap on the diagonal (between spokes): must be empty copper.
        let gap_mid = pad_radius + thermal::GAP / 2;
        let diag = ((gap_mid as f64) / std::f64::consts::SQRT_2).round() as Unit;
        assert!(
            !filled.contains_point(Point::new(diag, diag)),
            "thermal gap between spokes must clear the pour"
        );
        // On the +X spoke, just outside the pad: must be pour copper.
        let on_spoke = Point::new(pad_radius + thermal::GAP / 2, 0);
        assert!(
            filled.contains_point(on_spoke),
            "thermal spoke must reconnect pad to pour"
        );
        // Far from the pad: pour remains.
        assert!(filled.contains_point(Point::new(5 * MM, 5 * MM)));
    }

    #[test]
    fn crowded_thermals_refuse_fill_not_silent_solid() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        let r = MM / 2; // 0.5 mm
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
                hole_diameter: None,
            });
        }

        let err = fill(&outline, &board, &node).unwrap_err();
        assert!(matches!(err, FillZoneError::IllegalThermal { .. }));
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
            .map(|(x, y)| {
                Point::new(
                    (x * c - y * s).round() as Unit,
                    (x * s + y * c).round() as Unit,
                )
            })
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
            hole_diameter: None,
        });

        let islands = fill(&outline, &board, &node).unwrap();
        assert_eq!(islands.len(), 1);
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };

        // Local +X is first edge direction: (-hw,-hh)->(hw,-hh) rotated 45°
        // → (cos, sin) = (√2/2, √2/2).
        let ux = std::f64::consts::FRAC_1_SQRT_2;
        let uy = ux;
        let extent = pad_extent_along(&shape, ux, uy);
        let on_local = Point::new(
            ((extent + thermal::GAP / 2) as f64 * ux).round() as Unit,
            ((extent + thermal::GAP / 2) as f64 * uy).round() as Unit,
        );
        assert!(
            filled.contains_point(on_local),
            "spoke must follow rotated pad local axis"
        );

        // World +X mid-gap (between local axes at 45°): should be empty.
        let on_world = Point::new(extent + thermal::GAP / 2, 0);
        assert!(
            !filled.contains_point(on_world),
            "diagonal/world gap between local spokes must stay empty"
        );
    }

    #[test]
    fn sibling_pads_same_pin_do_not_block_each_other() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        let r = MM / 2;
        // Two paste openings of one pin, 1.05 mm apart — would block facing
        // spokes if counted as obstacles, but siblings must be excluded.
        let a = Point::new(0, 0);
        let b = Point::new((1.05 * MM as f64) as Unit, 0);
        node.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(a, r)),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
            hole_diameter: None,
        });
        node.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(b, r)),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
            hole_diameter: None,
        });
        let siblings = [(a.x, a.y), (b.x, b.y)];
        let islands = fill_zone(
            &outline,
            LayerId::FCu,
            NetId(1),
            &board,
            &node,
            &JlcpcbClearance,
            default_spoke(),
            &|c| {
                if siblings.iter().any(|&(x, y)| x == c.x && y == c.y) {
                    siblings.to_vec()
                } else {
                    self_exclude(c)
                }
            },
        )
        .unwrap();
        assert_eq!(islands.len(), 1);
        // With siblings excluded each pad still has ≥2 free dirs (+Y/-Y at least).
        let shape_a = PadShape::Circle(Circle::new(a, r));
        assert!(
            free_spoke_directions_in_node(
                &shape_a,
                LayerId::FCu,
                default_spoke(),
                &node,
                &siblings
            ) >= MIN_FREE_SPOKE_DIRS
        );
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
            hole_diameter: None,
        });

        let islands = fill(&outline, &board, &node).unwrap();
        assert_eq!(islands.len(), 1);
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };
        assert!(
            filled.contains_point(Point::new(0, 0)),
            "a different-layer obstacle must not affect this layer's fill"
        );
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

        let islands = fill(&outline, &board, &node).unwrap();
        assert_eq!(islands.len(), 1);
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };
        assert!(
            !filled.contains_point(Point::new(0, 0)),
            "a different-net track must clear a corridor through the pour"
        );
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
        let islands = fill(&outline, &board, &node).unwrap();
        assert_eq!(islands.len(), 1);
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };
        assert!(
            filled.contains_point(Point::new(0, 0)),
            "same-net vias stay solid (stitching)"
        );
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
            hole_diameter: None,
        });
        assert!(fill(&outline, &board, &node).unwrap().is_empty());
    }

    #[test]
    fn spoke_width_respects_min_track_floor() {
        assert_eq!(thermal::spoke_width(100_000), thermal::SPOKE_WIDTH);
        assert_eq!(thermal::spoke_width(160_000), thermal::SPOKE_WIDTH);
        assert_eq!(thermal::spoke_width(250_000), 250_000);
    }

    fn pth_pad(net: NetId, zone_connection: ZoneConnection) -> Item {
        Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), MM)),
            net: Some(net),
            layer: LayerId::FCu,
            zone_connection,
            hole_diameter: Some(MM),
        }
    }

    #[test]
    fn pth_thermal_pad_gets_relief_on_both_copper_layers() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        node.add(pth_pad(NetId(1), ZoneConnection::Thermal));

        let pad_radius = MM;
        let gap_mid = pad_radius + thermal::GAP / 2;
        let diag = ((gap_mid as f64) / std::f64::consts::SQRT_2).round() as Unit;
        let on_spoke = Point::new(pad_radius + thermal::GAP / 2, 0);
        for layer in [LayerId::FCu, LayerId::BCu] {
            let islands = fill_on(&outline, &board, &node, layer).unwrap();
            assert_eq!(islands.len(), 1, "{layer:?}");
            let Item::Zone {
                outline: filled, ..
            } = &islands[0]
            else {
                panic!("expected a zone")
            };
            assert!(
                !filled.contains_point(Point::new(diag, diag)),
                "thermal PTH gap must clear the pour on {layer:?}"
            );
            assert!(
                filled.contains_point(on_spoke),
                "thermal PTH spoke must reconnect on {layer:?}"
            );
        }
    }

    #[test]
    fn pth_solid_pad_stays_flooded_on_back_copper() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        node.add(pth_pad(NetId(1), ZoneConnection::Solid));
        let islands = fill_on(&outline, &board, &node, LayerId::BCu).unwrap();
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };
        assert!(
            filled.contains_point(Point::new(0, 0)),
            "solid PTH must flood on B.Cu"
        );
    }

    #[test]
    fn pth_foreign_net_is_cleared_from_a_back_copper_pour() {
        let outline = square(0.0, 0.0, 10.0);
        let board = vec![square(0.0, 0.0, 50.0)];
        let mut node = Node::new();
        node.add(pth_pad(NetId(2), ZoneConnection::Thermal));
        let islands = fill_on(&outline, &board, &node, LayerId::BCu).unwrap();
        let Item::Zone {
            outline: filled, ..
        } = &islands[0]
        else {
            panic!("expected a zone")
        };
        assert!(
            !filled.contains_point(Point::new(0, 0)),
            "a different-net PTH must not short into a B.Cu plane"
        );
    }
}
