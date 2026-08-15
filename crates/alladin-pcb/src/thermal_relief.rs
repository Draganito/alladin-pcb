//! Correct-by-construction thermal relief freedom checks.
//!
//! A pad with [`ZoneConnection::Thermal`] may only exist where a zone fill
//! can produce real thermals: at least [`MIN_FREE_SPOKE_DIRS`] pad-local
//! spoke corridors clear of same-layer copper (pads, vias, tracks, holes),
//! including same-net copper. Sibling pads of the same footprint pin
//! number are excluded from each other's probes.

use alladin_core::{thermal, Item, LayerId, NetId, Node, PadShape, ZoneConnection};
use alladin_geom::{
    circle_segment_collides, segment_polygon_collides, segment_segment_collides, Circle, Point,
    Segment, Unit,
};

/// Minimum free spoke directions for a legal Thermal pad.
pub const MIN_FREE_SPOKE_DIRS: usize = 2;

fn item_on_layer(item: &Item, layer: LayerId) -> bool {
    match item.layers() {
        (a, None) => a == layer,
        (a, Some(b)) => a == layer || b == layer,
    }
}

/// Pad-local unit X axis: world +X for circles; direction of the first
/// outline edge for polygons (so spokes follow a rotated rect's sides).
pub fn pad_local_x(shape: &PadShape) -> (f64, f64) {
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
pub fn pad_extent_along(shape: &PadShape, ux: f64, uy: f64) -> Unit {
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

fn center_key(p: Point) -> (Unit, Unit) {
    (p.x, p.y)
}

fn is_excluded_pad(item: &Item, exclude_centers: &[(Unit, Unit)]) -> bool {
    let Item::Pad { shape, .. } = item else {
        return false;
    };
    let c = shape.center();
    exclude_centers.iter().any(|&(x, y)| x == c.x && y == c.y)
}

/// Whether a track terminates on `pad_center` (endpoint coincides with the
/// pad). Such stubs are the pad's own copper connection — their end-cap
/// always overlaps every spoke corridor root and must not count as a
/// pour-neck obstacle for this pad.
fn track_terminates_on_pad(track: &Segment, pad_center: Point) -> bool {
    (track.a.x == pad_center.x && track.a.y == pad_center.y)
        || (track.b.x == pad_center.x && track.b.y == pad_center.y)
}

/// Whether `item` copper intersects the spoke corridor capsule (`corridor`
/// is the axis segment with width = spoke width). `pad_center` is the pad
/// under consideration (tracks that terminate there are skipped).
fn copper_hits_corridor(corridor: &Segment, item: &Item, pad_center: Point) -> bool {
    match item {
        Item::Pad {
            shape: PadShape::Circle(c),
            ..
        } => circle_segment_collides(c, corridor, 0),
        Item::Pad {
            shape: PadShape::Polygon { outline, .. },
            ..
        } => segment_polygon_collides(corridor, outline, 0),
        Item::Via { shape, .. } => circle_segment_collides(shape, corridor, 0),
        Item::Track { shape, .. } => {
            if track_terminates_on_pad(shape, pad_center) {
                return false;
            }
            segment_segment_collides(corridor, shape, 0)
        }
        Item::Hole { position, drill } => {
            // Match fill obstacle: screw-head keep-out radius = drill diameter.
            circle_segment_collides(&Circle::new(*position, *drill), corridor, 0)
        }
        Item::Zone { .. } => false,
    }
}

/// Corridor along `(ux, uy)` from the pad edge outward over `GAP + spoke_width`,
/// width `spoke_width`.
fn spoke_corridor(shape: &PadShape, ux: f64, uy: f64, spoke_width: Unit) -> Segment {
    let center = shape.center();
    let extent = pad_extent_along(shape, ux, uy);
    let len = thermal::GAP + spoke_width;
    let a = Point::new(
        center.x + (extent as f64 * ux).round() as Unit,
        center.y + (extent as f64 * uy).round() as Unit,
    );
    let b = Point::new(
        center.x + ((extent + len) as f64 * ux).round() as Unit,
        center.y + ((extent + len) as f64 * uy).round() as Unit,
    );
    Segment::new(a, b, spoke_width)
}

/// Whether a spoke stub along `(ux, uy)` has a clear pour neck.
pub fn spoke_direction_free<'a>(
    shape: &PadShape,
    ux: f64,
    uy: f64,
    layer: LayerId,
    spoke_width: Unit,
    obstacles: impl Iterator<Item = &'a Item>,
    exclude_centers: &[(Unit, Unit)],
) -> bool {
    let pad_center = shape.center();
    let corridor = spoke_corridor(shape, ux, uy, spoke_width);
    for item in obstacles.filter(|item| item_on_layer(item, layer)) {
        if is_excluded_pad(item, exclude_centers) {
            continue;
        }
        if copper_hits_corridor(&corridor, item, pad_center) {
            return false;
        }
    }
    true
}

/// Count of free pad-local spoke directions (0..=4).
pub fn free_spoke_directions<'a>(
    shape: &PadShape,
    layer: LayerId,
    spoke_width: Unit,
    obstacles: impl Iterator<Item = &'a Item>,
    exclude_centers: &[(Unit, Unit)],
) -> usize {
    let (ux, uy) = pad_local_x(shape);
    let dirs = [(ux, uy), (-ux, -uy), (-uy, ux), (uy, -ux)];
    // Collect obstacles once — callers often pass node.iter().
    let items: Vec<&Item> = obstacles.collect();
    dirs.into_iter()
        .filter(|(dx, dy)| {
            spoke_direction_free(
                shape,
                *dx,
                *dy,
                layer,
                spoke_width,
                items.iter().copied(),
                exclude_centers,
            )
        })
        .count()
}

/// Convenience over a live [`Node`].
pub fn free_spoke_directions_in_node(
    shape: &PadShape,
    layer: LayerId,
    spoke_width: Unit,
    node: &Node,
    exclude_centers: &[(Unit, Unit)],
) -> usize {
    free_spoke_directions(shape, layer, spoke_width, node.iter(), exclude_centers)
}

/// Whether this Thermal pad has enough free spoke room.
pub fn thermal_pad_is_legal(
    shape: &PadShape,
    layer: LayerId,
    spoke_width: Unit,
    node: &Node,
    exclude_centers: &[(Unit, Unit)],
) -> bool {
    free_spoke_directions_in_node(shape, layer, spoke_width, node, exclude_centers)
        >= MIN_FREE_SPOKE_DIRS
}

/// First same-net Thermal on `layer` that fails the ≥2-free-dirs rule, if any.
pub fn first_illegal_thermal_on_layer(
    node: &Node,
    layer: LayerId,
    net: NetId,
    spoke_width: Unit,
    exclude_for_center: &dyn Fn(Point) -> Vec<(Unit, Unit)>,
) -> Option<Point> {
    for item in node.iter().filter(|item| item_on_layer(item, layer)) {
        let Item::Pad {
            shape,
            zone_connection,
            net: pad_net,
            ..
        } = item
        else {
            continue;
        };
        if *zone_connection != ZoneConnection::Thermal || *pad_net != Some(net) {
            continue;
        }
        let center = shape.center();
        let excludes = exclude_for_center(center);
        if !thermal_pad_is_legal(shape, layer, spoke_width, node, &excludes) {
            return Some(center);
        }
    }
    None
}

/// Every Thermal pad on the board (any net/layer) must stay legal.
pub fn first_illegal_thermal_anywhere(
    node: &Node,
    spoke_width: Unit,
    exclude_for_center: &dyn Fn(Point) -> Vec<(Unit, Unit)>,
) -> Option<(Point, LayerId)> {
    for item in node.iter() {
        let Item::Pad {
            shape,
            zone_connection,
            ..
        } = item
        else {
            continue;
        };
        if *zone_connection != ZoneConnection::Thermal {
            continue;
        }
        let center = shape.center();
        let excludes = exclude_for_center(center);
        let (a, b) = item.layers();
        for layer in std::iter::once(a).chain(b) {
            if !thermal_pad_is_legal(shape, layer, spoke_width, node, &excludes) {
                return Some((center, layer));
            }
        }
    }
    None
}

/// Default exclude list: just the pad's own center.
pub fn self_exclude(center: Point) -> Vec<(Unit, Unit)> {
    vec![center_key(center)]
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_geom::MM;

    #[test]
    fn isolated_circle_has_four_free_dirs() {
        let mut node = Node::new();
        let shape = PadShape::Circle(Circle::new(Point::new(0, 0), MM / 2));
        node.add(Item::Pad {
            shape: shape.clone(),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
            hole_diameter: None,
        });
        let n = free_spoke_directions_in_node(
            &shape,
            LayerId::FCu,
            thermal::SPOKE_WIDTH,
            &node,
            &self_exclude(Point::new(0, 0)),
        );
        assert_eq!(n, 4);
    }

    #[test]
    fn neighbouring_pad_blocks_facing_dirs() {
        let mut node = Node::new();
        let r = MM / 2;
        let a = PadShape::Circle(Circle::new(Point::new(0, 0), r));
        let b = PadShape::Circle(Circle::new(Point::new((1.05 * MM as f64) as Unit, 0), r));
        node.add(Item::Pad {
            shape: a.clone(),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
            hole_diameter: None,
        });
        node.add(Item::Pad {
            shape: b,
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
            hole_diameter: None,
        });
        let n = free_spoke_directions_in_node(
            &a,
            LayerId::FCu,
            thermal::SPOKE_WIDTH,
            &node,
            &self_exclude(Point::new(0, 0)),
        );
        assert!(n < 4, "neighbour should block at least +X; got {n}");
    }
}
